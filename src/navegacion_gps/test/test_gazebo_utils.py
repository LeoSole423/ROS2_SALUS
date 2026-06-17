from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from interfaces.msg import CmdVelFinal
from navegacion_gps.gazebo_utils import GazeboUtilsNode
from navegacion_gps.gps_profiles import (
    SimGpsFixProcessor,
    build_custom_gps_profile,
    resolve_gps_profile,
)


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeCmdBridgeNode:
    def __init__(self, enabled: bool) -> None:
        self.cmd_vel_gazebo_pub = _FakePublisher() if enabled else None
        self.use_realistic_cmd_vel_bridge = False
        self.max_speed_mps = 4.0
        self.max_reverse_mps = 1.3
        self.vx_deadband_mps = 0.10
        self.vx_min_effective_mps = 0.75
        self.max_abs_angular_z = 0.4
        self.operational_steering_limit_rad = 0.3141592654
        self.invert_steer_from_cmd_vel = False
        self.auto_drive_enabled = True
        self.reverse_brake_pct = 20
        self.sim_max_forward_mps = 4.0
        self.sim_max_reverse_mps = 1.3
        self.sim_max_abs_angular_z = 0.4
        self.sim_max_steering_angle_rad = 0.5235987756

    _publish_cmd_vel_gazebo = GazeboUtilsNode._publish_cmd_vel_gazebo
    _translate_cmd_vel_final_to_gazebo = GazeboUtilsNode._translate_cmd_vel_final_to_gazebo


class _FakeFrameNode:
    _strip = GazeboUtilsNode._strip
    _resolve_frame = GazeboUtilsNode._resolve_frame

    def __init__(self, strip_prefix: bool) -> None:
        self.strip_prefix = strip_prefix
        self.odom_frame_id = "odom"
        self.base_link_frame_id = "base_footprint"
        self.odom_pub = _FakePublisher()


class _FakeGpsNode:
    _strip = GazeboUtilsNode._strip
    _resolve_frame = GazeboUtilsNode._resolve_frame
    _get_reference_time_ns = GazeboUtilsNode._get_reference_time_ns
    _apply_realistic_gps = GazeboUtilsNode._apply_realistic_gps
    _should_hold_gps = GazeboUtilsNode._should_hold_gps
    _gps_cb = GazeboUtilsNode._gps_cb

    def __init__(self, *, profile_name: str = "m8n", hold_when_stationary: bool = False) -> None:
        self.strip_prefix = True
        self.gps_frame_id = "gps_link"
        self.gps_pub = _FakePublisher()
        self.gps_rtk_status_pub = _FakePublisher()
        self.gps_hold_when_stationary = hold_when_stationary
        self.gps_hold_linear_speed_threshold_mps = 0.02
        self.gps_hold_yaw_rate_threshold_rps = 0.01
        self.use_realistic_gps = profile_name == "legacy_realistic"
        self.gps_publish_rate_hz = 5.0
        self.gps_publish_jitter_stddev_s = 0.0
        self.gps_horizontal_noise_stddev_m = 0.35
        self.gps_vertical_noise_stddev_m = 0.75
        self.gps_bias_walk_stddev_m_per_sqrt_s = 0.02
        if profile_name == "legacy_realistic":
            profile = build_custom_gps_profile(
                name="legacy_realistic",
                publish_rate_hz=self.gps_publish_rate_hz,
                publish_jitter_stddev_s=self.gps_publish_jitter_stddev_s,
                horizontal_noise_stddev_m=self.gps_horizontal_noise_stddev_m,
                vertical_noise_stddev_m=self.gps_vertical_noise_stddev_m,
                bias_walk_stddev_m_per_sqrt_s=self.gps_bias_walk_stddev_m_per_sqrt_s,
                navsat_status=NavSatFix().status.STATUS_FIX,
                rtk_status_text="3D_FIX",
                description="legacy",
            )
        else:
            profile = resolve_gps_profile(profile_name)
        self._gps_profile = profile
        self._gps_processor = SimGpsFixProcessor(profile, random_seed=123)
        self._last_odom_linear_speed_mps = 0.0
        self._last_odom_yaw_rate_rps = 0.0
        self._last_gps_out = None

    def get_clock(self):
        class _FakeClock:
            class _Now:
                nanoseconds = 0

            @staticmethod
            def now():
                return _FakeClock._Now()

        return _FakeClock()


def test_cmd_vel_final_without_brake_is_forwarded() -> None:
    node = _FakeCmdBridgeNode(enabled=True)
    msg = CmdVelFinal()
    msg.twist.linear.x = 1.1
    msg.twist.angular.z = -0.4
    msg.brake_pct = 0

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    assert len(node.cmd_vel_gazebo_pub.messages) == 1
    published = node.cmd_vel_gazebo_pub.messages[-1]
    assert isinstance(published, Twist)
    assert float(published.linear.x) == 1.1
    assert float(published.angular.z) == -0.4


def test_realistic_bridge_applies_min_effective_speed() -> None:
    node = _FakeCmdBridgeNode(enabled=True)
    node.use_realistic_cmd_vel_bridge = True
    msg = CmdVelFinal()
    msg.twist.linear.x = 0.2
    msg.twist.angular.z = 0.0
    msg.brake_pct = 0

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    published = node.cmd_vel_gazebo_pub.messages[-1]
    assert float(published.linear.x) == 0.75
    assert float(published.angular.z) == 0.0


def test_realistic_bridge_preserves_curvature_when_min_effective_speed_applies() -> None:
    node = _FakeCmdBridgeNode(enabled=True)
    node.use_realistic_cmd_vel_bridge = True
    node.vx_deadband_mps = 0.01
    node.vx_min_effective_mps = 0.5
    msg = CmdVelFinal()
    msg.twist.linear.x = 0.1
    msg.twist.angular.z = 0.08
    msg.brake_pct = 0

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    published = node.cmd_vel_gazebo_pub.messages[-1]
    assert float(published.linear.x) == 0.5
    assert float(published.angular.z) == 0.24


def test_realistic_bridge_applies_deadband_and_zeroes_small_reverse() -> None:
    node = _FakeCmdBridgeNode(enabled=True)
    node.use_realistic_cmd_vel_bridge = True
    msg = CmdVelFinal()
    msg.twist.linear.x = -0.05
    msg.twist.angular.z = 0.0
    msg.brake_pct = 0

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    published = node.cmd_vel_gazebo_pub.messages[-1]
    assert float(published.linear.x) == 0.0
    assert float(published.angular.z) == 0.0


def test_realistic_bridge_clamps_speed_and_angular_mapping() -> None:
    node = _FakeCmdBridgeNode(enabled=True)
    node.use_realistic_cmd_vel_bridge = True
    node.sim_max_forward_mps = 3.0
    node.sim_max_abs_angular_z = 0.6
    msg = CmdVelFinal()
    msg.twist.linear.x = 8.0
    msg.twist.angular.z = 1.0
    msg.brake_pct = 0

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    published = node.cmd_vel_gazebo_pub.messages[-1]
    assert float(published.linear.x) == 3.0
    assert float(published.angular.z) == 0.132


def test_realistic_bridge_clamps_reverse_speed() -> None:
    node = _FakeCmdBridgeNode(enabled=True)
    node.use_realistic_cmd_vel_bridge = True
    node.sim_max_reverse_mps = 1.0
    msg = CmdVelFinal()
    msg.twist.linear.x = -2.2
    msg.twist.angular.z = -0.4
    msg.brake_pct = 0

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    published = node.cmd_vel_gazebo_pub.messages[-1]
    assert float(published.linear.x) == -1.0
    assert float(published.angular.z) == 0.128


def test_cmd_vel_final_with_brake_publishes_zero_twist() -> None:
    node = _FakeCmdBridgeNode(enabled=True)
    node.use_realistic_cmd_vel_bridge = True
    msg = CmdVelFinal()
    msg.twist.linear.x = 0.9
    msg.twist.angular.z = 0.2
    msg.brake_pct = 35

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    published = node.cmd_vel_gazebo_pub.messages[-1]
    assert float(published.linear.x) == 0.0
    assert float(published.angular.z) == 0.0


def test_cmd_vel_bridge_disabled_does_not_publish() -> None:
    node = _FakeCmdBridgeNode(enabled=False)
    msg = CmdVelFinal()
    msg.twist.linear.x = 0.8
    msg.twist.angular.z = 0.1
    msg.brake_pct = 0

    GazeboUtilsNode._cmd_vel_final_cb(node, msg)

    assert node.cmd_vel_gazebo_pub is None


def test_odom_callback_preserves_frame_normalization_logic() -> None:
    node = _FakeFrameNode(strip_prefix=True)
    msg = Odometry()
    msg.header.frame_id = "model::odom"
    msg.child_frame_id = "model::base_footprint"

    GazeboUtilsNode._odom_cb(node, msg)

    assert len(node.odom_pub.messages) == 1
    published = node.odom_pub.messages[-1]
    assert published.header.frame_id == "odom"
    assert published.child_frame_id == "base_footprint"


def test_realistic_gps_adds_noise_and_covariance() -> None:
    node = _FakeGpsNode(profile_name="m8n")
    msg = NavSatFix()
    msg.header.stamp.sec = 10
    msg.header.frame_id = "model::gps_link"
    msg.latitude = -31.4858037
    msg.longitude = -64.2410570
    msg.altitude = 0.0

    GazeboUtilsNode._gps_cb(node, msg)

    assert len(node.gps_pub.messages) == 1
    published = node.gps_pub.messages[-1]
    assert published.header.frame_id == "gps_link"
    assert published.latitude != msg.latitude
    assert published.longitude != msg.longitude
    assert published.position_covariance_type == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    assert published.position_covariance[0] == 1.5**2
    assert published.position_covariance[8] == 2.5**2
    assert node.gps_rtk_status_pub.messages[-1].data == "3D_FIX"


def test_realistic_gps_throttles_publish_rate() -> None:
    node = _FakeGpsNode(profile_name="m8n")

    msg1 = NavSatFix()
    msg1.header.stamp.sec = 10
    msg1.latitude = -31.4858037
    msg1.longitude = -64.2410570

    msg2 = NavSatFix()
    msg2.header.stamp.sec = 10
    msg2.header.stamp.nanosec = 100_000_000
    msg2.latitude = -31.4858037
    msg2.longitude = -64.2410570

    msg3 = NavSatFix()
    msg3.header.stamp.sec = 10
    msg3.header.stamp.nanosec = 250_000_000
    msg3.latitude = -31.4858037
    msg3.longitude = -64.2410570

    GazeboUtilsNode._gps_cb(node, msg1)
    GazeboUtilsNode._gps_cb(node, msg2)
    GazeboUtilsNode._gps_cb(node, msg3)

    assert len(node.gps_pub.messages) == 2
    assert node.gps_rtk_status_pub.messages[-1].data == "3D_FIX"


def test_ideal_gps_profile_keeps_position_and_marks_sim_ideal() -> None:
    node = _FakeGpsNode(profile_name="ideal")
    msg = NavSatFix()
    msg.header.stamp.sec = 10
    msg.header.frame_id = "model::gps_link"
    msg.latitude = -31.4858037
    msg.longitude = -64.2410570
    msg.altitude = 12.0

    GazeboUtilsNode._gps_cb(node, msg)

    assert len(node.gps_pub.messages) == 1
    published = node.gps_pub.messages[-1]
    assert published.latitude == msg.latitude
    assert published.longitude == msg.longitude
    assert published.altitude == msg.altitude
    assert published.position_covariance[0] == 0.01**2
    assert published.position_covariance[8] == 0.02**2
    assert node.gps_rtk_status_pub.messages[-1].data == "SIM_IDEAL"


def test_gazebo_utils_holds_f9p_rtk_when_stationary() -> None:
    node = _FakeGpsNode(profile_name="f9p_rtk", hold_when_stationary=True)
    msg1 = NavSatFix()
    msg1.header.stamp.sec = 10
    msg1.header.frame_id = "model::gps_link"
    msg1.latitude = -31.4858037
    msg1.longitude = -64.2410570
    msg1.altitude = 0.0

    msg2 = NavSatFix()
    msg2.header.stamp.sec = 10
    msg2.header.stamp.nanosec = 100_000_000
    msg2.header.frame_id = "model::gps_link"
    msg2.latitude = -31.4858037
    msg2.longitude = -64.2410570
    msg2.altitude = 0.0

    GazeboUtilsNode._gps_cb(node, msg1)
    GazeboUtilsNode._gps_cb(node, msg2)

    first = node.gps_pub.messages[0]
    second = node.gps_pub.messages[1]
    assert second.latitude == first.latitude
    assert second.longitude == first.longitude
    assert second.altitude == first.altitude
    assert second.header.stamp.nanosec == 100_000_000
