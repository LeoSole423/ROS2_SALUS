import json
import math

from builtin_interfaces.msg import Time
from interfaces.msg import DriveTelemetry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String

from navegacion_gps.compass_heading_gate import CompassHeadingGateNode
from navegacion_gps.compass_heading_gate import compass_hdg_deg_to_yaw_enu_deg
from navegacion_gps.heading_math import yaw_deg_from_quaternion_xyzw


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeClock:
    def now(self):
        return self

    def to_msg(self) -> Time:
        return Time(sec=10)


class _FakeCompassGate:
    _on_compass_hdg = CompassHeadingGateNode._on_compass_hdg
    _on_imu = CompassHeadingGateNode._on_imu
    _on_drive_telemetry = CompassHeadingGateNode._on_drive_telemetry
    _on_gps_heading_debug = CompassHeadingGateNode._on_gps_heading_debug
    _age_s = CompassHeadingGateNode._age_s
    _stationary_drive_gate_active = (
        CompassHeadingGateNode._stationary_drive_gate_active
    )
    _gps_heading_gate_blocks = CompassHeadingGateNode._gps_heading_gate_blocks
    _acceptance_reason = CompassHeadingGateNode._acceptance_reason
    _build_imu_measurement = CompassHeadingGateNode._build_imu_measurement
    _publish_debug = CompassHeadingGateNode._publish_debug
    _on_publish_timer = CompassHeadingGateNode._on_publish_timer

    def __init__(self) -> None:
        self._base_frame = "base_footprint"
        self._yaw_variance_rad2 = 1.0
        self._startup_window_s = 10.0
        self._stationary_publish_after_s = 30.0
        self._stationary_speed_threshold_mps = 0.05
        self._max_abs_yaw_rate_rps = 0.05
        self._compass_max_age_s = 0.75
        self._imu_max_age_s = 0.75
        self._drive_telemetry_timeout_s = 0.75
        self._gps_heading_debug_timeout_s = 1.0
        self._block_when_gps_heading_valid = True
        self._max_abs_jump_deg = 45.0
        self._start_monotonic_s = 95.0
        self._last_compass_hdg_deg = None
        self._last_compass_monotonic_s = None
        self._last_imu_yaw_rate_rps = None
        self._last_imu_monotonic_s = None
        self._last_drive_telemetry = None
        self._last_drive_monotonic_s = None
        self._stationary_since_monotonic_s = None
        self._last_accepted_yaw_deg = None
        self._gps_heading_valid = False
        self._last_gps_debug_monotonic_s = None
        self._imu_pub = _FakePublisher()
        self._debug_pub = _FakePublisher()
        self._now_s = 100.0

    def _monotonic_now_s(self) -> float:
        return float(self._now_s)

    def get_clock(self) -> _FakeClock:
        return _FakeClock()


def _make_drive(speed_mps: float, *, fresh: bool = True) -> DriveTelemetry:
    msg = DriveTelemetry()
    msg.fresh = bool(fresh)
    msg.speed_valid = True
    msg.speed_mps_measured = float(speed_mps)
    return msg


def _make_imu(yaw_rate_rps: float) -> Imu:
    msg = Imu()
    msg.angular_velocity.z = float(yaw_rate_rps)
    return msg


def _prime_stationary_gate(
    node: _FakeCompassGate,
    *,
    compass_hdg_deg: float = 0.0,
) -> None:
    node._on_compass_hdg(Float64(data=float(compass_hdg_deg)))
    node._on_imu(_make_imu(0.0))
    node._on_drive_telemetry(_make_drive(0.0))


def test_compass_hdg_to_yaw_enu_conversion() -> None:
    assert compass_hdg_deg_to_yaw_enu_deg(0.0) == 90.0
    assert compass_hdg_deg_to_yaw_enu_deg(90.0) == 0.0
    assert compass_hdg_deg_to_yaw_enu_deg(180.0) == -90.0
    west_yaw_abs_deg = abs(compass_hdg_deg_to_yaw_enu_deg(270.0))
    assert abs(west_yaw_abs_deg - 180.0) < 1.0e-6


def test_gate_publishes_yaw_only_imu_during_startup_stationary() -> None:
    node = _FakeCompassGate()
    _prime_stationary_gate(node, compass_hdg_deg=0.0)

    node._on_publish_timer()

    published = node._imu_pub.messages[-1]
    yaw_deg = yaw_deg_from_quaternion_xyzw(
        published.orientation.x,
        published.orientation.y,
        published.orientation.z,
        published.orientation.w,
    )
    assert math.isclose(yaw_deg, 90.0)
    assert published.header.frame_id == "base_footprint"
    assert published.orientation_covariance[0] == 1.0e6
    assert published.orientation_covariance[4] == 1.0e6
    assert published.orientation_covariance[8] == 1.0
    assert published.angular_velocity_covariance[0] == -1.0
    assert published.linear_acceleration_covariance[0] == -1.0


def test_gate_blocks_when_vehicle_is_moving() -> None:
    node = _FakeCompassGate()
    node._on_compass_hdg(Float64(data=0.0))
    node._on_imu(_make_imu(0.0))
    node._on_drive_telemetry(_make_drive(0.2))

    node._on_publish_timer()

    assert node._imu_pub.messages == []
    debug = json.loads(node._debug_pub.messages[-1].data)
    assert debug["reason"] == "not_stationary"


def test_gate_blocks_when_yaw_rate_is_high() -> None:
    node = _FakeCompassGate()
    node._on_compass_hdg(Float64(data=0.0))
    node._on_imu(_make_imu(0.2))
    node._on_drive_telemetry(_make_drive(0.0))

    node._on_publish_timer()

    assert node._imu_pub.messages == []
    debug = json.loads(node._debug_pub.messages[-1].data)
    assert debug["reason"] == "yaw_rate_high"


def test_gate_blocks_when_gps_heading_is_valid() -> None:
    node = _FakeCompassGate()
    _prime_stationary_gate(node, compass_hdg_deg=0.0)
    node._on_gps_heading_debug(String(data=json.dumps({"valid": True})))

    node._on_publish_timer()

    assert node._imu_pub.messages == []
    debug = json.loads(node._debug_pub.messages[-1].data)
    assert debug["reason"] == "gps_heading_valid"


def test_gate_blocks_stale_compass_and_large_jumps() -> None:
    stale = _FakeCompassGate()
    _prime_stationary_gate(stale, compass_hdg_deg=0.0)
    stale._now_s = 101.0

    stale._on_publish_timer()

    assert stale._imu_pub.messages == []
    stale_debug = json.loads(stale._debug_pub.messages[-1].data)
    assert stale_debug["reason"] == "compass_stale"

    jumped = _FakeCompassGate()
    jumped._max_abs_jump_deg = 10.0
    jumped._last_accepted_yaw_deg = 0.0
    _prime_stationary_gate(jumped, compass_hdg_deg=0.0)

    jumped._on_publish_timer()

    assert jumped._imu_pub.messages == []
    jump_debug = json.loads(jumped._debug_pub.messages[-1].data)
    assert jump_debug["reason"] == "compass_jump"
