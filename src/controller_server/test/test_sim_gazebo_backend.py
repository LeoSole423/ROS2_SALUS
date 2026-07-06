import math
import time

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from controller_server.control_logic import DesiredCommand
from controller_server.rpy_esp32_comms.controller import CommandState
from controller_server.rpy_esp32_comms.telemetry import ControlSource
from controller_server.sim_gazebo_backend import (
    OdomSample,
    SIM_BATTERY_PRESETS,
    SimGazeboBackend,
    build_battery_status_flags,
    build_status_flags,
    select_physical_steering_angle_rad,
    steering_angle_from_wheel_angles,
    synthesize_telemetry,
    translate_command_state_to_gazebo_twist,
)
from controller_server.transport_backends import create_transport_backend


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeNode:
    def __init__(self) -> None:
        self.publishers = []
        self.subscriptions = []
        self.timers = []

    def create_publisher(self, _msg_type, _topic: str, _qos: int):
        pub = _FakePublisher()
        self.publishers.append(pub)
        return pub

    def create_subscription(self, _msg_type, _topic: str, callback, _qos: int):
        handle = {"callback": callback}
        self.subscriptions.append(handle)
        return handle

    def create_timer(self, _period_s: float, callback):
        handle = {"callback": callback}
        self.timers.append(handle)
        return handle


def test_translate_command_state_to_gazebo_twist_inverts_actuation_sign() -> None:
    command_state = CommandState(
        drive_enabled=True,
        estop=False,
        steer_pct=-50,
        speed_mps=0.8,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
    )

    linear_x, steering_angle = translate_command_state_to_gazebo_twist(
        command_state=command_state,
        max_steering_angle_rad=0.5235987756,
        invert_actuation_steer_sign=True,
    )

    assert math.isclose(linear_x, 0.8, rel_tol=0.0, abs_tol=1.0e-9)
    assert math.isclose(steering_angle, 0.2617993878, rel_tol=0.0, abs_tol=1.0e-9)


def test_select_physical_steering_angle_prefers_joint_state_over_odom() -> None:
    left_joint_angle = math.radians(13.0)
    right_joint_angle = math.radians(11.0)
    steering_angle = select_physical_steering_angle_rad(
        left_joint_angle_rad=left_joint_angle,
        right_joint_angle_rad=right_joint_angle,
        odom_linear_x_mps=1.0,
        odom_angular_z_rps=0.8,
        max_joint_odom_delta_rad=math.radians(30.0),
    )

    expected = steering_angle_from_wheel_angles(
        left_joint_angle_rad=left_joint_angle,
        right_joint_angle_rad=right_joint_angle,
    )
    assert expected is not None
    assert math.isclose(steering_angle, expected, rel_tol=0.0, abs_tol=1.0e-9)


@pytest.mark.parametrize(
    ("left_joint_deg", "right_joint_deg", "expected_center_deg"),
    [
        (23.064121, 17.631289, 20.0),
        (-17.631289, -23.064121, -20.0),
    ],
)
def test_steering_angle_from_wheel_angles_recovers_center_angle(
    left_joint_deg: float,
    right_joint_deg: float,
    expected_center_deg: float,
) -> None:
    steering_angle = steering_angle_from_wheel_angles(
        left_joint_angle_rad=math.radians(left_joint_deg),
        right_joint_angle_rad=math.radians(right_joint_deg),
    )

    assert steering_angle == pytest.approx(math.radians(expected_center_deg), abs=1.0e-6)


def test_select_physical_steering_angle_uses_odom_when_joint_estimate_disagrees() -> None:
    steering_angle = select_physical_steering_angle_rad(
        left_joint_angle_rad=math.radians(2.0),
        right_joint_angle_rad=math.radians(2.0),
        odom_linear_x_mps=1.0,
        odom_angular_z_rps=math.tan(math.radians(18.0)) / 0.94,
        max_joint_odom_delta_rad=math.radians(5.0),
    )

    assert steering_angle == pytest.approx(math.radians(18.0), abs=1.0e-6)


def test_synthesize_telemetry_marks_pi_and_inverts_measured_sign() -> None:
    command_state = CommandState(
        drive_enabled=True,
        estop=False,
        steer_pct=-40,
        speed_mps=0.7,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
    )
    odom_sample = OdomSample(
        linear_x_mps=0.7,
        linear_y_mps=0.0,
        angular_z_rps=0.1,
        rx_monotonic_s=__import__("time").monotonic(),
    )

    telemetry = synthesize_telemetry(
        command_state=command_state,
        odom_sample=odom_sample,
        left_joint_angle_rad=math.radians(12.7),
        right_joint_angle_rad=math.radians(11.3),
        invert_measured_steer_sign=True,
        telemetry_timeout_s=0.5,
    )

    assert telemetry is not None
    assert telemetry.control_source == ControlSource.PI
    assert telemetry.ready is True
    assert telemetry.pi_fresh is True
    assert telemetry.speed_mps == pytest.approx(0.7)
    assert telemetry.steer_deg == pytest.approx(-12.0)


def test_build_status_flags_encodes_bits() -> None:
    flags = build_status_flags(
        ready=True,
        estop_active=True,
        pi_fresh=True,
        control_source=ControlSource.PI,
    )
    assert flags & (1 << 0)
    assert flags & (1 << 1)
    assert flags & (1 << 3)
    assert ((flags >> 4) & 0x03) == int(ControlSource.PI)


def test_build_battery_status_flags_encodes_bits() -> None:
    flags = build_battery_status_flags(
        ready=True,
        fresh=True,
        suspect=True,
        calibrated=True,
    )

    assert flags & (1 << 0)
    assert flags & (1 << 1)
    assert flags & (1 << 2)
    assert flags & (1 << 3)


def test_create_transport_backend_builds_sim_backend() -> None:
    backend = create_transport_backend(
        node=_FakeNode(),
        transport_backend="sim_gazebo",
        serial_port="/dev/null",
        serial_baud=115200,
        serial_tx_hz=50.0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        sim_cmd_vel_topic="/cmd_vel_gazebo",
        sim_odom_topic="/odom_raw",
        sim_joint_states_topic="/joint_states",
        sim_front_left_steer_joint="front_left_steer_joint",
        sim_front_right_steer_joint="front_right_steer_joint",
        sim_wheelbase_m=0.94,
        sim_track_width_m=0.75,
        sim_max_steering_angle_rad=0.5235987756,
        sim_telemetry_timeout_s=0.5,
        sim_invert_actuation_steer_sign=True,
        sim_invert_measured_steer_sign=True,
        sim_max_joint_odom_steer_delta_deg=5.0,
    )

    assert isinstance(backend, SimGazeboBackend)


def test_create_transport_backend_rejects_unknown_backend() -> None:
    with pytest.raises(ValueError):
        create_transport_backend(
            node=_FakeNode(),
            transport_backend="unknown",
            serial_port="/dev/null",
            serial_baud=115200,
            serial_tx_hz=50.0,
            max_speed_mps=4.0,
            max_reverse_mps=1.3,
            sim_cmd_vel_topic="/cmd_vel_gazebo",
            sim_odom_topic="/odom_raw",
            sim_joint_states_topic="/joint_states",
            sim_front_left_steer_joint="front_left_steer_joint",
            sim_front_right_steer_joint="front_right_steer_joint",
            sim_wheelbase_m=0.94,
            sim_track_width_m=0.75,
            sim_max_steering_angle_rad=0.5235987756,
            sim_telemetry_timeout_s=0.5,
            sim_invert_actuation_steer_sign=True,
            sim_invert_measured_steer_sign=True,
            sim_max_joint_odom_steer_delta_deg=5.0,
        )


def test_sim_gazebo_backend_publishes_actuation_and_telemetry() -> None:
    fake_node = _FakeNode()
    backend = SimGazeboBackend(
        node=fake_node,
        tx_hz=50.0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        cmd_vel_topic="/cmd_vel_gazebo",
        odom_topic="/odom_raw",
        joint_states_topic="/joint_states",
        front_left_steer_joint="front_left_steer_joint",
        front_right_steer_joint="front_right_steer_joint",
        wheelbase_m=0.94,
        track_width_m=0.75,
        max_steering_angle_rad=0.5235987756,
        telemetry_timeout_s=0.5,
        invert_actuation_steer_sign=True,
        invert_measured_steer_sign=True,
    )
    backend.start()

    joint_msg = JointState()
    joint_msg.name = ["front_left_steer_joint", "front_right_steer_joint"]
    joint_msg.position = [0.2, 0.2]
    backend._on_joint_states(joint_msg)

    odom_msg = Odometry()
    odom_msg.twist.twist.linear.x = 0.8
    odom_msg.twist.twist.angular.z = 0.1
    backend._on_odom(odom_msg)

    backend.apply_command(
        DesiredCommand(
            drive_enabled=True,
            estop=False,
            speed_mps=0.8,
            steer_pct=-50,
            brake_pct=0,
        )
    )
    backend._publish_current_command()

    assert len(fake_node.publishers) == 1
    published = fake_node.publishers[0].messages[-1]
    assert published.linear.x == pytest.approx(0.8)
    assert published.angular.z == pytest.approx(0.2617993878)

    telemetry = backend.get_latest_telemetry()
    assert telemetry is not None
    assert telemetry.control_source == ControlSource.PI
    assert telemetry.speed_mps == pytest.approx(0.8)
    assert telemetry.steer_deg == pytest.approx(-math.degrees(0.2), abs=0.1)


def test_sim_gazebo_backend_battery_preset_generates_loaded_voltage() -> None:
    backend = SimGazeboBackend(
        node=_FakeNode(),
        tx_hz=50.0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        cmd_vel_topic="/cmd_vel_gazebo",
        odom_topic="/odom_raw",
        joint_states_topic="/joint_states",
        front_left_steer_joint="front_left_steer_joint",
        front_right_steer_joint="front_right_steer_joint",
        wheelbase_m=0.94,
        track_width_m=0.75,
        max_steering_angle_rad=0.5235987756,
        telemetry_timeout_s=0.5,
        invert_actuation_steer_sign=True,
        invert_measured_steer_sign=True,
    )

    preset = backend.set_sim_battery_preset("under_load")
    battery = backend.get_latest_battery_telemetry()

    assert preset == SIM_BATTERY_PRESETS["under_load"]
    assert battery is not None
    assert battery.ready is True
    assert battery.fresh is True
    assert battery.suspect is False
    assert battery.calibrated is True
    assert battery.battery_voltage_v == pytest.approx(59.3)
    assert battery.adc_pin_voltage_v == pytest.approx(59.3 / 25.0)


def test_sim_gazebo_backend_battery_state_uses_recovered_voltage_when_traction_disabled() -> None:
    backend = SimGazeboBackend(
        node=_FakeNode(),
        tx_hz=50.0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        cmd_vel_topic="/cmd_vel_gazebo",
        odom_topic="/odom_raw",
        joint_states_topic="/joint_states",
        front_left_steer_joint="front_left_steer_joint",
        front_right_steer_joint="front_right_steer_joint",
        wheelbase_m=0.94,
        track_width_m=0.75,
        max_steering_angle_rad=0.5235987756,
        telemetry_timeout_s=0.5,
        invert_actuation_steer_sign=True,
        invert_measured_steer_sign=True,
    )

    backend.set_sim_battery_state(
        recovered_voltage_v=57.0,
        loaded_voltage_v=56.2,
        traction_active_override=False,
        ready=True,
        fresh=False,
        suspect=True,
    )
    battery = backend.get_latest_battery_telemetry()

    assert battery is not None
    assert battery.battery_voltage_v == pytest.approx(57.0)
    assert battery.ready is True
    assert battery.fresh is False
    assert battery.suspect is True


def test_sim_gazebo_backend_fresh_battery_refreshes_rx_timestamp() -> None:
    backend = SimGazeboBackend(
        node=_FakeNode(),
        tx_hz=50.0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        cmd_vel_topic="/cmd_vel_gazebo",
        odom_topic="/odom_raw",
        joint_states_topic="/joint_states",
        front_left_steer_joint="front_left_steer_joint",
        front_right_steer_joint="front_right_steer_joint",
        wheelbase_m=0.94,
        track_width_m=0.75,
        max_steering_angle_rad=0.5235987756,
        telemetry_timeout_s=0.5,
        invert_actuation_steer_sign=True,
        invert_measured_steer_sign=True,
    )

    backend.set_sim_battery_preset("full")
    first = backend.get_latest_battery_telemetry()
    time.sleep(0.02)
    second = backend.get_latest_battery_telemetry()

    assert first is not None
    assert second is not None
    assert first.fresh is True
    assert second.fresh is True
    assert first.sample_age_s == pytest.approx(0.0)
    assert second.sample_age_s == pytest.approx(0.0)
    assert second.rx_monotonic_s >= first.rx_monotonic_s
