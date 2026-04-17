from interfaces.msg import DriveTelemetry
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu

from navegacion_gps.global_imu_stationary_gate import (
    GlobalImuStationaryGateNode,
    qos_profile_sensor_data,
)


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeGateNode:
    _on_imu = GlobalImuStationaryGateNode._on_imu
    _stationary_gate_active = GlobalImuStationaryGateNode._stationary_gate_active
    _zero_angular_velocity = staticmethod(GlobalImuStationaryGateNode._zero_angular_velocity)

    def __init__(self) -> None:
        self._stationary_speed_threshold_mps = 0.03
        self._drive_telemetry_timeout_s = 0.5
        self._last_drive_telemetry = None
        self._last_drive_telemetry_monotonic_s = None
        self._imu_pub = _FakePublisher()
        self._now_s = 100.0

    def _monotonic_now_s(self) -> float:
        return float(self._now_s)


def _make_drive(
    *,
    speed_mps: float,
    fresh: bool = True,
    speed_valid: bool = True,
) -> DriveTelemetry:
    msg = DriveTelemetry()
    msg.fresh = bool(fresh)
    msg.speed_valid = bool(speed_valid)
    msg.speed_mps_measured = float(speed_mps)
    return msg


def _make_imu() -> Imu:
    msg = Imu()
    msg.angular_velocity.x = 0.01
    msg.angular_velocity.y = -0.02
    msg.angular_velocity.z = 0.03
    return msg


def test_gate_zeroes_imu_angular_velocity_when_stationary() -> None:
    node = _FakeGateNode()
    node._last_drive_telemetry = _make_drive(speed_mps=0.01)
    node._last_drive_telemetry_monotonic_s = 99.8

    node._on_imu(_make_imu())

    published = node._imu_pub.messages[-1]
    assert published.angular_velocity.x == 0.0
    assert published.angular_velocity.y == 0.0
    assert published.angular_velocity.z == 0.0


def test_gate_passes_through_imu_when_vehicle_is_moving() -> None:
    node = _FakeGateNode()
    node._last_drive_telemetry = _make_drive(speed_mps=0.12)
    node._last_drive_telemetry_monotonic_s = 99.8

    node._on_imu(_make_imu())

    published = node._imu_pub.messages[-1]
    assert published.angular_velocity.x == 0.01
    assert published.angular_velocity.y == -0.02
    assert published.angular_velocity.z == 0.03


def test_gate_passes_through_imu_when_drive_telemetry_is_stale() -> None:
    node = _FakeGateNode()
    node._last_drive_telemetry = _make_drive(speed_mps=0.0)
    node._last_drive_telemetry_monotonic_s = 99.0

    node._on_imu(_make_imu())

    published = node._imu_pub.messages[-1]
    assert published.angular_velocity.x == 0.01
    assert published.angular_velocity.y == -0.02
    assert published.angular_velocity.z == 0.03


def test_gate_uses_sensor_data_qos_for_imu_input() -> None:
    assert qos_profile_sensor_data.reliability == ReliabilityPolicy.BEST_EFFORT
