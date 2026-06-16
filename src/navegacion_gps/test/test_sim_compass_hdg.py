import math

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu

from navegacion_gps.sim_compass_hdg import SimCompassHdgNode
from navegacion_gps.sim_compass_hdg import yaw_enu_deg_to_compass_hdg_deg


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeSimCompass:
    _on_imu = SimCompassHdgNode._on_imu
    _compass_heading_from_yaw = SimCompassHdgNode._compass_heading_from_yaw
    _on_publish_timer = SimCompassHdgNode._on_publish_timer

    def __init__(self) -> None:
        import random

        self._noise_stddev_deg = 0.0
        self._bias_deg = 0.0
        self._initial_yaw_offset_deg = 0.0
        self._rng = random.Random(1)
        self._last_yaw_enu_deg = None
        self._pub = _FakePublisher()


def _imu_from_yaw_deg(yaw_deg: float) -> Imu:
    msg = Imu()
    half = math.radians(float(yaw_deg)) * 0.5
    msg.header.stamp = Time(sec=1)
    msg.orientation.z = math.sin(half)
    msg.orientation.w = math.cos(half)
    return msg


def test_yaw_enu_to_compass_hdg_conversion() -> None:
    assert yaw_enu_deg_to_compass_hdg_deg(90.0) == 0.0
    assert yaw_enu_deg_to_compass_hdg_deg(0.0) == 90.0
    assert yaw_enu_deg_to_compass_hdg_deg(-90.0) == 180.0
    assert yaw_enu_deg_to_compass_hdg_deg(180.0) == 270.0


def test_sim_compass_publishes_heading_from_imu_with_bias() -> None:
    node = _FakeSimCompass()
    node._bias_deg = 5.0
    node._on_imu(_imu_from_yaw_deg(90.0))

    node._on_publish_timer()

    assert math.isclose(node._pub.messages[-1].data, 5.0)


def test_sim_compass_applies_initial_spawn_yaw_offset() -> None:
    node = _FakeSimCompass()
    node._initial_yaw_offset_deg = 90.0
    node._on_imu(_imu_from_yaw_deg(0.0))

    node._on_publish_timer()

    assert math.isclose(node._pub.messages[-1].data, 0.0)
