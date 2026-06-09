from __future__ import annotations

import random
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from navegacion_gps.heading_math import yaw_deg_from_quaternion_xyzw


def normalize_heading_deg(heading_deg: float) -> float:
    return float(heading_deg) % 360.0


def yaw_enu_deg_to_compass_hdg_deg(yaw_enu_deg: float) -> float:
    return normalize_heading_deg(90.0 - float(yaw_enu_deg))


class SimCompassHdgNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_compass_hdg")

        self.declare_parameter("input_imu_topic", "/imu/data")
        self.declare_parameter("output_topic", "/sim/compass_hdg")
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("noise_stddev_deg", 0.0)
        self.declare_parameter("bias_deg", 0.0)
        self.declare_parameter("initial_yaw_offset_deg", 0.0)
        self.declare_parameter("seed", 1)

        input_imu_topic = str(self.get_parameter("input_imu_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self._noise_stddev_deg = max(
            0.0, float(self.get_parameter("noise_stddev_deg").value)
        )
        self._bias_deg = float(self.get_parameter("bias_deg").value)
        self._initial_yaw_offset_deg = float(
            self.get_parameter("initial_yaw_offset_deg").value
        )
        self._rng = random.Random(int(self.get_parameter("seed").value))
        self._last_yaw_enu_deg: Optional[float] = None

        self._pub = self.create_publisher(Float64, output_topic, 10)
        self.create_subscription(
            Imu,
            input_imu_topic,
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / publish_hz, self._on_publish_timer)
        self.get_logger().info(
            "sim_compass_hdg ready "
            f"(imu={input_imu_topic}, output={output_topic}, "
            f"bias={self._bias_deg:.2f}deg, "
            f"initial_yaw={self._initial_yaw_offset_deg:.2f}deg, "
            f"noise={self._noise_stddev_deg:.2f}deg)"
        )

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        try:
            self._last_yaw_enu_deg = yaw_deg_from_quaternion_xyzw(
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )
        except ValueError:
            return

    def _compass_heading_from_yaw(self, yaw_enu_deg: float) -> float:
        noise_deg = 0.0
        if self._noise_stddev_deg > 0.0:
            noise_deg = self._rng.gauss(0.0, self._noise_stddev_deg)
        global_yaw_enu_deg = yaw_enu_deg + self._initial_yaw_offset_deg
        return normalize_heading_deg(
            yaw_enu_deg_to_compass_hdg_deg(global_yaw_enu_deg)
            + self._bias_deg
            + noise_deg
        )

    def _on_publish_timer(self) -> None:
        if self._last_yaw_enu_deg is None:
            return
        compass_hdg_deg = self._compass_heading_from_yaw(
            self._last_yaw_enu_deg
        )
        self._pub.publish(
            Float64(data=compass_hdg_deg)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimCompassHdgNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
