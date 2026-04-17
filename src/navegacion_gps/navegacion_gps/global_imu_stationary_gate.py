from __future__ import annotations

from copy import deepcopy
import time
from typing import Optional

import rclpy
from interfaces.msg import DriveTelemetry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class GlobalImuStationaryGateNode(Node):
    def __init__(self) -> None:
        super().__init__("global_imu_stationary_gate")

        self.declare_parameter("input_imu_topic", "/imu/data")
        self.declare_parameter("output_imu_topic", "/imu/data_global")
        self.declare_parameter("drive_telemetry_topic", "/controller/drive_telemetry")
        self.declare_parameter("stationary_speed_threshold_mps", 0.03)
        self.declare_parameter("drive_telemetry_timeout_s", 0.5)

        input_imu_topic = str(self.get_parameter("input_imu_topic").value)
        output_imu_topic = str(self.get_parameter("output_imu_topic").value)
        drive_telemetry_topic = str(self.get_parameter("drive_telemetry_topic").value)

        self._stationary_speed_threshold_mps = max(
            0.0,
            float(self.get_parameter("stationary_speed_threshold_mps").value),
        )
        self._drive_telemetry_timeout_s = max(
            0.0,
            float(self.get_parameter("drive_telemetry_timeout_s").value),
        )
        self._last_drive_telemetry: Optional[DriveTelemetry] = None
        self._last_drive_telemetry_monotonic_s: Optional[float] = None

        self._imu_pub = self.create_publisher(Imu, output_imu_topic, 10)
        # Match the canonical ROS 2 sensor QoS so the gate can consume
        # BEST_EFFORT IMU publishers such as MAVROS.
        self.create_subscription(
            Imu,
            input_imu_topic,
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DriveTelemetry,
            drive_telemetry_topic,
            self._on_drive_telemetry,
            10,
        )

        self.get_logger().info(
            "global_imu_stationary_gate ready "
            f"({input_imu_topic} -> {output_imu_topic}, "
            f"drive={drive_telemetry_topic}, "
            f"speed_threshold={self._stationary_speed_threshold_mps:.3f}m/s, "
            f"timeout={self._drive_telemetry_timeout_s:.3f}s)"
        )

    def _monotonic_now_s(self) -> float:
        return time.monotonic()

    def _on_drive_telemetry(self, msg: DriveTelemetry) -> None:
        self._last_drive_telemetry = msg
        self._last_drive_telemetry_monotonic_s = self._monotonic_now_s()

    def _stationary_gate_active(self) -> bool:
        msg = self._last_drive_telemetry
        if msg is None or self._last_drive_telemetry_monotonic_s is None:
            return False
        if (
            self._monotonic_now_s() - self._last_drive_telemetry_monotonic_s
        ) > self._drive_telemetry_timeout_s:
            return False
        if not bool(msg.fresh) or not bool(msg.speed_valid):
            return False
        return abs(float(msg.speed_mps_measured)) <= self._stationary_speed_threshold_mps

    @staticmethod
    def _zero_angular_velocity(msg: Imu) -> Imu:
        gated = deepcopy(msg)
        gated.angular_velocity.x = 0.0
        gated.angular_velocity.y = 0.0
        gated.angular_velocity.z = 0.0
        return gated

    def _on_imu(self, msg: Imu) -> None:
        if self._stationary_gate_active():
            self._imu_pub.publish(self._zero_angular_velocity(msg))
            return
        self._imu_pub.publish(deepcopy(msg))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalImuStationaryGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
