from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from interfaces.msg import DriveTelemetry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String

from navegacion_gps.heading_math import normalize_yaw_deg
from navegacion_gps.heading_math import shortest_angular_distance_deg


def compass_hdg_deg_to_yaw_enu_deg(compass_hdg_deg: float) -> float:
    return normalize_yaw_deg(90.0 - float(compass_hdg_deg))


def quaternion_xyzw_from_yaw_deg(
    yaw_deg: float,
) -> tuple[float, float, float, float]:
    yaw_rad = math.radians(float(yaw_deg))
    half_yaw = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class CompassHeadingGateNode(Node):
    def __init__(self) -> None:
        super().__init__("compass_heading_gate")

        self.declare_parameter("compass_hdg_topic", "/mavros_node/compass_hdg")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter(
            "drive_telemetry_topic",
            "/controller/drive_telemetry",
        )
        self.declare_parameter(
            "gps_course_heading_debug_topic",
            "/gps/course_heading/debug",
        )
        self.declare_parameter("output_topic", "/imu/compass_heading")
        self.declare_parameter("debug_topic", "/imu/compass_heading/debug")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("yaw_variance_rad2", 1.0)
        self.declare_parameter("startup_window_s", 10.0)
        self.declare_parameter("stationary_publish_after_s", 30.0)
        self.declare_parameter("stationary_speed_threshold_mps", 0.05)
        self.declare_parameter("max_abs_yaw_rate_rps", 0.05)
        self.declare_parameter("compass_max_age_s", 0.75)
        self.declare_parameter("imu_max_age_s", 0.75)
        self.declare_parameter("drive_telemetry_timeout_s", 0.75)
        self.declare_parameter("gps_heading_debug_timeout_s", 1.0)
        self.declare_parameter("block_when_gps_heading_valid", True)
        self.declare_parameter("max_abs_jump_deg", 45.0)

        compass_hdg_topic = str(self.get_parameter("compass_hdg_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        drive_telemetry_topic = str(
            self.get_parameter("drive_telemetry_topic").value
        )
        gps_debug_topic = str(
            self.get_parameter("gps_course_heading_debug_topic").value
        )
        output_topic = str(self.get_parameter("output_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)

        self._base_frame = str(self.get_parameter("base_frame").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self._yaw_variance_rad2 = max(
            1.0e-6, float(self.get_parameter("yaw_variance_rad2").value)
        )
        self._startup_window_s = max(
            0.0,
            float(self.get_parameter("startup_window_s").value),
        )
        self._stationary_publish_after_s = max(
            0.0,
            float(self.get_parameter("stationary_publish_after_s").value),
        )
        self._stationary_speed_threshold_mps = max(
            0.0,
            float(self.get_parameter("stationary_speed_threshold_mps").value),
        )
        self._max_abs_yaw_rate_rps = max(
            0.0, float(self.get_parameter("max_abs_yaw_rate_rps").value)
        )
        self._compass_max_age_s = max(
            0.05, float(self.get_parameter("compass_max_age_s").value)
        )
        self._imu_max_age_s = max(
            0.05,
            float(self.get_parameter("imu_max_age_s").value),
        )
        self._drive_telemetry_timeout_s = max(
            0.05,
            float(self.get_parameter("drive_telemetry_timeout_s").value),
        )
        self._gps_heading_debug_timeout_s = max(
            0.05,
            float(self.get_parameter("gps_heading_debug_timeout_s").value),
        )
        self._block_when_gps_heading_valid = bool(
            self.get_parameter("block_when_gps_heading_valid").value
        )
        self._max_abs_jump_deg = max(
            0.0,
            float(self.get_parameter("max_abs_jump_deg").value),
        )

        self._start_monotonic_s = time.monotonic()
        self._last_compass_hdg_deg: Optional[float] = None
        self._last_compass_monotonic_s: Optional[float] = None
        self._last_imu_yaw_rate_rps: Optional[float] = None
        self._last_imu_monotonic_s: Optional[float] = None
        self._last_drive_telemetry: Optional[DriveTelemetry] = None
        self._last_drive_monotonic_s: Optional[float] = None
        self._stationary_since_monotonic_s: Optional[float] = None
        self._last_accepted_yaw_deg: Optional[float] = None
        self._gps_heading_valid = False
        self._last_gps_debug_monotonic_s: Optional[float] = None

        self._imu_pub = self.create_publisher(Imu, output_topic, 10)
        self._debug_pub = self.create_publisher(String, debug_topic, 10)
        self.create_subscription(
            Float64,
            compass_hdg_topic,
            self._on_compass_hdg,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            imu_topic,
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DriveTelemetry,
            drive_telemetry_topic,
            self._on_drive_telemetry,
            10,
        )
        self.create_subscription(
            String,
            gps_debug_topic,
            self._on_gps_heading_debug,
            10,
        )
        self.create_timer(1.0 / publish_hz, self._on_publish_timer)

        self.get_logger().info(
            "compass_heading_gate ready "
            f"(compass={compass_hdg_topic}, imu={imu_topic}, "
            f"drive={drive_telemetry_topic}, output={output_topic})"
        )

    def _monotonic_now_s(self) -> float:
        return time.monotonic()

    def _on_compass_hdg(self, msg: Float64) -> None:
        value = float(msg.data)
        if not math.isfinite(value):
            return
        self._last_compass_hdg_deg = value % 360.0
        self._last_compass_monotonic_s = self._monotonic_now_s()

    def _on_imu(self, msg: Imu) -> None:
        yaw_rate = float(msg.angular_velocity.z)
        if not math.isfinite(yaw_rate):
            return
        self._last_imu_yaw_rate_rps = yaw_rate
        self._last_imu_monotonic_s = self._monotonic_now_s()

    def _on_drive_telemetry(self, msg: DriveTelemetry) -> None:
        now_s = self._monotonic_now_s()
        self._last_drive_telemetry = msg
        self._last_drive_monotonic_s = now_s
        if self._stationary_drive_gate_active(now_s):
            if self._stationary_since_monotonic_s is None:
                self._stationary_since_monotonic_s = now_s
        else:
            self._stationary_since_monotonic_s = None

    def _on_gps_heading_debug(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data))
        except (TypeError, ValueError):
            self._gps_heading_valid = False
            self._last_gps_debug_monotonic_s = self._monotonic_now_s()
            return
        self._gps_heading_valid = bool(payload.get("valid", False))
        self._last_gps_debug_monotonic_s = self._monotonic_now_s()

    def _age_s(
        self,
        now_s: float,
        stamp_s: Optional[float],
    ) -> Optional[float]:
        if stamp_s is None:
            return None
        return max(0.0, float(now_s) - float(stamp_s))

    def _stationary_drive_gate_active(self, now_s: float) -> bool:
        msg = self._last_drive_telemetry
        if msg is None or self._last_drive_monotonic_s is None:
            return False
        age_s = self._age_s(now_s, self._last_drive_monotonic_s)
        if age_s is None or age_s > self._drive_telemetry_timeout_s:
            return False
        if not bool(msg.fresh) or not bool(msg.speed_valid):
            return False
        return (
            abs(float(msg.speed_mps_measured))
            <= self._stationary_speed_threshold_mps
        )

    def _gps_heading_gate_blocks(self, now_s: float) -> bool:
        if not self._block_when_gps_heading_valid:
            return False
        age_s = self._age_s(now_s, self._last_gps_debug_monotonic_s)
        if age_s is None or age_s > self._gps_heading_debug_timeout_s:
            return False
        return bool(self._gps_heading_valid)

    def _acceptance_reason(
        self,
        now_s: float,
    ) -> tuple[bool, str, Optional[float]]:
        compass_age_s = self._age_s(now_s, self._last_compass_monotonic_s)
        if self._last_compass_hdg_deg is None or compass_age_s is None:
            return (False, "compass_missing", None)
        if compass_age_s > self._compass_max_age_s:
            return (False, "compass_stale", compass_age_s)

        imu_age_s = self._age_s(now_s, self._last_imu_monotonic_s)
        if self._last_imu_yaw_rate_rps is None or imu_age_s is None:
            return (False, "imu_missing", compass_age_s)
        if imu_age_s > self._imu_max_age_s:
            return (False, "imu_stale", compass_age_s)
        if (
            abs(float(self._last_imu_yaw_rate_rps))
            > self._max_abs_yaw_rate_rps
        ):
            return (False, "yaw_rate_high", compass_age_s)

        if not self._stationary_drive_gate_active(now_s):
            return (False, "not_stationary", compass_age_s)
        if self._gps_heading_gate_blocks(now_s):
            return (False, "gps_heading_valid", compass_age_s)

        yaw_deg = compass_hdg_deg_to_yaw_enu_deg(
            float(self._last_compass_hdg_deg)
        )
        if (
            self._last_accepted_yaw_deg is not None
            and self._max_abs_jump_deg > 0.0
        ):
            jump_deg = abs(
                shortest_angular_distance_deg(
                    self._last_accepted_yaw_deg,
                    yaw_deg,
                )
            )
            if jump_deg > self._max_abs_jump_deg:
                return (False, "compass_jump", compass_age_s)

        uptime_s = max(0.0, now_s - self._start_monotonic_s)
        if uptime_s <= self._startup_window_s:
            return (True, "startup_stationary", compass_age_s)
        if self._stationary_since_monotonic_s is not None:
            stationary_duration_s = now_s - self._stationary_since_monotonic_s
            if stationary_duration_s >= self._stationary_publish_after_s:
                return (True, "long_stationary", compass_age_s)
        return (False, "stationary_not_long_enough", compass_age_s)

    def _build_imu_measurement(self, yaw_deg: float) -> Imu:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        qx, qy, qz, qw = quaternion_xyzw_from_yaw_deg(yaw_deg)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance = [
            1.0e6,
            0.0,
            0.0,
            0.0,
            1.0e6,
            0.0,
            0.0,
            0.0,
            float(self._yaw_variance_rad2),
        ]
        msg.angular_velocity_covariance = [
            -1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        msg.linear_acceleration_covariance = [
            -1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        return msg

    def _publish_debug(
        self,
        *,
        valid: bool,
        reason: str,
        yaw_deg: Optional[float],
        compass_age_s: Optional[float],
        now_s: float,
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "valid": bool(valid),
                "reason": str(reason),
                "yaw_deg": yaw_deg,
                "compass_hdg_deg": self._last_compass_hdg_deg,
                "compass_age_s": compass_age_s,
                "yaw_rate_rps": self._last_imu_yaw_rate_rps,
                "stationary": self._stationary_drive_gate_active(now_s),
                "gps_heading_valid": bool(self._gps_heading_valid),
                "base_frame": self._base_frame,
            },
            sort_keys=True,
        )
        self._debug_pub.publish(msg)

    def _on_publish_timer(self) -> None:
        now_s = self._monotonic_now_s()
        valid, reason, compass_age_s = self._acceptance_reason(now_s)
        yaw_deg = None
        if self._last_compass_hdg_deg is not None:
            yaw_deg = compass_hdg_deg_to_yaw_enu_deg(
                float(self._last_compass_hdg_deg)
            )
        self._publish_debug(
            valid=valid,
            reason=reason,
            yaw_deg=yaw_deg,
            compass_age_s=compass_age_s,
            now_s=now_s,
        )
        if not valid or yaw_deg is None:
            return
        self._last_accepted_yaw_deg = yaw_deg
        self._imu_pub.publish(self._build_imu_measurement(yaw_deg))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompassHeadingGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
