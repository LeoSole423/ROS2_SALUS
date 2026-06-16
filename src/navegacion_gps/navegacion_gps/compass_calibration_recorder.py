from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Any, Optional

import rclpy
from interfaces.msg import DriveTelemetry
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64, String

from navegacion_gps.compass_heading_gate import compass_hdg_deg_to_yaw_enu_deg
from navegacion_gps.heading_math import circular_mean_deg
from navegacion_gps.heading_math import normalize_yaw_deg
from navegacion_gps.heading_math import shortest_angular_distance_deg
from navegacion_gps.heading_math import yaw_deg_from_quaternion_xyzw


SCHEMA_VERSION = 1
TOOL_NAME = "compass_calibration_recorder"


@dataclass(frozen=True)
class CalibrationThresholds:
    min_valid_samples: int = 10
    min_speed_mps: float = 0.5
    max_abs_steer_deg: float = 5.0
    max_abs_yaw_rate_rps: float = 0.08
    max_bias_std_deg: float = 3.0
    max_pair_age_s: float = 1.0


@dataclass(frozen=True)
class CalibrationTopics:
    compass_hdg: str = "/mavros_node/compass_hdg"
    mag: str = "/mavros_node/mag"
    gps_course_heading: str = "/gps/course_heading"
    gps_course_heading_debug: str = "/gps/course_heading/debug"
    drive_telemetry: str = "/controller/drive_telemetry"
    odometry_global: str = "/odometry/global"
    odometry_local: str = "/odometry/local"
    imu: str = "/imu/data"


@dataclass
class ComparisonSample:
    stamp_s: float
    compass_hdg_deg: float
    compass_yaw_enu_deg: float
    gps_course_yaw_enu_deg: float
    delta_yaw_compass_minus_gps_deg: float
    speed_mps: Optional[float]
    steer_deg: Optional[float]
    yaw_rate_rps: Optional[float]
    rtk_status: str


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _is_finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(float(value))


def _stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + (float(stamp.nanosec) / 1_000_000_000.0)


def _float_or_none(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _mean(values: list[float]) -> Optional[float]:
    if not values:
        return None
    return float(sum(values) / len(values))


def _stddev(values: list[float]) -> Optional[float]:
    if len(values) < 2:
        return 0.0 if values else None
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return float(math.sqrt(variance))


def _linear_summary(values: list[float]) -> dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "mean": None,
            "stddev": None,
            "min": None,
            "max": None,
        }
    return {
        "count": len(values),
        "mean": _mean(values),
        "stddev": _stddev(values),
        "min": float(min(values)),
        "max": float(max(values)),
    }


def _angle_summary(values_deg: list[float]) -> dict[str, Any]:
    if not values_deg:
        return {
            "count": 0,
            "mean_deg": None,
            "stddev_deg": None,
            "max_abs_error_deg": None,
            "min_deg": None,
            "max_deg": None,
        }
    mean_deg = circular_mean_deg(values_deg)
    if mean_deg is None:
        return {
            "count": len(values_deg),
            "mean_deg": None,
            "stddev_deg": None,
            "max_abs_error_deg": None,
            "min_deg": float(min(values_deg)),
            "max_deg": float(max(values_deg)),
        }
    errors = [
        shortest_angular_distance_deg(mean_deg, value)
        for value in values_deg
    ]
    return {
        "count": len(values_deg),
        "mean_deg": float(mean_deg),
        "stddev_deg": _stddev(errors),
        "max_abs_error_deg": float(max(abs(error) for error in errors)),
        "min_deg": float(min(values_deg)),
        "max_deg": float(max(values_deg)),
    }


def parse_gps_debug_payload(data: str) -> dict[str, Any]:
    try:
        payload = json.loads(str(data))
    except (TypeError, ValueError):
        return {"valid": False, "parse_error": True}
    if not isinstance(payload, dict):
        return {"valid": False, "parse_error": True}
    payload.setdefault("valid", False)
    return payload


def _confidence(
    enough_data: bool,
    stddev_deg: Optional[float],
    count: int,
) -> str:
    if not enough_data or stddev_deg is None:
        return "insufficient"
    if count >= 30 and stddev_deg <= 2.0:
        return "high"
    if stddev_deg <= 3.0:
        return "medium"
    return "low"


class CompassCalibrationCollector:
    def __init__(
        self,
        *,
        label: str,
        duration_s: float,
        thresholds: CalibrationThresholds,
        topics: CalibrationTopics,
    ) -> None:
        self.label = str(label)
        self.duration_s = float(duration_s)
        self.thresholds = thresholds
        self.topics = topics
        self.started_at = _utc_now_iso()
        self.finished_at: Optional[str] = None

        self.counts: dict[str, int] = {
            "compass": 0,
            "mag": 0,
            "gps_course_heading": 0,
            "gps_course_heading_debug": 0,
            "drive_telemetry": 0,
            "odometry_global": 0,
            "odometry_local": 0,
            "imu": 0,
            "valid_comparison": 0,
        }
        self.invalid_reasons: dict[str, int] = {}

        self.compass_hdg_deg: list[float] = []
        self.compass_yaw_enu_deg: list[float] = []
        self.gps_course_yaw_enu_deg: list[float] = []
        self.delta_yaw_compass_minus_gps_deg: list[float] = []
        self.mag_norm_ut: list[float] = []
        self.drive_speed_mps: list[float] = []
        self.drive_steer_deg: list[float] = []
        self.imu_yaw_rate_rps: list[float] = []
        self.odom_global_yaw_deg: list[float] = []
        self.odom_local_yaw_deg: list[float] = []

        self.valid_samples: list[ComparisonSample] = []
        self.last_gps_debug: dict[str, Any] = {"valid": False}
        self.last_compass_stamp_s: Optional[float] = None
        self.last_compass_hdg_deg: Optional[float] = None
        self.last_compass_yaw_enu_deg: Optional[float] = None
        self.last_speed_mps: Optional[float] = None
        self.last_steer_deg: Optional[float] = None
        self.last_yaw_rate_rps: Optional[float] = None

    def finish(self) -> None:
        self.finished_at = _utc_now_iso()

    def _reject(self, reason: str) -> None:
        self.invalid_reasons[reason] = self.invalid_reasons.get(reason, 0) + 1

    def add_compass(self, compass_hdg_deg: float, stamp_s: float) -> None:
        if not math.isfinite(float(compass_hdg_deg)):
            self._reject("compass_non_finite")
            return
        compass_hdg = float(compass_hdg_deg) % 360.0
        yaw_enu = compass_hdg_deg_to_yaw_enu_deg(compass_hdg)
        self.counts["compass"] += 1
        self.last_compass_stamp_s = float(stamp_s)
        self.last_compass_hdg_deg = compass_hdg
        self.last_compass_yaw_enu_deg = yaw_enu
        self.compass_hdg_deg.append(compass_hdg)
        self.compass_yaw_enu_deg.append(yaw_enu)

    def add_mag(self, x_t: float, y_t: float, z_t: float) -> None:
        components = (float(x_t), float(y_t), float(z_t))
        if not all(math.isfinite(value) for value in components):
            self._reject("mag_non_finite")
            return
        self.counts["mag"] += 1
        norm_t = math.sqrt(sum(value * value for value in components))
        self.mag_norm_ut.append(float(norm_t * 1.0e6))

    def add_gps_debug(self, payload: dict[str, Any]) -> None:
        self.counts["gps_course_heading_debug"] += 1
        self.last_gps_debug = dict(payload)

    def add_drive_telemetry(
        self,
        *,
        speed_mps: Optional[float],
        steer_deg: Optional[float],
    ) -> None:
        self.counts["drive_telemetry"] += 1
        if _is_finite(speed_mps):
            self.last_speed_mps = float(speed_mps)
            self.drive_speed_mps.append(float(speed_mps))
        if _is_finite(steer_deg):
            self.last_steer_deg = float(steer_deg)
            self.drive_steer_deg.append(float(steer_deg))

    def add_imu_yaw_rate(self, yaw_rate_rps: float) -> None:
        if not math.isfinite(float(yaw_rate_rps)):
            self._reject("imu_yaw_rate_non_finite")
            return
        self.counts["imu"] += 1
        self.last_yaw_rate_rps = float(yaw_rate_rps)
        self.imu_yaw_rate_rps.append(float(yaw_rate_rps))

    def add_odom_global_yaw(self, yaw_deg: float) -> None:
        if math.isfinite(float(yaw_deg)):
            self.counts["odometry_global"] += 1
            self.odom_global_yaw_deg.append(normalize_yaw_deg(yaw_deg))

    def add_odom_local_yaw(self, yaw_deg: float) -> None:
        if math.isfinite(float(yaw_deg)):
            self.counts["odometry_local"] += 1
            self.odom_local_yaw_deg.append(normalize_yaw_deg(yaw_deg))

    def add_gps_course_heading(
        self,
        yaw_enu_deg: float,
        stamp_s: float,
    ) -> None:
        if not math.isfinite(float(yaw_enu_deg)):
            self._reject("gps_heading_non_finite")
            return
        self.counts["gps_course_heading"] += 1
        gps_yaw = normalize_yaw_deg(yaw_enu_deg)
        self.gps_course_yaw_enu_deg.append(gps_yaw)

        if not bool(self.last_gps_debug.get("valid", False)):
            self._reject("gps_debug_invalid")
            return
        if (
            self.last_compass_stamp_s is None
            or self.last_compass_hdg_deg is None
            or self.last_compass_yaw_enu_deg is None
        ):
            self._reject("compass_missing")
            return
        compass_age_s = abs(float(stamp_s) - float(self.last_compass_stamp_s))
        if compass_age_s > self.thresholds.max_pair_age_s:
            self._reject("compass_pair_stale")
            return
        if not _is_finite(self.last_speed_mps):
            self._reject("speed_missing")
            return
        if abs(float(self.last_speed_mps)) < self.thresholds.min_speed_mps:
            self._reject("speed_low")
            return
        if (
            _is_finite(self.last_steer_deg)
            and abs(float(self.last_steer_deg))
            > self.thresholds.max_abs_steer_deg
        ):
            self._reject("steer_high")
            return
        if (
            _is_finite(self.last_yaw_rate_rps)
            and abs(float(self.last_yaw_rate_rps))
            > self.thresholds.max_abs_yaw_rate_rps
        ):
            self._reject("yaw_rate_high")
            return

        delta = shortest_angular_distance_deg(
            gps_yaw,
            self.last_compass_yaw_enu_deg,
        )
        self.delta_yaw_compass_minus_gps_deg.append(delta)
        self.counts["valid_comparison"] += 1
        self.valid_samples.append(
            ComparisonSample(
                stamp_s=float(stamp_s),
                compass_hdg_deg=float(self.last_compass_hdg_deg),
                compass_yaw_enu_deg=float(self.last_compass_yaw_enu_deg),
                gps_course_yaw_enu_deg=float(gps_yaw),
                delta_yaw_compass_minus_gps_deg=float(delta),
                speed_mps=self.last_speed_mps,
                steer_deg=self.last_steer_deg,
                yaw_rate_rps=self.last_yaw_rate_rps,
                rtk_status=str(
                    self.last_gps_debug.get(
                        "rtk_status_normalized",
                        self.last_gps_debug.get("rtk_status", ""),
                    )
                    or ""
                ),
            )
        )

    def build_report(self, *, include_samples: bool = False) -> dict[str, Any]:
        delta_summary = _angle_summary(self.delta_yaw_compass_minus_gps_deg)
        delta_mean = delta_summary["mean_deg"]
        delta_stddev = delta_summary["stddev_deg"]
        valid_count = self.counts["valid_comparison"]
        enough_data = (
            valid_count >= self.thresholds.min_valid_samples
            and delta_mean is not None
            and delta_stddev is not None
            and float(delta_stddev) <= self.thresholds.max_bias_std_deg
        )
        recommended_yaw_bias_deg = (
            normalize_yaw_deg(-float(delta_mean))
            if delta_mean is not None
            else None
        )
        recommended_compass_hdg_bias_deg = (
            normalize_yaw_deg(float(delta_mean))
            if delta_mean is not None
            else None
        )
        confidence = _confidence(
            bool(enough_data),
            float(delta_stddev) if delta_stddev is not None else None,
            valid_count,
        )

        action: str
        if enough_data:
            action = (
                "Bias estimate is stable enough for a controlled offset test."
            )
        elif valid_count < self.thresholds.min_valid_samples:
            action = (
                "Collect another straight RTK run with more valid samples."
            )
        else:
            action = (
                "Bias is noisy; inspect magnetic interference or repeat in "
                "another heading."
            )

        report: dict[str, Any] = {
            "schema_version": SCHEMA_VERSION,
            "tool": TOOL_NAME,
            "metadata": {
                "label": self.label,
                "duration_s": self.duration_s,
                "started_at": self.started_at,
                "finished_at": self.finished_at or _utc_now_iso(),
                "thresholds": asdict(self.thresholds),
                "topics": asdict(self.topics),
            },
            "sample_counts": dict(self.counts),
            "invalid_reasons": dict(sorted(self.invalid_reasons.items())),
            "summaries": {
                "compass_hdg_deg": _angle_summary(self.compass_hdg_deg),
                "compass_yaw_enu_deg": _angle_summary(
                    self.compass_yaw_enu_deg
                ),
                "gps_course_yaw_enu_deg": _angle_summary(
                    self.gps_course_yaw_enu_deg
                ),
                "delta_compass_minus_gps_yaw_deg": delta_summary,
                "mag_norm_uT": _linear_summary(self.mag_norm_ut),
                "drive_speed_mps": _linear_summary(self.drive_speed_mps),
                "drive_steer_deg": _linear_summary(self.drive_steer_deg),
                "imu_yaw_rate_rps": _linear_summary(self.imu_yaw_rate_rps),
                "odom_global_yaw_deg": _angle_summary(
                    self.odom_global_yaw_deg
                ),
                "odom_local_yaw_deg": _angle_summary(self.odom_local_yaw_deg),
            },
            "comparison": {
                "definition": (
                    "delta_yaw_compass_minus_gps = compass_yaw_enu - "
                    "gps_course_yaw_enu"
                ),
                "compass_yaw_enu_definition": (
                    "normalize(90 - compass_hdg)"
                ),
                "valid_filter": {
                    "gps_debug_valid": True,
                    "min_speed_mps": self.thresholds.min_speed_mps,
                    "max_abs_steer_deg": self.thresholds.max_abs_steer_deg,
                    "max_abs_yaw_rate_rps": (
                        self.thresholds.max_abs_yaw_rate_rps
                    ),
                    "max_pair_age_s": self.thresholds.max_pair_age_s,
                },
            },
            "recommendation": {
                "enough_data": bool(enough_data),
                "confidence": confidence,
                "recommended_yaw_bias_deg": recommended_yaw_bias_deg,
                "recommended_compass_hdg_bias_deg": (
                    recommended_compass_hdg_bias_deg
                ),
                "action": action,
            },
            "last_debug": {
                "gps_course_heading": dict(self.last_gps_debug),
            },
        }
        if include_samples:
            report["valid_samples"] = [
                asdict(sample) for sample in self.valid_samples
            ]
        else:
            report["valid_sample_preview"] = {
                "first": (
                    asdict(self.valid_samples[0])
                    if self.valid_samples
                    else None
                ),
                "last": (
                    asdict(self.valid_samples[-1])
                    if self.valid_samples
                    else None
                ),
            }
        return report


class CompassCalibrationRecorderNode(Node):
    def __init__(self, collector: CompassCalibrationCollector) -> None:
        super().__init__(TOOL_NAME)
        self.collector = collector
        topics = collector.topics
        self.create_subscription(
            Float64,
            topics.compass_hdg,
            self._on_compass_hdg,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            MagneticField,
            topics.mag,
            self._on_mag,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            topics.gps_course_heading,
            self._on_gps_course_heading,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            topics.gps_course_heading_debug,
            self._on_gps_course_heading_debug,
            10,
        )
        self.create_subscription(
            DriveTelemetry,
            topics.drive_telemetry,
            self._on_drive_telemetry,
            10,
        )
        self.create_subscription(
            Imu,
            topics.imu,
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            topics.odometry_global,
            self._on_odometry_global,
            10,
        )
        self.create_subscription(
            Odometry,
            topics.odometry_local,
            self._on_odometry_local,
            10,
        )

    def _node_time_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1.0e9

    def _message_stamp_s(self, msg: Any) -> float:
        stamp_s = _stamp_to_seconds(msg.header.stamp)
        if stamp_s <= 0.0:
            return self._node_time_s()
        return stamp_s

    def _on_compass_hdg(self, msg: Float64) -> None:
        self.collector.add_compass(float(msg.data), self._node_time_s())

    def _on_mag(self, msg: MagneticField) -> None:
        self.collector.add_mag(
            float(msg.magnetic_field.x),
            float(msg.magnetic_field.y),
            float(msg.magnetic_field.z),
        )

    def _on_gps_course_heading_debug(self, msg: String) -> None:
        self.collector.add_gps_debug(parse_gps_debug_payload(str(msg.data)))

    def _on_drive_telemetry(self, msg: DriveTelemetry) -> None:
        speed = (
            float(msg.speed_mps_measured) if bool(msg.speed_valid) else None
        )
        steer = (
            float(msg.steer_deg_measured) if bool(msg.steer_valid) else None
        )
        self.collector.add_drive_telemetry(speed_mps=speed, steer_deg=steer)

    def _on_imu(self, msg: Imu) -> None:
        self.collector.add_imu_yaw_rate(float(msg.angular_velocity.z))

    def _on_odometry_global(self, msg: Odometry) -> None:
        try:
            q = msg.pose.pose.orientation
            yaw = yaw_deg_from_quaternion_xyzw(
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )
        except ValueError:
            return
        self.collector.add_odom_global_yaw(yaw)

    def _on_odometry_local(self, msg: Odometry) -> None:
        try:
            q = msg.pose.pose.orientation
            yaw = yaw_deg_from_quaternion_xyzw(
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )
        except ValueError:
            return
        self.collector.add_odom_local_yaw(yaw)

    def _on_gps_course_heading(self, msg: Imu) -> None:
        try:
            q = msg.orientation
            yaw = yaw_deg_from_quaternion_xyzw(
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )
        except ValueError:
            self.collector._reject("gps_heading_bad_quaternion")
            return
        self.collector.add_gps_course_heading(yaw, self._message_stamp_s(msg))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record passive compass-vs-GPS heading calibration data and emit "
            "agent-readable JSON."
        )
    )
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--label", default="")
    parser.add_argument("--output", default="")
    parser.add_argument("--include-samples", action="store_true")
    parser.add_argument("--min-valid-samples", type=int, default=10)
    parser.add_argument("--min-speed-mps", type=float, default=0.5)
    parser.add_argument("--max-abs-steer-deg", type=float, default=5.0)
    parser.add_argument("--max-abs-yaw-rate-rps", type=float, default=0.08)
    parser.add_argument("--max-bias-std-deg", type=float, default=3.0)
    parser.add_argument("--max-pair-age-s", type=float, default=1.0)
    parser.add_argument(
        "--compass-hdg-topic",
        default=CalibrationTopics.compass_hdg,
    )
    parser.add_argument("--mag-topic", default=CalibrationTopics.mag)
    parser.add_argument(
        "--gps-course-heading-topic",
        default=CalibrationTopics.gps_course_heading,
    )
    parser.add_argument(
        "--gps-course-heading-debug-topic",
        default=CalibrationTopics.gps_course_heading_debug,
    )
    parser.add_argument(
        "--drive-telemetry-topic",
        default=CalibrationTopics.drive_telemetry,
    )
    parser.add_argument(
        "--odometry-global-topic",
        default=CalibrationTopics.odometry_global,
    )
    parser.add_argument(
        "--odometry-local-topic",
        default=CalibrationTopics.odometry_local,
    )
    parser.add_argument("--imu-topic", default=CalibrationTopics.imu)
    return parser


def _write_report(path: str, report_json: str) -> None:
    output_path = Path(path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(report_json + "\n", encoding="utf-8")


def main(args: Optional[list[str]] = None) -> None:
    cli_args = remove_ros_args(args=args)
    parser_args = cli_args[1:] if args is None and cli_args else cli_args
    parsed = build_arg_parser().parse_args(parser_args)
    duration_s = max(0.1, float(parsed.duration_s))
    thresholds = CalibrationThresholds(
        min_valid_samples=max(1, int(parsed.min_valid_samples)),
        min_speed_mps=max(0.0, float(parsed.min_speed_mps)),
        max_abs_steer_deg=max(0.0, float(parsed.max_abs_steer_deg)),
        max_abs_yaw_rate_rps=max(0.0, float(parsed.max_abs_yaw_rate_rps)),
        max_bias_std_deg=max(0.0, float(parsed.max_bias_std_deg)),
        max_pair_age_s=max(0.0, float(parsed.max_pair_age_s)),
    )
    topics = CalibrationTopics(
        compass_hdg=str(parsed.compass_hdg_topic),
        mag=str(parsed.mag_topic),
        gps_course_heading=str(parsed.gps_course_heading_topic),
        gps_course_heading_debug=str(parsed.gps_course_heading_debug_topic),
        drive_telemetry=str(parsed.drive_telemetry_topic),
        odometry_global=str(parsed.odometry_global_topic),
        odometry_local=str(parsed.odometry_local_topic),
        imu=str(parsed.imu_topic),
    )
    collector = CompassCalibrationCollector(
        label=str(parsed.label),
        duration_s=duration_s,
        thresholds=thresholds,
        topics=topics,
    )

    rclpy.init(args=args)
    node = CompassCalibrationRecorderNode(collector)
    try:
        end_time_s = node.get_clock().now().nanoseconds / 1.0e9 + duration_s
        while (
            rclpy.ok()
            and (node.get_clock().now().nanoseconds / 1.0e9) < end_time_s
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        collector.finish()
        node.destroy_node()
        rclpy.shutdown()

    report = collector.build_report(
        include_samples=bool(parsed.include_samples)
    )
    report_json = json.dumps(report, indent=2, sort_keys=True)
    print(report_json)
    if parsed.output:
        _write_report(str(parsed.output), report_json)


if __name__ == "__main__":
    main()
