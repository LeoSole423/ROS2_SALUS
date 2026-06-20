import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from interfaces.msg import NavEvent, NavTelemetry
from interfaces.srv import GetRouteMissionState
from nav2_msgs.msg import BehaviorTreeLog, CollisionMonitorState
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node


TERMINAL_ROUTE_CODES = {
    "ROUTE_MISSION_COMPLETED",
    "ROUTE_MISSION_CANCELLED",
    "ROUTE_MISSION_FAILED",
}
SIGNIFICANT_CODES = {
    "ROUTE_MISSION_STARTED",
    "ROUTE_CHUNK_REQUESTED",
    "ROUTE_CHUNK_DISPATCHED",
    "ROUTE_CHECKPOINT_REACHED",
    "ROUTE_SYNTHETIC_SKIPPED",
    "ROUTE_MISSION_COMPLETED",
    "ROUTE_MISSION_CANCELLED",
    "ROUTE_MISSION_FAILED",
    "REPLAN_STARTED",
    "REPLAN_FINISHED",
    "REPLAN_FAILED",
    "REPLAN_CANCELLED",
    "CLEARANCE_INVALID",
    "CLEARANCE_SLOW",
    "GOAL_RESULT_ABORTED",
}


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _normalize_angle_rad(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _path_length(points: Sequence[Tuple[float, float]]) -> float:
    return sum(
        math.hypot(end[0] - start[0], end[1] - start[1])
        for start, end in zip(points, points[1:])
    )


def _orientation_yaw(orientation: Any) -> float:
    x = float(getattr(orientation, "x", 0.0))
    y = float(getattr(orientation, "y", 0.0))
    z = float(getattr(orientation, "z", 0.0))
    w = float(getattr(orientation, "w", 1.0))
    return math.atan2(2.0 * ((w * z) + (x * y)), 1.0 - (2.0 * ((y * y) + (z * z))))


def _segments_intersect(
    a: Tuple[float, float],
    b: Tuple[float, float],
    c: Tuple[float, float],
    d: Tuple[float, float],
) -> bool:
    def cross(p: Tuple[float, float], q: Tuple[float, float], r: Tuple[float, float]) -> float:
        return ((q[0] - p[0]) * (r[1] - p[1])) - ((q[1] - p[1]) * (r[0] - p[0]))

    ab_c = cross(a, b, c)
    ab_d = cross(a, b, d)
    cd_a = cross(c, d, a)
    cd_b = cross(c, d, b)
    return (ab_c * ab_d) < 0.0 and (cd_a * cd_b) < 0.0


def _self_intersection_count(points: Sequence[Tuple[float, float]]) -> int:
    if len(points) < 4:
        return 0
    stride = max(1, int(math.ceil(len(points) / 200.0)))
    sampled = list(points[::stride])
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    count = 0
    for first in range(len(sampled) - 1):
        for second in range(first + 2, len(sampled) - 1):
            if second == first + 1:
                continue
            if _segments_intersect(
                sampled[first], sampled[first + 1], sampled[second], sampled[second + 1]
            ):
                count += 1
    return count


def analyze_path_points(
    points: Sequence[Tuple[float, float]],
    *,
    robot_pose: Optional[Tuple[float, float, float]] = None,
    previous_points: Optional[Sequence[Tuple[float, float]]] = None,
) -> Dict[str, Any]:
    clean = [(float(x), float(y)) for x, y in points if math.isfinite(x) and math.isfinite(y)]
    length_m = _path_length(clean)
    headings: List[Tuple[float, float]] = []
    travelled = 0.0
    for start, end in zip(clean, clean[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        if distance <= 1.0e-6:
            continue
        headings.append((travelled, math.atan2(end[1] - start[1], end[0] - start[0])))
        travelled += distance

    turn_first_20m_deg = 0.0
    max_heading_step_deg = 0.0
    for (distance, previous), (_, current) in zip(headings, headings[1:]):
        delta_deg = abs(math.degrees(_normalize_angle_rad(current - previous)))
        max_heading_step_deg = max(max_heading_step_deg, delta_deg)
        if distance <= 20.0:
            turn_first_20m_deg += delta_deg

    start_gap_m = None
    heading_error_deg = None
    if robot_pose is not None and clean:
        start_gap_m = math.hypot(clean[0][0] - robot_pose[0], clean[0][1] - robot_pose[1])
        if headings:
            heading_error_deg = abs(
                math.degrees(_normalize_angle_rad(headings[0][1] - robot_pose[2]))
            )

    mean_change_m = None
    max_change_m = None
    previous = list(previous_points or [])
    if clean and previous:
        sample_count = min(100, len(clean), len(previous))
        if sample_count > 1:
            distances = []
            for index in range(sample_count):
                current_index = round(index * (len(clean) - 1) / (sample_count - 1))
                previous_index = round(index * (len(previous) - 1) / (sample_count - 1))
                distances.append(
                    math.hypot(
                        clean[current_index][0] - previous[previous_index][0],
                        clean[current_index][1] - previous[previous_index][1],
                    )
                )
            mean_change_m = sum(distances) / len(distances)
            max_change_m = max(distances)

    intersections = _self_intersection_count(clean)
    suspected_o = intersections > 0 or turn_first_20m_deg >= 270.0
    return {
        "pose_count": len(clean),
        "length_m": round(length_m, 3),
        "start_gap_m": None if start_gap_m is None else round(start_gap_m, 3),
        "initial_heading_error_deg": (
            None if heading_error_deg is None else round(heading_error_deg, 3)
        ),
        "turn_first_20m_deg": round(turn_first_20m_deg, 3),
        "max_heading_step_deg": round(max_heading_step_deg, 3),
        "self_intersections": intersections,
        "mean_change_from_previous_m": (
            None if mean_change_m is None else round(mean_change_m, 3)
        ),
        "max_change_from_previous_m": (
            None if max_change_m is None else round(max_change_m, 3)
        ),
        "suspected_o_path": suspected_o,
    }


def detect_replan_bursts(records: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    starts = [record for record in records if record.get("code") == "REPLAN_STARTED"]
    bursts: List[Dict[str, Any]] = []
    for index, record in enumerate(starts):
        start_time = float(record.get("t_wall", 0.0))
        window = [
            item
            for item in starts[index:]
            if float(item.get("t_wall", 0.0)) - start_time <= 10.0
        ]
        if len(window) >= 3:
            burst = {
                "start_t": start_time,
                "count": len(window),
                "reasons": [str(item.get("data", {}).get("reason", "unknown")) for item in window],
            }
            if not bursts or burst["start_t"] > bursts[-1]["start_t"] + 10.0:
                bursts.append(burst)
    return bursts


def render_trace_summary(metadata: Dict[str, Any], records: Sequence[Dict[str, Any]]) -> str:
    mission_id = str(metadata.get("mission_id", "unknown"))
    replans = [record for record in records if record.get("code") == "REPLAN_STARTED"]
    reasons: Dict[str, int] = {}
    for record in replans:
        reason = str(record.get("data", {}).get("reason", "unknown"))
        reasons[reason] = reasons.get(reason, 0) + 1
    bursts = detect_replan_bursts(records)
    regressions = [record for record in records if record.get("code") == "PROGRESS_REGRESSION"]
    suspicious_plans = [
        record
        for record in records
        if record.get("kind") == "plan" and bool(record.get("data", {}).get("suspected_o_path"))
    ]
    start_gap_plans = [
        record
        for record in records
        if record.get("kind") == "plan"
        and record.get("data", {}).get("start_gap_m") is not None
        and float(record.get("data", {}).get("start_gap_m", 0.0)) > 2.0
    ]
    heading_mismatch_plans = [
        record
        for record in records
        if record.get("kind") == "plan"
        and record.get("data", {}).get("initial_heading_error_deg") is not None
        and float(record.get("data", {}).get("initial_heading_error_deg", 0.0)) > 60.0
    ]
    synthetic_pass_times = [
        float(record.get("t_wall", 0.0))
        for record in records
        if record.get("code") == "ROUTE_SYNTHETIC_PASSED"
    ]
    replans_near_synthetic = [
        record
        for record in replans
        if any(
            abs(float(record.get("t_wall", 0.0)) - synthetic_time) <= 1.5
            for synthetic_time in synthetic_pass_times
        )
    ]
    aborted = [record for record in records if "ABORTED" in str(record.get("code", ""))]
    clearance_events = [
        record for record in records if str(record.get("code", "")).startswith("CLEARANCE_")
    ]
    clearance_reasons: Dict[str, int] = {}
    for record in clearance_events:
        reason = str(record.get("data", {}).get("reason", "unknown"))
        clearance_reasons[reason] = clearance_reasons.get(reason, 0) + 1

    lines = [
        f"# Navigation trace {mission_id}",
        "",
        f"- Estado: `{metadata.get('status', 'active')}`",
        f"- Inicio UTC: `{metadata.get('started_at_utc', 'unknown')}`",
        f"- Registros: `{len(records)}`",
        f"- Replans: `{len(replans)}`",
        f"- Checkpoints alcanzados: `{sum(record.get('code') == 'ROUTE_CHECKPOINT_REACHED' for record in records)}`",
        "",
        "## Replanning",
        "",
    ]
    if reasons:
        lines.extend(f"- `{reason}`: {count}" for reason, count in sorted(reasons.items()))
    else:
        lines.append("- No se registraron replans.")
    lines.extend(["", "## Clearance Validator", ""])
    if clearance_events:
        lines.append(f"- Eventos: `{len(clearance_events)}`")
        lines.extend(
            f"- `{reason}`: {count}" for reason, count in sorted(clearance_reasons.items())
        )
    else:
        lines.append("- No se registraron eventos del validador.")
    lines.extend(["", "## Anomalías detectadas", ""])
    lines.append(f"- Bursts (>=3 replans/10s): `{len(bursts)}`")
    lines.append(f"- Regresiones de progreso: `{len(regressions)}`")
    lines.append(f"- Paths con posible O: `{len(suspicious_plans)}`")
    lines.append(f"- Paths iniciados a más de 2m del robot: `{len(start_gap_plans)}`")
    lines.append(f"- Paths con error inicial de heading mayor a 60deg: `{len(heading_mismatch_plans)}`")
    lines.append(f"- Replans a +/-1.5s de un synthetic pasado: `{len(replans_near_synthetic)}`")
    lines.append(f"- Aborts: `{len(aborted)}`")

    if bursts:
        lines.extend(["", "### Bursts", ""])
        for burst in bursts:
            lines.append(
                f"- t={burst['start_t']:.3f}: {burst['count']} replans, causas={','.join(burst['reasons'])}"
            )
    if suspicious_plans:
        lines.extend(["", "### Paths sospechosos", ""])
        for record in suspicious_plans:
            data = record.get("data", {})
            lines.append(
                f"- plan `{data.get('plan_id')}`: turn20={data.get('turn_first_20m_deg')}deg, "
                f"intersections={data.get('self_intersections')}, heading_error={data.get('initial_heading_error_deg')}deg"
            )
    if aborted:
        lines.extend(["", "### Contexto previo a abort", ""])
        for abort in aborted:
            abort_time = float(abort.get("t_wall", 0.0))
            previous = [
                record
                for record in records
                if 0.0 <= abort_time - float(record.get("t_wall", 0.0)) <= 5.0
            ]
            lines.append(f"- Abort t={abort_time:.3f}:")
            for record in previous[-20:]:
                lines.append(
                    f"  - {record.get('code', record.get('kind'))} "
                    f"{json.dumps(record.get('data', {}), sort_keys=True)}"
                )

    lines.extend(["", "## Timeline significativa", ""])
    significant = [
        record
        for record in records
        if record.get("code") in SIGNIFICANT_CODES or record.get("code") == "PROGRESS_REGRESSION"
    ]
    for record in significant[-200:]:
        data = record.get("data", {})
        detail = " ".join(
            f"{key}={data[key]}"
            for key in ("reason", "outcome", "input_index", "expanded_index", "error")
            if key in data
        )
        lines.append(
            f"- `{float(record.get('t_wall', 0.0)):.3f}` **{record.get('code', record.get('kind'))}** {detail}".rstrip()
        )
    lines.extend(
        [
            "",
            "## Archivos para análisis",
            "",
            "- `timeline.jsonl`: fuente cronológica completa.",
            "- `plans/`: geometría y métricas de cada path publicado.",
            "- `mission_path.json` y `chunks/`: geometría de ruta y segmentos enviados.",
            "- `context/`: YAML de Nav2 y XML del BT usados en la corrida.",
            "- `metadata.json`: contexto de ejecución y configuración.",
            "",
        ]
    )
    return "\n".join(lines)


def _event_details(msg: NavEvent) -> Dict[str, str]:
    return {
        str(item.key): str(item.value)
        for item in (msg.details or [])
        if str(item.key)
    }


class NavTraceRecorder(Node):
    def __init__(self) -> None:
        super().__init__("nav_trace_recorder")
        self.declare_parameter("output_root", "/ros2_ws/artifacts/nav_traces")
        self.declare_parameter("sample_hz", 2.0)
        self.declare_parameter("route_state_service", "/route_executor/get_state")
        self.declare_parameter("nav2_params_file", "")
        self.declare_parameter("bt_xml_file", "")
        self.declare_parameter("workspace_root", "/ros2_ws")

        self.output_root = Path(str(self.get_parameter("output_root").value))
        self.sample_hz = max(0.2, float(self.get_parameter("sample_hz").value))
        self._lock = threading.RLock()
        self._session_dir: Optional[Path] = None
        self._timeline_file: Optional[Path] = None
        self._metadata: Dict[str, Any] = {}
        self._records: List[Dict[str, Any]] = []
        self._sequence = 0
        self._plan_sequence = 0
        self._last_plan_points: List[Tuple[float, float]] = []
        self._robot_pose: Optional[Tuple[float, float, float]] = None
        self._route_state: Dict[str, Any] = {}
        self._last_progress: Optional[Tuple[int, int]] = None
        self._last_route_signature = ""
        self._last_collision_action: Optional[int] = None
        self._last_nav_result_key: Optional[Tuple[int, int, str]] = None
        self._state_request_inflight = False

        self._route_state_client = self.create_client(
            GetRouteMissionState, str(self.get_parameter("route_state_service").value)
        )
        self.create_subscription(NavEvent, "/nav_command_server/events", self._on_nav_event, 50)
        self.create_subscription(NavEvent, "/navigation_trace/events", self._on_trace_event, 50)
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(NavPath, "/route_executor/mission_path", self._on_mission_path, 10)
        self.create_subscription(NavPath, "/route_executor/active_chunk_path", self._on_chunk_path, 10)
        self.create_subscription(BehaviorTreeLog, "/behavior_tree_log", self._on_bt_log, 20)
        self.create_subscription(Odometry, "/odometry/global", self._on_odometry, 20)
        self.create_subscription(
            CollisionMonitorState, "/collision_monitor_state", self._on_collision_state, 20
        )
        self.create_subscription(
            NavTelemetry, "/nav_command_server/telemetry", self._on_nav_telemetry, 20
        )
        self.create_timer(1.0 / self.sample_hz, self._poll_route_state)
        self.get_logger().info(f"nav trace recorder ready (output_root={self.output_root})")

    def _git_metadata(self) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        try:
            configured_root = Path(str(self.get_parameter("workspace_root").value))
            root = subprocess.check_output(
                ["git", "rev-parse", "--show-toplevel"],
                cwd=configured_root,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=2.0,
            ).strip()
            result["git_root"] = root
            result["git_commit"] = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=root, text=True, timeout=2.0
            ).strip()
            result["git_branch"] = subprocess.check_output(
                ["git", "branch", "--show-current"], cwd=root, text=True, timeout=2.0
            ).strip()
            result["git_dirty"] = bool(
                subprocess.check_output(
                    ["git", "status", "--porcelain"], cwd=root, text=True, timeout=2.0
                ).strip()
            )
        except Exception as exc:
            result["git_error"] = str(exc)
        return result

    def _start_session(self, details: Dict[str, Any]) -> None:
        mission_id = str(details.get("mission_id") or f"unknown_{int(time.time())}")
        if self._session_dir is not None:
            self._finalize_session("superseded")
        stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
        session_dir = self.output_root / f"{stamp}_{mission_id}"
        (session_dir / "plans").mkdir(parents=True, exist_ok=True)
        self._session_dir = session_dir
        self._timeline_file = session_dir / "timeline.jsonl"
        self._records = []
        self._sequence = 0
        self._plan_sequence = 0
        self._last_plan_points = []
        self._last_progress = None
        self._last_route_signature = ""
        self._metadata = {
            "schema_version": 1,
            "mission_id": mission_id,
            "status": "active",
            "started_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
            "route": dict(details),
            "nav2_params_file": str(self.get_parameter("nav2_params_file").value),
            "bt_xml_file": str(self.get_parameter("bt_xml_file").value),
            **self._git_metadata(),
        }
        context_files: Dict[str, Any] = {}
        for label, parameter_name in (
            ("nav2_params", "nav2_params_file"),
            ("behavior_tree", "bt_xml_file"),
        ):
            source = Path(str(self.get_parameter(parameter_name).value))
            if not source.is_file():
                continue
            context_dir = session_dir / "context"
            context_dir.mkdir(exist_ok=True)
            destination = context_dir / source.name
            shutil.copy2(source, destination)
            content = destination.read_bytes()
            context_files[label] = {
                "file": str(destination.relative_to(session_dir)),
                "sha256": hashlib.sha256(content).hexdigest(),
            }
        self._metadata["context_files"] = context_files
        self._write_metadata()

    def _write_metadata(self) -> None:
        if self._session_dir is None:
            return
        (self._session_dir / "metadata.json").write_text(
            json.dumps(self._metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    def _write_summary(self) -> None:
        if self._session_dir is None:
            return
        (self._session_dir / "summary.md").write_text(
            render_trace_summary(self._metadata, self._records), encoding="utf-8"
        )

    def _record(self, kind: str, code: str, data: Optional[Dict[str, Any]] = None) -> None:
        with self._lock:
            if self._timeline_file is None:
                return
            self._sequence += 1
            context = {
                "mission_id": self._metadata.get("mission_id", ""),
                "chunk_id": self._route_state.get("chunk_id", 0),
                "loop_iteration": self._route_state.get("loop_iteration", 0),
            }
            record = _json_safe({
                "schema_version": 1,
                "seq": self._sequence,
                "t_wall": time.time(),
                "t_ros": self.get_clock().now().nanoseconds / 1.0e9,
                "kind": kind,
                "code": code,
                **context,
                "data": dict(data or {}),
            })
            with self._timeline_file.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
            self._records.append(record)
            if code in SIGNIFICANT_CODES or code == "PROGRESS_REGRESSION":
                self._write_summary()

    def _finalize_session(self, status: str) -> None:
        with self._lock:
            if self._session_dir is None:
                return
            self._metadata["status"] = status
            self._metadata["finished_at_utc"] = time.strftime(
                "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
            )
            self._write_metadata()
            self._write_summary()
            path = self._session_dir
            self._session_dir = None
            self._timeline_file = None
        self.get_logger().info(f"navigation trace finalized: {path}")

    def _on_nav_event(self, msg: NavEvent) -> None:
        details = _event_details(msg)
        code = str(msg.code)
        with self._lock:
            if code == "ROUTE_MISSION_STARTED":
                self._start_session(details)
            if self._session_dir is None:
                return
            self._record(
                "route_event" if str(msg.component) == "route_executor" else "nav_event",
                code,
                {
                    "component": str(msg.component),
                    "message": str(msg.message),
                    "severity": int(msg.severity),
                    **details,
                },
            )
            if code in TERMINAL_ROUTE_CODES:
                self._finalize_session(code.lower())

    def _on_trace_event(self, msg: NavEvent) -> None:
        details = _event_details(msg)
        code = str(msg.code)
        self._record(
            "clearance" if code.startswith("CLEARANCE_") else "replan",
            code,
            {
                "component": str(msg.component),
                "message": str(msg.message),
                "severity": int(msg.severity),
                **details,
            },
        )

    def _path_points(self, msg: NavPath) -> List[Tuple[float, float]]:
        return [
            (float(pose.pose.position.x), float(pose.pose.position.y)) for pose in msg.poses
        ]

    def _on_plan(self, msg: NavPath) -> None:
        with self._lock:
            if self._session_dir is None:
                return
            points = self._path_points(msg)
            self._plan_sequence += 1
            plan_id = self._plan_sequence
            metrics = analyze_path_points(
                points, robot_pose=self._robot_pose, previous_points=self._last_plan_points
            )
            encoded = json.dumps(points, separators=(",", ":")).encode("utf-8")
            payload = {
                "plan_id": plan_id,
                "frame_id": str(msg.header.frame_id),
                "sha256": hashlib.sha256(encoded).hexdigest(),
                **metrics,
                "points": [[round(x, 4), round(y, 4)] for x, y in points],
            }
            plan_path = self._session_dir / "plans" / f"plan_{plan_id:04d}.json"
            plan_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
            self._last_plan_points = points
            record_payload = dict(payload)
            record_payload.pop("points", None)
            record_payload["file"] = str(plan_path.relative_to(self._session_dir))
            self._record("plan", "PLAN_PUBLISHED", record_payload)

    def _record_path_shape(self, code: str, msg: NavPath) -> None:
        points = self._path_points(msg)
        if self._session_dir is not None:
            if code == "MISSION_PATH_UPDATED":
                destination = self._session_dir / "mission_path.json"
            else:
                chunks_dir = self._session_dir / "chunks"
                chunks_dir.mkdir(exist_ok=True)
                destination = chunks_dir / f"chunk_{int(self._route_state.get('chunk_id', 0)):04d}.json"
            destination.write_text(
                json.dumps(
                    {
                        "frame_id": str(msg.header.frame_id),
                        "points": [[round(x, 4), round(y, 4)] for x, y in points],
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
        self._record(
            "route_geometry",
            code,
            {"frame_id": str(msg.header.frame_id), "pose_count": len(points), "length_m": _path_length(points)},
        )

    def _on_mission_path(self, msg: NavPath) -> None:
        self._record_path_shape("MISSION_PATH_UPDATED", msg)

    def _on_chunk_path(self, msg: NavPath) -> None:
        self._record_path_shape("ACTIVE_CHUNK_PATH_UPDATED", msg)

    def _on_odometry(self, msg: Odometry) -> None:
        self._robot_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            _orientation_yaw(msg.pose.pose.orientation),
        )

    def _on_collision_state(self, msg: CollisionMonitorState) -> None:
        action_type = int(msg.action_type)
        if action_type == self._last_collision_action:
            return
        self._last_collision_action = action_type
        self._record(
            "safety",
            "COLLISION_MONITOR_STATE",
            {"action_type": action_type},
        )

    def _on_nav_telemetry(self, msg: NavTelemetry) -> None:
        key = (int(msg.nav_result_status), int(msg.nav_result_event_id), str(msg.failure_code))
        if key == self._last_nav_result_key:
            return
        self._last_nav_result_key = key
        if key[0] not in (0, 1, 2):
            self._record(
                "nav_result",
                "NAV_RESULT_STATE",
                {
                    "status": int(msg.nav_result_status),
                    "text": str(msg.nav_result_text),
                    "event_id": int(msg.nav_result_event_id),
                    "failure_code": str(msg.failure_code),
                },
            )

    def _on_bt_log(self, msg: BehaviorTreeLog) -> None:
        interesting = {
            "ComputePath",
            "ClearGlobalCostmap-Context",
            "ClearLocalCostmap-Context",
            "ClearLocalCostmap-Subtree",
            "ClearGlobalCostmap-Subtree",
            "FollowPath",
            "WaitAndReplan",
        }
        for event in msg.event_log or []:
            if str(event.node_name) not in interesting and str(event.current_status) != "FAILURE":
                continue
            self._record(
                "bt_transition",
                "BT_STATUS_CHANGED",
                {
                    "node_name": str(event.node_name),
                    "previous_status": str(event.previous_status),
                    "current_status": str(event.current_status),
                },
            )

    def _poll_route_state(self) -> None:
        if self._state_request_inflight or not self._route_state_client.service_is_ready():
            return
        self._state_request_inflight = True
        future = self._route_state_client.call_async(GetRouteMissionState.Request())
        future.add_done_callback(self._on_route_state)

    def _on_route_state(self, future: Any) -> None:
        self._state_request_inflight = False
        try:
            response = future.result()
        except Exception:
            return
        if response is None or not bool(response.ok):
            return
        state = {
            "mission_id": str(response.mission_id),
            "chunk_id": int(response.chunk_id),
            "loop_iteration": int(response.loop_iteration),
            "reached_checkpoint_count": int(response.reached_checkpoint_count),
            "current_start_index": int(response.current_start_index),
            "current_target_index": int(response.current_target_index),
            "current_checkpoint_index": int(response.current_checkpoint_index),
            "current_progress_expanded_index": int(response.current_progress_expanded_index),
            "current_progress_ratio": float(response.current_progress_ratio),
            "cross_track_error_m": float(response.cross_track_error_m),
            "distance_to_target_m": float(response.distance_to_target_m),
            "mission_key_flags": [bool(value) for value in response.mission_key_flags],
            "mission_input_indices": [int(value) for value in response.mission_input_indices],
            "mission_lats": [float(value) for value in response.mission_lats],
            "mission_lons": [float(value) for value in response.mission_lons],
            "mission_yaws_deg": [float(value) for value in response.mission_yaws_deg],
            "active": bool(response.active),
            "blocked_state": str(response.blocked_state),
        }
        with self._lock:
            if self._session_dir is None:
                self._route_state = state
                return
            previous = self._last_progress
            current = (state["loop_iteration"], state["current_progress_expanded_index"])
            if previous is not None:
                if current[0] == previous[0] and current[1] < previous[1]:
                    self._record(
                        "anomaly",
                        "PROGRESS_REGRESSION",
                        {"previous_index": previous[1], "current_index": current[1]},
                    )
                elif current[0] == previous[0] and current[1] > previous[1]:
                    crossed = range(previous[1] + 1, current[1] + 1)
                    synthetic = [
                        index
                        for index in crossed
                        if 0 <= index < len(state["mission_key_flags"])
                        and not state["mission_key_flags"][index]
                    ]
                    if synthetic:
                        self._record(
                            "progress",
                            "ROUTE_SYNTHETIC_PASSED",
                            {"indices": synthetic, "count": len(synthetic)},
                        )
            self._last_progress = current
            self._route_state = state
            route_signature = json.dumps(
                [state["mission_id"], state["mission_key_flags"], state["mission_input_indices"]],
                separators=(",", ":"),
            )
            if route_signature != self._last_route_signature:
                self._last_route_signature = route_signature
                self._record(
                    "route_definition",
                    "ROUTE_DEFINITION",
                    {
                        "mission_key_flags": state["mission_key_flags"],
                        "mission_input_indices": state["mission_input_indices"],
                        "mission_lats": state["mission_lats"],
                        "mission_lons": state["mission_lons"],
                        "mission_yaws_deg": state["mission_yaws_deg"],
                    },
                )
            sample = dict(state)
            sample.pop("mission_key_flags", None)
            sample.pop("mission_input_indices", None)
            sample.pop("mission_lats", None)
            sample.pop("mission_lons", None)
            sample.pop("mission_yaws_deg", None)
            self._record("progress", "ROUTE_PROGRESS_SAMPLE", sample)

    def destroy_node(self) -> bool:
        self._finalize_session("recorder_shutdown")
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = NavTraceRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
