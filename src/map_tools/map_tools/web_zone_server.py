import asyncio
import base64
from collections import deque
import json
import math
import os
import shlex
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple
from urllib import request as urllib_request

import cv2
import numpy as np
import rclpy
import websockets
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from nav2_msgs.msg import BehaviorTreeLog
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from interfaces.msg import CmdVelFinal, DriveTelemetry, NavEvent, NavTelemetry
from interfaces.srv import (
    BrakeNav,
    CameraPan,
    CameraStatus,
    CancelNavGoal,
    CancelRouteMission,
    GetDatum,
    GetNavSnapshot,
    GetNavState,
    GetRouteMissionState,
    GetZonesState,
    SetManualMode,
    SetNavGoalLL,
    SetRouteMissionLL,
    SetZonesGeoJson,
)
from .datum_file_utils import (
    build_datums_doc,
    load_datums_yaml_file,
    normalize_datum_profile,
    save_datums_yaml_file,
    unique_datum_id,
    utc_now_iso,
)
from .waypoints_file_utils import load_waypoints_yaml_file, save_waypoints_yaml_file


ROSBAG_TOPIC_PROFILES: Dict[str, Tuple[str, ...]] = {
    "core": (
        "/global_position/raw/fix",
        "/gps/fix",
        "/gps/rtk_status",
        "/gps/rtk_status_mavros",
        "/gps/odometry_map",
        "/gps/course_heading",
        "/gps/course_heading/debug",
        "/odometry/local",
        "/odometry/gps",
        "/odometry/local_global",
        "/odometry/local_yaw_hold",
        "/odometry/global",
        "/imu/data",
        "/imu/data_global",
        "/scan",
        "/cmd_vel",
        "/cmd_vel_safe",
        "/cmd_vel_final",
        "/collision_monitor_state",
        "/nav_command_server/telemetry",
        "/nav_command_server/events",
        "/controller/drive_telemetry",
        "/controller/status",
        "/controller/telemetry",
        "/diagnostics",
        "/tf",
        "/tf_static",
        "/rosout",
    ),
    "full_nav2": (
        "/global_position/raw/fix",
        "/gps/fix",
        "/gps/rtk_status",
        "/gps/rtk_status_mavros",
        "/gps/odometry_map",
        "/gps/course_heading",
        "/gps/course_heading/debug",
        "/odometry/local",
        "/odometry/gps",
        "/odometry/local_global",
        "/odometry/local_yaw_hold",
        "/odometry/global",
        "/imu/data",
        "/imu/data_global",
        "/scan",
        "/cmd_vel",
        "/cmd_vel_safe",
        "/cmd_vel_final",
        "/collision_monitor_state",
        "/nav_command_server/telemetry",
        "/nav_command_server/events",
        "/controller/drive_telemetry",
        "/controller/status",
        "/controller/telemetry",
        "/diagnostics",
        "/tf",
        "/tf_static",
        "/rosout",
        "/plan",
        "/local_costmap/costmap",
        "/global_costmap/costmap",
        "/local_costmap/published_footprint",
        "/behavior_tree_log",
    ),
}

MISSION_SESSION_DIR = Path("/tmp/mission_sessions")
MISSION_STATUS_FILE = "status.json"

UNSET = object()


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class WebZoneServerNode(Node):
    @staticmethod
    def _diag_level_value(value: Any) -> int:
        if isinstance(value, (bytes, bytearray)):
            return int.from_bytes(value, byteorder="little", signed=False)
        return int(value)

    @staticmethod
    def _safe_float(value: Any, default: float = 0.0) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return float(default)
        if not np.isfinite(parsed):
            return float(default)
        return float(parsed)

    @staticmethod
    def _safe_int(value: Any, default: int = 0) -> int:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return int(default)
        if not np.isfinite(parsed):
            return int(default)
        return int(parsed)

    @staticmethod
    def _normalize_camera_frame_encoding(value: str) -> str:
        normalized = value.strip().lower()
        if normalized in {"jpeg", "jpg", "png"}:
            return "jpeg" if normalized in {"jpeg", "jpg"} else "png"
        return "jpeg"

    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__("web_zone_server")
        self._loop = loop

        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 8766)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("gps_status_topic", "/gps/rtk_status")
        self.declare_parameter("odom_topic", "/odometry/local")
        self.declare_parameter("gps_broadcast_hz", 1.0)
        self.declare_parameter("request_timeout_s", 5.0)
        self.declare_parameter("snapshot_request_timeout_s", 2.0)
        self.declare_parameter("set_zones_timeout_s", 12.0)
        self.declare_parameter("set_goal_timeout_s", 12.0)
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("datums_file", "")

        self.declare_parameter("zones_set_geojson_service", "/zones_manager/set_geojson")
        self.declare_parameter("zones_get_state_service", "/zones_manager/get_state")
        self.declare_parameter("zones_reload_service", "/zones_manager/reload_from_disk")

        self.declare_parameter("nav_set_goal_service", "/nav_command_server/set_goal_ll")
        self.declare_parameter("nav_cancel_goal_service", "/nav_command_server/cancel_goal")
        self.declare_parameter("nav_brake_service", "/nav_command_server/brake")
        self.declare_parameter("nav_set_manual_mode_service", "/nav_command_server/set_manual_mode")
        self.declare_parameter("nav_get_state_service", "/nav_command_server/get_state")
        self.declare_parameter("route_set_service", "/route_executor/set_route_ll")
        self.declare_parameter("route_cancel_service", "/route_executor/cancel_route")
        self.declare_parameter("route_get_state_service", "/route_executor/get_state")
        self.declare_parameter("route_state_poll_hz", 2.0)
        self.declare_parameter("teleop_cmd_topic", "/cmd_vel_teleop")

        self.declare_parameter("nav_snapshot_service", "/nav_snapshot_server/get_nav_snapshot")
        self.declare_parameter("nav_telemetry_topic", "/nav_command_server/telemetry")
        self.declare_parameter("nav_events_topic", "/nav_command_server/events")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("rosbag_output_dir", "/ros2_ws/bags")
        self.declare_parameter("camera_pan_service", "/camara/camera_pan")
        self.declare_parameter("camera_zoom_toggle_service", "/camara/camera_zoom_toggle")
        self.declare_parameter("camera_status_service", "/camara/camera_status")
        self.declare_parameter("camera_image_topic", "/camera/image_raw")
        self.declare_parameter("camera_detections_topic", "/detections")
        self.declare_parameter("camera_frame_encoding", "jpeg")
        self.declare_parameter("camera_jpeg_quality", 90)
        self.declare_parameter("camera_ws_max_fps", 10.0)
        self.declare_parameter("camera_ws_width", 960)
        self.declare_parameter("enable_control_lock", False)
        self.declare_parameter("control_lock_start_locked", True)
        self.declare_parameter("control_lock_heartbeat_timeout_s", 2.5)
        self.declare_parameter("sensor_bridge_enabled", False)
        self.declare_parameter("sensor_bridge_http_url", "http://127.0.0.1:8000/data")
        self.declare_parameter("sensor_bridge_timeout_s", 0.35)
        self.declare_parameter("sensor_bridge_poll_hz", 2.0)
        self.declare_parameter("datum_get_service", "/datum_setter/get_datum")
        self.declare_parameter("fixed_datum_lat", float("nan"))
        self.declare_parameter("fixed_datum_lon", float("nan"))
        self.declare_parameter("fixed_datum_yaw_deg", 0.0)
        self.declare_parameter("fixed_datum_source", "real_global_v2_fixed")

        self.ws_host = str(self.get_parameter("ws_host").value)
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.gps_status_topic = str(self.get_parameter("gps_status_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.gps_broadcast_hz = float(self.get_parameter("gps_broadcast_hz").value)
        self.request_timeout_s = max(0.5, float(self.get_parameter("request_timeout_s").value))
        self.snapshot_request_timeout_s = max(
            0.5, float(self.get_parameter("snapshot_request_timeout_s").value)
        )
        self.set_zones_timeout_s = max(
            self.request_timeout_s, float(self.get_parameter("set_zones_timeout_s").value)
        )
        self.set_goal_timeout_s = max(
            self.request_timeout_s, float(self.get_parameter("set_goal_timeout_s").value)
        )
        configured_waypoints_file = str(self.get_parameter("waypoints_file").value)
        self.waypoints_file = self._resolve_waypoints_file(configured_waypoints_file)
        configured_datums_file = str(self.get_parameter("datums_file").value)
        self.datums_file = self._resolve_datums_file(configured_datums_file)

        self.zones_set_geojson_service = str(
            self.get_parameter("zones_set_geojson_service").value
        )
        self.zones_get_state_service = str(
            self.get_parameter("zones_get_state_service").value
        )
        self.zones_reload_service = str(self.get_parameter("zones_reload_service").value)

        self.nav_set_goal_service = str(self.get_parameter("nav_set_goal_service").value)
        self.nav_cancel_goal_service = str(
            self.get_parameter("nav_cancel_goal_service").value
        )
        self.nav_brake_service = str(self.get_parameter("nav_brake_service").value)
        self.nav_set_manual_mode_service = str(
            self.get_parameter("nav_set_manual_mode_service").value
        )
        self.nav_get_state_service = str(self.get_parameter("nav_get_state_service").value)
        self.route_set_service = str(self.get_parameter("route_set_service").value)
        self.route_cancel_service = str(self.get_parameter("route_cancel_service").value)
        self.route_get_state_service = str(
            self.get_parameter("route_get_state_service").value
        )
        self.route_state_poll_hz = max(
            0.2, float(self.get_parameter("route_state_poll_hz").value)
        )
        self.teleop_cmd_topic = str(self.get_parameter("teleop_cmd_topic").value)

        self.nav_snapshot_service = str(self.get_parameter("nav_snapshot_service").value)
        self.nav_telemetry_topic = str(self.get_parameter("nav_telemetry_topic").value)
        self.nav_events_topic = str(self.get_parameter("nav_events_topic").value)
        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.rosbag_output_dir = str(self.get_parameter("rosbag_output_dir").value)
        self.camera_pan_service = str(self.get_parameter("camera_pan_service").value)
        self.camera_zoom_toggle_service = str(
            self.get_parameter("camera_zoom_toggle_service").value
        )
        self.camera_status_service = str(self.get_parameter("camera_status_service").value)
        self.camera_image_topic = str(self.get_parameter("camera_image_topic").value)
        self.camera_detections_topic = str(
            self.get_parameter("camera_detections_topic").value
        )
        self.camera_frame_encoding = self._normalize_camera_frame_encoding(
            str(self.get_parameter("camera_frame_encoding").value)
        )
        self.camera_jpeg_quality = min(
            95, max(40, int(self.get_parameter("camera_jpeg_quality").value))
        )
        self.camera_ws_max_fps = max(
            1.0, float(self.get_parameter("camera_ws_max_fps").value)
        )
        self.camera_ws_width = max(0, int(self.get_parameter("camera_ws_width").value))
        self.enable_control_lock = _coerce_bool(
            self.get_parameter("enable_control_lock").value
        )
        self.control_lock_start_locked = _coerce_bool(
            self.get_parameter("control_lock_start_locked").value
        )
        self.control_lock_heartbeat_timeout_s = max(
            0.5, float(self.get_parameter("control_lock_heartbeat_timeout_s").value)
        )
        self.sensor_bridge_enabled = _coerce_bool(
            self.get_parameter("sensor_bridge_enabled").value
        )
        self.sensor_bridge_http_url = str(
            self.get_parameter("sensor_bridge_http_url").value
        ).strip()
        self.sensor_bridge_timeout_s = max(
            0.1, float(self.get_parameter("sensor_bridge_timeout_s").value)
        )
        self.sensor_bridge_poll_hz = max(
            0.2, float(self.get_parameter("sensor_bridge_poll_hz").value)
        )
        self.datum_get_service = str(self.get_parameter("datum_get_service").value)
        self.fixed_datum_lat = float(self.get_parameter("fixed_datum_lat").value)
        self.fixed_datum_lon = float(self.get_parameter("fixed_datum_lon").value)
        self.fixed_datum_yaw_deg = float(self.get_parameter("fixed_datum_yaw_deg").value)
        self.fixed_datum_source = str(self.get_parameter("fixed_datum_source").value).strip()

        self._lock = threading.Lock()
        self._ws_clients: Set[Any] = set()
        self._ws_send_locks: Dict[Any, asyncio.Lock] = {}
        self._last_camera_ws_frame_monotonic = 0.0
        self._camera_bridge = CvBridge()
        self._recent_camera_frames: deque[Dict[str, int]] = deque(maxlen=20)
        self._latest_camera_frame_shape = {"width": 0, "height": 0}

        self._last_robot_pose: Optional[Dict[str, float]] = None
        self._last_robot_heading_deg: Optional[float] = None
        self._last_imu_heading_deg: Optional[float] = None
        self._last_gps_broadcast_monotonic: Optional[float] = None
        self._gps_status_payload = self._build_gps_status_payload(
            raw="",
            source="unavailable",
            available=False,
        )
        self._last_explicit_gps_status_monotonic: Optional[float] = None

        self._zones: List[Dict[str, Any]] = []
        self._zones_geojson: Dict[str, Any] = {"type": "FeatureCollection", "features": []}
        self._mask_ready = False
        self._mask_source = "none"

        self._cmd_vel_safe = {
            "available": False,
            "linear_x": 0.0,
            "angular_z": 0.0,
        }
        self._manual_control = {
            "enabled": False,
            "linear_x_cmd": 0.0,
            "angular_z_cmd": 0.0,
            "last_cmd_age_s": None,
        }
        self._goal_active = False
        self._nav_result_status = 0
        self._nav_result_text = "idle"
        self._nav_result_event_id = 0
        self._camera_status = {
            "ok": False,
            "error": "camera status unavailable",
            "last_command": "none",
            "zoom_in": False,
        }
        self._route_mission = self._build_default_route_mission_payload()
        self._recent_nav_events: deque[Dict[str, Any]] = deque(maxlen=30)
        self._active_alerts: List[Dict[str, Any]] = []
        self._rosbag_process: Optional[subprocess.Popen] = None
        self._rosbag_profile = ""
        self._rosbag_output_path = ""
        self._rosbag_log_path = ""
        self._rosbag_started_at_epoch_ms: Optional[int] = None
        self._rosbag_last_exit_code: Optional[int] = None
        self._rosbag_last_error = ""
        self._mission_active: bool = False
        self._mission_file: Optional[Path] = None
        self._mission_message_count: int = 0
        self._mission_pending_send: List[str] = []
        self._mission_last_telemetry_key: Optional[str] = None
        self._mission_last_drive_key: Optional[str] = None
        self._mission_last_controller_telemetry_key: Optional[str] = None
        self._mission_last_controller_status_key: Optional[str] = None
        self._mission_last_diag_key: Dict[str, str] = {}
        self._control_locked = bool(
            self.enable_control_lock and self.control_lock_start_locked
        )
        self._control_lock_reason = (
            "STARTUP_LOCKED" if self._control_locked else ""
        )
        self._last_control_heartbeat_monotonic: Optional[float] = None
        self._sensor_bridge_snapshot: Dict[str, Any] = {}
        self._sensor_bridge_ok = False
        self._sensor_bridge_error = ""
        self._sensor_bridge_last_poll_monotonic: Optional[float] = None
        self._datum_snapshot = self._build_default_datum_snapshot()
        self._datums_doc = build_datums_doc([], "")
        self._datums_error = ""

        self._manual_cmd_last_monotonic: Optional[float] = None
        self._route_state_poll_inflight = False

        self._gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self._on_gps_fix, qos_profile_sensor_data
        )
        self._imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data
        )
        self._gps_status_sub = self.create_subscription(
            String, self.gps_status_topic, self._on_gps_status, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odometry, 10
        )
        self._nav_telemetry_sub = self.create_subscription(
            NavTelemetry, self.nav_telemetry_topic, self._on_nav_telemetry, 10
        )
        self._nav_events_sub = self.create_subscription(
            NavEvent, self.nav_events_topic, self._on_nav_event, 10
        )
        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray, self.diagnostics_topic, self._on_diagnostics, 10
        )
        self._drive_telemetry_sub = self.create_subscription(
            DriveTelemetry, "/controller/drive_telemetry", self._on_drive_telemetry, 10
        )
        self._controller_telemetry_sub = self.create_subscription(
            String, "/controller/telemetry", self._on_controller_telemetry, 10
        )
        self._controller_status_sub = self.create_subscription(
            String, "/controller/status", self._on_controller_status, 10
        )
        self._rosout_sub = self.create_subscription(
            Log, "/rosout", self._on_rosout, 10
        )
        self._behavior_tree_log_sub = self.create_subscription(
            BehaviorTreeLog, "/behavior_tree_log", self._on_behavior_tree_log, 10
        )
        self._camera_image_sub = self.create_subscription(
            Image, self.camera_image_topic, self._on_camera_image, qos_profile_sensor_data
        )
        self._camera_detections_sub = self.create_subscription(
            Detection2DArray,
            self.camera_detections_topic,
            self._on_camera_detections,
            10,
        )

        self._zones_set_geojson_client = self.create_client(
            SetZonesGeoJson, self.zones_set_geojson_service
        )
        self._zones_get_state_client = self.create_client(
            GetZonesState, self.zones_get_state_service
        )
        self._zones_reload_client = self.create_client(Trigger, self.zones_reload_service)
        self._nav_set_goal_client = self.create_client(SetNavGoalLL, self.nav_set_goal_service)
        self._nav_cancel_goal_client = self.create_client(
            CancelNavGoal, self.nav_cancel_goal_service
        )
        self._nav_brake_client = self.create_client(BrakeNav, self.nav_brake_service)
        self._nav_set_manual_mode_client = self.create_client(
            SetManualMode, self.nav_set_manual_mode_service
        )
        self._teleop_cmd_pub = self.create_publisher(CmdVelFinal, self.teleop_cmd_topic, 10)
        self._nav_get_state_client = self.create_client(GetNavState, self.nav_get_state_service)
        self._route_set_client = self.create_client(SetRouteMissionLL, self.route_set_service)
        self._route_cancel_client = self.create_client(
            CancelRouteMission, self.route_cancel_service
        )
        self._route_get_state_client = self.create_client(
            GetRouteMissionState, self.route_get_state_service
        )
        self._nav_snapshot_client = self.create_client(GetNavSnapshot, self.nav_snapshot_service)
        self._camera_pan_client = self.create_client(CameraPan, self.camera_pan_service)
        self._camera_zoom_toggle_client = self.create_client(
            Trigger, self.camera_zoom_toggle_service
        )
        self._camera_status_client = self.create_client(
            CameraStatus, self.camera_status_service
        )
        self._datum_get_client = self.create_client(GetDatum, self.datum_get_service)
        self._control_lock_watchdog_timer = self.create_timer(
            0.25, self._control_lock_watchdog_tick
        )
        self._sensor_bridge_timer = self.create_timer(
            1.0 / float(self.sensor_bridge_poll_hz),
            self._sensor_bridge_poll_tick,
        )
        self._route_state_poll_timer = self.create_timer(
            1.0 / float(self.route_state_poll_hz),
            self._route_state_poll_tick,
        )
        self.get_logger().info(
            "Web gateway ready "
            f"(ws={self.ws_host}:{self.ws_port}, zones_set={self.zones_set_geojson_service}, "
            f"goal_set={self.nav_set_goal_service}, snapshot={self.nav_snapshot_service}, "
            f"route_set={self.route_set_service}, route_get_state={self.route_get_state_service}, "
            f"nav_events={self.nav_events_topic}, diagnostics={self.diagnostics_topic}, "
            f"rosbag_dir={self.rosbag_output_dir}, "
            f"camera_pan={self.camera_pan_service}, camera_zoom_toggle={self.camera_zoom_toggle_service}, "
            f"camera_status={self.camera_status_service}, "
            f"camera_image={self.camera_image_topic}, "
            f"camera_detections={self.camera_detections_topic}, "
            f"camera_frame_encoding={self.camera_frame_encoding}, "
            f"camera_ws_width={self.camera_ws_width}, "
            f"teleop_topic={self.teleop_cmd_topic}, gps_topic={self.gps_topic}, "
            f"imu_topic={self.imu_topic}, "
            f"gps_status_topic={self.gps_status_topic}, "
            f"odom_topic={self.odom_topic}, control_lock={self.enable_control_lock}, "
            f"sensor_bridge={self.sensor_bridge_enabled})"
        )
        self.get_logger().info(f"Waypoints file path: {self.waypoints_file}")
        self.get_logger().info(f"Datums file path: {self.datums_file}")

    def add_client(self, ws: Any) -> None:
        with self._lock:
            self._ws_clients.add(ws)
            self._ws_send_locks[ws] = asyncio.Lock()
            count = len(self._ws_clients)
        self.get_logger().info(f"WS client connected (clients={count})")

    def remove_client(self, ws: Any) -> None:
        with self._lock:
            self._ws_clients.discard(ws)
            self._ws_send_locks.pop(ws, None)
            count = len(self._ws_clients)
        self.get_logger().info(f"WS client disconnected (clients={count})")

    async def send_ws_text(self, ws: Any, text: str) -> bool:
        with self._lock:
            lock = self._ws_send_locks.get(ws)
        if lock is None:
            return False
        async with lock:
            await ws.send(text)
        return True

    async def send_ws_json(self, ws: Any, payload: Dict[str, Any]) -> bool:
        return await self.send_ws_text(ws, json.dumps(payload))

    def snapshot_state(self) -> Dict[str, Any]:
        datums_payload = self._build_datums_state_payload()
        with self._lock:
            connection_status = self._connection_status_locked()
            return {
                "op": "state",
                "ok": True,
                "frame_id": self.map_frame,
                "zones": list(self._zones),
                "geojson": dict(self._zones_geojson),
                "mask_ready": bool(self._mask_ready),
                "mask_source": str(self._mask_source),
                "robot_pose": self._last_robot_pose,
                "gps_status": dict(self._gps_status_payload),
                "cmd_vel_safe": dict(self._cmd_vel_safe),
                "manual_control": dict(self._manual_control),
                "goal_active": bool(self._goal_active),
                "nav_result_status": int(self._nav_result_status),
                "nav_result_text": str(self._nav_result_text),
                "nav_result_event_id": int(self._nav_result_event_id),
                "route_mission": dict(self._route_mission),
                "alerts": list(self._active_alerts),
                "recent_events": list(self._recent_nav_events),
                "rosbag": self._build_rosbag_status_payload_locked(),
                "camera_status": dict(self._camera_status),
                "datum": dict(self._datum_snapshot),
                "datums": datums_payload,
                **connection_status,
            }

    def _build_nav_telemetry_payload(self) -> Dict[str, Any]:
        with self._lock:
            cmd_vel_safe = dict(self._cmd_vel_safe)
            manual_control = dict(self._manual_control)
            goal_active = bool(self._goal_active)
            nav_result_status = int(self._nav_result_status)
            nav_result_text = str(self._nav_result_text)
            nav_result_event_id = int(self._nav_result_event_id)
            robot_pose = dict(self._last_robot_pose) if self._last_robot_pose is not None else None
            route_mission = dict(self._route_mission)
            alerts = list(self._active_alerts)
            recent_events = list(self._recent_nav_events)
            connection_status = self._connection_status_locked()
        return {
            "op": "nav_telemetry",
            "cmd_vel_safe": cmd_vel_safe,
            "manual_control": manual_control,
            "goal_active": goal_active,
            "nav_result_status": nav_result_status,
            "nav_result_text": nav_result_text,
            "nav_result_event_id": nav_result_event_id,
            "robot_pose": robot_pose,
            "route_mission": route_mission,
            "alerts": alerts,
            "recent_events": recent_events,
            **connection_status,
        }

    @staticmethod
    def _normalize_gps_status_text(status_text: Any) -> str:
        text = str(status_text or "").strip().lower()
        for old, new in (("-", "_"), (" ", "_")):
            text = text.replace(old, new)
        return "_".join(part for part in text.split("_") if part)

    @classmethod
    def _gps_status_label_and_level(cls, normalized_status: str) -> Tuple[str, str]:
        if not normalized_status:
            return "Unavailable", "bad"
        if "rtk_fixed" in normalized_status:
            return "RTK FIXED", "good"
        if "rtk_float" in normalized_status:
            return "RTK FLOAT", "warn"
        if normalized_status in {"3d_fix", "gps_only", "fix"}:
            return "3D FIX", "warn"
        if normalized_status in {"rtk_fix", "gbas_fix", "sbas_fix"}:
            return "RTK FIX", "good"
        if normalized_status in {"no_fix", "gps_no_fix"}:
            return "NO FIX", "bad"
        if normalized_status == "rtcm_stale":
            return "RTCM STALE", "bad"
        if normalized_status == "rtcm_ok":
            return "RTCM OK", "warn"
        if normalized_status == "waiting_for_gps":
            return "WAITING GPS", "bad"
        if normalized_status == "waiting_for_mavros_gps_rtk":
            return "WAITING RTK LINK", "warn"
        return normalized_status.replace("_", " ").upper(), "warn"

    @classmethod
    def _build_gps_status_payload(
        cls,
        *,
        raw: Any,
        source: str,
        available: bool = True,
    ) -> Dict[str, Any]:
        normalized = cls._normalize_gps_status_text(raw)
        label, level = cls._gps_status_label_and_level(normalized)
        return {
            "available": bool(available),
            "raw": str(raw or ""),
            "normalized": normalized,
            "label": label,
            "level": level,
            "source": str(source),
        }

    @classmethod
    def _build_gps_status_payload_from_navsat(cls, status_value: Any) -> Dict[str, Any]:
        try:
            status_code = int(status_value)
        except (TypeError, ValueError):
            status_code = int(NavSatStatus.STATUS_NO_FIX)
        if status_code >= int(NavSatStatus.STATUS_GBAS_FIX):
            raw = "RTK_FIX"
        elif status_code == int(NavSatStatus.STATUS_SBAS_FIX):
            raw = "SBAS_FIX"
        elif status_code == int(NavSatStatus.STATUS_FIX):
            raw = "3D_FIX"
        else:
            raw = "NO_FIX"
        return cls._build_gps_status_payload(raw=raw, source="gps_fix", available=True)

    @staticmethod
    def _gps_status_payload_changed(
        previous: Dict[str, Any],
        current: Dict[str, Any],
    ) -> bool:
        for key in ("available", "raw", "normalized", "label", "level", "source"):
            if previous.get(key) != current.get(key):
                return True
        return False

    def _build_default_datum_snapshot(self) -> Dict[str, Any]:
        if np.isfinite(self.fixed_datum_lat) and np.isfinite(self.fixed_datum_lon):
            return {
                "available": True,
                "ok": True,
                "already_set": True,
                "datum_lat": float(self.fixed_datum_lat),
                "datum_lon": float(self.fixed_datum_lon),
                "datum_yaw_deg": float(self.fixed_datum_yaw_deg),
                "last_set_source": self.fixed_datum_source or "fixed_config",
                "last_set_epoch_ms": None,
                "last_set_with_rtk": False,
                "gps_is_rtk": False,
            }
        return {
            "available": False,
            "ok": False,
            "already_set": False,
            "datum_lat": None,
            "datum_lon": None,
            "datum_yaw_deg": None,
            "last_set_source": "",
            "last_set_epoch_ms": None,
            "last_set_with_rtk": False,
            "gps_is_rtk": False,
        }

    @staticmethod
    def _build_default_route_mission_payload() -> Dict[str, Any]:
        return {
            "active": False,
            "paused": False,
            "loop": False,
            "status": "idle",
            "input_waypoint_count": 0,
            "expanded_waypoint_count": 0,
            "current_start_index": 0,
            "current_target_index": 0,
            "active_chunk_size": 0,
            "leg_spacing_m": 0.0,
            "chunk_span_m": 0.0,
            "chunk_max_waypoints": 0,
            "blocked_state": "",
            "blocked_reason_code": "",
            "blocked_reason_text": "",
            "blocked_retry_attempt": 0,
            "blocked_retry_max_attempts": 0,
            "blocked_wait_remaining_s": 0.0,
            "mission_waypoints": [],
            "active_chunk_waypoints": [],
        }

    @staticmethod
    def _route_waypoints_from_state(
        lats: Sequence[float], lons: Sequence[float], yaws_deg: Sequence[float]
    ) -> List[Dict[str, float]]:
        if not (len(lats) == len(lons) == len(yaws_deg)):
            return []
        waypoints: List[Dict[str, float]] = []
        for lat, lon, yaw_deg in zip(lats, lons, yaws_deg):
            lat_value = float(lat)
            lon_value = float(lon)
            yaw_value = float(yaw_deg)
            if not (
                np.isfinite(lat_value)
                and np.isfinite(lon_value)
                and np.isfinite(yaw_value)
            ):
                continue
            waypoints.append(
                {
                    "lat": lat_value,
                    "lon": lon_value,
                    "yaw_deg": yaw_value,
                }
            )
        return waypoints

    def _update_route_state(self, response: GetRouteMissionState.Response) -> None:
        payload = {
            "active": bool(response.active),
            "paused": bool(response.paused),
            "loop": bool(response.loop),
            "status": str(response.status),
            "input_waypoint_count": int(response.input_waypoint_count),
            "expanded_waypoint_count": int(response.expanded_waypoint_count),
            "current_start_index": int(response.current_start_index),
            "current_target_index": int(response.current_target_index),
            "active_chunk_size": int(response.active_chunk_size),
            "leg_spacing_m": float(response.leg_spacing_m),
            "chunk_span_m": float(response.chunk_span_m),
            "chunk_max_waypoints": int(response.chunk_max_waypoints),
            "blocked_state": str(getattr(response, "blocked_state", "")),
            "blocked_reason_code": str(getattr(response, "blocked_reason_code", "")),
            "blocked_reason_text": str(getattr(response, "blocked_reason_text", "")),
            "blocked_retry_attempt": int(getattr(response, "blocked_retry_attempt", 0)),
            "blocked_retry_max_attempts": int(
                getattr(response, "blocked_retry_max_attempts", 0)
            ),
            "blocked_wait_remaining_s": float(
                getattr(response, "blocked_wait_remaining_s", 0.0)
            ),
            "mission_waypoints": self._route_waypoints_from_state(
                response.mission_lats, response.mission_lons, response.mission_yaws_deg
            ),
            "active_chunk_waypoints": self._route_waypoints_from_state(
                response.active_lats, response.active_lons, response.active_yaws_deg
            ),
        }
        with self._lock:
            self._route_mission = payload

    @staticmethod
    def _stamp_to_epoch_ms(stamp: Any) -> Optional[int]:
        if stamp is None:
            return None
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if not isinstance(sec, (int, float)) or not isinstance(nanosec, (int, float)):
            return None
        return int(float(sec) * 1000.0 + float(nanosec) / 1_000_000.0)

    @staticmethod
    def _clamp_unit_interval(value: float) -> float:
        return max(0.0, min(1.0, float(value)))

    def _record_camera_frame_shape(self, stamp_ms: int, width: int, height: int) -> None:
        if stamp_ms <= 0 or width <= 0 or height <= 0:
            return
        with self._lock:
            self._recent_camera_frames.append(
                {"stamp_ms": int(stamp_ms), "width": int(width), "height": int(height)}
            )
            self._latest_camera_frame_shape = {"width": int(width), "height": int(height)}

    def _resolve_camera_frame_shape(self, stamp_ms: int) -> Tuple[int, int]:
        with self._lock:
            recent_frames = list(self._recent_camera_frames)
            fallback_width = int(self._latest_camera_frame_shape.get("width", 0))
            fallback_height = int(self._latest_camera_frame_shape.get("height", 0))

        best_width = fallback_width
        best_height = fallback_height
        best_diff = 251
        for frame in recent_frames:
            diff = abs(int(frame.get("stamp_ms", 0)) - int(stamp_ms))
            if diff > 250 or diff >= best_diff:
                continue
            best_diff = diff
            best_width = int(frame.get("width", 0))
            best_height = int(frame.get("height", 0))
        return best_width, best_height

    def _normalize_detection_bbox(
        self,
        *,
        cx: float,
        cy: float,
        width: float,
        height: float,
        frame_width: int,
        frame_height: int,
    ) -> Optional[List[float]]:
        if not all(np.isfinite(value) for value in (cx, cy, width, height)):
            return None
        if width <= 0.0 or height <= 0.0:
            return None

        looks_normalized = max(abs(cx), abs(cy), abs(width), abs(height)) <= 1.5
        if looks_normalized:
            left = cx - width * 0.5
            top = cy - height * 0.5
            right = cx + width * 0.5
            bottom = cy + height * 0.5
            return [
                self._clamp_unit_interval(left),
                self._clamp_unit_interval(top),
                self._clamp_unit_interval(right - left),
                self._clamp_unit_interval(bottom - top),
            ]

        if frame_width <= 0 or frame_height <= 0:
            return None

        left_px = max(0.0, min(float(frame_width), cx - width * 0.5))
        top_px = max(0.0, min(float(frame_height), cy - height * 0.5))
        right_px = max(0.0, min(float(frame_width), cx + width * 0.5))
        bottom_px = max(0.0, min(float(frame_height), cy + height * 0.5))
        if right_px <= left_px or bottom_px <= top_px:
            return None

        return [
            self._clamp_unit_interval(left_px / float(frame_width)),
            self._clamp_unit_interval(top_px / float(frame_height)),
            self._clamp_unit_interval((right_px - left_px) / float(frame_width)),
            self._clamp_unit_interval((bottom_px - top_px) / float(frame_height)),
        ]

    def _serialize_detection(
        self,
        detection: Any,
        *,
        frame_width: int,
        frame_height: int,
    ) -> Optional[Dict[str, Any]]:
        top_label = ""
        top_score = 0.0
        results = list(getattr(detection, "results", []) or [])
        if results:
            top_result = results[0]
            hypothesis = getattr(top_result, "hypothesis", None)
            if hypothesis is not None:
                top_label = str(getattr(hypothesis, "class_id", "") or "")
                try:
                    top_score = float(getattr(hypothesis, "score", 0.0))
                except (TypeError, ValueError):
                    top_score = 0.0

        bbox = getattr(detection, "bbox", None)
        center = getattr(bbox, "center", None)
        position = getattr(center, "position", None)
        try:
            cx = float(getattr(position, "x"))
            cy = float(getattr(position, "y"))
            width = float(getattr(bbox, "size_x"))
            height = float(getattr(bbox, "size_y"))
        except (TypeError, ValueError, AttributeError):
            return None

        normalized_bbox = self._normalize_detection_bbox(
            cx=cx,
            cy=cy,
            width=width,
            height=height,
            frame_width=frame_width,
            frame_height=frame_height,
        )
        if normalized_bbox is None:
            return None

        return {
            "id": str(getattr(detection, "id", "") or ""),
            "class": top_label or "unknown",
            "confidence": max(0.0, min(1.0, top_score)),
            "bbox": normalized_bbox,
        }

    def _refresh_datum_snapshot(self) -> Dict[str, Any]:
        default_snapshot = self._build_default_datum_snapshot()
        req = GetDatum.Request()
        res = self._call_service(self._datum_get_client, req, min(self.request_timeout_s, 1.0))
        if res is None:
            with self._lock:
                self._datum_snapshot = default_snapshot
                return dict(self._datum_snapshot)

        snapshot = {
            "available": bool(getattr(res, "ok", False)),
            "ok": bool(getattr(res, "ok", False)),
            "already_set": bool(getattr(res, "already_set", False)),
            "datum_lat": (
                float(res.datum_lat) if np.isfinite(float(getattr(res, "datum_lat", float("nan")))) else None
            ),
            "datum_lon": (
                float(res.datum_lon) if np.isfinite(float(getattr(res, "datum_lon", float("nan")))) else None
            ),
            "datum_yaw_deg": float(self.fixed_datum_yaw_deg),
            "last_set_source": str(getattr(res, "last_set_source", "") or ""),
            "last_set_epoch_ms": self._stamp_to_epoch_ms(getattr(res, "last_set_stamp", None)),
            "last_set_with_rtk": bool(getattr(res, "last_set_with_rtk", False)),
            "gps_is_rtk": bool(getattr(res, "gps_is_rtk", False)),
        }
        if not snapshot["already_set"] and default_snapshot.get("available"):
            snapshot = default_snapshot
        with self._lock:
            self._datum_snapshot = snapshot
            return dict(self._datum_snapshot)

    def _sensor_bridge_poll_tick(self) -> None:
        if not self.sensor_bridge_enabled or not self.sensor_bridge_http_url:
            return
        try:
            payload = self._poll_sensor_bridge_snapshot()
            with self._lock:
                self._sensor_bridge_snapshot = payload
                self._sensor_bridge_ok = True
                self._sensor_bridge_error = ""
                self._sensor_bridge_last_poll_monotonic = time.monotonic()
        except Exception as exc:
            with self._lock:
                self._sensor_bridge_ok = False
                self._sensor_bridge_error = str(exc)
                self._sensor_bridge_last_poll_monotonic = time.monotonic()

    def _poll_sensor_bridge_snapshot(self) -> Dict[str, Any]:
        req = urllib_request.Request(
            self.sensor_bridge_http_url,
            headers={"Accept": "application/json"},
            method="GET",
        )
        with urllib_request.urlopen(req, timeout=self.sensor_bridge_timeout_s) as response:
            raw = response.read().decode("utf-8")
        payload = json.loads(raw or "{}")
        if not isinstance(payload, dict):
            raise ValueError("sensor bridge response must be a JSON object")
        return payload

    @staticmethod
    def _precision_from_gps_snapshot(snapshot: Dict[str, Any]) -> Optional[float]:
        gps_meta = snapshot.get("gps_meta")
        if isinstance(gps_meta, dict):
            for key in ("estimated_precision_m", "hdop_m", "eph_m"):
                value = gps_meta.get(key)
                if isinstance(value, (int, float)) and np.isfinite(float(value)):
                    return float(value)
            eph_raw = gps_meta.get("eph")
            if isinstance(eph_raw, (int, float)) and np.isfinite(float(eph_raw)):
                return float(eph_raw) / 100.0
        gps = snapshot.get("gps")
        if isinstance(gps, dict):
            covariance = gps.get("position_covariance")
            if isinstance(covariance, list) and len(covariance) >= 2:
                xx = covariance[0]
                yy = covariance[4] if len(covariance) > 4 else covariance[1]
                if isinstance(xx, (int, float)) and isinstance(yy, (int, float)):
                    if np.isfinite(float(xx)) and np.isfinite(float(yy)):
                        return math.sqrt(max(0.0, float(xx)) + max(0.0, float(yy)))
        return None

    @staticmethod
    def _derive_mode(goal_active: bool, manual_enabled: bool) -> str:
        if manual_enabled:
            return "manual"
        if goal_active:
            return "navigating"
        return "idle"

    @staticmethod
    def _normalize_delta_deg(delta_deg: float) -> float:
        while delta_deg <= -180.0:
            delta_deg += 360.0
        while delta_deg > 180.0:
            delta_deg -= 360.0
        return float(delta_deg)

    def _fallback_diagnostics_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            imu_heading = self._last_imu_heading_deg
            robot_heading = self._last_robot_heading_deg
        yaw_delta_deg = None
        if imu_heading is not None and robot_heading is not None:
            yaw_delta_deg = self._normalize_delta_deg(float(imu_heading) - float(robot_heading))
        return {
            "yaw_delta_deg": yaw_delta_deg,
            "diferencias": yaw_delta_deg,
        }

    def _fallback_rtk_source_state(self, gps_status: Dict[str, Any]) -> Dict[str, Any]:
        available = bool(gps_status.get("available", False))
        label = str(gps_status.get("label") or "").strip()
        normalized = str(gps_status.get("normalized") or "").strip()
        source = str(gps_status.get("source") or "").strip()
        return {
            "connected": available,
            "active_source_label": label or "sim_gps",
            "active_source_id": normalized or source or "sim_gps",
            "rtcm_age_s": None,
            "received_count": None,
            "last_error": "" if available else "gps unavailable",
        }

    def _connection_status_locked(self) -> Dict[str, Any]:
        return {
            "connected": True,
            "mode": self._derive_mode(self._goal_active, bool(self._manual_control.get("enabled", False))),
            "battery_pct": 0.0,
            "control_locked": bool(self._control_locked),
            "control_lock_reason": str(self._control_lock_reason),
            "locked": bool(self._control_locked),
            "lock_reason": str(self._control_lock_reason),
        }

    def _control_lock_watchdog_tick(self) -> None:
        if not self.enable_control_lock:
            return
        with self._lock:
            locked = bool(self._control_locked)
            last_heartbeat = self._last_control_heartbeat_monotonic
        if locked or last_heartbeat is None:
            return
        if (time.monotonic() - last_heartbeat) <= self.control_lock_heartbeat_timeout_s:
            return
        with self._lock:
            self._control_locked = True
            self._control_lock_reason = "UI_HEARTBEAT_TIMEOUT"
            self._last_control_heartbeat_monotonic = None
        self._broadcast_from_thread(self._build_nav_telemetry_payload())
        self._broadcast_from_thread(self.snapshot_state())

    def set_control_lock(self, locked: bool) -> Tuple[bool, str, bool, str]:
        if not self.enable_control_lock:
            return True, "", False, ""

        next_locked = bool(locked)
        with self._lock:
            self._control_locked = next_locked
            self._control_lock_reason = "UI_LOCK_REQUEST" if next_locked else ""
            self._last_control_heartbeat_monotonic = (
                None if next_locked else time.monotonic()
            )
            locked_after = bool(self._control_locked)
            reason = str(self._control_lock_reason)
        return True, "", locked_after, reason

    def control_heartbeat(self) -> Tuple[bool, str, bool, str]:
        if not self.enable_control_lock:
            return True, "", False, ""

        with self._lock:
            if not self._control_locked:
                self._last_control_heartbeat_monotonic = time.monotonic()
            locked_after = bool(self._control_locked)
            reason = str(self._control_lock_reason)
        return True, "", locked_after, reason

    def is_ui_control_locked(self) -> bool:
        if not self.enable_control_lock:
            return False
        with self._lock:
            return bool(self._control_locked)

    def get_ui_control_lock_reason(self) -> str:
        if not self.enable_control_lock:
            return ""
        with self._lock:
            return str(self._control_lock_reason)

    def _build_general_sensor_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            bridge_snapshot = dict(self._sensor_bridge_snapshot)
            datum_snapshot = dict(self._datum_snapshot)
            bridge_ok = bool(self._sensor_bridge_ok)
            bridge_error = str(self._sensor_bridge_error)
            gps_status = dict(self._gps_status_payload)

        gps_meta = {}
        if isinstance(bridge_snapshot.get("gps_meta"), dict):
            gps_meta = dict(bridge_snapshot["gps_meta"])
        if "fix_type_name" not in gps_meta:
            gps_meta["fix_type_name"] = gps_status.get("label", "UNKNOWN")
        if "rtk_status" not in gps_meta:
            gps_meta["rtk_status"] = gps_status.get("normalized") or gps_status.get("raw", "")
        precision = self._precision_from_gps_snapshot(bridge_snapshot)
        if precision is not None and "estimated_precision_m" not in gps_meta:
            gps_meta["estimated_precision_m"] = precision

        rtk_source_state = (
            dict(bridge_snapshot.get("rtk_source_state"))
            if isinstance(bridge_snapshot.get("rtk_source_state"), dict)
            else self._fallback_rtk_source_state(gps_status)
        )
        diagnostics = (
            dict(bridge_snapshot.get("diagnostics"))
            if isinstance(bridge_snapshot.get("diagnostics"), dict)
            else self._fallback_diagnostics_snapshot()
        )

        snapshot = {
            "gps_meta": gps_meta,
            "gps_status": gps_status,
            "rtk_source_state": rtk_source_state,
            "rtk_sources": list(bridge_snapshot.get("rtk_sources") or []),
            "datum": datum_snapshot,
            "datums": self._build_datums_state_payload(),
            "diagnostics": diagnostics,
        }
        if bridge_ok:
            snapshot["sensor_bridge_ok"] = True
        elif bridge_error:
            snapshot["sensor_bridge_error"] = bridge_error
        return snapshot

    def build_sensor_info_message(
        self, *, tab: str, interval_s: float, topic_name: Optional[str] = None
    ) -> Dict[str, Any]:
        normalized_tab = str(tab or "").strip()
        payload: Dict[str, Any] = {
            "op": "sensor_info",
            "tab": normalized_tab,
            "interval_s": float(interval_s),
            "enabled": True,
            "implemented": False,
            "ok": True,
            "snapshot": {},
        }

        if normalized_tab == "general":
            payload["implemented"] = True
            payload["snapshot"] = self._build_general_sensor_snapshot()
            return payload

        if normalized_tab == "pixhawk_gps":
            with self._lock:
                bridge_snapshot = dict(self._sensor_bridge_snapshot)
                bridge_ok = bool(self._sensor_bridge_ok)
                bridge_error = str(self._sensor_bridge_error)
            payload["implemented"] = True
            if not self.sensor_bridge_enabled:
                payload["snapshot"] = {
                    "gps_meta": {
                        "fix_type_name": self._gps_status_payload.get("label", "UNKNOWN"),
                        "rtk_status": self._gps_status_payload.get("normalized", ""),
                    },
                    "diagnostics": self._fallback_diagnostics_snapshot(),
                }
                return payload
            payload["snapshot"] = bridge_snapshot
            if not bridge_ok:
                payload["ok"] = False
                payload["error"] = bridge_error or "sensor bridge unavailable"
            return payload

        if normalized_tab == "topics":
            payload["implemented"] = False
            payload["snapshot"] = {
                "selected_topic": str(topic_name or ""),
                "selected_type": "",
                "topics_catalog": [],
                "history_text": "",
                "error": "topic stream bridge not implemented in SALUS yet",
            }
            return payload

        if normalized_tab in {"lidar", "camera"}:
            payload["implemented"] = False
            return payload

        payload["ok"] = False
        payload["error"] = f"unknown sensor tab: {normalized_tab}"
        return payload

    @staticmethod
    def _rosbag_topics_for_profile(profile: str) -> Optional[Tuple[str, ...]]:
        return ROSBAG_TOPIC_PROFILES.get(str(profile))

    def _build_rosbag_status_payload_locked(self) -> Dict[str, Any]:
        active = self._rosbag_process is not None and self._rosbag_process.poll() is None
        pid = None
        if active and self._rosbag_process is not None:
            pid = int(self._rosbag_process.pid)
        return {
            "active": bool(active),
            "profile": str(self._rosbag_profile),
            "output_dir": str(self._rosbag_output_path),
            "log_path": str(self._rosbag_log_path),
            "pid": pid,
            "started_at_epoch_ms": (
                int(self._rosbag_started_at_epoch_ms)
                if self._rosbag_started_at_epoch_ms is not None
                else None
            ),
            "last_exit_code": (
                int(self._rosbag_last_exit_code)
                if self._rosbag_last_exit_code is not None
                else None
            ),
            "last_error": str(self._rosbag_last_error),
            "available_profiles": sorted(ROSBAG_TOPIC_PROFILES.keys()),
        }

    def _rosbag_status_payload(self) -> Dict[str, Any]:
        with self._lock:
            return self._build_rosbag_status_payload_locked()

    @staticmethod
    def _nav_event_details_to_dict(msg: NavEvent) -> Dict[str, str]:
        details: Dict[str, str] = {}
        for item in getattr(msg, "details", []) or []:
            key = str(getattr(item, "key", "") or "")
            if not key:
                continue
            details[key] = str(getattr(item, "value", "") or "")
        return details

    @staticmethod
    def _diagnostic_values_to_dict(status: Any) -> Dict[str, str]:
        values: Dict[str, str] = {}
        for item in getattr(status, "values", []) or []:
            key = str(getattr(item, "key", "") or "")
            if not key:
                continue
            values[key] = str(getattr(item, "value", "") or "")
        return values

    def _nav_event_to_payload(self, msg: NavEvent) -> Dict[str, Any]:
        return {
            "stamp": {
                "sec": int(getattr(msg.stamp, "sec", 0)),
                "nanosec": int(getattr(msg.stamp, "nanosec", 0)),
            },
            "severity": int(msg.severity),
            "component": str(msg.component),
            "code": str(msg.code),
            "message": str(msg.message),
            "event_id": int(msg.event_id),
            "details": self._nav_event_details_to_dict(msg),
        }

    def _diagnostic_status_to_payload(self, status: Any) -> Dict[str, Any]:
        return {
            "name": str(getattr(status, "name", "")),
            "level": self._diag_level_value(
                getattr(
                    status,
                    "level",
                    self._diag_level_value(DiagnosticStatus.OK),
                )
            ),
            "message": str(getattr(status, "message", "")),
            "hardware_id": str(getattr(status, "hardware_id", "")),
            "values": self._diagnostic_values_to_dict(status),
        }

    def _should_surface_diagnostic(self, status: Any) -> bool:
        level = self._diag_level_value(
            getattr(
                status,
                "level",
                self._diag_level_value(DiagnosticStatus.OK),
            )
        )
        if level == self._diag_level_value(DiagnosticStatus.OK):
            return False
        name = str(getattr(status, "name", "") or "")
        if not name.startswith("navigation/"):
            return False
        message = str(getattr(status, "message", "") or "")
        if name == "navigation/collision_monitor" and message == "no collision monitor state yet":
            return False
        return True

    def _broadcast_from_thread(self, payload: Dict[str, Any]) -> None:
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    def _broadcast_rosbag_status(self) -> None:
        self._broadcast_from_thread(
            {
                "op": "rosbag_status",
                "rosbag": self._rosbag_status_payload(),
            }
        )

    def _update_rosbag_state_locked(
        self,
        *,
        process: Any = UNSET,
        profile: Any = UNSET,
        output_path: Any = UNSET,
        log_path: Any = UNSET,
        started_at_epoch_ms: Any = UNSET,
        last_exit_code: Any = UNSET,
        last_error: Any = UNSET,
    ) -> None:
        if process is not UNSET:
            self._rosbag_process = process
        if profile is not UNSET:
            self._rosbag_profile = str(profile)
        if output_path is not UNSET:
            self._rosbag_output_path = str(output_path)
        if log_path is not UNSET:
            self._rosbag_log_path = str(log_path)
        if started_at_epoch_ms is not UNSET:
            if started_at_epoch_ms is None:
                self._rosbag_started_at_epoch_ms = None
            else:
                self._rosbag_started_at_epoch_ms = int(started_at_epoch_ms)
        if last_exit_code is not UNSET:
            if last_exit_code is None:
                self._rosbag_last_exit_code = None
            else:
                self._rosbag_last_exit_code = int(last_exit_code)
        if last_error is not UNSET:
            self._rosbag_last_error = str(last_error)

    def _rosbag_waiter(self, process: subprocess.Popen) -> None:
        exit_code = process.wait()
        with self._lock:
            if self._rosbag_process is not process:
                return
            self._rosbag_process = None
            self._rosbag_last_exit_code = int(exit_code)
            if exit_code == 0:
                self._rosbag_last_error = ""
            elif not self._rosbag_last_error:
                self._rosbag_last_error = f"rosbag exited with code {exit_code}"
        self._broadcast_rosbag_status()

    def get_rosbag_status(self) -> Dict[str, Any]:
        return self._rosbag_status_payload()

    def start_rosbag(self, profile: str = "core") -> Tuple[bool, str, Dict[str, Any]]:
        profile_name = str(profile or "core").strip() or "core"
        topics = self._rosbag_topics_for_profile(profile_name)
        if topics is None:
            return False, f"unknown rosbag profile: {profile_name}", self.get_rosbag_status()

        with self._lock:
            if self._rosbag_process is not None and self._rosbag_process.poll() is None:
                return False, "rosbag is already running", self._build_rosbag_status_payload_locked()

        bags_dir = Path(self.rosbag_output_dir)
        bags_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        output_dir = bags_dir / f"nav_debug_{profile_name}_{stamp}"
        log_path = bags_dir / f"nav_debug_{profile_name}_{stamp}.log"

        output_dir_quoted = shlex.quote(str(output_dir))
        topics_quoted = " ".join(shlex.quote(topic) for topic in topics)
        cmd = (
            "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && "
            "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi && "
            "cd /ros2_ws && "
            f"exec ros2 bag record -o {output_dir_quoted} {topics_quoted}"
        )

        with log_path.open("ab") as log_file:
            process = subprocess.Popen(
                ["bash", "-lc", cmd],
                cwd="/ros2_ws",
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )

        time.sleep(0.4)
        exit_code = process.poll()
        if exit_code is not None:
            err = f"rosbag failed to start (exit_code={exit_code})"
            with self._lock:
                self._update_rosbag_state_locked(
                    process=None,
                    profile=profile_name,
                    output_path=str(output_dir),
                    log_path=str(log_path),
                    started_at_epoch_ms=None,
                    last_exit_code=int(exit_code),
                    last_error=err,
                )
            self._broadcast_rosbag_status()
            return False, err, self.get_rosbag_status()

        started_at_epoch_ms = int(time.time() * 1000.0)
        with self._lock:
            self._update_rosbag_state_locked(
                process=process,
                profile=profile_name,
                output_path=str(output_dir),
                log_path=str(log_path),
                started_at_epoch_ms=started_at_epoch_ms,
                last_exit_code=None,
                last_error="",
            )
        waiter = threading.Thread(
            target=self._rosbag_waiter,
            args=(process,),
            daemon=True,
            name="rosbag_waiter",
        )
        waiter.start()
        self._broadcast_rosbag_status()
        return True, "", self.get_rosbag_status()

    def stop_rosbag(self) -> Tuple[bool, str, Dict[str, Any]]:
        with self._lock:
            process = self._rosbag_process
        if process is None or process.poll() is not None:
            with self._lock:
                self._rosbag_process = None
            return False, "rosbag is not running", self.get_rosbag_status()

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except Exception:
            try:
                process.send_signal(signal.SIGINT)
            except Exception as exc:
                return False, f"failed to stop rosbag: {exc}", self.get_rosbag_status()

        deadline = time.time() + 10.0
        while time.time() < deadline:
            if process.poll() is not None:
                break
            time.sleep(0.1)
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception:
                process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    process.kill()
                process.wait(timeout=5.0)

        with self._lock:
            if self._rosbag_process is process:
                self._rosbag_process = None
                self._rosbag_last_exit_code = int(process.returncode or 0)
                if int(process.returncode or 0) == 0:
                    self._rosbag_last_error = ""
        self._broadcast_rosbag_status()
        return True, "", self.get_rosbag_status()

    def close(self) -> None:
        try:
            self.stop_rosbag()
        except Exception:
            pass

    async def _broadcast(self, payload: Dict[str, Any]) -> None:
        text = json.dumps(payload)
        with self._lock:
            clients = list(self._ws_clients)
        if not clients:
            return
        failed = []
        for ws in clients:
            try:
                sent = await self.send_ws_text(ws, text)
                if not sent:
                    failed.append(ws)
            except Exception:
                failed.append(ws)
        if failed:
            with self._lock:
                for ws in failed:
                    self._ws_clients.discard(ws)
                    self._ws_send_locks.pop(ws, None)

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        if not np.isfinite(msg.latitude) or not np.isfinite(msg.longitude):
            return

        gps_status_payload = self._build_gps_status_payload_from_navsat(msg.status.status)
        gps_status_broadcast = None
        now = time.monotonic()
        with self._lock:
            explicit_status_fresh = (
                self._last_explicit_gps_status_monotonic is not None
                and (now - self._last_explicit_gps_status_monotonic) <= 3.0
            )
            if not explicit_status_fresh and self._gps_status_payload_changed(
                self._gps_status_payload, gps_status_payload
            ):
                self._gps_status_payload = gps_status_payload
                gps_status_broadcast = {
                    "op": "gps_status",
                    "gps_status": dict(self._gps_status_payload),
                }

        with self._lock:
            heading_deg = self._last_robot_heading_deg
        pose = self._build_robot_pose(
            lat=float(msg.latitude),
            lon=float(msg.longitude),
            heading_deg=heading_deg,
        )
        with self._lock:
            self._last_robot_pose = pose
            last_sent = self._last_gps_broadcast_monotonic

        min_interval = 1.0 / max(0.1, float(self.gps_broadcast_hz))
        if last_sent is not None and (now - last_sent) < min_interval:
            if gps_status_broadcast is not None:
                asyncio.run_coroutine_threadsafe(self._broadcast(gps_status_broadcast), self._loop)
            return

        with self._lock:
            self._last_gps_broadcast_monotonic = now

        payload = {"op": "robot_pose", "pose": pose}
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)
        if gps_status_broadcast is not None:
            asyncio.run_coroutine_threadsafe(self._broadcast(gps_status_broadcast), self._loop)

    def _on_gps_status(self, msg: String) -> None:
        payload = self._build_gps_status_payload(raw=msg.data, source="rtk_status", available=True)
        should_broadcast = False
        with self._lock:
            self._last_explicit_gps_status_monotonic = time.monotonic()
            if self._gps_status_payload_changed(self._gps_status_payload, payload):
                self._gps_status_payload = payload
                should_broadcast = True
        if should_broadcast:
            asyncio.run_coroutine_threadsafe(
                self._broadcast(
                    {
                        "op": "gps_status",
                        "gps_status": dict(payload),
                    }
                ),
                self._loop,
            )

    def _yaw_deg_from_quaternion(
        self, x: float, y: float, z: float, w: float
    ) -> Optional[float]:
        if (
            (not np.isfinite(x))
            or (not np.isfinite(y))
            or (not np.isfinite(z))
            or (not np.isfinite(w))
        ):
            return None
        norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
        if norm < 1.0e-9:
            return None
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        siny_cosp = 2.0 * ((w * z) + (x * y))
        cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        while yaw_deg <= -180.0:
            yaw_deg += 360.0
        while yaw_deg > 180.0:
            yaw_deg -= 360.0
        return float(yaw_deg)

    def _build_robot_pose(
        self, lat: float, lon: float, heading_deg: Optional[float] = None
    ) -> Dict[str, float]:
        pose = {"lat": float(lat), "lon": float(lon)}
        if heading_deg is not None and np.isfinite(heading_deg):
            pose["heading_deg"] = float(heading_deg)
        return pose

    def _on_odometry(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        heading_deg = self._yaw_deg_from_quaternion(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        if heading_deg is None:
            return
        with self._lock:
            self._last_robot_heading_deg = float(heading_deg)
            if self._last_robot_pose is not None:
                self._last_robot_pose["heading_deg"] = float(heading_deg)

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        heading_deg = self._yaw_deg_from_quaternion(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        if heading_deg is None:
            return
        with self._lock:
            self._last_imu_heading_deg = float(heading_deg)

    def _on_nav_telemetry(self, msg: NavTelemetry) -> None:
        robot_pose_payload = None
        with self._lock:
            self._cmd_vel_safe = {
                "available": bool(msg.cmd_vel_available),
                "linear_x": float(msg.cmd_vel_linear_x),
                "angular_z": float(msg.cmd_vel_angular_z),
            }
            self._goal_active = bool(msg.goal_active)
            self._nav_result_status = int(getattr(msg, "nav_result_status", 0))
            self._nav_result_text = str(getattr(msg, "nav_result_text", ""))
            self._nav_result_event_id = int(getattr(msg, "nav_result_event_id", 0))

            last_cmd_age = None
            if self._manual_cmd_last_monotonic is not None:
                last_cmd_age = max(0.0, time.monotonic() - self._manual_cmd_last_monotonic)

            self._manual_control = {
                "enabled": bool(msg.manual_enabled),
                "linear_x_cmd": float(msg.manual_linear_x_cmd),
                "angular_z_cmd": float(msg.manual_angular_z_cmd),
                "last_cmd_age_s": last_cmd_age,
            }

            if np.isfinite(msg.robot_lat) and np.isfinite(msg.robot_lon):
                self._last_robot_pose = self._build_robot_pose(
                    lat=float(msg.robot_lat),
                    lon=float(msg.robot_lon),
                    heading_deg=self._last_robot_heading_deg,
                )
                robot_pose_payload = dict(self._last_robot_pose)

        asyncio.run_coroutine_threadsafe(
            self._broadcast(self._build_nav_telemetry_payload()), self._loop
        )
        if robot_pose_payload is not None:
            asyncio.run_coroutine_threadsafe(
                self._broadcast({"op": "robot_pose", "pose": robot_pose_payload}),
                self._loop,
            )

    def _on_nav_event(self, msg: NavEvent) -> None:
        payload = self._nav_event_to_payload(msg)
        with self._lock:
            self._recent_nav_events.append(payload)
        asyncio.run_coroutine_threadsafe(
            self._broadcast({"op": "nav_event", "event": payload}), self._loop
        )
        asyncio.run_coroutine_threadsafe(
            self._broadcast(self._build_nav_telemetry_payload()), self._loop
        )

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        alerts = [
            self._diagnostic_status_to_payload(status)
            for status in (msg.status or [])
            if self._should_surface_diagnostic(status)
        ]
        alerts.sort(key=lambda item: (-int(item.get("level", 0)), str(item.get("name", ""))))
        with self._lock:
            self._active_alerts = alerts
        asyncio.run_coroutine_threadsafe(
            self._broadcast({"op": "nav_alerts", "alerts": alerts}), self._loop
        )
        asyncio.run_coroutine_threadsafe(
            self._broadcast(self._build_nav_telemetry_payload()), self._loop
        )
        try:
            for status in (msg.status or []):
                name = str(status.name)
                level = self._diag_level_value(status.level)
                key = f"{status.name}:{level}:{status.message}"
                should_record = False
                with self._lock:
                    if key != self._mission_last_diag_key.get(name):
                        self._mission_last_diag_key[name] = key
                        should_record = True
                if should_record:
                    self._mission_record(
                        {
                            "t": time.time(),
                            "topic": "/diagnostics",
                            "data": {
                                "name": str(status.name),
                                "level": int(level),
                                "message": str(status.message),
                                "hardware_id": str(status.hardware_id),
                            },
                        }
                    )
        except Exception as exc:
            self.get_logger().error(f"mission diagnostics recording failed: {exc}")

    @staticmethod
    def _mission_line_count(path: Path) -> int:
        count = 0
        with path.open("rb") as handle:
            for _ in handle:
                count += 1
        return count

    @staticmethod
    def _mission_session_path(filename: Any) -> Tuple[Optional[Path], str]:
        name = str(filename or "").strip()
        if not name:
            return None, "filename is required"
        candidate = Path(name)
        if candidate.name != name or candidate.suffix != ".jsonl":
            return None, "invalid mission session filename"
        return MISSION_SESSION_DIR / name, ""

    def mission_list_sessions(self) -> List[Dict[str, Any]]:
        MISSION_SESSION_DIR.mkdir(parents=True, exist_ok=True)
        sessions: List[Dict[str, Any]] = []

        def _mtime(path: Path) -> float:
            try:
                return path.stat().st_mtime
            except OSError:
                return 0.0

        for path in sorted(MISSION_SESSION_DIR.glob("*.jsonl"), key=_mtime, reverse=True):
            try:
                stat = path.stat()
                line_count = self._mission_line_count(path)
            except OSError:
                continue
            sessions.append(
                {
                    "filename": path.name,
                    "size_bytes": int(stat.st_size),
                    "line_count": int(line_count),
                    "mtime_epoch_ms": int(stat.st_mtime * 1000.0),
                }
            )
        return sessions

    def mission_get_session(self, filename: Any) -> Tuple[bool, str, List[Dict[str, Any]]]:
        path, err = self._mission_session_path(filename)
        if path is None:
            return False, err, []
        if not path.is_file():
            return False, "mission session not found", []
        records: List[Dict[str, Any]] = []
        try:
            with path.open("r", encoding="utf-8") as handle:
                for line_number, raw in enumerate(handle, start=1):
                    line = raw.strip()
                    if not line:
                        continue
                    try:
                        parsed = json.loads(line)
                    except json.JSONDecodeError as exc:
                        return False, f"invalid JSONL at line {line_number}: {exc}", []
                    if isinstance(parsed, dict):
                        records.append(parsed)
                    else:
                        return False, f"invalid JSONL record at line {line_number}", []
        except OSError as exc:
            return False, f"failed to read mission session: {exc}", []
        return True, "", records

    def mission_get_status(self) -> Dict[str, Any]:
        MISSION_SESSION_DIR.mkdir(parents=True, exist_ok=True)
        status_path = MISSION_SESSION_DIR / MISSION_STATUS_FILE
        payload: Dict[str, Any] = {}
        try:
            if status_path.is_file():
                loaded = json.loads(status_path.read_text(encoding="utf-8"))
                if isinstance(loaded, dict):
                    payload = loaded
        except (OSError, json.JSONDecodeError):
            payload = {}
        active = bool(payload.get("active", False))
        filename = payload.get("current_session")
        if filename is not None:
            filename = str(filename)
        message_count = int(payload.get("message_count", 0) or 0)
        if active and filename:
            path, err = self._mission_session_path(filename)
            if err or path is None or not path.is_file():
                active = False
                filename = None
                message_count = 0
            else:
                try:
                    message_count = self._mission_line_count(path)
                except OSError:
                    message_count = int(payload.get("message_count", 0) or 0)
        elif not active:
            filename = None
        return {
            "active": bool(active),
            "filename": filename,
            "current_session": filename,
            "message_count": int(message_count),
            "updated_at": payload.get("updated_at"),
        }

    def _mission_record(self, record: dict) -> None:
        with self._lock:
            if not self._mission_active or self._mission_file is None:
                return
            mission_file = self._mission_file
        try:
            with mission_file.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record) + "\n")
        except Exception as exc:
            self.get_logger().error(f"mission record write failed: {exc}")
            return
        with self._lock:
            self._mission_message_count += 1
            current_session = self._mission_file.name if self._mission_file else None
            status = {
                "active": bool(self._mission_active),
                "current_session": current_session,
                "message_count": int(self._mission_message_count),
                "updated_at": time.time(),
            }
        try:
            MISSION_SESSION_DIR.mkdir(parents=True, exist_ok=True)
            (MISSION_SESSION_DIR / MISSION_STATUS_FILE).write_text(json.dumps(status), encoding="utf-8")
        except Exception as exc:
            self.get_logger().error(f"mission status write failed: {exc}")

    def _mission_start(self) -> None:
        try:
            MISSION_SESSION_DIR.mkdir(parents=True, exist_ok=True)
            sessions = sorted(MISSION_SESSION_DIR.glob("*.jsonl"), key=lambda p: p.stat().st_mtime)
            while len(sessions) > 7:
                oldest = sessions.pop(0)
                oldest.unlink()
        except Exception as exc:
            self.get_logger().error(f"mission session cleanup failed: {exc}")
        filename = f"mission_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}.jsonl"
        mission_file = MISSION_SESSION_DIR / filename
        try:
            mission_file.touch(exist_ok=True)
        except Exception as exc:
            self.get_logger().error(f"mission session create failed: {exc}")
            return
        with self._lock:
            self._mission_active = True
            self._mission_file = mission_file
            self._mission_message_count = 0
        self._mission_record({"t": time.time(), "topic": "system/session_start"})

    def _mission_stop(self) -> None:
        with self._lock:
            if not self._mission_active:
                return
            filename = self._mission_file.name if self._mission_file else ""
            message_count = int(self._mission_message_count)
        self._mission_record(
            {
                "t": time.time(),
                "topic": "system/session_end",
                "data": {"message_count": message_count},
            }
        )
        with self._lock:
            self._mission_active = False
            status = {
                "active": False,
                "current_session": None,
                "message_count": int(self._mission_message_count),
                "updated_at": time.time(),
            }
            if filename:
                self._mission_pending_send.append(filename)
        try:
            MISSION_SESSION_DIR.mkdir(parents=True, exist_ok=True)
            (MISSION_SESSION_DIR / MISSION_STATUS_FILE).write_text(json.dumps(status), encoding="utf-8")
        except Exception as exc:
            self.get_logger().error(f"mission status write failed: {exc}")
        self._mission_broadcast_pending()

    def _mission_broadcast_pending(self) -> None:
        with self._lock:
            if not self._ws_clients:
                return
            pending = list(self._mission_pending_send)
        sent: List[str] = []
        for filename in pending:
            ok, _, records = self.mission_get_session(filename)
            if not ok:
                continue
            payload = {"op": "mission.session_ready", "filename": filename, "lines": records}
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)
            sent.append(filename)
        if sent:
            with self._lock:
                for f in sent:
                    try:
                        self._mission_pending_send.remove(f)
                    except ValueError:
                        pass

    def _on_drive_telemetry(self, msg: DriveTelemetry) -> None:
        try:
            key = f"{msg.estop}:{msg.drive_enabled}"
            should_record = False
            with self._lock:
                if key != self._mission_last_drive_key:
                    self._mission_last_drive_key = key
                    should_record = True
            if not should_record:
                return
            self._mission_record(
                {
                    "t": time.time(),
                    "topic": "/controller/drive_telemetry",
                    "data": {
                        "estop": bool(msg.estop),
                        "drive_enabled": bool(msg.drive_enabled),
                        "speed_mps_measured": self._safe_float(msg.speed_mps_measured),
                        "steer_deg_measured": self._safe_float(msg.steer_deg_measured),
                        "brake_applied_pct": self._safe_int(msg.brake_applied_pct),
                        "ready": bool(msg.ready),
                        "fresh": bool(msg.fresh),
                    },
                }
            )
        except Exception as exc:
            self.get_logger().error(f"mission drive telemetry recording failed: {exc}")

    def _on_controller_telemetry(self, msg: String) -> None:
        try:
            raw = str(msg.data)
            try:
                parsed = json.loads(raw)
            except Exception:
                parsed = {}
            payload = parsed if isinstance(parsed, dict) else {}
            telemetry = payload.get("telemetry")
            telemetry = telemetry if isinstance(telemetry, dict) else {}
            command = payload.get("requested_auto_command")
            command = command if isinstance(command, dict) else {}
            key_payload = {
                "source": payload.get("source"),
                "ready": telemetry.get("ready"),
                "estop_active": telemetry.get("estop_active"),
                "failsafe_active": telemetry.get("failsafe_active"),
                "pi_fresh": telemetry.get("pi_fresh"),
                "control_source": telemetry.get("control_source"),
                "drive_enabled": command.get("drive_enabled"),
                "estop": command.get("estop"),
                "brake_pct": command.get("brake_pct"),
            }
            key = json.dumps(key_payload, sort_keys=True)
            should_record = False
            with self._lock:
                if key != self._mission_last_controller_telemetry_key:
                    self._mission_last_controller_telemetry_key = key
                    should_record = True
            if not should_record:
                return
            if payload:
                data = {
                    "source": str(payload.get("source", "")),
                    "telemetry": {
                        "ready": bool(telemetry.get("ready", False)),
                        "estop_active": bool(telemetry.get("estop_active", False)),
                        "failsafe_active": bool(telemetry.get("failsafe_active", False)),
                        "pi_fresh": bool(telemetry.get("pi_fresh", False)),
                        "control_source": str(telemetry.get("control_source", "")),
                        "speed_mps": self._safe_float(telemetry.get("speed_mps", 0.0)),
                        "steer_deg": self._safe_float(telemetry.get("steer_deg", 0.0)),
                        "brake_applied_pct": self._safe_int(telemetry.get("brake_applied_pct", 0)),
                    },
                    "requested_auto_command": {
                        "drive_enabled": bool(command.get("drive_enabled", False)),
                        "estop": bool(command.get("estop", False)),
                        "speed_mps": self._safe_float(command.get("speed_mps", 0.0)),
                        "steer_pct": self._safe_int(command.get("steer_pct", 0)),
                        "brake_pct": self._safe_int(command.get("brake_pct", 0)),
                    },
                }
            else:
                data = {"raw": raw}
            self._mission_record({"t": time.time(), "topic": "/controller/telemetry", "data": data})
        except Exception as exc:
            self.get_logger().error(f"mission controller telemetry recording failed: {exc}")

    def _on_controller_status(self, msg: String) -> None:
        try:
            raw = str(msg.data)
            try:
                parsed = json.loads(raw)
            except Exception:
                parsed = {}
            payload = parsed if isinstance(parsed, dict) else {}
            telemetry = payload.get("telemetry") if isinstance(payload.get("telemetry"), dict) else {}
            key_fields = {
                "ready": telemetry.get("ready"),
                "estop_active": telemetry.get("estop_active"),
                "failsafe_active": telemetry.get("failsafe_active"),
                "control_source": telemetry.get("control_source"),
                "overspeed_active": telemetry.get("overspeed_active"),
            }
            key = json.dumps(key_fields, sort_keys=True)
            should_record = False
            with self._lock:
                if key != self._mission_last_controller_status_key:
                    self._mission_last_controller_status_key = key
                    should_record = True
            if not should_record:
                return
            self._mission_record(
                {
                    "t": time.time(),
                    "topic": "/controller/status",
                    "data": {
                        "ready": bool(telemetry.get("ready", False)),
                        "estop_active": bool(telemetry.get("estop_active", False)),
                        "failsafe_active": bool(telemetry.get("failsafe_active", False)),
                        "pi_fresh": bool(telemetry.get("pi_fresh", False)),
                        "control_source": str(telemetry.get("control_source", "")),
                        "overspeed_active": bool(telemetry.get("overspeed_active", False)),
                        "speed_mps": self._safe_float(telemetry.get("speed_mps", 0.0)),
                        "steer_deg": self._safe_float(telemetry.get("steer_deg", 0.0)),
                        "brake_applied_pct": self._safe_int(telemetry.get("brake_applied_pct", 0)),
                    } if telemetry else {"raw": raw},
                }
            )
        except Exception as exc:
            self.get_logger().error(f"mission controller status recording failed: {exc}")

    def _on_rosout(self, msg: Log) -> None:
        try:
            if self._safe_int(msg.level) < 30:
                return
            self._mission_record(
                {
                    "t": time.time(),
                    "topic": "/rosout",
                    "data": {
                        "level": self._safe_int(msg.level),
                        "name": str(msg.name),
                        "msg": str(msg.msg),
                        "file": str(msg.file),
                        "function": str(msg.function),
                        "line": self._safe_int(msg.line),
                    },
                }
            )
        except Exception as exc:
            self.get_logger().error(f"mission rosout recording failed: {exc}")

    def _on_behavior_tree_log(self, msg: BehaviorTreeLog) -> None:
        try:
            events = [
                {
                    "node_name": str(e.node_name),
                    "previous_status": str(e.previous_status),
                    "current_status": str(e.current_status),
                }
                for e in (msg.event_log or [])
                if str(e.current_status) == "FAILURE"
            ]
            if not events:
                return
            self._mission_record(
                {
                    "t": time.time(),
                    "topic": "/behavior_tree_log",
                    "data": {"events": events},
                }
            )
        except Exception as exc:
            self.get_logger().error(f"mission behavior tree recording failed: {exc}")

    def _on_camera_image(self, msg: Image) -> None:
        stamp_ms = self._stamp_to_epoch_ms(msg.header.stamp) or int(time.time() * 1000.0)
        now = time.monotonic()
        min_period = 1.0 / self.camera_ws_max_fps if self.camera_ws_max_fps > 0.0 else 0.0
        try:
            frame = self._camera_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"camera frame decode failed: {exc}")
            return

        height = int(frame.shape[0]) if len(frame.shape) >= 1 else 0
        width = int(frame.shape[1]) if len(frame.shape) >= 2 else 0
        self._record_camera_frame_shape(stamp_ms, width, height)

        with self._lock:
            has_clients = bool(self._ws_clients)
        if not has_clients:
            return
        if min_period > 0.0:
            with self._lock:
                if (now - self._last_camera_ws_frame_monotonic) < min_period:
                    return
                self._last_camera_ws_frame_monotonic = now

        if self.camera_ws_width > 0 and width > self.camera_ws_width:
            ws_height = int(round(height * self.camera_ws_width / width))
            frame = cv2.resize(frame, (self.camera_ws_width, ws_height), interpolation=cv2.INTER_AREA)
            width = self.camera_ws_width
            height = ws_height

        encode_extension = ".png"
        encode_params: List[int] = [int(cv2.IMWRITE_PNG_COMPRESSION), 1]
        payload_encoding = "png"
        if self.camera_frame_encoding == "jpeg":
            encode_extension = ".jpg"
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.camera_jpeg_quality)]
            payload_encoding = "jpeg"

        ok, encoded = cv2.imencode(encode_extension, frame, encode_params)
        if not ok:
            self.get_logger().warning(
                f"camera frame {payload_encoding.upper()} encode failed"
            )
            return

        self._broadcast_from_thread(
            {
                "op": "camera_frame",
                "data": base64.b64encode(encoded.tobytes()).decode("ascii"),
                "encoding": payload_encoding,
                "stamp_ms": int(stamp_ms),
                "width": int(width),
                "height": int(height),
            }
        )

    def _on_camera_detections(self, msg: Detection2DArray) -> None:
        stamp_ms = self._stamp_to_epoch_ms(msg.header.stamp) or int(time.time() * 1000.0)
        frame_width, frame_height = self._resolve_camera_frame_shape(stamp_ms)

        with self._lock:
            has_clients = bool(self._ws_clients)
        if not has_clients:
            return

        detections: List[Dict[str, Any]] = []
        for detection in list(getattr(msg, "detections", []) or []):
            serialized = self._serialize_detection(
                detection,
                frame_width=frame_width,
                frame_height=frame_height,
            )
            if serialized is not None:
                detections.append(serialized)

        self._broadcast_from_thread(
            {
                "op": "camera_detections",
                "detections": detections,
                "stamp_ms": int(stamp_ms),
            }
        )

    def _wait_for_future(self, future: Any, timeout_s: float) -> Optional[Any]:
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                return future.result()
            if (time.monotonic() - start) >= timeout_s:
                return None
            time.sleep(0.01)
        return None

    def _call_service(self, client: Any, request: Any, timeout_s: float) -> Optional[Any]:
        service_name = getattr(client, "srv_name", "<unknown_service>")
        request_name = type(request).__name__
        if not client.wait_for_service(timeout_sec=min(timeout_s, 2.0)):
            self.get_logger().warning(
                f"Service unavailable: {service_name} (request={request_name})"
            )
            return None
        future = client.call_async(request)
        result = self._wait_for_future(future, timeout_s)
        if result is None:
            self.get_logger().warning(
                f"Service timeout: {service_name} (request={request_name}, timeout_s={timeout_s:.2f})"
            )
        return result

    def _resolve_waypoints_file(self, configured_path: str) -> Path:
        if configured_path:
            return Path(configured_path)

        config_dir = self._resolve_navegacion_config_dir()
        return config_dir / "saved_waypoints.yaml"

    def _resolve_datums_file(self, configured_path: str) -> Path:
        if configured_path:
            return Path(configured_path)

        config_dir = self._resolve_navegacion_config_dir()
        return config_dir / "datums.yaml"

    def _resolve_navegacion_config_dir(self) -> Path:
        try:
            pkg_dir = Path(get_package_share_directory("navegacion_gps"))
            default_dir = pkg_dir / "config"
            try:
                workspace_root = pkg_dir.parents[3]
                source_dir = workspace_root / "src" / "navegacion_gps" / "config"
                if source_dir.exists():
                    return source_dir
            except Exception:
                pass
            return default_dir
        except Exception:
            pass

        fallback = Path(__file__).resolve().parents[3] / "src" / "navegacion_gps" / "config"
        return fallback

    def _geojson_string_to_zones(self, geojson_text: str) -> List[Dict[str, Any]]:
        try:
            payload = json.loads(geojson_text)
        except Exception:
            return []
        if not isinstance(payload, dict):
            return []
        if str(payload.get("type", "")) != "FeatureCollection":
            return []
        features = payload.get("features")
        if not isinstance(features, list):
            return []

        zones: List[Dict[str, Any]] = []
        for feature_idx, feature in enumerate(features):
            if not isinstance(feature, dict):
                continue
            props = feature.get("properties")
            if not isinstance(props, dict):
                props = {}

            zone_id = str(props.get("id", f"zone_{feature_idx + 1}"))
            zone_type = str(props.get("type", "no_go"))
            enabled = bool(props.get("enabled", True))

            geometry = feature.get("geometry")
            if not isinstance(geometry, dict):
                continue
            geometry_type = str(geometry.get("type", ""))
            coordinates = geometry.get("coordinates", [])
            polygons: List[Any] = []
            if geometry_type == "Polygon":
                polygons = [coordinates]
            elif geometry_type == "MultiPolygon" and isinstance(coordinates, list):
                polygons = coordinates
            else:
                continue

            for poly_idx, polygon in enumerate(polygons):
                if not isinstance(polygon, list) or len(polygon) == 0:
                    continue
                outer = polygon[0]
                if not isinstance(outer, list):
                    continue
                points: List[Dict[str, float]] = []
                for coord in outer:
                    if not isinstance(coord, (list, tuple)) or len(coord) < 2:
                        continue
                    try:
                        lon = float(coord[0])
                        lat = float(coord[1])
                    except Exception:
                        continue
                    points.append({"lat": lat, "lon": lon})
                if len(points) < 3:
                    continue
                if points[0] == points[-1]:
                    points = points[:-1]
                if len(points) < 3:
                    continue

                polygon_id = zone_id if len(polygons) == 1 else f"{zone_id}__{poly_idx + 1}"
                zones.append(
                    {
                        "id": polygon_id,
                        "type": zone_type,
                        "enabled": enabled,
                        "polygon": points,
                    }
                )
        return zones

    def _normalize_geojson_payload(
        self, payload: Any
    ) -> Tuple[Optional[str], Optional[Dict[str, Any]], str]:
        try:
            if isinstance(payload, str):
                obj = json.loads(payload)
            elif isinstance(payload, dict):
                obj = payload
            else:
                return None, None, "geojson must be an object or string"
        except Exception as exc:
            return None, None, f"invalid geojson json: {exc}"
        if not isinstance(obj, dict):
            return None, None, "geojson root must be an object"
        if str(obj.get("type", "")) != "FeatureCollection":
            return None, None, "geojson root type must be FeatureCollection"
        features = obj.get("features")
        if not isinstance(features, list):
            return None, None, "geojson.features must be a list"
        return json.dumps(obj, ensure_ascii=True), obj, ""

    def _update_zones_state(self, response: GetZonesState.Response) -> None:
        geojson_text = str(response.geojson)
        zones_geojson: Dict[str, Any]
        try:
            parsed = json.loads(geojson_text) if geojson_text else {}
            zones_geojson = parsed if isinstance(parsed, dict) else {}
        except Exception:
            zones_geojson = {}

        with self._lock:
            self._zones = self._geojson_string_to_zones(geojson_text)
            self._zones_geojson = (
                zones_geojson
                if zones_geojson
                else {"type": "FeatureCollection", "features": []}
            )
            self._mask_ready = bool(response.mask_ready)
            self._mask_source = str(response.mask_source)
            if response.frame_id:
                self.map_frame = str(response.frame_id)

    def _update_nav_state(self, response: GetNavState.Response) -> None:
        with self._lock:
            self._goal_active = bool(response.goal_active)
            self._cmd_vel_safe = {
                "available": bool(response.cmd_vel_available),
                "linear_x": float(response.cmd_vel_linear_x),
                "angular_z": float(response.cmd_vel_angular_z),
            }
            self._manual_control = {
                "enabled": bool(response.manual_enabled),
                "linear_x_cmd": float(response.manual_linear_x_cmd),
                "angular_z_cmd": float(response.manual_angular_z_cmd),
                "last_cmd_age_s": None,
            }
            if np.isfinite(response.robot_lat) and np.isfinite(response.robot_lon):
                self._last_robot_pose = self._build_robot_pose(
                    lat=float(response.robot_lat),
                    lon=float(response.robot_lon),
                    heading_deg=self._last_robot_heading_deg,
                )

    def get_zones_state(self) -> Tuple[bool, str]:
        req = GetZonesState.Request()
        res = self._call_service(self._zones_get_state_client, req, self.request_timeout_s)
        if res is None:
            return False, "zones get_state timeout"
        if not res.ok:
            return False, str(res.error)
        self._update_zones_state(res)
        return True, ""

    def set_zones_geojson(self, payload: Any) -> Tuple[bool, str, bool]:
        geojson_text, _, err = self._normalize_geojson_payload(payload)
        if geojson_text is None:
            return False, err, False
        self.get_logger().info("WS->ROS set_zones_geojson")
        req = SetZonesGeoJson.Request()
        req.geojson = geojson_text
        res = self._call_service(self._zones_set_geojson_client, req, self.set_zones_timeout_s)
        if res is None:
            return False, "zones set_geojson timeout", False
        if not res.ok:
            self.get_logger().warning(f"set_zones_geojson failed: {res.error}")
            return False, str(res.error), bool(res.map_reloaded)
        self.get_zones_state()
        self.get_logger().info(
            "set_zones_geojson ok "
            f"(features={int(res.feature_count)}, polygons={int(res.polygon_count)}, "
            f"map_reloaded={bool(res.map_reloaded)})"
        )
        return True, "", bool(res.map_reloaded)

    def reload_zones_from_disk(self) -> Tuple[bool, str]:
        self.get_logger().info("WS->ROS reload_zones_from_disk")
        req = Trigger.Request()
        res = self._call_service(
            self._zones_reload_client,
            req,
            self.set_zones_timeout_s,
        )
        if res is None:
            return False, "zones reload timeout"
        if not res.success:
            return False, str(res.message or "reload failed")
        ok_state, err_state = self.get_zones_state()
        if not ok_state:
            return False, err_state
        return True, ""

    def get_nav_state(self) -> Tuple[bool, str]:
        req = GetNavState.Request()
        res = self._call_service(self._nav_get_state_client, req, self.request_timeout_s)
        if res is None:
            return False, "nav get_state timeout"
        if not res.ok:
            return False, str(res.error)
        self._update_nav_state(res)
        return True, ""

    def get_route_state(self) -> Tuple[bool, str]:
        req = GetRouteMissionState.Request()
        res = self._call_service(self._route_get_state_client, req, self.request_timeout_s)
        if res is None:
            return False, "route get_state timeout"
        if not res.ok:
            return False, str(res.error)
        self._update_route_state(res)
        return True, ""

    def _route_state_poll_tick(self) -> None:
        with self._lock:
            if self._route_state_poll_inflight:
                return
            self._route_state_poll_inflight = True
        thread = threading.Thread(
            target=self._run_route_state_poll,
            daemon=True,
            name="route_state_poll",
        )
        thread.start()

    def _run_route_state_poll(self) -> None:
        try:
            ok, err = self.get_route_state()
            if not ok and err:
                self.get_logger().debug(f"route state poll skipped: {err}")
        finally:
            with self._lock:
                self._route_state_poll_inflight = False

    @staticmethod
    def _normalize_yaw_deg(yaw_deg: float) -> float:
        yaw = float(yaw_deg)
        while yaw <= -180.0:
            yaw += 360.0
        while yaw > 180.0:
            yaw -= 360.0
        return float(yaw)

    @staticmethod
    def _bearing_deg_between_ll(
        start_lat: float, start_lon: float, end_lat: float, end_lon: float
    ) -> Optional[float]:
        if not (
            np.isfinite(start_lat)
            and np.isfinite(start_lon)
            and np.isfinite(end_lat)
            and np.isfinite(end_lon)
        ):
            return None
        meters_per_deg_lat = 111_320.0
        cos_lat = max(1.0e-6, abs(math.cos(math.radians(float(start_lat)))))
        meters_per_deg_lon = meters_per_deg_lat * cos_lat
        north_m = (float(end_lat) - float(start_lat)) * meters_per_deg_lat
        east_m = (float(end_lon) - float(start_lon)) * meters_per_deg_lon
        if math.hypot(north_m, east_m) <= 1.0e-6:
            return None
        return WebZoneServerNode._normalize_yaw_deg(math.degrees(math.atan2(north_m, east_m)))

    @staticmethod
    def _route_tangent_bearing_deg(
        incoming_bearing_deg: Optional[float], outgoing_bearing_deg: Optional[float]
    ) -> Optional[float]:
        if incoming_bearing_deg is None:
            return outgoing_bearing_deg
        if outgoing_bearing_deg is None:
            return incoming_bearing_deg

        incoming_rad = math.radians(float(incoming_bearing_deg))
        outgoing_rad = math.radians(float(outgoing_bearing_deg))
        east = math.cos(incoming_rad) + math.cos(outgoing_rad)
        north = math.sin(incoming_rad) + math.sin(outgoing_rad)
        if math.hypot(east, north) <= 1.0e-6:
            return outgoing_bearing_deg
        return WebZoneServerNode._normalize_yaw_deg(math.degrees(math.atan2(north, east)))

    def _resolve_waypoint_yaws(
        self, waypoints: List[Dict[str, Any]], loop: bool
    ) -> List[float]:
        resolved: List[Optional[float]] = [None] * len(waypoints)
        with self._lock:
            robot_pose = dict(self._last_robot_pose) if self._last_robot_pose is not None else None
            robot_heading_deg = self._last_robot_heading_deg

        for idx, wp in enumerate(waypoints):
            yaw_deg = wp.get("yaw_deg", UNSET)
            if yaw_deg is not UNSET and yaw_deg is not None and np.isfinite(float(yaw_deg)):
                resolved[idx] = self._normalize_yaw_deg(float(yaw_deg))

        for idx in range(len(waypoints)):
            if resolved[idx] is not None:
                continue
            lat = float(waypoints[idx]["lat"])
            lon = float(waypoints[idx]["lon"])

            next_idx: Optional[int] = None
            if idx + 1 < len(waypoints):
                next_idx = idx + 1
            elif loop and len(waypoints) > 1:
                next_idx = 0

            prev_idx: Optional[int] = None
            if idx > 0:
                prev_idx = idx - 1
            elif loop and len(waypoints) > 1:
                prev_idx = len(waypoints) - 1

            incoming_bearing: Optional[float] = None
            outgoing_bearing: Optional[float] = None
            if prev_idx is not None:
                prev_wp = waypoints[prev_idx]
                incoming_bearing = self._bearing_deg_between_ll(
                    float(prev_wp["lat"]), float(prev_wp["lon"]), lat, lon
                )
            if next_idx is not None:
                next_wp = waypoints[next_idx]
                outgoing_bearing = self._bearing_deg_between_ll(
                    lat, lon, float(next_wp["lat"]), float(next_wp["lon"])
                )

            tangent_bearing = self._route_tangent_bearing_deg(
                incoming_bearing, outgoing_bearing
            )
            if tangent_bearing is not None:
                resolved[idx] = tangent_bearing
                continue

            if robot_pose is not None:
                bearing = self._bearing_deg_between_ll(
                    float(robot_pose["lat"]),
                    float(robot_pose["lon"]),
                    lat,
                    lon,
                )
                if bearing is not None:
                    resolved[idx] = bearing
                    continue
                heading_deg = robot_pose.get("heading_deg", None)
                if heading_deg is not None and np.isfinite(float(heading_deg)):
                    resolved[idx] = self._normalize_yaw_deg(float(heading_deg))
                    continue

            if robot_heading_deg is not None and np.isfinite(float(robot_heading_deg)):
                resolved[idx] = self._normalize_yaw_deg(float(robot_heading_deg))
            else:
                resolved[idx] = 0.0

        return [float(yaw) for yaw in resolved]

    def set_nav_goals(
        self, waypoints: List[Dict[str, Any]], loop: bool
    ) -> Tuple[bool, str, int, bool]:
        if len(waypoints) == 0:
            return False, "at least one waypoint is required", 0, False

        self.get_logger().info(
            f"WS->ROS set_nav_goals (count={len(waypoints)}, loop={bool(loop)})"
        )
        req = SetNavGoalLL.Request()
        resolved_yaws_deg = self._resolve_waypoint_yaws(waypoints, loop)

        req.lats = [float(wp["lat"]) for wp in waypoints]
        req.lons = [float(wp["lon"]) for wp in waypoints]
        req.yaws_deg = [float(yaw_deg) for yaw_deg in resolved_yaws_deg]
        req.loop = bool(loop)

        # Keep legacy single-goal fields populated for compatibility.
        req.lat = float(req.lats[0])
        req.lon = float(req.lons[0])
        req.yaw_deg = float(req.yaws_deg[0])

        res = self._call_service(self._nav_set_goal_client, req, self.set_goal_timeout_s)
        if res is None:
            return False, "set_goal_ll timeout", len(waypoints), bool(loop)
        if not res.ok:
            self.get_logger().warning(f"set_nav_goals failed: {res.error}")
        else:
            self.get_logger().info("set_nav_goals ok")
        return bool(res.ok), str(res.error), len(waypoints), bool(loop)

    def set_route_mission(
        self,
        waypoints: List[Dict[str, Any]],
        loop: bool,
        leg_spacing_m: Optional[float] = None,
        chunk_span_m: Optional[float] = None,
        chunk_max_waypoints: Optional[int] = None,
    ) -> Tuple[bool, str, int, int]:
        if len(waypoints) == 0:
            return False, "at least one waypoint is required", 0, 0

        self.get_logger().info(
            "WS->ROS set_route_mission "
            f"(count={len(waypoints)}, loop={bool(loop)}, "
            f"leg_spacing_m={leg_spacing_m}, chunk_span_m={chunk_span_m}, "
            f"chunk_max_waypoints={chunk_max_waypoints})"
        )
        req = SetRouteMissionLL.Request()
        resolved_yaws_deg = self._resolve_waypoint_yaws(waypoints, loop)
        req.lats = [float(wp["lat"]) for wp in waypoints]
        req.lons = [float(wp["lon"]) for wp in waypoints]
        req.yaws_deg = [float(yaw_deg) for yaw_deg in resolved_yaws_deg]
        req.loop = bool(loop)
        req.leg_spacing_m = (
            float(leg_spacing_m)
            if leg_spacing_m is not None and np.isfinite(float(leg_spacing_m))
            else 0.0
        )
        req.chunk_span_m = (
            float(chunk_span_m)
            if chunk_span_m is not None and np.isfinite(float(chunk_span_m))
            else 0.0
        )
        req.chunk_max_waypoints = max(0, int(chunk_max_waypoints or 0))

        res = self._call_service(self._route_set_client, req, self.set_goal_timeout_s)
        if res is None:
            return False, "set_route_ll timeout", len(waypoints), 0
        if not res.ok:
            self.get_logger().warning(f"set_route_mission failed: {res.error}")
        else:
            self.get_logger().info("set_route_mission ok")
            self.get_route_state()
        return (
            bool(res.ok),
            str(res.error),
            int(getattr(res, "input_waypoint_count", len(waypoints))),
            int(getattr(res, "expanded_waypoint_count", 0)),
        )

    def cancel_route_mission(self) -> Tuple[bool, str]:
        req = CancelRouteMission.Request()
        res = self._call_service(self._route_cancel_client, req, self.request_timeout_s)
        if res is None:
            return False, "cancel_route timeout"
        if bool(res.ok):
            self.get_route_state()
        return bool(res.ok), str(res.error)

    def save_waypoints_file(self, waypoints: List[Dict[str, float]]) -> Tuple[bool, str, int]:
        ok, err, count = save_waypoints_yaml_file(self.waypoints_file, waypoints)
        if not ok:
            self.get_logger().warning(f"save_waypoints_file failed: {err}")
            return False, err, 0
        self.get_logger().info(f"save_waypoints_file ok (count={count})")
        return True, "", int(count)

    def load_waypoints_file(self) -> Tuple[bool, str, List[Dict[str, float]]]:
        ok, err, waypoints = load_waypoints_yaml_file(self.waypoints_file)
        if not ok:
            self.get_logger().warning(f"load_waypoints_file failed: {err}")
            return False, err, []
        self.get_logger().info(f"load_waypoints_file ok (count={len(waypoints)})")
        return True, "", waypoints

    def _runtime_datum_payload(self) -> Dict[str, Any]:
        with self._lock:
            snapshot = dict(self._datum_snapshot)
        return {
            "lat": snapshot.get("datum_lat"),
            "lon": snapshot.get("datum_lon"),
            "yaw_deg": snapshot.get("datum_yaw_deg"),
            "source": snapshot.get("last_set_source") or self.fixed_datum_source,
            "already_set": bool(snapshot.get("already_set", False)),
            "available": bool(snapshot.get("available", False)),
        }

    @staticmethod
    def _datum_matches_runtime(profile: Optional[Dict[str, Any]], runtime: Dict[str, Any]) -> bool:
        if not profile:
            return False
        try:
            plat = float(profile.get("lat"))
            plon = float(profile.get("lon"))
            pyaw = float(profile.get("yaw_deg", 0.0))
            rlat = float(runtime.get("lat"))
            rlon = float(runtime.get("lon"))
            ryaw = float(runtime.get("yaw_deg", 0.0))
        except (TypeError, ValueError):
            return False
        if not all(np.isfinite(value) for value in (plat, plon, pyaw, rlat, rlon, ryaw)):
            return False
        return (
            abs(plat - rlat) <= 1.0e-9
            and abs(plon - rlon) <= 1.0e-9
            and abs(WebZoneServerNode._normalize_yaw_deg(pyaw - ryaw)) <= 1.0e-6
        )

    def _build_datums_state_payload(self) -> Dict[str, Any]:
        with self._lock:
            doc = {
                "version": int(self._datums_doc.get("version", 1)),
                "selected_id": str(self._datums_doc.get("selected_id") or ""),
                "datums": [dict(item) for item in self._datums_doc.get("datums", [])],
            }
            datums_error = str(self._datums_error)
        runtime = self._runtime_datum_payload()
        selected = None
        for profile in doc["datums"]:
            if str(profile.get("id") or "") == doc["selected_id"]:
                selected = profile
                break
        pending_restart = selected is not None and not self._datum_matches_runtime(selected, runtime)
        return {
            "datums": doc["datums"],
            "selected_id": doc["selected_id"],
            "selected": dict(selected) if selected else None,
            "runtime": runtime,
            "pending_restart": bool(pending_restart),
            "apply_mode": "next_restart",
            "file_path": str(self.datums_file),
            "error": datums_error,
        }

    def load_datums_file(self) -> Tuple[bool, str, Dict[str, Any]]:
        ok, err, doc = load_datums_yaml_file(self.datums_file)
        with self._lock:
            self._datums_doc = doc
            self._datums_error = "" if ok else err
        if not ok:
            self.get_logger().warning(f"load_datums_file failed: {err}")
            return False, err, self._build_datums_state_payload()
        return True, "", self._build_datums_state_payload()

    def get_datums(self) -> Tuple[bool, str, Dict[str, Any]]:
        ok, err, payload = self.load_datums_file()
        return ok, err, payload

    def _save_datums_doc(self, doc: Dict[str, Any]) -> Tuple[bool, str, Dict[str, Any]]:
        ok, err = save_datums_yaml_file(self.datums_file, doc)
        if not ok:
            with self._lock:
                self._datums_error = err
            self.get_logger().warning(f"save_datums_file failed: {err}")
            return False, err, self._build_datums_state_payload()
        ok_load, err_load, payload = self.load_datums_file()
        return ok_load, err_load, payload

    def save_datum(self, datum: Dict[str, Any]) -> Tuple[bool, str, Dict[str, Any]]:
        ok, err, payload = self.load_datums_file()
        if not ok:
            return False, err, payload
        datums = [dict(item) for item in self._datums_doc.get("datums", [])]
        requested_id = str(datum.get("id") or "").strip()
        existing_index = None
        if requested_id:
            for idx, item in enumerate(datums):
                if str(item.get("id") or "") == requested_id:
                    existing_index = idx
                    break

        existing_ids = {
            str(item.get("id") or "")
            for idx, item in enumerate(datums)
            if idx != existing_index and str(item.get("id") or "")
        }
        profile, parse_err = normalize_datum_profile(
            datum,
            existing_ids=existing_ids,
            allow_existing_id=bool(requested_id),
        )
        if profile is None:
            return False, parse_err, self._build_datums_state_payload()

        now = utc_now_iso()
        if existing_index is not None:
            previous = datums[existing_index]
            profile["id"] = str(previous.get("id") or profile["id"])
            profile["created_at"] = str(previous.get("created_at") or profile["created_at"])
            profile["updated_at"] = now
            datums[existing_index] = profile
        else:
            profile["id"] = unique_datum_id(profile["name"], existing_ids, profile.get("id"))
            profile["created_at"] = now
            profile["updated_at"] = now
            datums.append(profile)

        selected_id = str(self._datums_doc.get("selected_id") or "")
        if bool(datum.get("select", False)):
            selected_id = str(profile["id"])
        return self._save_datums_doc(build_datums_doc(datums, selected_id))

    def delete_datum(self, datum_id: str) -> Tuple[bool, str, Dict[str, Any]]:
        datum_id = str(datum_id or "").strip()
        if not datum_id:
            return False, "id is required", self._build_datums_state_payload()
        ok, err, payload = self.load_datums_file()
        if not ok:
            return False, err, payload
        datums = [dict(item) for item in self._datums_doc.get("datums", [])]
        next_datums = [item for item in datums if str(item.get("id") or "") != datum_id]
        if len(next_datums) == len(datums):
            return False, f"datum not found: {datum_id}", self._build_datums_state_payload()
        selected_id = str(self._datums_doc.get("selected_id") or "")
        if selected_id == datum_id:
            selected_id = ""
        return self._save_datums_doc(build_datums_doc(next_datums, selected_id))

    def select_datum(self, datum_id: str) -> Tuple[bool, str, Dict[str, Any]]:
        datum_id = str(datum_id or "").strip()
        if not datum_id:
            return False, "id is required", self._build_datums_state_payload()
        ok, err, payload = self.load_datums_file()
        if not ok:
            return False, err, payload
        datums = [dict(item) for item in self._datums_doc.get("datums", [])]
        if not any(str(item.get("id") or "") == datum_id for item in datums):
            return False, f"datum not found: {datum_id}", self._build_datums_state_payload()
        return self._save_datums_doc(build_datums_doc(datums, datum_id))

    def capture_current_gps_datum(
        self,
        name: str,
        yaw_deg: Optional[float] = None,
        notes: str = "",
        select: bool = False,
    ) -> Tuple[bool, str, Dict[str, Any]]:
        with self._lock:
            robot_pose = dict(self._last_robot_pose) if self._last_robot_pose is not None else None
            gps_status = dict(self._gps_status_payload)
        if robot_pose is None:
            return False, "current GPS is unavailable", self._build_datums_state_payload()
        lat = robot_pose.get("lat")
        lon = robot_pose.get("lon")
        if not (
            isinstance(lat, (int, float))
            and isinstance(lon, (int, float))
            and np.isfinite(float(lat))
            and np.isfinite(float(lon))
        ):
            return False, "current GPS lat/lon is invalid", self._build_datums_state_payload()
        applied_yaw = (
            float(yaw_deg)
            if yaw_deg is not None and np.isfinite(float(yaw_deg))
            else float(self.fixed_datum_yaw_deg)
        )
        datum = {
            "name": name,
            "lat": float(lat),
            "lon": float(lon),
            "yaw_deg": applied_yaw,
            "source": "current_gps",
            "notes": notes,
            "select": bool(select),
            "metadata": {
                "captured_at": utc_now_iso(),
                "gps_status": gps_status,
            },
        }
        return self.save_datum(datum)

    def cancel_nav_goal(self) -> Tuple[bool, str]:
        req = CancelNavGoal.Request()
        res = self._call_service(self._nav_cancel_goal_client, req, self.request_timeout_s)
        if res is None:
            return False, "cancel_goal timeout"
        return bool(res.ok), str(res.error)

    def brake_nav(self) -> Tuple[bool, str]:
        req = BrakeNav.Request()
        res = self._call_service(self._nav_brake_client, req, self.request_timeout_s)
        if res is None:
            return False, "brake timeout"
        return bool(res.ok), str(res.error)

    def set_manual_mode(self, enabled: bool) -> Tuple[bool, str, bool]:
        req = SetManualMode.Request()
        req.enabled = bool(enabled)
        res = self._call_service(self._nav_set_manual_mode_client, req, self.request_timeout_s)
        if res is None:
            return False, "set_manual_mode timeout", bool(enabled)
        if res.ok:
            self.get_nav_state()
        return bool(res.ok), str(res.error), bool(res.enabled_after)

    def set_manual_cmd(
        self, linear_x: float, angular_z: float, brake_pct: int = 0
    ) -> Tuple[bool, str]:
        if not np.isfinite(linear_x) or not np.isfinite(angular_z):
            return False, "invalid manual command values"

        with self._lock:
            manual_enabled = bool(self._manual_control.get("enabled", False))
        if not manual_enabled:
            return False, "manual control is disabled"

        brake_pct_clamped = max(0, min(100, int(brake_pct)))
        cmd = CmdVelFinal()
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        cmd.brake_pct = brake_pct_clamped
        self._teleop_cmd_pub.publish(cmd)

        with self._lock:
            self._manual_cmd_last_monotonic = time.monotonic()
            self._manual_control["linear_x_cmd"] = float(linear_x)
            self._manual_control["angular_z_cmd"] = float(angular_z)
            self._manual_control["last_cmd_age_s"] = 0.0

        return True, ""

    def get_nav_snapshot(self) -> Tuple[bool, str, Dict[str, Any]]:
        started = time.perf_counter()
        req = GetNavSnapshot.Request()
        res = self._call_service(
            self._nav_snapshot_client, req, self.snapshot_request_timeout_s
        )
        if res is None:
            return False, "nav snapshot timeout", {}
        if not res.ok:
            self.get_logger().warning(f"get_nav_snapshot failed: {res.error}")
            return False, str(res.error), {}

        image_bytes = bytes(res.image_png)
        payload = {
            "op": "nav_snapshot",
            "ok": True,
            "mime": res.mime or "image/png",
            "width": int(res.width),
            "height": int(res.height),
            "frame_id": str(res.frame_id),
            "stamp": {
                "sec": int(res.stamp.sec),
                "nanosec": int(res.stamp.nanosec),
            },
            "layers": {
                "local_costmap": bool(res.layers.local_costmap),
                "global_costmap": bool(res.layers.global_costmap),
                "keepout_mask": bool(res.layers.keepout_mask),
                "footprint": bool(res.layers.footprint),
                "stop_zone": bool(res.layers.stop_zone),
                "scan": bool(res.layers.scan),
                "plan": bool(res.layers.plan),
                "collision_polygons": bool(res.layers.collision_polygons),
                "global_inset": bool(res.layers.global_inset),
            },
            "image_b64": base64.b64encode(image_bytes).decode("ascii"),
            "image_size_bytes": int(len(image_bytes)),
        }
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.get_logger().info(
            f"get_nav_snapshot ok (elapsed_ms={elapsed_ms:.1f}, bytes={len(image_bytes)})"
        )
        return True, "", payload

    def camera_pan(self, angle_deg: float) -> Tuple[bool, str, float]:
        req = CameraPan.Request()
        req.angle_deg = float(angle_deg)
        res = self._call_service(self._camera_pan_client, req, self.request_timeout_s)
        if res is None:
            return False, "camera_pan timeout", 0.0

        applied = float(res.applied_angle_deg)
        if res.ok:
            with self._lock:
                self._camera_status["ok"] = True
                self._camera_status["error"] = ""
                self._camera_status["last_command"] = f"angle:{applied:.1f}"
        else:
            with self._lock:
                self._camera_status["ok"] = False
                self._camera_status["error"] = str(res.error)
        return bool(res.ok), str(res.error), applied

    def camera_zoom_toggle(self) -> Tuple[bool, str]:
        req = Trigger.Request()
        res = self._call_service(
            self._camera_zoom_toggle_client,
            req,
            self.request_timeout_s,
        )
        if res is None:
            return False, "camera_zoom_toggle timeout"
        if res.success:
            with self._lock:
                self._camera_status["ok"] = True
                self._camera_status["error"] = ""
                self._camera_status["last_command"] = "zoom_toggle"
        else:
            with self._lock:
                self._camera_status["ok"] = False
                self._camera_status["error"] = str(res.message)
        return bool(res.success), str(res.message)

    def get_camera_status(self) -> Tuple[bool, str, Dict[str, Any]]:
        req = CameraStatus.Request()
        res = self._call_service(self._camera_status_client, req, self.request_timeout_s)
        if res is None:
            payload = {
                "op": "camera_status",
                "ok": False,
                "error": "camera_status timeout",
                "last_command": "",
                "zoom_in": False,
            }
            return False, payload["error"], payload

        payload = {
            "op": "camera_status",
            "ok": bool(res.ok),
            "error": str(res.error),
            "last_command": str(res.last_command),
            "zoom_in": bool(res.zoom_in),
        }
        with self._lock:
            self._camera_status = {
                "ok": bool(res.ok),
                "error": str(res.error),
                "last_command": str(res.last_command),
                "zoom_in": bool(res.zoom_in),
            }
        return bool(res.ok), str(res.error), payload

    def bootstrap_backend_state(self) -> None:
        self.get_logger().info("Bootstrapping gateway state from backend services...")
        ok_k, err_k = self.get_zones_state()
        if not ok_k and err_k:
            self.get_logger().warning(f"zones bootstrap failed: {err_k}")
        ok_n, err_n = self.get_nav_state()
        if not ok_n and err_n:
            self.get_logger().warning(f"nav bootstrap failed: {err_n}")
        ok_r, err_r = self.get_route_state()
        if not ok_r and err_r:
            self.get_logger().warning(f"route bootstrap failed: {err_r}")
        ok_c, err_c, _ = self.get_camera_status()
        if not ok_c and err_c:
            self.get_logger().warning(f"camera bootstrap failed: {err_c}")
        self._refresh_datum_snapshot()
        ok_d, err_d, _ = self.load_datums_file()
        if not ok_d and err_d:
            self.get_logger().warning(f"datums bootstrap failed: {err_d}")
        if self.sensor_bridge_enabled and self.sensor_bridge_http_url:
            self._sensor_bridge_poll_tick()
        self.get_logger().info("Gateway bootstrap finished")


class WebSocketApi:
    def __init__(self, node: WebZoneServerNode):
        self.node = node
        self._sensor_info_views: Dict[Any, Dict[str, Any]] = {}
        self._sensor_info_tasks: Dict[Any, asyncio.Task[Any]] = {}

    async def _reload_zones_on_connect(self) -> None:
        try:
            ok, err = await asyncio.to_thread(self.node.reload_zones_from_disk)
            if not ok:
                if err:
                    self.node.get_logger().warning(
                        f"zones reload on WS connect failed: {err}"
                    )
                return
            await self.node._broadcast(self.node.snapshot_state())
        except Exception as exc:
            self.node.get_logger().warning(f"zones reload on WS connect crashed: {exc}")

    async def _refresh_nav_state_on_connect(self) -> None:
        try:
            ok_nav, err_nav = await asyncio.to_thread(self.node.get_nav_state)
            if not ok_nav and err_nav:
                self.node.get_logger().warning(
                    f"nav refresh on WS connect failed: {err_nav}"
                )
            await self.node._broadcast(self.node.snapshot_state())
            ok_route, err_route = await asyncio.to_thread(self.node.get_route_state)
            if not ok_route and err_route:
                self.node.get_logger().warning(
                    f"route refresh on WS connect failed: {err_route}"
                )
                return
            await self.node._broadcast(self.node.snapshot_state())
        except Exception as exc:
            self.node.get_logger().warning(f"nav refresh on WS connect crashed: {exc}")

    def _parse_waypoints_from_message(
        self, msg: Dict[str, Any]
    ) -> Tuple[Optional[List[Dict[str, float]]], bool, str]:
        loop = bool(msg.get("loop", False))
        waypoints_raw = msg.get("waypoints")

        if waypoints_raw is None:
            try:
                lat = float(msg["lat"])
                lon = float(msg["lon"])
                yaw_raw = msg.get("yaw_deg", UNSET)
            except (KeyError, ValueError, TypeError) as exc:
                return None, False, f"invalid parameters: {exc}"
            if (not np.isfinite(lat)) or (not np.isfinite(lon)):
                return None, False, "lat/lon must be finite numbers"
            waypoint = {"lat": lat, "lon": lon}
            if yaw_raw is not UNSET:
                yaw_deg = float(yaw_raw)
                if not np.isfinite(yaw_deg):
                    return None, False, "yaw_deg must be a finite number"
                waypoint["yaw_deg"] = float(yaw_deg)
            return [waypoint], loop, ""

        if not isinstance(waypoints_raw, list) or len(waypoints_raw) == 0:
            return None, False, "waypoints must be a non-empty list"

        waypoints: List[Dict[str, float]] = []
        for idx, item in enumerate(waypoints_raw):
            if not isinstance(item, dict):
                return None, False, f"waypoint[{idx}] must be an object"
            try:
                lat = float(item["lat"])
                lon = float(item["lon"])
                yaw_raw = item.get("yaw_deg", UNSET)
            except (KeyError, ValueError, TypeError) as exc:
                return None, False, f"invalid waypoint[{idx}] values: {exc}"
            if (not np.isfinite(lat)) or (not np.isfinite(lon)):
                return None, False, f"waypoint[{idx}] lat/lon must be finite"
            waypoint: Dict[str, Any] = {"lat": lat, "lon": lon}
            if yaw_raw is not UNSET:
                yaw_deg = float(yaw_raw)
                if not np.isfinite(yaw_deg):
                    return None, False, f"waypoint[{idx}] yaw_deg must be finite"
                waypoint["yaw_deg"] = float(yaw_deg)
            waypoints.append(waypoint)

        return waypoints, loop, ""

    @staticmethod
    def _extract_client_req_id(msg: Dict[str, Any]) -> Optional[str]:
        req_id = msg.get("client_req_id")
        if req_id is None:
            return None
        if isinstance(req_id, (str, int, float, bool)):
            return str(req_id)
        return None

    def _build_ack_payload(
        self,
        request: str,
        ok: bool,
        error: Optional[str],
        client_req_id: Optional[str],
        extra: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "op": "ack",
            "ok": bool(ok),
            "request": str(request),
            "error": None if ok else str(error or "unknown error"),
        }
        if client_req_id is not None:
            payload["client_req_id"] = client_req_id
        if extra:
            payload.update(extra)
        return payload

    def _control_lock_extra(self, *, locked: Optional[bool] = None, reason: Optional[str] = None) -> Dict[str, Any]:
        locked_value = self.node.is_ui_control_locked() if locked is None else bool(locked)
        reason_value = self.node.get_ui_control_lock_reason() if reason is None else str(reason or "")
        return {
            "control_locked": bool(locked_value),
            "control_lock_reason": reason_value,
            "locked": bool(locked_value),
            "lock_reason": reason_value,
        }

    def _is_controlled_robot_op(self, op: Any, msg: Dict[str, Any]) -> bool:
        if not self.node.enable_control_lock:
            return False
        op_name = str(op or "")
        if op_name == "set_goal_ll":
            return True
        if op_name == "set_route_ll":
            return True
        if op_name == "set_manual_cmd":
            return True
        if op_name == "set_manual_mode":
            return bool(msg.get("enabled") is True)
        return False

    @staticmethod
    def _parse_route_options(
        msg: Dict[str, Any]
    ) -> Tuple[Optional[float], Optional[float], Optional[int], str]:
        leg_spacing_raw = msg.get("leg_spacing_m", None)
        chunk_span_raw = msg.get("chunk_span_m", None)
        chunk_max_raw = msg.get("chunk_max_waypoints", None)

        leg_spacing_m: Optional[float] = None
        chunk_span_m: Optional[float] = None
        chunk_max_waypoints: Optional[int] = None

        if leg_spacing_raw is not None:
            try:
                leg_spacing_m = float(leg_spacing_raw)
            except (TypeError, ValueError):
                return None, None, None, "leg_spacing_m must be a number"
            if (not np.isfinite(leg_spacing_m)) or leg_spacing_m <= 0.0:
                return None, None, None, "leg_spacing_m must be > 0"

        if chunk_span_raw is not None:
            try:
                chunk_span_m = float(chunk_span_raw)
            except (TypeError, ValueError):
                return None, None, None, "chunk_span_m must be a number"
            if (not np.isfinite(chunk_span_m)) or chunk_span_m <= 0.0:
                return None, None, None, "chunk_span_m must be > 0"

        if chunk_max_raw is not None:
            try:
                chunk_max_waypoints = int(chunk_max_raw)
            except (TypeError, ValueError):
                return None, None, None, "chunk_max_waypoints must be an integer"
            if chunk_max_waypoints <= 0:
                return None, None, None, "chunk_max_waypoints must be > 0"

        return leg_spacing_m, chunk_span_m, chunk_max_waypoints, ""

    async def _send_sensor_info_snapshot(self, ws: Any) -> None:
        view = self._sensor_info_views.get(ws)
        if not view or not bool(view.get("enabled")):
            return
        payload = self.node.build_sensor_info_message(
            tab=str(view.get("tab") or ""),
            interval_s=float(view.get("interval_s") or 0.1),
            topic_name=view.get("topic_name"),
        )
        await self._send_json(ws, payload)

    async def _sensor_info_loop(self, ws: Any) -> None:
        try:
            while True:
                view = self._sensor_info_views.get(ws)
                if not view or not bool(view.get("enabled")):
                    return
                await self._send_sensor_info_snapshot(ws)
                await asyncio.sleep(max(0.1, float(view.get("interval_s") or 0.1)))
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.node.get_logger().warning(f"sensor_info loop stopped: {exc}")
        finally:
            current = self._sensor_info_tasks.get(ws)
            if current is not None and current.done():
                self._sensor_info_tasks.pop(ws, None)

    def _restart_sensor_info_loop(self, ws: Any) -> None:
        current = self._sensor_info_tasks.pop(ws, None)
        if current is not None:
            current.cancel()
        view = self._sensor_info_views.get(ws)
        if not view or not bool(view.get("enabled")):
            return
        task = asyncio.create_task(self._sensor_info_loop(ws))
        self._sensor_info_tasks[ws] = task

    def _clear_sensor_info_client(self, ws: Any) -> None:
        task = self._sensor_info_tasks.pop(ws, None)
        if task is not None:
            task.cancel()
        self._sensor_info_views.pop(ws, None)

    async def _send_json(self, ws: Any, payload: Dict[str, Any]) -> None:
        await self.node.send_ws_json(ws, payload)

    async def _send_ack(
        self,
        ws: Any,
        request: str,
        ok: bool,
        error: Optional[str] = None,
        *,
        client_req_id: Optional[str] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        await self._send_json(
            ws,
            self._build_ack_payload(
                request=request,
                ok=ok,
                error=error,
                client_req_id=client_req_id,
                extra=extra,
            ),
        )

    async def handle(self, ws: Any, path: Optional[str] = None) -> None:
        _ = path
        pending_tasks: Set[asyncio.Task[Any]] = set()
        self.node.add_client(ws)
        self._sensor_info_views[ws] = {
            "enabled": False,
            "tab": None,
            "interval_s": 0.1,
            "topic_name": None,
        }
        try:
            await self._send_json(ws, self.node.snapshot_state())
            sessions = await asyncio.to_thread(self.node.mission_list_sessions)
            await self._send_json(ws, {"op": "mission.sessions_on_connect", "sessions": sessions})
            self.node._mission_broadcast_pending()
            connect_reload_task = asyncio.create_task(self._reload_zones_on_connect())
            pending_tasks.add(connect_reload_task)
            connect_reload_task.add_done_callback(
                lambda done: pending_tasks.discard(done)
            )
            connect_nav_refresh_task = asyncio.create_task(self._refresh_nav_state_on_connect())
            pending_tasks.add(connect_nav_refresh_task)
            connect_nav_refresh_task.add_done_callback(
                lambda done: pending_tasks.discard(done)
            )
            async for raw in ws:
                task = asyncio.create_task(self._handle_message_safe(ws, raw))
                pending_tasks.add(task)
                task.add_done_callback(lambda done: pending_tasks.discard(done))
        finally:
            for task in list(pending_tasks):
                task.cancel()
            if pending_tasks:
                await asyncio.gather(*pending_tasks, return_exceptions=True)
            self._clear_sensor_info_client(ws)
            self.node.remove_client(ws)

    async def _handle_message_safe(self, ws: Any, raw: str) -> None:
        try:
            await self._handle_message(ws, raw)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.node.get_logger().error(f"WS request handling failed: {exc}")

    async def _handle_message(self, ws: Any, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            self.node.get_logger().warning("Invalid WS JSON payload received")
            await self._send_ack(ws, "invalid_json", False, "invalid json")
            return

        client_req_id = self._extract_client_req_id(msg)
        op = msg.get("op")
        if op != "set_manual_cmd":
            self.node.get_logger().info(f"WS op received: {op}")
        if op == "get_state":
            ok_nav, err_nav = await asyncio.to_thread(self.node.get_nav_state)
            if not ok_nav and err_nav:
                self.node.get_logger().warning(f"get_state nav refresh failed: {err_nav}")
            ok_route, err_route = await asyncio.to_thread(self.node.get_route_state)
            if not ok_route and err_route:
                self.node.get_logger().warning(
                    f"get_state route refresh failed: {err_route}"
                )
            payload = self.node.snapshot_state()
            if client_req_id is not None:
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        if op == "get_rosbag_status":
            payload = {
                "op": "rosbag_status",
                "rosbag": await asyncio.to_thread(self.node.get_rosbag_status),
            }
            if client_req_id is not None:
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        if op == "set_control_lock":
            locked_raw = msg.get("locked")
            if not isinstance(locked_raw, bool):
                await self._send_ack(
                    ws,
                    "set_control_lock",
                    False,
                    "locked must be boolean",
                    client_req_id=client_req_id,
                    extra=self._control_lock_extra(),
                )
                return
            ok, err, locked_after, reason = await asyncio.to_thread(
                self.node.set_control_lock, locked_raw
            )
            await self._send_ack(
                ws,
                "set_control_lock",
                ok,
                err,
                client_req_id=client_req_id,
                extra=self._control_lock_extra(locked=locked_after, reason=reason),
            )
            if ok:
                await self.node._broadcast(self.node._build_nav_telemetry_payload())
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "control_heartbeat":
            ok, err, locked_after, reason = await asyncio.to_thread(
                self.node.control_heartbeat
            )
            await self._send_ack(
                ws,
                "control_heartbeat",
                ok,
                err,
                client_req_id=client_req_id,
                extra=self._control_lock_extra(locked=locked_after, reason=reason),
            )
            return

        if op == "set_sensor_info_view":
            enabled = bool(msg.get("enabled"))
            tab = None if msg.get("tab") is None else str(msg.get("tab"))
            topic_name = None if msg.get("topic_name") is None else str(msg.get("topic_name"))
            try:
                interval_s = float(msg.get("interval_s", 0.1))
            except (TypeError, ValueError):
                interval_s = 0.1
            interval_s = max(0.1, min(5.0, interval_s))

            self._sensor_info_views[ws] = {
                "enabled": enabled,
                "tab": tab,
                "interval_s": interval_s,
                "topic_name": topic_name,
            }
            if enabled:
                message = self.node.build_sensor_info_message(
                    tab=tab or "",
                    interval_s=interval_s,
                    topic_name=topic_name,
                )
                await self._send_ack(
                    ws,
                    "set_sensor_info_view",
                    True,
                    None,
                    client_req_id=client_req_id,
                    extra={
                        "enabled": True,
                        "tab": tab,
                        "interval_s": interval_s,
                        "topic_name": topic_name,
                        "implemented": bool(message.get("implemented", False)),
                    },
                )
                self._restart_sensor_info_loop(ws)
                await self._send_json(ws, message)
            else:
                self._restart_sensor_info_loop(ws)
                await self._send_ack(
                    ws,
                    "set_sensor_info_view",
                    True,
                    None,
                    client_req_id=client_req_id,
                    extra={
                        "enabled": False,
                        "tab": tab,
                        "interval_s": interval_s,
                        "topic_name": topic_name,
                        "implemented": False,
                    },
                )
            return

        if self._is_controlled_robot_op(op, msg) and self.node.is_ui_control_locked():
            await self._send_ack(
                ws,
                str(op),
                False,
                f"controls are locked ({self.node.get_ui_control_lock_reason() or 'locked'})",
                client_req_id=client_req_id,
                extra=self._control_lock_extra(),
            )
            return

        if op == "set_zones_geojson":
            geojson_payload = msg.get("geojson")
            if geojson_payload is None:
                await self._send_ack(
                    ws,
                    "set_zones_geojson",
                    False,
                    "geojson field is required",
                    client_req_id=client_req_id,
                    extra={"published": False},
                )
                return
            ok, err, published = await asyncio.to_thread(
                self.node.set_zones_geojson, geojson_payload
            )
            await self._send_ack(
                ws,
                "set_zones_geojson",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"published": bool(published)},
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "load_zones_file":
            ok, err = await asyncio.to_thread(self.node.reload_zones_from_disk)
            await self._send_ack(
                ws,
                "load_zones_file",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"published": bool(ok)},
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "save_waypoints_file":
            waypoints, _, parse_err = self._parse_waypoints_from_message(msg)
            if waypoints is None:
                await self._send_ack(
                    ws,
                    "save_waypoints_file",
                    False,
                    parse_err,
                    client_req_id=client_req_id,
                )
                return
            ok, err, count = await asyncio.to_thread(self.node.save_waypoints_file, waypoints)
            await self._send_ack(
                ws,
                "save_waypoints_file",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"waypoint_count": int(count)},
            )
            return

        if op == "load_waypoints_file":
            ok, err, waypoints = await asyncio.to_thread(self.node.load_waypoints_file)
            await self._send_ack(
                ws,
                "load_waypoints_file",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "waypoint_count": int(len(waypoints)),
                    "waypoints": list(waypoints) if ok else [],
                },
            )
            return

        if op == "get_datums":
            ok, err, datums_payload = await asyncio.to_thread(self.node.get_datums)
            payload = {
                "op": "datums",
                "ok": bool(ok),
                "error": None if ok else err,
                **datums_payload,
            }
            if client_req_id is not None:
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        if op == "save_datum":
            datum_raw = msg.get("datum")
            if not isinstance(datum_raw, dict):
                datum_raw = {
                    "id": msg.get("id"),
                    "name": msg.get("name"),
                    "lat": msg.get("lat"),
                    "lon": msg.get("lon"),
                    "yaw_deg": msg.get("yaw_deg", 0.0),
                    "notes": msg.get("notes", ""),
                    "select": msg.get("select", False),
                }
            ok, err, datums_payload = await asyncio.to_thread(self.node.save_datum, datum_raw)
            await self._send_ack(
                ws,
                "save_datum",
                ok,
                err,
                client_req_id=client_req_id,
                extra=datums_payload,
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "delete_datum":
            datum_id = str(msg.get("id") or msg.get("datum_id") or "")
            ok, err, datums_payload = await asyncio.to_thread(self.node.delete_datum, datum_id)
            await self._send_ack(
                ws,
                "delete_datum",
                ok,
                err,
                client_req_id=client_req_id,
                extra=datums_payload,
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "select_datum":
            datum_id = str(msg.get("id") or msg.get("datum_id") or "")
            ok, err, datums_payload = await asyncio.to_thread(self.node.select_datum, datum_id)
            await self._send_ack(
                ws,
                "select_datum",
                ok,
                err,
                client_req_id=client_req_id,
                extra=datums_payload,
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "capture_current_gps_datum":
            name = str(msg.get("name") or "").strip()
            notes = str(msg.get("notes") or "").strip()
            yaw_raw = msg.get("yaw_deg", None)
            yaw_deg = None
            if yaw_raw is not None:
                try:
                    yaw_deg = float(yaw_raw)
                except (TypeError, ValueError):
                    await self._send_ack(
                        ws,
                        "capture_current_gps_datum",
                        False,
                        "yaw_deg must be numeric",
                        client_req_id=client_req_id,
                    )
                    return
                if not np.isfinite(yaw_deg):
                    await self._send_ack(
                        ws,
                        "capture_current_gps_datum",
                        False,
                        "yaw_deg must be finite",
                        client_req_id=client_req_id,
                    )
                    return
            ok, err, datums_payload = await asyncio.to_thread(
                self.node.capture_current_gps_datum,
                name,
                yaw_deg,
                notes,
                _coerce_bool(msg.get("select", True)),
            )
            await self._send_ack(
                ws,
                "capture_current_gps_datum",
                ok,
                err,
                client_req_id=client_req_id,
                extra=datums_payload,
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "set_goal_ll":
            waypoints, loop_enabled, parse_err = self._parse_waypoints_from_message(msg)
            if waypoints is None:
                await self._send_ack(
                    ws,
                    "set_goal_ll",
                    False,
                    parse_err,
                    client_req_id=client_req_id,
                )
                return
            ok, err, waypoint_count, loop_used = await asyncio.to_thread(
                self.node.set_nav_goals, waypoints, loop_enabled
            )
            await self._send_ack(
                ws,
                "set_goal_ll",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "waypoint_count": int(waypoint_count),
                    "loop": bool(loop_used),
                    **self._control_lock_extra(),
                },
            )
            return

        if op == "set_route_ll":
            waypoints, loop_enabled, parse_err = self._parse_waypoints_from_message(msg)
            if waypoints is None:
                await self._send_ack(
                    ws,
                    "set_route_ll",
                    False,
                    parse_err,
                    client_req_id=client_req_id,
                )
                return
            leg_spacing_m, chunk_span_m, chunk_max_waypoints, options_err = (
                self._parse_route_options(msg)
            )
            if options_err:
                await self._send_ack(
                    ws,
                    "set_route_ll",
                    False,
                    options_err,
                    client_req_id=client_req_id,
                )
                return
            ok, err, input_count, expanded_count = await asyncio.to_thread(
                self.node.set_route_mission,
                waypoints,
                loop_enabled,
                leg_spacing_m,
                chunk_span_m,
                chunk_max_waypoints,
            )
            await self._send_ack(
                ws,
                "set_route_ll",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "input_waypoint_count": int(input_count),
                    "expanded_waypoint_count": int(expanded_count),
                    **self._control_lock_extra(),
                },
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "cancel_goal":
            ok, err = await asyncio.to_thread(self.node.cancel_nav_goal)
            await self._send_ack(
                ws, "cancel_goal", ok, err, client_req_id=client_req_id
            )
            return

        if op == "cancel_route":
            ok, err = await asyncio.to_thread(self.node.cancel_route_mission)
            await self._send_ack(
                ws, "cancel_route", ok, err, client_req_id=client_req_id
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "brake":
            ok, err = await asyncio.to_thread(self.node.brake_nav)
            await self._send_ack(
                ws, "brake", ok, err, client_req_id=client_req_id
            )
            return

        if op == "set_manual_mode":
            enabled_raw = msg.get("enabled")
            if not isinstance(enabled_raw, bool):
                await self._send_ack(
                    ws,
                    "set_manual_mode",
                    False,
                    "enabled must be boolean",
                    client_req_id=client_req_id,
                )
                return
            ok, err, enabled_after = await asyncio.to_thread(
                self.node.set_manual_mode,
                enabled_raw,
            )
            await self._send_ack(
                ws,
                "set_manual_mode",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "enabled": bool(enabled_after),
                    **self._control_lock_extra(),
                },
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "set_manual_cmd":
            try:
                linear_x = float(msg["linear_x"])
                angular_z = float(msg["angular_z"])
                brake_pct = int(float(msg.get("brake_pct", 0)))
            except (KeyError, ValueError, TypeError) as exc:
                await self._send_ack(
                    ws,
                    "set_manual_cmd",
                    False,
                    f"invalid parameters: {exc}",
                    client_req_id=client_req_id,
                )
                return
            ok, err = await asyncio.to_thread(
                self.node.set_manual_cmd,
                linear_x,
                angular_z,
                brake_pct,
            )
            await self._send_ack(
                ws,
                "set_manual_cmd",
                ok,
                err,
                client_req_id=client_req_id,
                extra=self._control_lock_extra(),
            )
            return

        if op == "get_nav_snapshot":
            ok, err, payload = await asyncio.to_thread(self.node.get_nav_snapshot)
            if client_req_id is not None:
                payload = dict(payload)
                payload["client_req_id"] = client_req_id
            if ok:
                await self._send_json(ws, payload)
                return
            await self._send_json(
                ws,
                {
                    "op": "nav_snapshot",
                    "ok": False,
                    "error": err or "snapshot request failed",
                    "client_req_id": client_req_id,
                },
            )
            return

        if op == "mission.list_sessions":
            payload = {
                "op": "mission.list_sessions",
                "ok": True,
                "sessions": await asyncio.to_thread(self.node.mission_list_sessions),
            }
            if client_req_id is not None:
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        if op == "mission.get_session":
            filename = msg.get("filename")
            ok, err, records = await asyncio.to_thread(self.node.mission_get_session, filename)
            payload = {
                "op": "mission.get_session",
                "ok": bool(ok),
                "filename": str(filename or ""),
                "lines": records if ok else [],
                "error": None if ok else err,
            }
            if client_req_id is not None:
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        if op == "mission.download_session":
            filename = msg.get("filename")
            ok, err, records = await asyncio.to_thread(self.node.mission_get_session, filename)
            payload = {
                "op": "mission.download_session",
                "ok": bool(ok),
                "filename": str(filename or ""),
                "lines": records if ok else [],
                "error": None if ok else err,
                "download": True,
            }
            if client_req_id is not None:
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        if op == "mission.get_status":
            payload = {
                "op": "mission.get_status",
                "ok": True,
                "status": await asyncio.to_thread(self.node.mission_get_status),
            }
            if client_req_id is not None:
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        if op == "start_rosbag":
            profile = str(msg.get("profile", "core") or "core")
            ok, err, status_payload = await asyncio.to_thread(self.node.start_rosbag, profile)
            await self._send_ack(
                ws,
                "start_rosbag",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"rosbag": status_payload},
            )
            return

        if op == "stop_rosbag":
            ok, err, status_payload = await asyncio.to_thread(self.node.stop_rosbag)
            await self._send_ack(
                ws,
                "stop_rosbag",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"rosbag": status_payload},
            )
            return

        if op == "camera_pan":
            angle_raw = msg.get("angle")
            try:
                angle = float(angle_raw)
            except (ValueError, TypeError):
                await self._send_ack(
                    ws,
                    "camera_pan",
                    False,
                    "angle must be numeric",
                    client_req_id=client_req_id,
                )
                return
            if not np.isfinite(angle):
                await self._send_ack(
                    ws,
                    "camera_pan",
                    False,
                    "angle must be finite",
                    client_req_id=client_req_id,
                )
                return
            ok, err, _ = await asyncio.to_thread(self.node.camera_pan, angle)
            await self._send_ack(
                ws, "camera_pan", ok, err, client_req_id=client_req_id
            )
            return

        if op == "camera_zoom_toggle":
            ok, err = await asyncio.to_thread(self.node.camera_zoom_toggle)
            await self._send_ack(
                ws,
                "camera_zoom_toggle",
                ok,
                err,
                client_req_id=client_req_id,
            )
            return

        if op == "get_camera_status":
            _, _, payload = await asyncio.to_thread(self.node.get_camera_status)
            if client_req_id is not None:
                payload = dict(payload)
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        await self._send_ack(
            ws,
            str(op),
            False,
            "unknown op",
            client_req_id=client_req_id,
            extra={"published": False},
        )
        self.node.get_logger().warning(f"Unknown WS op received: {op}")


async def async_main() -> None:
    rclpy.init()
    loop = asyncio.get_running_loop()
    node = WebZoneServerNode(loop)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    await asyncio.to_thread(node.bootstrap_backend_state)

    api = WebSocketApi(node)
    server = await websockets.serve(api.handle, node.ws_host, node.ws_port)
    node.get_logger().info(
        f"WebSocket server listening on ws://{node.ws_host}:{node.ws_port}"
    )

    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        pass
    finally:
        server.close()
        await server.wait_closed()
        await asyncio.to_thread(node.close)
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def main() -> None:
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
