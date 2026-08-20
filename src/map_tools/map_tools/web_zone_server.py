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
from sensor_msgs.msg import BatteryState, Image, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from interfaces.msg import CmdVelFinal, DriveTelemetry, GeoRing, NavEvent
from interfaces.msg import NavTelemetry, NoGoPoint
from interfaces.srv import (
    BrakeNav,
    CameraPan,
    CameraPreset,
    CameraSavePreset,
    CameraPtz,
    CameraPtzState,
    CameraStatus,
    CancelNavGoal,
    CancelPatrolMission,
    CancelRouteMission,
    GenerateCoveragePlanLL,
    GetDatum,
    GetNavSnapshot,
    GetNavState,
    GetPatrolMissionState,
    GetRouteMissionState,
    RequestReturnHome,
    GetZonesState,
    SetManualMode,
    SetNavigationProfile,
    SetNavGoalLL,
    SetPatrolMissionLL,
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


def _geo_ring(raw: Any) -> GeoRing:
    """Anillo del cockpit al mensaje GeoRing.

    Acepta ``{"vertices": [{"lat": .., "lon": ..}]}`` y tambien una lista pelada
    de vertices. Los anillos viajan ABIERTOS: el cockpit no repite el primer
    vertice y el route_executor descarta el cierre si igual llegara.
    """
    if raw is None:
        return GeoRing()
    vertices = raw.get("vertices") if isinstance(raw, dict) else raw
    if not isinstance(vertices, list):
        return GeoRing()
    anillo = GeoRing()
    for vertex in vertices:
        if not isinstance(vertex, dict):
            continue
        try:
            anillo.vertices.append(
                NoGoPoint(lat=float(vertex["lat"]), lon=float(vertex["lon"]))
            )
        except (KeyError, TypeError, ValueError):
            continue
    return anillo


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
        self.declare_parameter("snapshot_request_timeout_s", 5.0)
        self.declare_parameter("set_zones_timeout_s", 12.0)
        self.declare_parameter("set_goal_timeout_s", 12.0)
        self.declare_parameter("coverage_plan_timeout_s", 5.0)
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
        self.declare_parameter(
            "coverage_plan_service",
            "/route_executor/generate_coverage_plan_ll",
        )
        self.declare_parameter("route_cancel_service", "/route_executor/cancel_route")
        self.declare_parameter("route_get_state_service", "/route_executor/get_state")
        self.declare_parameter("set_navigation_profile_service", "/route_executor/set_navigation_profile")
        self.declare_parameter("patrol_set_service", "/route_executor/set_patrol_ll")
        self.declare_parameter("patrol_cancel_service", "/route_executor/cancel_patrol")
        self.declare_parameter("patrol_get_state_service", "/route_executor/get_patrol_state")
        self.declare_parameter("patrol_return_home_service", "/route_executor/request_return_home")
        self.declare_parameter("route_state_poll_hz", 2.0)
        self.declare_parameter("coverage_reference_max_age_s", 5.0)
        self.declare_parameter("coverage_start_max_distance_m", 5.0)
        self.declare_parameter("coverage_start_max_heading_error_deg", 30.0)
        # Opt-in de simulacion: en real se conservan solamente los extremos de
        # pasada. Cuando esta activo, cada cabecera se envia como
        # guia-exterior + inicio-de-la-fila-siguiente dentro del mismo chunk.
        self.declare_parameter("coverage_use_headland_guides", False)
        self.declare_parameter("teleop_cmd_topic", "/cmd_vel_teleop")

        self.declare_parameter("nav_snapshot_service", "/nav_snapshot_server/get_nav_snapshot")
        self.declare_parameter("nav_telemetry_topic", "/nav_command_server/telemetry")
        self.declare_parameter("nav_events_topic", "/nav_command_server/events")
        self.declare_parameter("battery_state_topic", "/battery_state")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("rosbag_output_dir", "/ros2_ws/bags")
        self.declare_parameter("camera_pan_service", "/camara/camera_pan")
        self.declare_parameter("camera_zoom_toggle_service", "/camara/camera_zoom_toggle")
        self.declare_parameter("camera_status_service", "/camara/camera_status")
        self.declare_parameter("camera_ptz_service", "/camara/camera_ptz")
        self.declare_parameter("camera_preset_service", "/camara/camera_preset")
        self.declare_parameter("camera_save_preset_service", "/camara/camera_save_preset")
        self.declare_parameter("camera_ptz_state_service", "/camara/camera_ptz_state")
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
        self.coverage_plan_timeout_s = max(
            0.5,
            float(self.get_parameter("coverage_plan_timeout_s").value),
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
        self.coverage_plan_service = str(
            self.get_parameter("coverage_plan_service").value
        )
        self.route_cancel_service = str(self.get_parameter("route_cancel_service").value)
        self.route_get_state_service = str(
            self.get_parameter("route_get_state_service").value
        )
        self.set_navigation_profile_service = str(
            self.get_parameter("set_navigation_profile_service").value
        )
        self.patrol_set_service = str(self.get_parameter("patrol_set_service").value)
        self.patrol_cancel_service = str(self.get_parameter("patrol_cancel_service").value)
        self.patrol_get_state_service = str(
            self.get_parameter("patrol_get_state_service").value
        )
        self.patrol_return_home_service = str(
            self.get_parameter("patrol_return_home_service").value
        )
        self.route_state_poll_hz = max(
            0.2, float(self.get_parameter("route_state_poll_hz").value)
        )
        self.coverage_reference_max_age_s = max(
            0.5,
            float(self.get_parameter("coverage_reference_max_age_s").value),
        )
        self.coverage_start_max_distance_m = max(
            0.1,
            float(self.get_parameter("coverage_start_max_distance_m").value),
        )
        self.coverage_start_max_heading_error_deg = min(
            180.0,
            max(
                1.0,
                float(
                    self.get_parameter(
                        "coverage_start_max_heading_error_deg"
                    ).value
                ),
            ),
        )
        self.coverage_use_headland_guides = bool(
            self.get_parameter("coverage_use_headland_guides").value
        )
        self.teleop_cmd_topic = str(self.get_parameter("teleop_cmd_topic").value)

        self.nav_snapshot_service = str(self.get_parameter("nav_snapshot_service").value)
        self.nav_telemetry_topic = str(self.get_parameter("nav_telemetry_topic").value)
        self.nav_events_topic = str(self.get_parameter("nav_events_topic").value)
        self.battery_state_topic = str(self.get_parameter("battery_state_topic").value)
        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.rosbag_output_dir = str(self.get_parameter("rosbag_output_dir").value)
        self.camera_pan_service = str(self.get_parameter("camera_pan_service").value)
        self.camera_zoom_toggle_service = str(
            self.get_parameter("camera_zoom_toggle_service").value
        )
        self.camera_status_service = str(self.get_parameter("camera_status_service").value)
        self.camera_ptz_service = str(self.get_parameter("camera_ptz_service").value)
        self.camera_preset_service = str(
            self.get_parameter("camera_preset_service").value
        )
        self.camera_save_preset_service = str(
            self.get_parameter("camera_save_preset_service").value
        )
        self.camera_ptz_state_service = str(
            self.get_parameter("camera_ptz_state_service").value
        )
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
        self._last_robot_pose_monotonic: Optional[float] = None
        self._last_robot_heading_monotonic: Optional[float] = None
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
        self._drive_telemetry = {
            "available": False,
            "ready": False,
            "fresh": False,
            "drive_enabled": False,
            "estop": False,
            "reverse_requested": False,
            "speed_valid": False,
            "steer_valid": False,
            "control_source": "NONE",
            "speed_mps_measured": 0.0,
            "steer_deg_measured": 0.0,
            "brake_applied_pct": 0,
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
            "pan_deg": 0.0,
            "tilt_deg": 0.0,
            "zoom_level": 0.0,
            "active_preset": "",
        }
        self._route_mission = self._build_default_route_mission_payload()
        self._patrol_mission = self._build_default_patrol_mission_payload()
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
        self._battery_pct: Optional[float] = None
        self._battery_voltage_v: Optional[float] = None
        self._battery_state: str = ""
        self._battery_mission_state: str = ""
        self._battery_return_home_recommended: Optional[bool] = None
        self._battery_recovered_voltage_v: Optional[float] = None
        self._battery_loaded_voltage_v: Optional[float] = None
        self._battery_present: Optional[bool] = None
        self._battery_updated_age_s: Optional[float] = None
        self._battery_ws_key: Optional[str] = None
        self._battery_use_controller_telemetry = False
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
        self._battery_state_sub = self.create_subscription(
            BatteryState, self.battery_state_topic, self._on_battery_state, 10
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
        self._rtk_source_select_pub = self.create_publisher(String, "/gps/rtk_source/select", 10)
        self._rtk_source_manage_pub = self.create_publisher(String, "/gps/rtk_source/manage_json", 10)
        self._rtk_sources_list = []
        self._rtk_source_status = {}
        self.create_subscription(String, "/gps/rtk_sources/json", self._rtk_sources_cb, 2)
        self.create_subscription(String, "/gps/rtk_source/status_json", self._rtk_source_status_cb, 2)
        self._nav_get_state_client = self.create_client(GetNavState, self.nav_get_state_service)
        self._route_set_client = self.create_client(SetRouteMissionLL, self.route_set_service)
        self._coverage_plan_client = self.create_client(
            GenerateCoveragePlanLL,
            self.coverage_plan_service,
        )
        self._route_cancel_client = self.create_client(
            CancelRouteMission, self.route_cancel_service
        )
        self._route_get_state_client = self.create_client(
            GetRouteMissionState, self.route_get_state_service
        )
        self._navigation_profile_client = self.create_client(
            SetNavigationProfile, self.set_navigation_profile_service
        )
        self._patrol_set_client = self.create_client(
            SetPatrolMissionLL, self.patrol_set_service
        )
        self._patrol_cancel_client = self.create_client(
            CancelPatrolMission, self.patrol_cancel_service
        )
        self._patrol_get_state_client = self.create_client(
            GetPatrolMissionState, self.patrol_get_state_service
        )
        self._patrol_return_home_client = self.create_client(
            RequestReturnHome, self.patrol_return_home_service
        )
        self._nav_snapshot_client = self.create_client(GetNavSnapshot, self.nav_snapshot_service)
        self._camera_pan_client = self.create_client(CameraPan, self.camera_pan_service)
        self._camera_zoom_toggle_client = self.create_client(
            Trigger, self.camera_zoom_toggle_service
        )
        self._camera_status_client = self.create_client(
            CameraStatus, self.camera_status_service
        )
        self._camera_ptz_client = self.create_client(CameraPtz, self.camera_ptz_service)
        self._camera_preset_client = self.create_client(
            CameraPreset, self.camera_preset_service
        )
        self._camera_save_preset_client = self.create_client(
            CameraSavePreset, self.camera_save_preset_service
        )
        self._camera_ptz_state_client = self.create_client(
            CameraPtzState, self.camera_ptz_state_service
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
            f"camera_ptz={self.camera_ptz_service}, camera_preset={self.camera_preset_service}, "
            f"camera_save_preset={self.camera_save_preset_service}, "
            f"camera_ptz_state={self.camera_ptz_state_service}, "
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
                "drive_telemetry": dict(self._drive_telemetry),
                "manual_control": dict(self._manual_control),
                "goal_active": bool(self._goal_active),
                "nav_result_status": int(self._nav_result_status),
                "nav_result_text": str(self._nav_result_text),
                "nav_result_event_id": int(self._nav_result_event_id),
                "route_mission": dict(self._route_mission),
                "patrol_mission": dict(self._patrol_mission),
                "alerts": list(self._active_alerts),
                "recent_events": list(self._recent_nav_events),
                "rosbag": self._build_rosbag_status_payload_locked(),
                "camera_status": dict(self._camera_status),
                "datum": dict(self._datum_snapshot),
                "datums": datums_payload,
                "rtk_source_state": dict(self._rtk_source_status) if self._rtk_source_status else None,
                "rtk_sources": [dict(item) for item in self._rtk_sources_list],
                **connection_status,
            }

    def _build_nav_telemetry_payload(self) -> Dict[str, Any]:
        with self._lock:
            cmd_vel_safe = dict(self._cmd_vel_safe)
            drive_telemetry = dict(self._drive_telemetry)
            manual_control = dict(self._manual_control)
            goal_active = bool(self._goal_active)
            nav_result_status = int(self._nav_result_status)
            nav_result_text = str(self._nav_result_text)
            nav_result_event_id = int(self._nav_result_event_id)
            robot_pose = dict(self._last_robot_pose) if self._last_robot_pose is not None else None
            route_mission = dict(self._route_mission)
            patrol_mission = dict(self._patrol_mission)
            alerts = list(self._active_alerts)
            recent_events = list(self._recent_nav_events)
            connection_status = self._connection_status_locked()
        return {
            "op": "nav_telemetry",
            "cmd_vel_safe": cmd_vel_safe,
            "drive_telemetry": drive_telemetry,
            "manual_control": manual_control,
            "goal_active": goal_active,
            "nav_result_status": nav_result_status,
            "nav_result_text": nav_result_text,
            "nav_result_event_id": nav_result_event_id,
            "robot_pose": robot_pose,
            "route_mission": route_mission,
            "patrol_mission": patrol_mission,
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
            "low_battery_active": False,
            "return_home_requested": False,
            "return_home_active": False,
            "return_home_exit_waypoint_index": -1,
            "return_home_phase": "idle",
            "home_available": False,
            "home_waypoint": None,
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
            "action_active": False,
            "action_waypoint_index": 0,
            "action_type": "",
            "action_remaining_s": 0.0,
            "mission_waypoints": [],
            "active_chunk_waypoints": [],
        }

    @staticmethod
    def _build_default_patrol_mission_payload() -> Dict[str, Any]:
        return {
            "active": False,
            "phase": "idle",
            "low_battery_active": False,
            "return_home_requested": False,
            "return_home_active": False,
            "return_exit_loop_index": -1,
            "depart_entry_loop_index": -1,
            "home_available": False,
            "mission_id": "",
            "status": "idle",
            "home_waypoint": None,
            "loop_waypoints": [],
            "return_waypoints": [],
            "depart_waypoints": [],
            "active_chunk_waypoints": [],
        }

    @staticmethod
    def _route_waypoints_from_state(
        lats: Sequence[float],
        lons: Sequence[float],
        yaws_deg: Sequence[float],
        action_jsons: Optional[Sequence[str]] = None,
        roles: Optional[Sequence[str]] = None,
    ) -> List[Dict[str, Any]]:
        if not (len(lats) == len(lons) == len(yaws_deg)):
            return []
        actions = list(action_jsons or [])
        role_values = list(roles or [])
        waypoints: List[Dict[str, Any]] = []
        for idx, (lat, lon, yaw_deg) in enumerate(zip(lats, lons, yaws_deg)):
            lat_value = float(lat)
            lon_value = float(lon)
            yaw_value = float(yaw_deg)
            if not (
                np.isfinite(lat_value)
                and np.isfinite(lon_value)
                and np.isfinite(yaw_value)
            ):
                continue
            waypoint: Dict[str, Any] = {
                "lat": lat_value,
                "lon": lon_value,
                "yaw_deg": yaw_value,
            }
            action_json = str(actions[idx] if idx < len(actions) else "").strip()
            if action_json:
                try:
                    parsed_actions = json.loads(action_json)
                except Exception:
                    parsed_actions = []
                if isinstance(parsed_actions, list) and parsed_actions:
                    waypoint["actions"] = parsed_actions
            role = str(role_values[idx] if idx < len(role_values) else "normal").strip().lower()
            if role and role != "normal":
                waypoint["role"] = role
            waypoints.append(waypoint)
        return waypoints

    def _update_route_state(self, response: GetRouteMissionState.Response) -> None:
        payload = {
            "active": bool(response.active),
            "paused": bool(response.paused),
            "loop": bool(response.loop),
            "low_battery_active": bool(getattr(response, "low_battery_active", False)),
            "return_home_requested": bool(getattr(response, "return_home_requested", False)),
            "return_home_active": bool(getattr(response, "return_home_active", False)),
            "return_home_exit_waypoint_index": int(
                getattr(response, "return_home_exit_waypoint_index", -1)
            ),
            "return_home_phase": str(getattr(response, "return_home_phase", "idle")),
            "home_available": bool(getattr(response, "home_available", False)),
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
            "action_active": bool(getattr(response, "action_active", False)),
            "action_waypoint_index": int(getattr(response, "action_waypoint_index", 0)),
            "action_type": str(getattr(response, "action_type", "")),
            "action_remaining_s": float(getattr(response, "action_remaining_s", 0.0)),
            "mission_waypoints": self._route_waypoints_from_state(
                response.mission_lats,
                response.mission_lons,
                response.mission_yaws_deg,
                getattr(response, "mission_action_jsons", []),
                getattr(response, "mission_waypoint_roles", []),
            ),
            "active_chunk_waypoints": self._route_waypoints_from_state(
                response.active_lats, response.active_lons, response.active_yaws_deg
            ),
        }
        if bool(getattr(response, "home_available", False)):
            payload["home_waypoint"] = {
                "lat": float(getattr(response, "home_lat", 0.0)),
                "lon": float(getattr(response, "home_lon", 0.0)),
                "yaw_deg": float(getattr(response, "home_yaw_deg", 0.0)),
                "role": "home",
            }
        with self._lock:
            self._route_mission = payload

    def _update_patrol_state(self, response: GetPatrolMissionState.Response) -> None:
        payload = {
            "active": bool(getattr(response, "active", False)),
            "phase": str(getattr(response, "phase", "idle")),
            "low_battery_active": bool(getattr(response, "low_battery_active", False)),
            "return_home_requested": bool(getattr(response, "return_home_requested", False)),
            "return_home_active": bool(getattr(response, "return_home_active", False)),
            "return_exit_loop_index": int(getattr(response, "return_exit_loop_index", -1)),
            "depart_entry_loop_index": int(getattr(response, "depart_entry_loop_index", -1)),
            "home_available": bool(getattr(response, "home_available", False)),
            "mission_id": str(getattr(response, "mission_id", "")),
            "status": str(getattr(response, "status", "idle")),
            "loop_waypoints": self._route_waypoints_from_state(
                response.loop_lats,
                response.loop_lons,
                response.loop_yaws_deg,
                getattr(response, "loop_action_jsons", []),
            ),
            "return_waypoints": self._route_waypoints_from_state(
                response.return_lats,
                response.return_lons,
                response.return_yaws_deg,
                getattr(response, "return_action_jsons", []),
            ),
            "depart_waypoints": self._route_waypoints_from_state(
                response.depart_lats,
                response.depart_lons,
                response.depart_yaws_deg,
                getattr(response, "depart_action_jsons", []),
            ),
            "active_chunk_waypoints": self._route_waypoints_from_state(
                response.active_lats,
                response.active_lons,
                response.active_yaws_deg,
            ),
        }
        if bool(getattr(response, "home_available", False)):
            payload["home_waypoint"] = {
                "lat": float(getattr(response, "home_lat", 0.0)),
                "lon": float(getattr(response, "home_lon", 0.0)),
                "yaw_deg": float(getattr(response, "home_yaw_deg", 0.0)),
                "role": "home",
            }
        with self._lock:
            self._patrol_mission = payload

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

    def _rtk_sources_cb(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data) or "{}")
        except (ValueError, TypeError):
            return
        sources = payload.get("sources") if isinstance(payload, dict) else None
        if isinstance(sources, list):
            next_sources = [dict(item) for item in sources if isinstance(item, dict)]
            with self._lock:
                self._rtk_sources_list = [dict(item) for item in next_sources]
                current_status = dict(self._rtk_source_status)
            self._broadcast_from_thread(
                {
                    "op": "state",
                    "rtk_sources": next_sources,
                    "rtk_source_state": current_status if current_status else None,
                }
            )

    def _rtk_source_status_cb(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data) or "{}")
        except (ValueError, TypeError):
            return
        if isinstance(payload, dict):
            next_status = dict(payload)
            with self._lock:
                self._rtk_source_status = dict(next_status)
                current_sources = [dict(item) for item in self._rtk_sources_list]
            self._broadcast_from_thread(
                {
                    "op": "state",
                    "rtk_source_state": next_status,
                    "rtk_sources": current_sources,
                }
            )

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
        battery_pct = 0.0
        if self._battery_pct is not None and np.isfinite(float(self._battery_pct)):
            battery_pct = float(self._battery_pct)
        return {
            "connected": True,
            "mode": self._derive_mode(self._goal_active, bool(self._manual_control.get("enabled", False))),
            "battery_pct": battery_pct,
            "battery_voltage_v": (
                float(self._battery_voltage_v)
                if self._battery_voltage_v is not None and np.isfinite(float(self._battery_voltage_v))
                else None
            ),
            "battery_state": str(self._battery_state),
            "battery_mission_state": str(self._battery_mission_state),
            "battery_return_home_recommended": (
                bool(self._battery_return_home_recommended)
                if self._battery_return_home_recommended is not None
                else None
            ),
            "battery_recovered_voltage_v": (
                float(self._battery_recovered_voltage_v)
                if self._battery_recovered_voltage_v is not None
                and np.isfinite(float(self._battery_recovered_voltage_v))
                else None
            ),
            "battery_loaded_voltage_v": (
                float(self._battery_loaded_voltage_v)
                if self._battery_loaded_voltage_v is not None
                and np.isfinite(float(self._battery_loaded_voltage_v))
                else None
            ),
            "battery_present": bool(self._battery_present) if self._battery_present is not None else None,
            "battery_updated_age_s": (
                float(self._battery_updated_age_s)
                if self._battery_updated_age_s is not None and np.isfinite(float(self._battery_updated_age_s))
                else None
            ),
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
        rtk_sources = list(bridge_snapshot.get("rtk_sources") or [])
        # Prefer live data straight from rtk_source_manager (works even when the
        # sensor bridge is disabled, e.g. in simulation).
        with self._lock:
            mgr_status = dict(self._rtk_source_status)
            mgr_sources = [dict(item) for item in self._rtk_sources_list]
        if mgr_status:
            rtk_source_state = mgr_status
        if mgr_sources:
            rtk_sources = mgr_sources
        diagnostics = (
            dict(bridge_snapshot.get("diagnostics"))
            if isinstance(bridge_snapshot.get("diagnostics"), dict)
            else self._fallback_diagnostics_snapshot()
        )

        snapshot = {
            "gps_meta": gps_meta,
            "gps_status": gps_status,
            "rtk_source_state": rtk_source_state,
            "rtk_sources": rtk_sources,
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
            self._last_robot_pose_monotonic = now
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
            self._last_robot_heading_monotonic = time.monotonic()
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

    def _on_battery_state(self, msg: BatteryState) -> None:
        percentage = float(getattr(msg, "percentage", float("nan")))
        if not np.isfinite(percentage):
            return
        if percentage <= 1.0:
            percentage *= 100.0
        percentage = max(0.0, min(100.0, percentage))
        with self._lock:
            if self._battery_use_controller_telemetry:
                return
            self._battery_pct = float(percentage)

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

        nav_telemetry_record = {
            "goal_active": bool(msg.goal_active),
            "auto_mode": str(getattr(msg, "auto_mode", "")),
            "active_action": str(getattr(msg, "active_action", "")),
            "cmd_vel_available": bool(msg.cmd_vel_available),
            "gps_fix_available": bool(getattr(msg, "gps_fix_available", False)),
            "failure_code": str(getattr(msg, "failure_code", "")),
            "nav_result_status": int(getattr(msg, "nav_result_status", 0)),
            "nav_result_text": str(getattr(msg, "nav_result_text", "")),
            "nav_result_event_id": int(getattr(msg, "nav_result_event_id", 0)),
        }
        nav_telemetry_key = json.dumps(nav_telemetry_record, sort_keys=True)
        should_record_nav_telemetry = False
        with self._lock:
            if nav_telemetry_key != self._mission_last_telemetry_key:
                self._mission_last_telemetry_key = nav_telemetry_key
                should_record_nav_telemetry = True
        if should_record_nav_telemetry:
            self._mission_record(
                {"t": time.time(), "topic": "/nav_command_server/telemetry", "data": nav_telemetry_record}
            )

        asyncio.run_coroutine_threadsafe(
            self._broadcast(self._build_nav_telemetry_payload()), self._loop
        )
        if robot_pose_payload is not None:
            asyncio.run_coroutine_threadsafe(
                self._broadcast({"op": "robot_pose", "pose": robot_pose_payload}),
                self._loop,
            )

    _MISSION_START_CODES = frozenset({"GOAL_ACCEPTED"})
    _MISSION_STOP_CODES = frozenset(
        {"GOAL_RESULT_SUCCEEDED", "GOAL_RESULT_ABORTED", "GOAL_CANCELLED", "GOAL_REJECTED"}
    )

    def _on_nav_event(self, msg: NavEvent) -> None:
        payload = self._nav_event_to_payload(msg)
        with self._lock:
            self._recent_nav_events.append(payload)
        code = str(payload.get("code", "")).upper()
        if code in self._MISSION_START_CODES:
            with self._lock:
                already_active = self._mission_active
            if not already_active:
                self._mission_start()
        self._mission_record({"t": time.time(), "topic": "/nav_command_server/events", "data": payload})
        if code in self._MISSION_STOP_CODES:
            self._mission_stop()
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
            payload = {"op": "mission.new_session", "filename": filename, "lines": records}
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
            payload = {
                "available": True,
                "ready": bool(msg.ready),
                "fresh": bool(msg.fresh),
                "drive_enabled": bool(msg.drive_enabled),
                "estop": bool(msg.estop),
                "reverse_requested": bool(msg.reverse_requested),
                "speed_valid": bool(msg.speed_valid),
                "steer_valid": bool(msg.steer_valid),
                "control_source": str(msg.control_source),
                "speed_mps_measured": self._safe_float(msg.speed_mps_measured),
                "steer_deg_measured": self._safe_float(msg.steer_deg_measured),
                "brake_applied_pct": self._safe_int(msg.brake_applied_pct),
            }
            with self._lock:
                self._drive_telemetry = dict(payload)
            self._broadcast_from_thread(
                {"op": "drive_telemetry", "drive_telemetry": payload}
            )

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
                        "estop": payload["estop"],
                        "drive_enabled": payload["drive_enabled"],
                        "speed_mps_measured": payload["speed_mps_measured"],
                        "steer_deg_measured": payload["steer_deg_measured"],
                        "brake_applied_pct": payload["brake_applied_pct"],
                        "ready": payload["ready"],
                        "fresh": payload["fresh"],
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
            battery = payload.get("battery")
            battery = battery if isinstance(battery, dict) else {}
            next_battery_key = None
            should_broadcast_battery = False
            filtered_percentage = battery.get("filtered_percentage", battery.get("percentage"))
            filtered_voltage_v = battery.get("filtered_voltage_v", battery.get("battery_voltage_v"))
            battery_state = battery.get("state", "")
            battery_mission_state = battery.get(
                "mission_guard_state", battery.get("state", "")
            )
            battery_return_home_recommended = battery.get("return_home_recommended")
            battery_recovered_voltage_v = battery.get("recovered_voltage_v")
            battery_loaded_voltage_v = battery.get(
                "loaded_voltage_slow_v", battery.get("filtered_voltage_v")
            )
            battery_present = battery.get("ready", battery.get("present"))
            battery_updated_age_s = battery.get("link_age_s", battery.get("sample_age_s"))
            with self._lock:
                if battery:
                    self._battery_use_controller_telemetry = True
                if filtered_percentage is not None:
                    battery_pct = self._safe_float(filtered_percentage, float("nan"))
                    if np.isfinite(battery_pct):
                        if battery_pct <= 1.0:
                            battery_pct *= 100.0
                        self._battery_pct = max(0.0, min(100.0, float(battery_pct)))
                battery_voltage = self._safe_float(filtered_voltage_v, float("nan"))
                self._battery_voltage_v = float(battery_voltage) if np.isfinite(battery_voltage) else None
                self._battery_state = str(battery_state or "")
                self._battery_mission_state = str(battery_mission_state or "")
                self._battery_return_home_recommended = (
                    bool(battery_return_home_recommended)
                    if battery_return_home_recommended is not None
                    else None
                )
                recovered_voltage = self._safe_float(
                    battery_recovered_voltage_v, float("nan")
                )
                self._battery_recovered_voltage_v = (
                    float(recovered_voltage) if np.isfinite(recovered_voltage) else None
                )
                loaded_voltage = self._safe_float(
                    battery_loaded_voltage_v, float("nan")
                )
                self._battery_loaded_voltage_v = (
                    float(loaded_voltage) if np.isfinite(loaded_voltage) else None
                )
                self._battery_present = (
                    bool(battery_present) if battery_present is not None else None
                )
                battery_age = self._safe_float(battery_updated_age_s, float("nan"))
                self._battery_updated_age_s = (
                    float(battery_age) if np.isfinite(battery_age) else None
                )
                next_battery_key = json.dumps(
                    {
                        "battery_pct": self._battery_pct,
                        "battery_voltage_v": self._battery_voltage_v,
                        "battery_state": self._battery_state,
                        "battery_mission_state": self._battery_mission_state,
                        "battery_return_home_recommended": self._battery_return_home_recommended,
                        "battery_recovered_voltage_v": self._battery_recovered_voltage_v,
                        "battery_loaded_voltage_v": self._battery_loaded_voltage_v,
                        "battery_present": self._battery_present,
                        "battery_updated_age_s": self._battery_updated_age_s,
                    },
                    sort_keys=True,
                )
                if next_battery_key != self._battery_ws_key:
                    self._battery_ws_key = next_battery_key
                    should_broadcast_battery = True
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
            if should_broadcast_battery:
                self._broadcast_from_thread(self._build_nav_telemetry_payload())
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
                    "battery": {
                        "raw_voltage_v": self._safe_float(
                            battery.get("raw_voltage_v", battery.get("battery_voltage_v", 0.0)),
                            0.0,
                        ),
                        "filtered_voltage_v": self._safe_float(
                            battery.get("filtered_voltage_v", battery.get("battery_voltage_v", 0.0)),
                            0.0,
                        ),
                        "raw_percentage": self._safe_float(
                            battery.get("raw_percentage", battery.get("percentage", 0.0)),
                            0.0,
                        ),
                        "filtered_percentage": self._safe_float(
                            battery.get("filtered_percentage", battery.get("percentage", 0.0)),
                            0.0,
                        ),
                        "state": str(battery.get("state", "")),
                        "present": bool(battery.get("ready", False)),
                        "updated_age_s": self._safe_float(
                            battery.get("link_age_s", battery.get("sample_age_s", 0.0)),
                            0.0,
                        ),
                    },
                }
            else:
                data = {"raw": raw}
            if not should_record:
                return
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

    def get_patrol_state(self) -> Tuple[bool, str]:
        req = GetPatrolMissionState.Request()
        res = self._call_service(self._patrol_get_state_client, req, self.request_timeout_s)
        if res is None:
            return False, "patrol get_state timeout"
        if not res.ok:
            return False, str(res.error)
        self._update_patrol_state(res)
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
            ok_patrol, err_patrol = self.get_patrol_state()
            if not ok_patrol and err_patrol:
                self.get_logger().debug(f"patrol state poll skipped: {err_patrol}")
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

    def set_navigation_profile(self, profile: str) -> Tuple[bool, str, str]:
        target = str(profile or "").strip().lower()
        if target not in {"urban", "rural"}:
            return False, "profile must be 'urban' or 'rural'", ""
        req = SetNavigationProfile.Request()
        req.profile = target
        res = self._call_service(
            self._navigation_profile_client,
            req,
            self.request_timeout_s,
        )
        if res is None:
            return False, "set_navigation_profile timeout", ""
        active_profile = str(getattr(res, "active_profile", "") or "")
        return bool(res.ok), str(res.error), active_profile

    def current_coverage_reference(
        self, *, require_heading: bool = True
    ) -> Tuple[Optional[Dict[str, float]], str]:
        now = time.monotonic()
        with self._lock:
            pose = (
                dict(self._last_robot_pose)
                if self._last_robot_pose is not None
                else None
            )
            pose_updated = self._last_robot_pose_monotonic
            heading_updated = self._last_robot_heading_monotonic
        if pose is None:
            return None, "current robot GPS pose is unavailable"
        if require_heading and "heading_deg" not in pose:
            return None, "current robot map heading is unavailable"
        if pose_updated is None or (now - float(pose_updated)) > float(
            self.coverage_reference_max_age_s
        ):
            return None, "current robot GPS pose is stale"
        if require_heading and (
            heading_updated is None or (now - float(heading_updated)) > float(
            self.coverage_reference_max_age_s
            )
        ):
            return None, "current robot map heading is stale"
        reference = {
            "lat": float(pose["lat"]),
            "lon": float(pose["lon"]),
            # Fields2Cover elige la direccion de la primera pasada; su
            # preflight no compara rumbo. Se conserva un valor finito para el
            # payload de auditoria sin inventar que habia una medicion.
            "yaw_deg": float(pose.get("heading_deg", 0.0)),
        }
        if not all(np.isfinite(value) for value in reference.values()):
            return None, "current robot reference contains non-finite values"
        return reference, ""

    def generate_coverage_plan(
        self,
        parameters: Dict[str, Any],
    ) -> Tuple[bool, str, Dict[str, Any]]:
        req = GenerateCoveragePlanLL.Request()
        req.start_lat = float(parameters["start_lat"])
        req.start_lon = float(parameters["start_lon"])
        req.start_yaw_deg = float(parameters["start_yaw_deg"])
        # Con referencia explicita el operador marco la esquina fisica del lote y
        # el proveedor aplica el inset del implemento. Sin referencia la unica
        # informacion es donde esta el vehiculo, y ahi la esquina es la lectura
        # equivocada: el inset deja la primera pasada media pasada adelante y
        # media pasada al costado, un corrimiento puramente lateral que con radio
        # minimo no se puede tomar de frente y obliga a un rulo de aproximacion
        # antes de empezar a trabajar. Tomando la pose como centro de la primera
        # pasada el vehiculo arranca ya alineado.
        req.start_is_field_corner = bool(
            parameters.get("start_is_field_corner", True)
        )
        req.field_length_m = float(parameters["field_length_m"])
        req.field_width_m = float(parameters["field_width_m"])
        # Lote como poligono. Vacio = modo rectangulo legacy; la validacion fina
        # la hace el route_executor, aca solo se traduce.
        req.coverage_polygon = _geo_ring(parameters.get("coverage_polygon"))
        req.coverage_exclusions = [
            _geo_ring(ring) for ring in parameters.get("coverage_exclusions", [])
        ]
        req.cutter_width_m = float(parameters["cutter_width_m"])
        req.overlap_ratio = float(parameters["overlap_ratio"])
        req.min_turning_radius_m = float(parameters["min_turning_radius_m"])
        req.waypoint_spacing_m = float(parameters["waypoint_spacing_m"])
        req.side = str(parameters["side"])

        res = self._call_service(
            self._coverage_plan_client,
            req,
            self.coverage_plan_timeout_s,
        )
        if res is None:
            return False, "generate_coverage_plan_ll timeout", {}
        if not bool(res.ok):
            return False, str(res.error), {}

        sampled_lengths = {
            len(res.sampled_lats),
            len(res.sampled_lons),
            len(res.sampled_yaws_deg),
            len(res.sampled_phases),
            len(res.sampled_row_indices),
            len(res.sampled_key_flags),
        }
        if len(sampled_lengths) != 1:
            return False, "generate_coverage_plan_ll returned inconsistent sampled arrays", {}
        key_lengths = {
            len(res.key_lats),
            len(res.key_lons),
            len(res.key_yaws_deg),
        }
        if len(key_lengths) != 1:
            return False, "generate_coverage_plan_ll returned inconsistent key arrays", {}
        route_lengths = {
            len(res.route_lats),
            len(res.route_lons),
            len(res.route_yaws_deg),
            len(res.route_key_flags),
        }
        if len(route_lengths) != 1:
            return False, "generate_coverage_plan_ll returned inconsistent route arrays", {}

        sampled_key_values = [
            (float(lat), float(lon), float(yaw_deg))
            for lat, lon, yaw_deg, is_key in zip(
                res.sampled_lats,
                res.sampled_lons,
                res.sampled_yaws_deg,
                res.sampled_key_flags,
            )
            if bool(is_key)
        ]
        response_key_values = [
            (float(lat), float(lon), float(yaw_deg))
            for lat, lon, yaw_deg in zip(
                res.key_lats,
                res.key_lons,
                res.key_yaws_deg,
            )
        ]
        if len(sampled_key_values) != len(response_key_values) or any(
            not all(
                math.isclose(sampled_value, key_value, rel_tol=0.0, abs_tol=1.0e-12)
                for sampled_value, key_value in zip(sampled_item, key_item)
            )
            for sampled_item, key_item in zip(
                sampled_key_values,
                response_key_values,
            )
        ):
            return (
                False,
                "generate_coverage_plan_ll key arrays do not match sampled key flags",
                {},
            )
        if not response_key_values:
            return False, "generate_coverage_plan_ll returned no key waypoints", {}
        # "Dos metas por pasada" y "una guia por cabecera" describen el zigzag
        # nominal. Con zonas no-go el trazado se aparta de esa forma a proposito:
        # los rodeos agregan metas de parada y una zona que tapa la punta de una
        # pasada le borra el extremo. No hay formula que lo prediga —depende de
        # la forma de la zona—, asi que los invariantes de forma se exigen solo
        # cuando no hubo recorte. El resto de las validaciones sigue corriendo
        # siempre.
        # Los invariantes de forma que siguen describen el zigzag del
        # planificador propio. `topology_audited=false` significa que el plan no
        # salio de ese planificador —hoy, Fields2Cover— y ahi no aplican: su
        # trayectoria no tiene guias de cabecera ni dos metas por pasada.
        # La interfaz actual siempre expone este campo; el fallback conserva
        # compatibilidad con un route_executor anterior durante un despliegue
        # escalonado y hace que sus invariantes se validen de forma estricta.
        legacy_shape = bool(getattr(res, "topology_audited", True))
        nogo_applied = int(getattr(res, "nogo_polygon_count", 0)) > 0 or not legacy_shape
        expected_key_count = 2 * int(res.row_count)
        if not nogo_applied and len(response_key_values) != expected_key_count:
            return (
                False,
                "generate_coverage_plan_ll key waypoint count does not match "
                "two endpoints per row",
                {},
            )
        if not all(
            np.isfinite(value)
            for waypoint in response_key_values
            for value in waypoint
        ):
            return False, "generate_coverage_plan_ll returned non-finite key values", {}
        # Las acciones por waypoint son opcionales: un backend viejo no manda el
        # arreglo. Se rellena con vacio para que el zip no recorte la ruta.
        response_route_actions = list(getattr(res, "route_action_jsons", []) or [])
        if len(response_route_actions) != len(res.route_lats):
            response_route_actions = ["" for _ in res.route_lats]
        response_route_values = [
            (float(lat), float(lon), float(yaw_deg), bool(is_key), str(action or ""))
            for lat, lon, yaw_deg, is_key, action in zip(
                res.route_lats,
                res.route_lons,
                res.route_yaws_deg,
                res.route_key_flags,
                response_route_actions,
            )
        ]
        if not response_route_values:
            return False, "generate_coverage_plan_ll returned no route waypoints", {}
        if not response_route_values[0][3] or not response_route_values[-1][3]:
            return False, "generate_coverage_plan_ll route must start and end at key waypoints", {}
        route_key_values = [item[:3] for item in response_route_values if item[3]]
        if len(route_key_values) != len(response_key_values) or any(
            not all(
                math.isclose(route_value, key_value, rel_tol=0.0, abs_tol=1.0e-12)
                for route_value, key_value in zip(route_item, key_item)
            )
            for route_item, key_item in zip(route_key_values, response_key_values)
        ):
            return False, "generate_coverage_plan_ll route keys do not match key arrays", {}
        expected_route_count = expected_key_count + max(0, int(res.row_count) - 1)
        if not nogo_applied and len(response_route_values) != expected_route_count:
            return (
                False,
                "generate_coverage_plan_ll route must contain one guide per turn",
                {},
            )
        if not all(
            np.isfinite(value)
            for waypoint in response_route_values
            for value in waypoint[:3]
        ):
            return False, "generate_coverage_plan_ll returned non-finite route values", {}

        strict_crossing_count = int(res.strict_crossing_count)
        nonadjacent_touch_count = int(res.nonadjacent_touch_count)
        collinear_overlap_count = int(res.collinear_overlap_count)
        derived_conflict_count = (
            strict_crossing_count
            + nonadjacent_touch_count
            + collinear_overlap_count
        )
        if int(res.topology_conflict_count) != derived_conflict_count:
            return (
                False,
                "generate_coverage_plan_ll topology conflict count is inconsistent",
                {},
            )
        field_strict_crossing_count = int(res.field_strict_crossing_count)
        field_nonadjacent_touch_count = int(
            res.field_nonadjacent_touch_count
        )
        field_collinear_overlap_count = int(
            res.field_collinear_overlap_count
        )
        derived_field_conflict_count = (
            field_strict_crossing_count
            + field_nonadjacent_touch_count
            + field_collinear_overlap_count
        )
        if int(res.field_topology_conflict_count) != derived_field_conflict_count:
            return (
                False,
                "generate_coverage_plan_ll field topology conflict count is "
                "inconsistent",
                {},
            )
        topology_scope = str(res.topology_scope or "").strip().lower()
        if topology_scope not in {"global", "field_interior", "fields2cover"}:
            return (
                False,
                "generate_coverage_plan_ll returned an invalid topology_scope",
                {},
            )
        derived_topology_safe = bool(
            derived_field_conflict_count == 0
            if topology_scope == "field_interior"
            else derived_conflict_count == 0 and int(res.omega_turn_count) == 0
        )
        # Sin auditoria no hay nada con que contrastar topology_safe: los
        # contadores estan en cero porque no se midieron.
        if legacy_shape and bool(res.topology_safe) != derived_topology_safe:
            return (
                False,
                "generate_coverage_plan_ll topology_safe invariant is inconsistent",
                {},
            )

        sampled_waypoints = [
            {
                "lat": float(lat),
                "lon": float(lon),
                "yaw_deg": float(yaw_deg),
                "phase": str(phase),
                "row_index": int(row_index),
                "key": bool(is_key),
            }
            for lat, lon, yaw_deg, phase, row_index, is_key in zip(
                res.sampled_lats,
                res.sampled_lons,
                res.sampled_yaws_deg,
                res.sampled_phases,
                res.sampled_row_indices,
                res.sampled_key_flags,
            )
        ]
        key_waypoints = [
            {
                "lat": float(lat),
                "lon": float(lon),
                "yaw_deg": float(yaw_deg),
            }
            for lat, lon, yaw_deg in zip(
                res.key_lats,
                res.key_lons,
                res.key_yaws_deg,
            )
        ]
        route_waypoints = [
            {
                "lat": lat,
                "lon": lon,
                "yaw_deg": yaw_deg,
                "key": is_key,
                "guide": not is_key,
                # La marcha atras de la cabecera de tres puntos viaja pegada al
                # waypoint donde hay que hacerla. Vacio en todos los demas.
                "action_json": action_json,
            }
            for lat, lon, yaw_deg, is_key, action_json in response_route_values
        ]
        global_topology_conflicts = {
            "strict_crossings": strict_crossing_count,
            "nonadjacent_touches": nonadjacent_touch_count,
            "collinear_overlaps": collinear_overlap_count,
            "total": derived_conflict_count,
        }
        field_topology_conflicts = {
            "strict_crossings": field_strict_crossing_count,
            "nonadjacent_touches": field_nonadjacent_touch_count,
            "collinear_overlaps": field_collinear_overlap_count,
            "total": derived_field_conflict_count,
            "scope": "field_interior",
        }
        topology_conflicts = (
            field_topology_conflicts
            if topology_scope == "field_interior"
            else {**global_topology_conflicts, "scope": "global"}
        )
        warnings: List[Dict[str, Any]] = []
        if int(res.omega_turn_count) > 0:
            warnings.append(
                {
                    "code": "COVERAGE_OMEGA_TURNS",
                    "count": int(res.omega_turn_count),
                }
            )
        if derived_conflict_count > 0:
            warnings.append(
                {
                    "code": "COVERAGE_HEADLAND_TOPOLOGY_CONFLICTS",
                    **global_topology_conflicts,
                }
            )

        headland_guidance_enabled = bool(
            getattr(self, "coverage_use_headland_guides", False)
        )
        execution_waypoints = (
            route_waypoints
            if headland_guidance_enabled
            else [
                {
                    **waypoint,
                    "key": True,
                    "guide": False,
                }
                for waypoint in key_waypoints
            ]
        )
        # El ejecutor conserva la semántica habitual de rutas, PATROL y goals
        # individuales. CAMPO distingue los extremos de pasada, que se siguen
        # con precisión, de las guías de cabecera. Estas últimas son tránsito:
        # Nav2 puede recuperar y corregir afuera del lote sin deformar el
        # trazado que importa dentro.
        execution_waypoints = [
            {
                **waypoint,
                "role": "coverage" if bool(waypoint.get("key", True)) else "coverage_transit",
            }
            for waypoint in execution_waypoints
        ]
        payload = {
            "reference": {
                "lat": float(req.start_lat),
                "lon": float(req.start_lon),
                "yaw_deg": float(req.start_yaw_deg),
            },
            "route_start_reference": {
                "lat": float(res.route_start_lat),
                "lon": float(res.route_start_lon),
                "yaw_deg": float(res.route_start_yaw_deg),
            },
            "parameters": {
                "field_length_m": float(req.field_length_m),
                "field_width_m": float(req.field_width_m),
                "cutter_width_m": float(req.cutter_width_m),
                "overlap_ratio": float(req.overlap_ratio),
                "min_turning_radius_m": float(req.min_turning_radius_m),
                "waypoint_spacing_m": float(req.waypoint_spacing_m),
                "side": str(req.side),
                "reference_mode": (
                    "field_corner" if req.start_is_field_corner else "first_row_start"
                ),
                "centerline_length_m": float(res.centerline_length_m),
            },
            "sampled_waypoints": sampled_waypoints,
            "key_waypoints": key_waypoints,
            "route_waypoints": route_waypoints,
            "headland_guidance_enabled": headland_guidance_enabled,
            "metrics": {
                "row_count": int(res.row_count),
                "lane_spacing_m": float(res.lane_spacing_m),
                "row_visit_order": [int(value) for value in res.row_visit_order],
                "turn_separations_m": [
                    float(value) for value in res.turn_separations_m
                ],
                "clean_uturn_count": int(res.clean_uturn_count),
                "omega_turn_count": int(res.omega_turn_count),
                "estimated_path_length_m": float(res.estimated_path_length_m),
                "headland_before_m": float(res.headland_before_m),
                "headland_after_m": float(res.headland_after_m),
                "lateral_overflow_m": float(res.lateral_overflow_m),
                "topology_conflicts": topology_conflicts,
                "field_topology_conflicts": field_topology_conflicts,
                "global_topology_conflicts": global_topology_conflicts,
                "topology_safe": bool(res.topology_safe),
                "topology_scope": topology_scope,
                "topology_audited": legacy_shape,
                "topology_audit_spacing_m": float(
                    res.topology_audit_spacing_m
                ),
                "planner_min_turning_radius_m": float(
                    res.planner_min_turning_radius_m
                ),
            },
            "topology_safe": bool(res.topology_safe),
            # Efecto de las zonas no-go. El cockpit repite el recorte por su
            # cuenta y compara contra esto: si el ve zonas y aca viene un cero,
            # sabe que la ruta que se va a ejecutar no es la que esta dibujando
            # y bloquea el arranque. Por eso los campos tienen que viajar aunque
            # no haya zonas.
            # Con que definicion de lote planifico el backend, y si corrio la
            # auditoria topologica. El cockpit los necesita para no leer
            # topology_safe como si lo hubiera verificado el verificador legacy.
            "field_mode": str(getattr(res, "field_mode", "rectangle") or "rectangle"),
            "topology_audited": legacy_shape,
            "nogo_polygon_count": int(getattr(res, "nogo_polygon_count", 0)),
            "nogo_dropped_count": int(getattr(res, "nogo_dropped_count", 0)),
            "nogo_detour_count": int(getattr(res, "nogo_detour_count", 0)),
            "nogo_note": str(getattr(res, "nogo_note", "") or ""),
            "warnings": warnings,
            "route_request": {
                "op": "set_route_ll",
                "waypoints": execution_waypoints,
                "loop": False,
                "leg_spacing_m": float(res.recommended_leg_spacing_m),
                "chunk_span_m": float(res.recommended_chunk_span_m),
                "chunk_max_waypoints": int(res.recommended_chunk_max_waypoints),
            },
        }
        return True, "", payload

    def start_coverage(
        self,
        parameters: Dict[str, Any],
    ) -> Tuple[bool, str, Dict[str, Any]]:
        ok, error, coverage_plan = self.generate_coverage_plan(parameters)
        result: Dict[str, Any] = {
            "route_started": False,
            "route_submission_state": "not_started",
        }
        if not ok:
            return False, error, result

        result["coverage_plan"] = coverage_plan
        topology_conflicts = coverage_plan["metrics"]["topology_conflicts"]
        if not bool(coverage_plan["topology_safe"]):
            topology_scope = str(
                coverage_plan["metrics"].get("topology_scope", "global")
            )
            scope_text = (
                "inside the field"
                if topology_scope == "field_interior"
                else "in the complete nominal path"
            )
            return (
                False,
                f"coverage has topology conflicts {scope_text} "
                f"(strict_crossings={int(topology_conflicts['strict_crossings'])}, "
                f"nonadjacent_touches={int(topology_conflicts['nonadjacent_touches'])}, "
                f"collinear_overlaps={int(topology_conflicts['collinear_overlaps'])})",
                result,
            )

        # El planner propio parte alineado con el vehiculo y necesita rumbo.
        # Fields2Cover selecciona la primera pasada por geometria, y mas abajo
        # ya excluimos el chequeo de rumbo cuando topology_audited es falso.
        # Exigirlo aca bloqueaba CAMPO estacionario aunque no se usara.
        exige_rumbo = bool(coverage_plan.get("topology_audited", True))
        current_reference, reference_error = self.current_coverage_reference(
            require_heading=exige_rumbo
        )
        if current_reference is None:
            return False, f"coverage approach rejected: {reference_error}", result

        first_key = coverage_plan["key_waypoints"][0]
        meters_per_deg_lat = 111_320.0
        average_lat_rad = math.radians(
            0.5 * (float(current_reference["lat"]) + float(first_key["lat"]))
        )
        meters_per_deg_lon = meters_per_deg_lat * max(
            1.0e-6,
            abs(math.cos(average_lat_rad)),
        )
        north_m = (
            float(first_key["lat"]) - float(current_reference["lat"])
        ) * meters_per_deg_lat
        east_m = (
            float(first_key["lon"]) - float(current_reference["lon"])
        ) * meters_per_deg_lon
        distance_m = float(math.hypot(north_m, east_m))
        heading_error_deg = abs(
            self._normalize_yaw_deg(
                float(current_reference["yaw_deg"]) - float(first_key["yaw_deg"])
            )
        )
        # El criterio de rumbo describe el trazado del planificador propio: ahi
        # la primera pasada arranca bajo el vehiculo y alineada con el, asi que
        # un rumbo distinto es senal de que algo no cierra. Fields2Cover elige
        # el swath inicial y su sentido segun la forma del lote, y el vehiculo
        # puede estar parado en cualquier lado: un rumbo distinto es lo normal,
        # no una falla. Nav2 lo lleva hasta la primera meta como con cualquier
        # otra. El criterio de distancia se mantiene en los dos casos, que es lo
        # que evita arrancar la cobertura de un lote que esta lejos.
        approach = {
            "checks_heading": exige_rumbo,
            "distance_m": distance_m,
            "max_distance_m": float(self.coverage_start_max_distance_m),
            "heading_error_deg": float(heading_error_deg),
            "max_heading_error_deg": float(
                self.coverage_start_max_heading_error_deg
            ),
            "robot_reference": dict(current_reference),
            "first_key_waypoint": dict(first_key),
        }
        result["approach"] = approach
        rumbo_fuera = exige_rumbo and heading_error_deg > float(
            self.coverage_start_max_heading_error_deg
        )
        if distance_m > float(self.coverage_start_max_distance_m) or rumbo_fuera:
            detalle_rumbo = (
                f"; heading_error={heading_error_deg:.1f} deg, "
                f"limit={float(self.coverage_start_max_heading_error_deg):.1f} deg"
                if exige_rumbo
                else " (sin criterio de rumbo: el planificador elige la pasada inicial)"
            )
            return (
                False,
                "coverage approach rejected "
                f"(distance={distance_m:.2f} m, "
                f"limit={float(self.coverage_start_max_distance_m):.2f} m"
                f"{detalle_rumbo})",
                result,
            )

        route_request = coverage_plan["route_request"]
        route_ok, route_error, input_count, expanded_count = self.set_route_mission(
            list(route_request["waypoints"]),
            False,
            float(route_request["leg_spacing_m"]),
            float(route_request["chunk_span_m"]),
            int(route_request["chunk_max_waypoints"]),
            # La cobertura se recorre completa desde su primera meta. Sin esto, un
            # lote corrido respecto del vehiculo deja lazos de cabecera a pocos
            # metros del robot y route_executor engancha la ruta por el medio,
            # descartando las primeras pasadas.
            start_from_first_waypoint=True,
        )
        result["input_waypoint_count"] = int(input_count)
        result["expanded_waypoint_count"] = int(expanded_count)
        result["input_key_waypoint_count"] = int(
            len(coverage_plan["key_waypoints"])
        )
        result["guide_waypoint_count"] = int(
            sum(
                bool(waypoint.get("guide"))
                for waypoint in route_request["waypoints"]
            )
        )
        if route_ok:
            result["route_started"] = True
            result["route_submission_state"] = "started"
            return True, "", result
        if str(route_error) == "set_route_ll timeout":
            result["route_started"] = None
            result["route_submission_state"] = "unknown_timeout"
        return False, str(route_error), result

    def set_route_mission(
        self,
        waypoints: List[Dict[str, Any]],
        loop: bool,
        leg_spacing_m: Optional[float] = None,
        chunk_span_m: Optional[float] = None,
        chunk_max_waypoints: Optional[int] = None,
        start_from_first_waypoint: bool = False,
    ) -> Tuple[bool, str, int, int]:
        if len(waypoints) == 0:
            return False, "at least one waypoint is required", 0, 0

        self.get_logger().info(
            "WS->ROS set_route_mission "
            f"(count={len(waypoints)}, loop={bool(loop)}, "
            f"leg_spacing_m={leg_spacing_m}, chunk_span_m={chunk_span_m}, "
            f"chunk_max_waypoints={chunk_max_waypoints}, "
            f"start_from_first_waypoint={bool(start_from_first_waypoint)})"
        )
        req = SetRouteMissionLL.Request()
        resolved_yaws_deg = self._resolve_waypoint_yaws(waypoints, loop)
        req.lats = [float(wp["lat"]) for wp in waypoints]
        req.lons = [float(wp["lon"]) for wp in waypoints]
        req.yaws_deg = [float(yaw_deg) for yaw_deg in resolved_yaws_deg]
        req.waypoint_action_jsons = [
            # `action_json` ya viene serializado por el backend (la marcha atras
            # de la cabecera de Campo). `actions` es el formato del editor de
            # waypoints. El primero gana porque es el que calculo el planner.
            str(wp.get("action_json") or "")
            or (
                json.dumps(wp.get("actions", []), separators=(",", ":"), sort_keys=True)
                if wp.get("actions")
                else ""
            )
            for wp in waypoints
        ]
        req.waypoint_roles = [
            str(wp.get("role", "normal") or "normal").strip().lower() for wp in waypoints
        ]
        req.waypoint_key_flags = [
            bool(wp.get("key", True)) for wp in waypoints
        ]
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
        req.start_from_first_waypoint = bool(start_from_first_waypoint)

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

    def set_patrol_mission(
        self,
        payload: Dict[str, Any],
        leg_spacing_m: Optional[float] = None,
        chunk_span_m: Optional[float] = None,
        chunk_max_waypoints: Optional[int] = None,
    ) -> Tuple[bool, str, int, int]:
        loop_waypoints = list(payload.get("loop_waypoints", []))
        home_waypoint = payload.get("home_waypoint")
        if not loop_waypoints:
            return False, "loop_waypoints must be a non-empty list", 0, 0
        if not isinstance(home_waypoint, dict):
            return False, "home_waypoint is required", 0, 0

        req = SetPatrolMissionLL.Request()
        loop_resolved_yaws = self._resolve_waypoint_yaws(loop_waypoints, True)
        req.loop_lats = [float(wp["lat"]) for wp in loop_waypoints]
        req.loop_lons = [float(wp["lon"]) for wp in loop_waypoints]
        req.loop_yaws_deg = [float(yaw_deg) for yaw_deg in loop_resolved_yaws]
        req.loop_waypoint_action_jsons = [
            json.dumps(wp.get("actions", []), separators=(",", ":"), sort_keys=True)
            if wp.get("actions")
            else ""
            for wp in loop_waypoints
        ]
        req.home_lat = float(home_waypoint["lat"])
        req.home_lon = float(home_waypoint["lon"])
        req.home_yaw_deg = float(
            home_waypoint.get(
                "yaw_deg",
                self._resolve_waypoint_yaws([home_waypoint], False)[0],
            )
        )

        for prefix, waypoints in (
            ("return", list(payload.get("return_waypoints", []))),
            ("depart", list(payload.get("depart_waypoints", []))),
        ):
            resolved_yaws = self._resolve_waypoint_yaws(waypoints, False) if waypoints else []
            setattr(req, f"{prefix}_lats", [float(wp["lat"]) for wp in waypoints])
            setattr(req, f"{prefix}_lons", [float(wp["lon"]) for wp in waypoints])
            setattr(req, f"{prefix}_yaws_deg", [float(yaw_deg) for yaw_deg in resolved_yaws])
            setattr(
                req,
                f"{prefix}_waypoint_action_jsons",
                [
                    json.dumps(wp.get("actions", []), separators=(",", ":"), sort_keys=True)
                    if wp.get("actions")
                    else ""
                    for wp in waypoints
                ],
            )

        req.depart_entry_loop_index = int(payload.get("depart_entry_loop_index", -1))
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

        res = self._call_service(self._patrol_set_client, req, self.set_goal_timeout_s)
        if res is None:
            return False, "set_patrol_ll timeout", 0, 0
        if bool(res.ok):
            self.get_patrol_state()
            self.get_route_state()
        return (
            bool(res.ok),
            str(res.error),
            int(getattr(res, "loop_input_waypoint_count", len(loop_waypoints))),
            int(getattr(res, "loop_expanded_waypoint_count", 0)),
        )

    def cancel_patrol_mission(self) -> Tuple[bool, str]:
        req = CancelPatrolMission.Request()
        res = self._call_service(self._patrol_cancel_client, req, self.request_timeout_s)
        if res is None:
            return False, "cancel_patrol timeout"
        if bool(res.ok):
            self.get_patrol_state()
            self.get_route_state()
        return bool(res.ok), str(res.error)

    def request_patrol_return_home(self) -> Tuple[bool, str]:
        req = RequestReturnHome.Request()
        res = self._call_service(self._patrol_return_home_client, req, self.request_timeout_s)
        if res is None:
            return False, "request_return_home timeout"
        if bool(res.ok):
            self.get_patrol_state()
            self.get_route_state()
        return bool(res.ok), str(res.error)

    def save_waypoints_file(
        self,
        waypoints: List[Dict[str, float]],
        patrol_profile: Optional[Dict[str, Any]] = None,
    ) -> Tuple[bool, str, int]:
        ok, err, count = save_waypoints_yaml_file(
            self.waypoints_file,
            waypoints,
            patrol_profile,
        )
        if not ok:
            self.get_logger().warning(f"save_waypoints_file failed: {err}")
            return False, err, 0
        self.get_logger().info(f"save_waypoints_file ok (count={count})")
        return True, "", int(count)

    def load_waypoints_file(
        self,
    ) -> Tuple[bool, str, List[Dict[str, float]], Optional[Dict[str, Any]]]:
        ok, err, waypoints, patrol_profile = load_waypoints_yaml_file(self.waypoints_file)
        if not ok:
            self.get_logger().warning(f"load_waypoints_file failed: {err}")
            return False, err, [], None
        self.get_logger().info(f"load_waypoints_file ok (count={len(waypoints)})")
        return True, "", waypoints, patrol_profile

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

    def select_rtk_source(self, source_id: str) -> Tuple[bool, str]:
        source_id = str(source_id or "").strip()
        if not source_id:
            return False, "source id is required"
        with self._lock:
            known_sources = [dict(item) for item in self._rtk_sources_list]
            current_status = dict(self._rtk_source_status)
        if known_sources and not any(str(item.get("id") or "").strip() == source_id for item in known_sources):
            return False, f"unknown RTK source: {source_id}"
        selected_source = next(
            (item for item in known_sources if str(item.get("id") or "").strip() == source_id),
            None,
        )
        optimistic_status = {
            **current_status,
            "active_source_id": source_id,
            "active_source_label": str(
                (selected_source or {}).get("label")
                or (selected_source or {}).get("name")
                or current_status.get("active_source_label")
                or source_id
            ),
            "connected": False,
            "last_error": "",
            "rtcm_age_s": None,
        }
        with self._lock:
            self._rtk_source_status = dict(optimistic_status)
        try:
            self._rtk_source_select_pub.publish(String(data=source_id))
        except Exception as exc:  # pragma: no cover - defensive
            return False, f"failed to publish rtk source select: {exc}"
        self._broadcast_from_thread(
            {
                "op": "state",
                "rtk_source_state": optimistic_status,
                "rtk_sources": known_sources,
            }
        )
        return True, ""

    def upsert_rtk_source(self, payload: Dict[str, Any]) -> Tuple[bool, str]:
        if not isinstance(payload, dict):
            return False, "payload must be an object"
        source_id = str(payload.get("id") or "").strip()
        host = str(payload.get("host") or "").strip()
        mountpoint = str(payload.get("mountpoint") or "").strip()
        if not source_id:
            return False, "source id is required"
        if not host:
            return False, "host is required"
        if not mountpoint:
            return False, "mountpoint is required"
        try:
            port = int(payload.get("port") or 2101)
        except (TypeError, ValueError):
            return False, "port must be a number"
        if port <= 0 or port > 65535:
            return False, "port must be between 1 and 65535"

        message = {
            "action": "upsert",
            "id": source_id,
            "label": str(payload.get("label") or source_id).strip() or source_id,
            "host": host,
            "port": port,
            "mountpoint": mountpoint,
            "username": str(payload.get("username") or "").strip(),
            "password": str(payload.get("password") or "").strip(),
            "activate": bool(payload.get("activate", True)),
        }
        try:
            self._rtk_source_manage_pub.publish(String(data=json.dumps(message)))
        except Exception as exc:  # pragma: no cover - defensive
            return False, f"failed to publish rtk source management: {exc}"
        return True, ""

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
            self.get_camera_ptz_state()
        else:
            with self._lock:
                self._camera_status["ok"] = False
                self._camera_status["error"] = str(res.error)
        return bool(res.ok), str(res.error), applied

    def _store_camera_status_payload(self, payload: Dict[str, Any]) -> None:
        with self._lock:
            self._camera_status = {
                "ok": bool(payload.get("ok", False)),
                "error": str(payload.get("error", "")),
                "last_command": str(payload.get("last_command", "none")),
                "zoom_in": bool(payload.get("zoom_in", False)),
                "pan_deg": float(payload.get("pan_deg", 0.0)),
                "tilt_deg": float(payload.get("tilt_deg", 0.0)),
                "zoom_level": float(payload.get("zoom_level", 0.0)),
                "active_preset": str(payload.get("active_preset", "")),
            }

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
            self.get_camera_ptz_state()
        else:
            with self._lock:
                self._camera_status["ok"] = False
                self._camera_status["error"] = str(res.message)
        return bool(res.success), str(res.message)

    def camera_ptz_move(
        self,
        *,
        relative: bool,
        pan_deg: Optional[float] = None,
        tilt_deg: Optional[float] = None,
        zoom_level: Optional[float] = None,
    ) -> Tuple[bool, str, Dict[str, Any]]:
        req = CameraPtz.Request()
        req.relative = bool(relative)
        req.apply_pan = pan_deg is not None
        req.pan_deg = float(pan_deg if pan_deg is not None else 0.0)
        req.apply_tilt = tilt_deg is not None
        req.tilt_deg = float(tilt_deg if tilt_deg is not None else 0.0)
        req.apply_zoom = zoom_level is not None
        req.zoom_level = float(zoom_level if zoom_level is not None else 0.0)
        res = self._call_service(self._camera_ptz_client, req, self.request_timeout_s)
        if res is None:
            payload = {
                "op": "camera_ptz_state",
                "ok": False,
                "error": "camera_ptz timeout",
                "pan_deg": 0.0,
                "tilt_deg": 0.0,
                "zoom_level": 0.0,
            }
            return False, payload["error"], payload

        payload = {
            "op": "camera_ptz_state",
            "ok": bool(res.ok),
            "error": str(res.error),
            "pan_deg": float(res.pan_deg),
            "tilt_deg": float(res.tilt_deg),
            "zoom_level": float(res.zoom_level),
        }
        if res.ok:
            _, _, state_payload = self.get_camera_ptz_state()
            return True, "", state_payload
        return False, str(res.error), payload

    def camera_preset(self, preset: str) -> Tuple[bool, str, Dict[str, Any]]:
        req = CameraPreset.Request()
        req.preset = str(preset)
        res = self._call_service(self._camera_preset_client, req, self.request_timeout_s)
        if res is None:
            payload = {
                "op": "camera_ptz_state",
                "ok": False,
                "error": "camera_preset timeout",
                "applied_preset": "",
                "pan_deg": 0.0,
                "tilt_deg": 0.0,
                "zoom_level": 0.0,
            }
            return False, payload["error"], payload

        payload = {
            "op": "camera_ptz_state",
            "ok": bool(res.ok),
            "error": str(res.error),
            "applied_preset": str(res.applied_preset),
            "pan_deg": float(res.pan_deg),
            "tilt_deg": float(res.tilt_deg),
            "zoom_level": float(res.zoom_level),
        }
        if res.ok:
            _, _, state_payload = self.get_camera_ptz_state()
            return True, "", state_payload
        return False, str(res.error), payload

    def camera_save_preset(
        self, preset: str, *, save_zoom: bool
    ) -> Tuple[bool, str, Dict[str, Any]]:
        req = CameraSavePreset.Request()
        req.preset = str(preset)
        req.save_zoom = bool(save_zoom)
        res = self._call_service(
            self._camera_save_preset_client, req, self.request_timeout_s
        )
        if res is None:
            payload = {
                "op": "camera_ptz_state",
                "ok": False,
                "error": "camera_save_preset timeout",
                "saved_preset": "",
                "pan_deg": 0.0,
                "tilt_deg": 0.0,
                "zoom_level": 0.0,
            }
            return False, payload["error"], payload

        payload = {
            "op": "camera_ptz_state",
            "ok": bool(res.ok),
            "error": str(res.error),
            "saved_preset": str(res.saved_preset),
            "pan_deg": float(res.pan_deg),
            "tilt_deg": float(res.tilt_deg),
            "zoom_level": float(res.zoom_level),
        }
        if res.ok:
            _, _, state_payload = self.get_camera_ptz_state()
            state_payload = dict(state_payload)
            state_payload["saved_preset"] = str(res.saved_preset)
            state_payload["saved_zoom"] = bool(save_zoom)
            return True, "", state_payload
        return False, str(res.error), payload

    def get_camera_ptz_state(self) -> Tuple[bool, str, Dict[str, Any]]:
        req = CameraPtzState.Request()
        res = self._call_service(
            self._camera_ptz_state_client, req, self.request_timeout_s
        )
        if res is None:
            payload = {
                "op": "camera_ptz_state",
                "ok": False,
                "error": "camera_ptz_state timeout",
                "last_command": "",
                "zoom_in": False,
                "pan_deg": 0.0,
                "tilt_deg": 0.0,
                "zoom_level": 0.0,
                "active_preset": "",
            }
            return False, payload["error"], payload

        payload = {
            "op": "camera_ptz_state",
            "ok": bool(res.ok),
            "error": str(res.error),
            "last_command": str(res.last_command),
            "zoom_in": bool(res.zoom_in),
            "pan_deg": float(res.pan_deg),
            "tilt_deg": float(res.tilt_deg),
            "zoom_level": float(res.zoom_level),
            "active_preset": str(res.active_preset),
        }
        self._store_camera_status_payload(payload)
        return bool(res.ok), str(res.error), payload

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
                "pan_deg": 0.0,
                "tilt_deg": 0.0,
                "zoom_level": 0.0,
                "active_preset": "",
            }
            return False, payload["error"], payload

        payload = {
            "op": "camera_status",
            "ok": bool(res.ok),
            "error": str(res.error),
            "last_command": str(res.last_command),
            "zoom_in": bool(res.zoom_in),
            "pan_deg": float(res.pan_deg),
            "tilt_deg": float(res.tilt_deg),
            "zoom_level": float(res.zoom_level),
            "active_preset": str(res.active_preset),
        }
        self._store_camera_status_payload(payload)
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
        ok_p, err_p = self.get_patrol_state()
        if not ok_p and err_p:
            self.get_logger().warning(f"patrol bootstrap failed: {err_p}")
        ok_c, err_c, _ = self.get_camera_ptz_state()
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
            ok_patrol, err_patrol = await asyncio.to_thread(self.node.get_patrol_state)
            if not ok_patrol and err_patrol:
                self.node.get_logger().warning(
                    f"patrol refresh on WS connect failed: {err_patrol}"
                )
            await self.node._broadcast(self.node.snapshot_state())
        except Exception as exc:
            self.node.get_logger().warning(f"nav refresh on WS connect crashed: {exc}")

    def _parse_coverage_parameters(
        self,
        msg: Dict[str, Any],
    ) -> Tuple[Optional[Dict[str, Any]], str]:
        reference_raw = msg.get("reference")
        if reference_raw is not None and not isinstance(reference_raw, dict):
            return None, "reference must be an object"

        explicit_values = {
            "start_lat": msg.get("start_lat", UNSET),
            "start_lon": msg.get("start_lon", UNSET),
            "start_yaw_deg": msg.get("start_yaw_deg", UNSET),
        }
        if isinstance(reference_raw, dict):
            explicit_values = {
                "start_lat": reference_raw.get("lat", UNSET),
                "start_lon": reference_raw.get("lon", UNSET),
                "start_yaw_deg": reference_raw.get(
                    "yaw_deg",
                    reference_raw.get("heading_deg", UNSET),
                ),
            }

        explicit_count = sum(value is not UNSET for value in explicit_values.values())
        if explicit_count not in {0, 3}:
            return None, "reference requires lat, lon and yaw_deg"
        start_is_field_corner = True
        if explicit_count == 0:
            current_reference, reference_error = self.node.current_coverage_reference()
            if current_reference is None:
                return None, reference_error
            explicit_values = {
                "start_lat": current_reference["lat"],
                "start_lon": current_reference["lon"],
                "start_yaw_deg": current_reference["yaw_deg"],
            }
            # La pose del vehiculo es el centro de la primera pasada, no la
            # esquina del lote: asi arranca alineado y sin rulo de aproximacion.
            start_is_field_corner = False

        raw_values = {
            **explicit_values,
            "field_length_m": msg.get("field_length_m", UNSET),
            "field_width_m": msg.get("field_width_m", UNSET),
            "cutter_width_m": msg.get("cutter_width_m", 2.0),
            "overlap_ratio": msg.get("overlap_ratio", 0.15),
            "min_turning_radius_m": msg.get("min_turning_radius_m", 4.0),
            "waypoint_spacing_m": msg.get("waypoint_spacing_m", 2.0),
        }
        if raw_values["field_length_m"] is UNSET:
            return None, "field_length_m is required"
        if raw_values["field_width_m"] is UNSET:
            return None, "field_width_m is required"

        parsed: Dict[str, Any] = {}
        for key, raw_value in raw_values.items():
            try:
                value = float(raw_value)
            except (TypeError, ValueError):
                return None, f"{key} must be a number"
            if not np.isfinite(value):
                return None, f"{key} must be finite"
            parsed[key] = float(value)
        parsed["side"] = str(msg.get("side", "left") or "left").strip().lower()
        if parsed["side"] not in {"left", "right"}:
            return None, "side must be 'left' or 'right'"
        parsed["start_is_field_corner"] = bool(start_is_field_corner)
        # El poligono viaja crudo: la geometria la valida el route_executor, que
        # es el unico que sabe si el planner activo la soporta. Duplicar esa
        # validacion aca daria dos verdades que se desincronizan.
        parsed["coverage_polygon"] = msg.get("coverage_polygon")
        exclusiones = msg.get("coverage_exclusions")
        parsed["coverage_exclusions"] = (
            exclusiones if isinstance(exclusiones, list) else []
        )
        return parsed, ""

    def _parse_waypoints_from_message(
        self, msg: Dict[str, Any]
    ) -> Tuple[Optional[List[Dict[str, Any]]], bool, str]:
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
            role_raw = msg.get("role", "normal")
            role = str(role_raw or "normal").strip().lower()
            if role not in ("normal", "home"):
                return None, False, "role must be 'normal' or 'home'"
            if role != "normal":
                waypoint["role"] = role
            actions, actions_err = self._normalize_waypoint_actions(msg.get("actions", []), 0)
            if actions_err:
                return None, False, actions_err
            if actions:
                if role == "home":
                    return None, False, "HOME waypoint cannot include actions"
                waypoint["actions"] = actions
            return [waypoint], loop, ""

        if not isinstance(waypoints_raw, list) or len(waypoints_raw) == 0:
            return None, False, "waypoints must be a non-empty list"

        waypoints: List[Dict[str, Any]] = []
        home_count = 0
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
            role_raw = item.get("role", "normal")
            role = str(role_raw or "normal").strip().lower()
            if role not in ("normal", "home"):
                return None, False, f"waypoint[{idx}] role must be 'normal' or 'home'"
            if role == "home":
                home_count += 1
                if home_count > 1:
                    return None, False, "only one HOME waypoint is allowed"
                waypoint["role"] = role
            actions, actions_err = self._normalize_waypoint_actions(
                item.get("actions", []),
                idx,
            )
            if actions_err:
                return None, False, actions_err
            if actions:
                if role == "home":
                    return None, False, f"waypoint[{idx}] HOME waypoint cannot include actions"
                waypoint["actions"] = actions
            waypoints.append(waypoint)

        return waypoints, loop, ""

    def _parse_patrol_mission_from_message(
        self, msg: Dict[str, Any]
    ) -> Tuple[Optional[Dict[str, Any]], str]:
        payload = msg.get("patrol_mission", msg)
        if not isinstance(payload, dict):
            return None, "patrol_mission must be an object"

        home_waypoint = payload.get("home_waypoint")
        if not isinstance(home_waypoint, dict):
            return None, "home_waypoint is required"

        def parse_segment(name: str, allow_empty: bool) -> Tuple[Optional[List[Dict[str, Any]]], str]:
            raw_waypoints = payload.get(f"{name}_waypoints", [])
            if not isinstance(raw_waypoints, list):
                return None, f"{name}_waypoints must be a list"
            if not raw_waypoints:
                if allow_empty:
                    return [], ""
                return None, f"{name}_waypoints must be a non-empty list"
            parsed, _, err = self._parse_waypoints_from_message({"waypoints": raw_waypoints, "loop": False})
            if parsed is None:
                return None, err
            for waypoint in parsed:
                waypoint.pop("role", None)
            return parsed, ""

        loop_waypoints, loop_err = parse_segment("loop", False)
        if loop_waypoints is None:
            return None, loop_err
        return_waypoints, return_err = parse_segment("return", True)
        if return_waypoints is None:
            return None, return_err
        depart_waypoints, depart_err = parse_segment("depart", True)
        if depart_waypoints is None:
            return None, depart_err

        home_segment, _, home_err = self._parse_waypoints_from_message(
            {"waypoints": [home_waypoint], "loop": False}
        )
        if home_segment is None or len(home_segment) != 1:
            return None, home_err or "invalid home_waypoint"
        home_payload = dict(home_segment[0])
        home_payload["role"] = "home"
        home_payload.pop("actions", None)

        depart_entry_loop_index = payload.get("depart_entry_loop_index", None)
        try:
            depart_entry_loop_index = int(depart_entry_loop_index)
        except (TypeError, ValueError):
            return None, "depart_entry_loop_index must be an integer"

        return (
            {
                "loop_waypoints": list(loop_waypoints),
                "home_waypoint": home_payload,
                "return_waypoints": list(return_waypoints),
                "depart_waypoints": list(depart_waypoints),
                "depart_entry_loop_index": depart_entry_loop_index,
            },
            "",
        )

    @staticmethod
    def _normalize_waypoint_actions(raw_actions: Any, waypoint_index: int) -> Tuple[List[Dict[str, Any]], str]:
        if raw_actions in (None, "", []):
            return [], ""
        if not isinstance(raw_actions, list):
            return [], f"waypoint[{waypoint_index}] actions must be a list"
        actions: List[Dict[str, Any]] = []
        for action_index, raw_action in enumerate(raw_actions):
            if not isinstance(raw_action, dict):
                return [], f"waypoint[{waypoint_index}].actions[{action_index}] must be an object"
            action_type = str(raw_action.get("type", "")).strip()
            label = str(raw_action.get("label", "") or "").strip()
            if action_type == "brake_hold":
                duration_s = WebZoneServerNode._safe_float(raw_action.get("duration_s"), 0.0)
                if duration_s <= 0.0 or duration_s > 600.0:
                    return [], f"waypoint[{waypoint_index}] brake_hold duration_s must be > 0 and <= 600"
                action: Dict[str, Any] = {
                    "type": "brake_hold",
                    "duration_s": float(duration_s),
                    "brake_pct": int(
                        max(0, min(100, WebZoneServerNode._safe_int(raw_action.get("brake_pct"), 100)))
                    ),
                }
            elif action_type == "set_navigation_profile":
                profile = str(raw_action.get("profile", "") or "").strip().lower()
                if profile not in {"urban", "rural"}:
                    return [], (
                        f"waypoint[{waypoint_index}] set_navigation_profile profile must be 'urban' or 'rural'"
                    )
                action = {"type": "set_navigation_profile", "profile": profile}
            else:
                return [], f"unsupported waypoint action type: {action_type or '<empty>'}"
            if label:
                action["label"] = label[:80]
            actions.append(action)
        return actions, ""

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
        if op_name == "start_coverage":
            return True
        if op_name == "set_patrol_ll":
            return True
        if op_name == "set_navigation_profile":
            return True
        if op_name == "request_return_home":
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
            ok_patrol, err_patrol = await asyncio.to_thread(self.node.get_patrol_state)
            if not ok_patrol and err_patrol:
                self.node.get_logger().warning(
                    f"get_state patrol refresh failed: {err_patrol}"
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
            patrol_profile_raw = msg.get("patrol_profile")
            patrol_profile = patrol_profile_raw if isinstance(patrol_profile_raw, dict) else None
            ok, err, count = await asyncio.to_thread(
                self.node.save_waypoints_file,
                waypoints,
                patrol_profile,
            )
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
            ok, err, waypoints, patrol_profile = await asyncio.to_thread(self.node.load_waypoints_file)
            await self._send_ack(
                ws,
                "load_waypoints_file",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "waypoint_count": int(len(waypoints)),
                    "waypoints": list(waypoints) if ok else [],
                    **({"patrol_profile": patrol_profile} if ok and patrol_profile else {}),
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

        if op == "select_rtk_source":
            source_id = str(msg.get("id") or msg.get("source_id") or "")
            ok, err = await asyncio.to_thread(self.node.select_rtk_source, source_id)
            await self._send_ack(
                ws,
                "select_rtk_source",
                ok,
                err,
                client_req_id=client_req_id,
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "upsert_rtk_source":
            source_payload = msg.get("source") if isinstance(msg.get("source"), dict) else msg
            ok, err = await asyncio.to_thread(self.node.upsert_rtk_source, source_payload)
            await self._send_ack(
                ws,
                "upsert_rtk_source",
                ok,
                err,
                client_req_id=client_req_id,
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

        if op == "preview_coverage":
            parameters, parse_error = self._parse_coverage_parameters(msg)
            if parameters is None:
                await self._send_ack(
                    ws,
                    "preview_coverage",
                    False,
                    parse_error,
                    client_req_id=client_req_id,
                )
                return
            ok, error, coverage_plan = await asyncio.to_thread(
                self.node.generate_coverage_plan,
                parameters,
            )
            await self._send_ack(
                ws,
                "preview_coverage",
                ok,
                error,
                client_req_id=client_req_id,
                extra={"coverage_plan": coverage_plan} if ok else None,
            )
            return

        if op == "start_coverage":
            parameters, parse_error = self._parse_coverage_parameters(msg)
            if parameters is None:
                await self._send_ack(
                    ws,
                    "start_coverage",
                    False,
                    parse_error,
                    client_req_id=client_req_id,
                    extra={
                        "route_started": False,
                        "route_submission_state": "not_started",
                        **self._control_lock_extra(),
                    },
                )
                return
            ok, error, start_result = await asyncio.to_thread(
                self.node.start_coverage,
                parameters,
            )
            await self._send_ack(
                ws,
                "start_coverage",
                ok,
                error,
                client_req_id=client_req_id,
                extra={
                    **start_result,
                    **self._control_lock_extra(),
                },
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

        if op == "set_navigation_profile":
            profile = str(msg.get("profile", "") or "").strip().lower()
            if profile not in {"urban", "rural"}:
                await self._send_ack(
                    ws,
                    "set_navigation_profile",
                    False,
                    "profile must be 'urban' or 'rural'",
                    client_req_id=client_req_id,
                    extra=self._control_lock_extra(),
                )
                return
            ok, err, active_profile = await asyncio.to_thread(
                self.node.set_navigation_profile,
                profile,
            )
            await self._send_ack(
                ws,
                "set_navigation_profile",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "active_profile": active_profile,
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

        if op == "set_patrol_ll":
            patrol_payload, parse_err = self._parse_patrol_mission_from_message(msg)
            if patrol_payload is None:
                await self._send_ack(
                    ws,
                    "set_patrol_ll",
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
                    "set_patrol_ll",
                    False,
                    options_err,
                    client_req_id=client_req_id,
                )
                return
            ok, err, input_count, expanded_count = await asyncio.to_thread(
                self.node.set_patrol_mission,
                patrol_payload,
                leg_spacing_m,
                chunk_span_m,
                chunk_max_waypoints,
            )
            await self._send_ack(
                ws,
                "set_patrol_ll",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "loop_input_waypoint_count": int(input_count),
                    "loop_expanded_waypoint_count": int(expanded_count),
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

        if op == "cancel_patrol":
            ok, err = await asyncio.to_thread(self.node.cancel_patrol_mission)
            await self._send_ack(
                ws, "cancel_patrol", ok, err, client_req_id=client_req_id
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "request_return_home":
            ok, err = await asyncio.to_thread(self.node.request_patrol_return_home)
            await self._send_ack(
                ws,
                "request_return_home",
                ok,
                err,
                client_req_id=client_req_id,
                extra=self._control_lock_extra(),
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

        if op == "camera_ptz_move":
            relative = bool(msg.get("relative", False))
            payload_fields: Dict[str, Optional[float]] = {
                "pan_deg": None,
                "tilt_deg": None,
                "zoom_level": None,
            }
            for key in payload_fields:
                raw_value = msg.get(key)
                if raw_value is None:
                    continue
                try:
                    numeric = float(raw_value)
                except (ValueError, TypeError):
                    await self._send_ack(
                        ws,
                        "camera_ptz_move",
                        False,
                        f"{key} must be numeric",
                        client_req_id=client_req_id,
                    )
                    return
                if not np.isfinite(numeric):
                    await self._send_ack(
                        ws,
                        "camera_ptz_move",
                        False,
                        f"{key} must be finite",
                        client_req_id=client_req_id,
                    )
                    return
                payload_fields[key] = numeric
            if all(value is None for value in payload_fields.values()):
                await self._send_ack(
                    ws,
                    "camera_ptz_move",
                    False,
                    "at least one PTZ axis is required",
                    client_req_id=client_req_id,
                )
                return
            ok, err, payload = await asyncio.to_thread(
                self.node.camera_ptz_move,
                relative=relative,
                pan_deg=payload_fields["pan_deg"],
                tilt_deg=payload_fields["tilt_deg"],
                zoom_level=payload_fields["zoom_level"],
            )
            await self._send_ack(
                ws,
                "camera_ptz_move",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"payload": payload},
            )
            return

        if op == "camera_ptz_preset":
            preset = str(msg.get("preset", "")).strip()
            if not preset:
                await self._send_ack(
                    ws,
                    "camera_ptz_preset",
                    False,
                    "preset is required",
                    client_req_id=client_req_id,
                )
                return
            ok, err, payload = await asyncio.to_thread(self.node.camera_preset, preset)
            await self._send_ack(
                ws,
                "camera_ptz_preset",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"payload": payload},
            )
            return

        if op == "camera_ptz_set_preset":
            preset = str(msg.get("preset", "")).strip()
            if not preset:
                await self._send_ack(
                    ws,
                    "camera_ptz_set_preset",
                    False,
                    "preset is required",
                    client_req_id=client_req_id,
                )
                return
            ok, err, payload = await asyncio.to_thread(
                self.node.camera_save_preset,
                preset,
                save_zoom=bool(msg.get("save_zoom", False)),
            )
            await self._send_ack(
                ws,
                "camera_ptz_set_preset",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"payload": payload},
            )
            return

        if op == "get_camera_ptz_state":
            _, _, payload = await asyncio.to_thread(self.node.get_camera_ptz_state)
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
