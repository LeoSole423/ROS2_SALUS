import threading
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticStatus
import pytest

from map_tools.web_zone_server import ROSBAG_TOPIC_PROFILES, WebZoneServerNode


def _diag_level(value) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(value, byteorder="little", signed=False)
    return int(value)


class _FakeNode:
    _diag_level_value = staticmethod(WebZoneServerNode._diag_level_value)
    _should_surface_diagnostic = WebZoneServerNode._should_surface_diagnostic
    _rosbag_topics_for_profile = staticmethod(WebZoneServerNode._rosbag_topics_for_profile)
    _normalize_gps_status_text = staticmethod(WebZoneServerNode._normalize_gps_status_text)
    _build_gps_status_payload = staticmethod(WebZoneServerNode._build_gps_status_payload)
    _build_gps_status_payload_from_navsat = staticmethod(
        WebZoneServerNode._build_gps_status_payload_from_navsat
    )


class _FakeStatus:
    def __init__(self, name: str, level, message: str) -> None:
        self.name = name
        self.level = level
        self.message = message


class _FakeSensorNode(_FakeNode):
    _build_default_datum_snapshot = WebZoneServerNode._build_default_datum_snapshot
    _precision_from_gps_snapshot = staticmethod(WebZoneServerNode._precision_from_gps_snapshot)
    _derive_mode = staticmethod(WebZoneServerNode._derive_mode)
    _connection_status_locked = WebZoneServerNode._connection_status_locked
    _build_general_sensor_snapshot = WebZoneServerNode._build_general_sensor_snapshot
    build_sensor_info_message = WebZoneServerNode.build_sensor_info_message
    is_ui_control_locked = WebZoneServerNode.is_ui_control_locked
    get_ui_control_lock_reason = WebZoneServerNode.get_ui_control_lock_reason

    def _build_datums_state_payload(self):
        return {
            "datums": [],
            "selected_id": "",
            "selected": None,
            "runtime": {
                "lat": self.fixed_datum_lat,
                "lon": self.fixed_datum_lon,
                "yaw_deg": self.fixed_datum_yaw_deg,
                "source": self.fixed_datum_source,
                "available": True,
                "already_set": True,
            },
            "pending_restart": False,
            "apply_mode": "next_restart",
            "file_path": "",
            "error": "",
        }

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.fixed_datum_lat = -31.4858037
        self.fixed_datum_lon = -64.2410570
        self.fixed_datum_yaw_deg = 0.0
        self.fixed_datum_source = "real_global_v2_fixed"
        self._goal_active = False
        self._manual_control = {"enabled": False}
        self._control_locked = False
        self._control_lock_reason = ""
        self.sensor_bridge_enabled = True
        self._gps_status_payload = {
            "available": True,
            "raw": "RTK_FIXED",
            "normalized": "rtk_fixed",
            "label": "RTK FIXED",
            "level": "good",
            "source": "rtk_status",
        }
        self._sensor_bridge_ok = True
        self._sensor_bridge_error = ""
        self._sensor_bridge_snapshot = {
            "gps_meta": {
                "fix_type_name": "RTK_FIXED",
                "rtk_status": "rtk_fixed",
                "satellites_visible": 18,
                "eph": 85,
            },
            "rtk_source_state": {
                "connected": True,
                "active_source_label": "Base Norte",
                "rtcm_age_s": 0.4,
            },
            "rtk_sources": [{"id": "base-norte", "label": "Base Norte"}],
            "gps": {
                "position_covariance": [0.04, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 0.0]
            },
            "diagnostics": {"yaw_delta_deg": 1.7},
        }
        self._datum_snapshot = self._build_default_datum_snapshot()


class _FakeWaypointYawNode(_FakeNode):
    _normalize_yaw_deg = staticmethod(WebZoneServerNode._normalize_yaw_deg)
    _bearing_deg_between_ll = staticmethod(WebZoneServerNode._bearing_deg_between_ll)
    _route_tangent_bearing_deg = staticmethod(WebZoneServerNode._route_tangent_bearing_deg)
    _resolve_waypoint_yaws = WebZoneServerNode._resolve_waypoint_yaws

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._last_robot_pose = None
        self._last_robot_heading_deg = None


class _FakeRouteStateNode(_FakeNode):
    _build_default_route_mission_payload = staticmethod(
        WebZoneServerNode._build_default_route_mission_payload
    )
    _route_waypoints_from_state = staticmethod(WebZoneServerNode._route_waypoints_from_state)
    _update_route_state = WebZoneServerNode._update_route_state

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._route_mission = self._build_default_route_mission_payload()


def test_should_surface_diagnostic_accepts_navigation_errors():
    node = _FakeNode()
    status = _FakeStatus(
        "navigation/nav_command_server",
        DiagnosticStatus.ERROR,
        "failure=GOAL_RESULT_ABORTED",
    )

    assert node._should_surface_diagnostic(status) is True


def test_should_surface_diagnostic_filters_non_navigation_status():
    node = _FakeNode()
    status = _FakeStatus("ekf_filter_node_map", DiagnosticStatus.ERROR, "stale")

    assert node._should_surface_diagnostic(status) is False


def test_should_surface_diagnostic_filters_idle_collision_monitor_warning():
    node = _FakeNode()
    status = _FakeStatus(
        "navigation/collision_monitor",
        DiagnosticStatus.WARN,
        "no collision monitor state yet",
    )

    assert node._should_surface_diagnostic(status) is False


def test_rosbag_topics_for_profile_matches_declared_profiles():
    topics = _FakeNode._rosbag_topics_for_profile("core")

    assert topics == ROSBAG_TOPIC_PROFILES["core"]
    assert "/global_position/raw/fix" in topics
    assert "/gps/rtk_status_mavros" in topics
    assert "/gps/odometry_map" in topics
    assert "/gps/course_heading" in topics
    assert "/gps/course_heading/debug" in topics
    assert "/odometry/global" in topics
    assert "/controller/drive_telemetry" in topics
    assert "/diagnostics" in topics
    assert "/nav_command_server/events" in topics
    assert _FakeNode._rosbag_topics_for_profile("missing") is None


def test_normalize_gps_status_text_handles_common_variants():
    assert _FakeNode._normalize_gps_status_text("RTK_FIXED") == "rtk_fixed"
    assert _FakeNode._normalize_gps_status_text("3D-FIX") == "3d_fix"
    assert _FakeNode._normalize_gps_status_text(" waiting for gps ") == "waiting_for_gps"


def test_build_gps_status_payload_maps_quality_to_label_and_level():
    payload = _FakeNode._build_gps_status_payload(
        raw="RTK_FLOAT",
        source="rtk_status",
        available=True,
    )

    assert payload["label"] == "RTK FLOAT"
    assert payload["level"] == "warn"
    assert payload["normalized"] == "rtk_float"
    assert payload["source"] == "rtk_status"


def test_build_gps_status_payload_from_navsat_falls_back_to_3d_fix():
    payload = _FakeNode._build_gps_status_payload_from_navsat(0)

    assert payload["label"] == "3D FIX"
    assert payload["level"] == "warn"
    assert payload["source"] == "gps_fix"


def test_build_default_datum_snapshot_uses_fixed_global_v2_values():
    node = _FakeSensorNode()

    snapshot = node._build_default_datum_snapshot()

    assert snapshot["available"] is True
    assert snapshot["already_set"] is True
    assert snapshot["datum_lat"] == node.fixed_datum_lat
    assert snapshot["datum_lon"] == node.fixed_datum_lon
    assert snapshot["last_set_source"] == "real_global_v2_fixed"


def test_build_general_sensor_snapshot_merges_bridge_state_and_precision():
    node = _FakeSensorNode()

    snapshot = node._build_general_sensor_snapshot()

    assert snapshot["gps_meta"]["fix_type_name"] == "RTK_FIXED"
    assert snapshot["gps_meta"]["estimated_precision_m"] == 0.85
    assert snapshot["rtk_source_state"]["active_source_label"] == "Base Norte"
    assert snapshot["datum"]["last_set_source"] == "real_global_v2_fixed"


def test_build_sensor_info_message_reports_bridge_errors_for_pixhawk_tab():
    node = _FakeSensorNode()
    node._sensor_bridge_ok = False
    node._sensor_bridge_error = "bridge offline"

    payload = node.build_sensor_info_message(tab="pixhawk_gps", interval_s=0.5)

    assert payload["implemented"] is True
    assert payload["ok"] is False
    assert payload["error"] == "bridge offline"


def test_resolve_waypoint_yaws_uses_route_tangent_for_auto_points():
    node = _FakeWaypointYawNode()
    waypoints = [
        {"lat": -31.0, "lon": -64.0},
        {"lat": -30.999, "lon": -64.0},
        {"lat": -30.999, "lon": -63.999},
    ]

    yaws = node._resolve_waypoint_yaws(waypoints, loop=False)

    assert yaws[0] == pytest.approx(90.0)
    assert yaws[1] == pytest.approx(45.0)
    assert yaws[2] == pytest.approx(0.0)


def test_resolve_waypoint_yaws_uses_loop_bearing_for_last_auto_point():
    node = _FakeWaypointYawNode()
    waypoints = [
        {"lat": -31.0, "lon": -64.0},
        {"lat": -30.999, "lon": -64.0},
    ]

    yaws = node._resolve_waypoint_yaws(waypoints, loop=True)

    assert yaws[0] == 90.0
    assert yaws[1] == -90.0


def test_resolve_waypoint_yaws_preserves_manual_and_uses_robot_for_single_auto():
    node = _FakeWaypointYawNode()
    node._last_robot_pose = {"lat": -31.0, "lon": -64.0, "heading_deg": 42.0}

    assert node._resolve_waypoint_yaws([{"lat": -30.999, "lon": -64.0}], loop=False) == [90.0]
    assert node._resolve_waypoint_yaws([{"lat": -31.0, "lon": -64.0}], loop=False) == [42.0]
    assert node._resolve_waypoint_yaws([{"lat": -31.0, "lon": -64.0, "yaw_deg": 181.0}], loop=False) == [-179.0]


def test_default_route_mission_payload_includes_blocked_fields():
    payload = _FakeRouteStateNode._build_default_route_mission_payload()

    assert payload["blocked_state"] == ""
    assert payload["blocked_reason_code"] == ""
    assert payload["blocked_reason_text"] == ""
    assert payload["blocked_retry_attempt"] == 0
    assert payload["blocked_retry_max_attempts"] == 0
    assert payload["blocked_wait_remaining_s"] == 0.0


def test_update_route_state_exposes_blocked_fields_for_websocket_payload():
    node = _FakeRouteStateNode()
    response = SimpleNamespace(
        active=True,
        paused=False,
        loop=False,
        input_waypoint_count=2,
        expanded_waypoint_count=3,
        current_start_index=1,
        current_target_index=2,
        active_chunk_size=2,
        leg_spacing_m=30.0,
        chunk_span_m=80.0,
        chunk_max_waypoints=4,
        status="route blocked: waiting",
        blocked_state="BLOCKED_WAITING",
        blocked_reason_code="NO_VALID_PATH",
        blocked_reason_text="no valid path found",
        blocked_retry_attempt=1,
        blocked_retry_max_attempts=3,
        blocked_wait_remaining_s=7.5,
        mission_lats=[-31.0, -31.001],
        mission_lons=[-64.0, -64.001],
        mission_yaws_deg=[0.0, 90.0],
        active_lats=[-31.001],
        active_lons=[-64.001],
        active_yaws_deg=[90.0],
    )

    node._update_route_state(response)

    assert node._route_mission["blocked_state"] == "BLOCKED_WAITING"
    assert node._route_mission["blocked_reason_code"] == "NO_VALID_PATH"
    assert node._route_mission["blocked_reason_text"] == "no valid path found"
    assert node._route_mission["blocked_retry_attempt"] == 1
    assert node._route_mission["blocked_retry_max_attempts"] == 3
    assert node._route_mission["blocked_wait_remaining_s"] == pytest.approx(7.5)
