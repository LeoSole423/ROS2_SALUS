import threading

from diagnostic_msgs.msg import DiagnosticStatus

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
