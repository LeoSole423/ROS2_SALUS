import threading
import time

from action_msgs.msg import GoalStatus
import pytest
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import PoseStamped

from navegacion_gps.route_executor import (
    BLOCKED_STATE_NEEDS_OPERATOR,
    BLOCKED_STATE_NONE,
    BLOCKED_STATE_RETRYING,
    BLOCKED_STATE_WAITING,
    RouteExecutorNode,
    RouteWaypoint,
    _parse_route_action_jsons,
    _poses_to_debug_path,
    _yaw_to_quaternion,
    build_chunk_waypoints,
    drop_duplicate_loop_closure,
    expand_route_waypoints,
    expand_route_waypoints_with_actions,
    expanded_input_indices,
    next_chunk_start_index,
    prepare_route_waypoints,
    resolve_blocked_retry_start,
    route_progress,
    skip_passed_synthetic_chunk_start,
    skip_reached_chunk_start,
    should_suppress_chunk_success_brake,
)
from interfaces.msg import NavEvent, NavTelemetry


def _converted_pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation = _yaw_to_quaternion(yaw_deg)
    return pose


class _FakeLogger:
    def __init__(self) -> None:
        self.records = []

    def info(self, message: str) -> None:
        self.records.append(("info", message))

    def warning(self, message: str) -> None:
        self.records.append(("warning", message))

    def error(self, message: str) -> None:
        self.records.append(("error", message))


def _fake_blocking_node() -> RouteExecutorNode:
    node = object.__new__(RouteExecutorNode)
    node._lock = threading.RLock()
    node._route_input = []
    node._route_expanded = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
    ]
    node._route_key_waypoint_flags = [True, True]
    node._route_input_indices = [0, 1]
    node._active_chunk = list(node._route_expanded)
    node._mission_active = True
    node._mission_paused = False
    node._mission_loop = False
    node._mission_status = "route active"
    node._mission_note = ""
    node._current_start_index = 0
    node._current_target_index = 1
    node._awaiting_chunk_result = True
    node._last_robot_pose = None
    node._last_nav_goal_active = True
    node._last_nav_result_status = int(GoalStatus.STATUS_UNKNOWN)
    node._last_nav_result_event_id = 0
    node._last_handled_nav_result_event_id = 0
    node._last_blocking_nav_event_code = ""
    node._last_blocking_nav_event_text = ""
    node._last_collision_stop_started = None
    node._last_collision_stop_handled = False
    node._blocked_state = BLOCKED_STATE_NONE
    node._blocked_reason_code = ""
    node._blocked_reason_text = ""
    node._blocked_retry_attempt = 0
    node._blocked_wait_until = None
    node._blocked_retry_inflight = False
    node.blocked_retry_max_attempts = 3
    node.blocked_retry_wait_s = 5.0
    node.blocked_retry_reanchor_on_current_pose = True
    node.blocked_retry_reanchor_tolerance_m = 8.0
    node.collision_stop_persistent_s = 3.0
    node.route_waypoint_reached_tolerance_m = 1.2
    node.clear_costmaps_before_blocked_retry = True
    node.request_timeout_s = 0.01
    node.events = []
    node.brake_calls = 0
    node.cancel_calls = 0
    node.clear_costmap_calls = 0
    node.sent_chunk_starts = []
    node._fake_logger = _FakeLogger()
    node.get_logger = lambda: node._fake_logger
    node._publish_route_event = (
        lambda severity, code, message, details=None: node.events.append(
            (severity, code, message, dict(details or {}))
        )
        or len(node.events)
    )
    node._apply_brake = lambda: setattr(node, "brake_calls", node.brake_calls + 1)
    node._cancel_nav_goal = (
        lambda: setattr(node, "cancel_calls", node.cancel_calls + 1) or (True, "")
    )
    node._clear_costmaps_for_retry = lambda: setattr(
        node,
        "clear_costmap_calls",
        node.clear_costmap_calls + 1,
    )
    node._publish_empty_active_chunk_path = lambda: None
    node._publish_empty_route_paths = lambda: None
    node._start_next_chunk_after_success = lambda: setattr(node, "advanced", True)

    def _send_chunk(*, start_index: int):
        node.sent_chunk_starts.append(start_index)
        with node._lock:
            node._awaiting_chunk_result = True
            node._current_start_index = start_index
            node._mission_status = "route active"
        return True, ""

    node._send_chunk = _send_chunk
    return node


def _telemetry_result(status: int, *, event_id: int = 1, text: str = "") -> NavTelemetry:
    msg = NavTelemetry()
    msg.goal_active = False
    msg.nav_result_status = int(status)
    msg.nav_result_event_id = int(event_id)
    msg.nav_result_text = str(text)
    msg.robot_lat = float("nan")
    msg.robot_lon = float("nan")
    return msg


def test_debug_path_preserves_converted_waypoints_and_orientations():
    stamp = Time(sec=12, nanosec=34)
    poses = [
        _converted_pose(1.0, 2.0, 0.0),
        _converted_pose(3.0, 4.0, 90.0),
    ]

    path = _poses_to_debug_path(poses, frame_id="map", stamp=stamp)

    assert path.header.frame_id == "map"
    assert path.header.stamp == stamp
    assert len(path.poses) == 2
    assert path.poses[0].pose.position.x == pytest.approx(1.0)
    assert path.poses[1].pose.position.y == pytest.approx(4.0)
    assert path.poses[1].pose.orientation.z == pytest.approx(0.70710678, abs=1.0e-6)
    assert path.poses[1].pose.orientation.w == pytest.approx(0.70710678, abs=1.0e-6)


def test_debug_path_is_empty_without_converted_waypoints():
    path = _poses_to_debug_path([], frame_id="map", stamp=Time(sec=1))

    assert path.header.frame_id == "map"
    assert path.poses == []


def test_debug_mission_and_active_chunk_paths_have_expected_scopes():
    stamp = Time(sec=5)
    mission_poses = [
        _converted_pose(0.0, 0.0, 0.0),
        _converted_pose(10.0, 0.0, 0.0),
        _converted_pose(20.0, 0.0, 0.0),
    ]
    active_chunk_poses = mission_poses[1:]

    mission_path = _poses_to_debug_path(mission_poses, frame_id="map", stamp=stamp)
    active_chunk_path = _poses_to_debug_path(active_chunk_poses, frame_id="map", stamp=stamp)

    assert len(mission_path.poses) == 3
    assert len(active_chunk_path.poses) == 2
    assert active_chunk_path.poses[0].pose.position.x == pytest.approx(10.0)


def test_expand_route_waypoints_inserts_intermediate_points_for_long_legs():
    base = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
    ]

    expanded = expand_route_waypoints(base, leg_spacing_m=30.0, loop=False)

    assert len(expanded) >= 4
    assert expanded[0] == base[0]
    assert expanded[-1] == base[-1]


def test_expand_route_waypoints_handles_loop_closure_without_duplicating_start():
    base = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.001, lon=0.001, yaw_deg=90.0),
    ]

    expanded = expand_route_waypoints(base, leg_spacing_m=40.0, loop=True)

    assert expanded[0] == base[0]
    assert len(expanded) > len(base)
    assert expanded.count(base[0]) == 1


def test_route_action_jsons_accept_brake_hold_only():
    actions, err = _parse_route_action_jsons(
        ['[{"type":"brake_hold","duration_s":5,"brake_pct":100}]', ""],
        2,
    )

    assert err == ""
    assert actions is not None
    assert actions[0] == '[{"brake_pct":100,"duration_s":5.0,"type":"brake_hold"}]'
    assert actions[1] == ""


def test_expand_route_waypoints_with_actions_marks_only_original_waypoints():
    base = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
    ]
    action_json = '[{"brake_pct":100,"duration_s":5.0,"type":"brake_hold"}]'

    expanded, actions, key_flags = expand_route_waypoints_with_actions(
        base,
        ["", action_json],
        leg_spacing_m=30.0,
        loop=False,
    )

    assert len(expanded) == len(actions)
    assert len(expanded) == len(key_flags)
    assert actions[-1] == action_json
    assert all(action == "" for action in actions[:-1])
    assert key_flags[0] is True
    assert key_flags[-1] is True
    assert any(flag is False for flag in key_flags[1:-1])


def test_expanded_input_indices_maps_only_manual_checkpoints():
    assert expanded_input_indices([True, False, False, True, False, True]) == [
        0,
        -1,
        -1,
        1,
        -1,
        2,
    ]


def test_route_progress_projects_robot_on_active_chunk_only():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
    ]

    progress = route_progress(
        route,
        start_index=1,
        target_index=3,
        loop=False,
        robot_lat=0.0,
        robot_lon=0.0003,
    )

    assert progress is not None
    assert progress.expanded_index == 1
    assert progress.ratio == pytest.approx(0.5, abs=0.02)
    assert progress.cross_track_error_m == pytest.approx(0.0, abs=0.05)
    assert progress.distance_to_target_m > 30.0


def test_route_progress_supports_loop_closure_chunk():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
    ]

    progress = route_progress(
        route,
        start_index=3,
        target_index=0,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.0003,
    )

    assert progress is not None
    assert progress.expanded_index == 3
    assert progress.ratio == pytest.approx(0.5, abs=0.02)


def test_drop_duplicate_loop_closure_removes_repeated_first_waypoint():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=180.0),
    ]

    normalized, dropped = drop_duplicate_loop_closure(
        route,
        loop=True,
        closure_tolerance_m=1.2,
    )

    assert dropped is True
    assert normalized == route[:2]


def test_build_chunk_waypoints_limits_chunk_by_span_and_advances_after_target():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0008, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=0,
        loop=False,
        chunk_span_m=50.0,
        chunk_max_waypoints=5,
    )

    assert len(chunk) in (2, 3)
    assert end_index == len(chunk) - 1

    next_chunk, next_end_index = build_chunk_waypoints(
        route,
        start_index=end_index + 1,
        loop=False,
        chunk_span_m=50.0,
        chunk_max_waypoints=5,
    )

    assert next_chunk[0] == route[end_index + 1]
    assert next_end_index >= end_index + 1


def test_build_chunk_waypoints_stops_at_programmed_action_index():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=0,
        loop=False,
        chunk_span_m=200.0,
        chunk_max_waypoints=4,
        action_stop_indices={2},
    )

    assert chunk == route[:3]
    assert end_index == 2


def test_build_chunk_waypoints_keeps_programmed_action_at_chunk_start():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=0,
        loop=False,
        chunk_span_m=200.0,
        chunk_max_waypoints=3,
        action_stop_indices={0},
    )

    assert chunk == [route[0]]
    assert end_index == 0


def test_build_chunk_waypoints_uses_synthetic_points_but_stops_at_next_key_waypoint():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0008, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=0,
        loop=False,
        chunk_span_m=20.0,
        chunk_max_waypoints=2,
        key_stop_indices={0, 4},
    )

    assert chunk == route
    assert end_index == 4


def test_build_chunk_waypoints_loop_closes_at_key_instead_of_synthetic_waypoint():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=3,
        loop=True,
        chunk_span_m=20.0,
        chunk_max_waypoints=2,
        key_stop_indices={0, 2},
    )

    assert chunk == [route[3], route[0]]
    assert end_index == 0


def test_build_chunk_waypoints_wraps_for_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=2,
        loop=True,
        chunk_span_m=80.0,
        chunk_max_waypoints=3,
    )

    assert chunk[0] == route[2]
    assert chunk[1] == route[0]
    assert len(chunk) == 2
    assert end_index == 0


def test_build_chunk_waypoints_does_not_send_full_loop_cycle():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0004, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0008, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=0,
        loop=True,
        chunk_span_m=200.0,
        chunk_max_waypoints=5,
    )

    assert chunk == route[:4]
    assert end_index == 3


def test_build_chunk_waypoints_allows_single_target_for_two_point_loop():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
    ]

    chunk, end_index = build_chunk_waypoints(
        route,
        start_index=1,
        loop=True,
        chunk_span_m=200.0,
        chunk_max_waypoints=2,
    )

    assert chunk == [route[1]]
    assert end_index == 1


def test_next_chunk_start_index_advances_for_non_loop_routes():
    assert next_chunk_start_index(current_target_index=3, route_size=6, loop=False) == 4
    assert next_chunk_start_index(current_target_index=5, route_size=6, loop=False) == 6


def test_next_chunk_start_index_advances_for_loop_routes():
    assert next_chunk_start_index(current_target_index=3, route_size=4, loop=True) == 0
    assert next_chunk_start_index(current_target_index=1, route_size=4, loop=True) == 2


def test_should_suppress_chunk_success_brake_for_intermediate_non_loop_chunk():
    assert (
        should_suppress_chunk_success_brake(
            current_target_index=3,
            route_size=6,
            loop=False,
        )
        is True
    )


def test_should_not_suppress_chunk_success_brake_for_final_non_loop_chunk():
    assert (
        should_suppress_chunk_success_brake(
            current_target_index=5,
            route_size=6,
            loop=False,
        )
        is False
    )


def test_should_suppress_chunk_success_brake_for_loop_chunk():
    assert (
        should_suppress_chunk_success_brake(
            current_target_index=5,
            route_size=6,
            loop=True,
        )
        is True
    )


def test_prepare_route_waypoints_skips_reached_prefix_for_non_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.00001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0005, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=False,
        robot_lat=0.0,
        robot_lon=0.0,
        waypoint_reached_tolerance_m=1.2,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.skipped_waypoints == 2
    assert prepared.note == "skipped 2 reached waypoints"
    assert prepared.waypoints == [route[2]]


def test_prepare_route_waypoints_joins_nearest_segment_for_non_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.002, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=False,
        robot_lat=0.0,
        robot_lon=0.0005,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=3.0,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.start_index == 1
    assert prepared.skipped_waypoints == 1
    assert prepared.note == "joined nearest segment 1->2"
    assert prepared.waypoints == [route[1], route[2]]
    assert prepared.input_indices == [1, 2]


def test_prepare_route_waypoints_does_not_join_far_segment_for_non_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.002, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=False,
        robot_lat=0.0001,
        robot_lon=0.0005,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=3.0,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.start_index == 0
    assert prepared.note == ""
    assert prepared.waypoints == route


def test_prepare_route_waypoints_rejects_non_loop_when_final_is_already_reached():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=False,
        robot_lat=0.0,
        robot_lon=0.0,
        waypoint_reached_tolerance_m=1.2,
    )

    assert prepared is None
    assert "final waypoint already within 1.20 m" in error


def test_prepare_route_waypoints_rotates_loop_to_next_useful_waypoint():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0003, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0006, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.0003,
        waypoint_reached_tolerance_m=1.2,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.rotated is True
    assert prepared.start_index == 1
    assert prepared.note == "loop rotated to waypoint 2"
    assert prepared.waypoints == [route[1], route[2], route[0]]
    assert prepared.input_indices == [1, 2, 0]


def test_prepare_route_waypoints_joins_nearest_segment_for_loop_routes():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.001, lon=0.001, yaw_deg=90.0),
    ]

    prepared, error = prepare_route_waypoints(
        route,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.0005,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=3.0,
    )

    assert error == ""
    assert prepared is not None
    assert prepared.rotated is True
    assert prepared.start_index == 1
    assert prepared.note == "loop joined nearest segment 1->2"
    assert prepared.waypoints == [route[1], route[2], route[0]]
    assert prepared.input_indices == [1, 2, 0]


def test_skip_reached_chunk_start_advances_loop_chunk_from_current_pose():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.001, lon=0.001, yaw_deg=90.0),
    ]

    start, skipped = skip_reached_chunk_start(
        route,
        start_index=0,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.0,
        waypoint_reached_tolerance_m=1.2,
    )

    assert start == 1
    assert skipped == 1


def test_skip_reached_chunk_start_keeps_reached_action_waypoint_in_loop():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.001, lon=0.001, yaw_deg=90.0),
    ]

    start, skipped = skip_reached_chunk_start(
        route,
        start_index=0,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.0,
        waypoint_reached_tolerance_m=1.2,
        protected_indices={0},
    )

    assert start == 0
    assert skipped == 0


def test_skip_reached_chunk_start_wraps_past_reached_loop_closure():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
        RouteWaypoint(lat=0.001, lon=0.001, yaw_deg=90.0),
    ]

    start, skipped = skip_reached_chunk_start(
        route,
        start_index=2,
        loop=True,
        robot_lat=0.001,
        robot_lon=0.001,
        waypoint_reached_tolerance_m=1.2,
    )

    assert start == 0
    assert skipped == 1


def test_skip_reached_chunk_start_does_not_skip_only_remaining_non_loop_waypoint():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.001, yaw_deg=0.0),
    ]

    start, skipped = skip_reached_chunk_start(
        route,
        start_index=1,
        loop=False,
        robot_lat=0.0,
        robot_lon=0.001,
        waypoint_reached_tolerance_m=1.2,
    )

    assert start == 1
    assert skipped == 0


def test_skip_passed_synthetic_chunk_start_advances_on_route_progress():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
    ]

    start, skipped = skip_passed_synthetic_chunk_start(
        route,
        start_index=1,
        loop=False,
        robot_lat=0.00002,
        robot_lon=0.00015,
        segment_tolerance_m=5.0,
        skippable_indices={1},
    )

    assert start == 2
    assert skipped == 1


def test_skip_passed_synthetic_chunk_start_keeps_key_waypoint():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
    ]

    start, skipped = skip_passed_synthetic_chunk_start(
        route,
        start_index=1,
        loop=False,
        robot_lat=0.00002,
        robot_lon=0.00015,
        segment_tolerance_m=5.0,
        skippable_indices=set(),
    )

    assert start == 1
    assert skipped == 0


def test_skip_passed_synthetic_chunk_start_keeps_action_waypoint_protected():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
    ]

    start, skipped = skip_passed_synthetic_chunk_start(
        route,
        start_index=1,
        loop=False,
        robot_lat=0.00002,
        robot_lon=0.00015,
        segment_tolerance_m=5.0,
        skippable_indices={1},
        protected_indices={1},
    )

    assert start == 1
    assert skipped == 0


def test_nav_event_records_blocking_failure_reason_for_route_abort():
    node = _fake_blocking_node()
    event = NavEvent()
    event.component = "nav_command_server"
    event.code = "GOAL_RESULT_ABORTED"
    event.message = "planner failed"
    event.details = [
        KeyValue(key="failure_reason_code", value="NO_VALID_PATH"),
        KeyValue(key="failure_reason", value="no valid path through blocked street"),
    ]

    RouteExecutorNode._on_nav_event(node, event)
    RouteExecutorNode._on_nav_telemetry(
        node,
        _telemetry_result(
            GoalStatus.STATUS_ABORTED,
            text="NavigateThroughPoses aborted",
        ),
    )

    assert node._mission_active is True
    assert node._awaiting_chunk_result is False
    assert node._blocked_state == BLOCKED_STATE_WAITING
    assert node._blocked_reason_code == "NO_VALID_PATH"
    assert node._blocked_reason_text == "no valid path through blocked street"
    assert node.brake_calls == 1
    assert node.events[-1][1] == "ROUTE_BLOCKED_WAITING"


def test_costmap_clear_timeout_abort_waits_for_controlled_retry():
    node = _fake_blocking_node()
    event = NavEvent()
    event.component = "nav_command_server"
    event.code = "GOAL_RESULT_ABORTED"
    event.message = "costmap clear timed out"
    event.details = [
        KeyValue(key="failure_reason_code", value="COSTMAP_CLEAR_TIMEOUT"),
        KeyValue(key="failure_reason", value="Nav2 no pudo limpiar el costmap a tiempo"),
    ]

    RouteExecutorNode._on_nav_event(node, event)
    RouteExecutorNode._on_nav_telemetry(
        node,
        _telemetry_result(
            GoalStatus.STATUS_ABORTED,
            text="NavigateThroughPoses aborted",
        ),
    )

    assert node._mission_active is True
    assert node._blocked_state == BLOCKED_STATE_WAITING
    assert node._blocked_reason_code == "COSTMAP_CLEAR_TIMEOUT"
    assert node._blocked_reason_text == "Nav2 no pudo limpiar el costmap a tiempo"
    assert node.events[-1][1] == "ROUTE_BLOCKED_WAITING"


def test_blocking_abort_does_not_finish_mission_without_nav_event():
    node = _fake_blocking_node()

    RouteExecutorNode._on_nav_telemetry(
        node,
        _telemetry_result(
            GoalStatus.STATUS_ABORTED,
            text="controller detected collision near obstacle",
        ),
    )

    assert node._mission_active is True
    assert node._blocked_state == BLOCKED_STATE_WAITING
    assert node._blocked_reason_code == "CONTROLLER_COLLISION"
    assert "collision" in node._blocked_reason_text


def test_blocked_retry_clears_costmaps_and_resends_same_chunk():
    node = _fake_blocking_node()
    node._blocked_state = BLOCKED_STATE_RETRYING
    node._blocked_reason_code = "NO_VALID_PATH"
    node._blocked_reason_text = "no valid path found"
    node._blocked_retry_attempt = 1
    node._blocked_retry_inflight = True
    node._current_start_index = 1

    RouteExecutorNode._run_blocked_retry(node)

    assert node.clear_costmap_calls == 1
    assert node.sent_chunk_starts == [1]
    assert node._blocked_state == BLOCKED_STATE_NONE
    assert node._blocked_retry_inflight is False
    assert node.events[0][1] == "ROUTE_BLOCKED_RETRYING"
    assert node.events[0][3]["requested_start_index"] == 1
    assert node.events[0][3]["resolved_start_index"] == 1
    assert node.events[-1][1] == "ROUTE_BLOCKED_CLEARED"


def test_blocked_retry_reanchors_to_current_route_segment():
    node = _fake_blocking_node()
    node._route_expanded = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
    ]
    node._blocked_state = BLOCKED_STATE_RETRYING
    node._blocked_reason_code = "RECOVERY_FAILED"
    node._blocked_reason_text = "recovery failed"
    node._blocked_retry_attempt = 1
    node._blocked_retry_inflight = True
    node._current_start_index = 0
    node._last_robot_pose = (0.0, 0.00015)

    RouteExecutorNode._run_blocked_retry(node)

    assert node.sent_chunk_starts == [2]
    retry_details = node.events[0][3]
    assert retry_details["requested_start_index"] == 0
    assert retry_details["resolved_start_index"] == 2
    assert retry_details["reanchored"] is True
    assert retry_details["reanchor_reason"] == "nearest_segment"
    cleared_details = node.events[-1][3]
    assert cleared_details["requested_start_index"] == 0
    assert cleared_details["resolved_start_index"] == 2
    assert cleared_details["reanchored"] is True


def test_blocked_retry_reanchor_never_moves_non_loop_route_backwards():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
    ]

    resolution = resolve_blocked_retry_start(
        route,
        current_start_index=2,
        loop=False,
        robot_lat=0.0,
        robot_lon=0.00005,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=8.0,
        reanchor_enabled=True,
    )

    assert resolution.requested_start_index == 2
    assert resolution.resolved_start_index == 2
    assert resolution.reanchored is False
    assert resolution.reason == "no_forward_reanchor"


def test_blocked_retry_reanchor_never_moves_loop_route_backwards_before_closure():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0003, yaw_deg=0.0),
    ]

    resolution = resolve_blocked_retry_start(
        route,
        current_start_index=2,
        current_target_index=3,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.00005,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=8.0,
        reanchor_enabled=True,
    )

    assert resolution.requested_start_index == 2
    assert resolution.resolved_start_index == 2
    assert resolution.reanchored is False
    assert resolution.reason == "no_forward_reanchor_loop"


def test_blocked_retry_reanchor_does_not_repeat_second_waypoint_in_two_point_loop():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=180.0),
    ]

    resolution = resolve_blocked_retry_start(
        route,
        current_start_index=0,
        current_target_index=0,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.00018,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=8.0,
        reanchor_enabled=True,
    )

    assert resolution.requested_start_index == 0
    assert resolution.resolved_start_index == 0
    assert resolution.reanchored is False
    assert resolution.reason == "no_forward_reanchor_loop"


def test_blocked_retry_reanchor_allows_loop_wrap_after_closure():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0000, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0001, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0002, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0003, yaw_deg=0.0),
    ]

    resolution = resolve_blocked_retry_start(
        route,
        current_start_index=3,
        current_target_index=1,
        loop=True,
        robot_lat=0.0,
        robot_lon=0.00005,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=8.0,
        reanchor_enabled=True,
    )

    assert resolution.requested_start_index == 3
    assert resolution.resolved_start_index == 1
    assert resolution.reanchored is True
    assert resolution.reason == "nearest_segment"


def test_blocked_retry_reanchor_falls_back_without_pose_or_match():
    route = [
        RouteWaypoint(lat=0.0, lon=0.0, yaw_deg=0.0),
        RouteWaypoint(lat=0.0, lon=0.0001, yaw_deg=0.0),
    ]

    without_pose = resolve_blocked_retry_start(
        route,
        current_start_index=1,
        loop=False,
        robot_lat=None,
        robot_lon=None,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=8.0,
        reanchor_enabled=True,
    )
    far_away = resolve_blocked_retry_start(
        route,
        current_start_index=1,
        loop=False,
        robot_lat=1.0,
        robot_lon=1.0,
        waypoint_reached_tolerance_m=1.2,
        segment_start_tolerance_m=8.0,
        reanchor_enabled=True,
    )

    assert without_pose.resolved_start_index == 1
    assert without_pose.reanchored is False
    assert without_pose.reason == "pose_unavailable"
    assert far_away.resolved_start_index == 1
    assert far_away.reanchored is False
    assert far_away.reason == "no_match"


def test_exhausted_blocked_retries_hold_mission_for_operator():
    node = _fake_blocking_node()
    node._blocked_retry_attempt = 3

    RouteExecutorNode._enter_blocked_waiting(
        node,
        "NO_VALID_PATH",
        "still blocked",
    )

    assert node._mission_active is True
    assert node._awaiting_chunk_result is False
    assert node._blocked_state == BLOCKED_STATE_NEEDS_OPERATOR
    assert node._mission_status == "route blocked: needs operator (NO_VALID_PATH)"
    assert node.events[-1][1] == "ROUTE_BLOCKED_NEEDS_OPERATOR"
    assert node.brake_calls == 1


def test_persistent_collision_stop_cancels_goal_and_waits_before_retry():
    node = _fake_blocking_node()
    node._last_collision_stop_started = time.monotonic() - 3.1
    msg = NavTelemetry()
    msg.goal_active = True
    msg.collision_stop_active = True
    msg.nav_result_status = int(GoalStatus.STATUS_UNKNOWN)
    msg.nav_result_event_id = 0
    msg.robot_lat = float("nan")
    msg.robot_lon = float("nan")

    RouteExecutorNode._on_nav_telemetry(node, msg)

    assert node._blocked_state == BLOCKED_STATE_WAITING
    assert node._blocked_reason_code == "COLLISION_STOP_ACTIVE"
    assert node.cancel_calls == 1
    assert node.brake_calls == 1


def test_short_collision_stop_does_not_enter_blocked_state():
    node = _fake_blocking_node()
    msg = NavTelemetry()
    msg.goal_active = True
    msg.collision_stop_active = True
    msg.nav_result_status = int(GoalStatus.STATUS_UNKNOWN)
    msg.nav_result_event_id = 0
    msg.robot_lat = float("nan")
    msg.robot_lon = float("nan")

    RouteExecutorNode._on_nav_telemetry(node, msg)

    assert node._blocked_state == BLOCKED_STATE_NONE
    assert node.cancel_calls == 0
    assert node.brake_calls == 0


def test_manual_takeover_clears_blocked_state():
    node = _fake_blocking_node()
    node._blocked_state = BLOCKED_STATE_WAITING
    node._blocked_reason_code = "NO_VALID_PATH"
    node._blocked_retry_attempt = 1
    msg = NavTelemetry()
    msg.goal_active = True
    msg.manual_enabled = True
    msg.nav_result_status = int(GoalStatus.STATUS_UNKNOWN)
    msg.nav_result_event_id = 0
    msg.robot_lat = float("nan")
    msg.robot_lon = float("nan")

    RouteExecutorNode._on_nav_telemetry(node, msg)

    assert node._mission_paused is True
    assert node._blocked_state == BLOCKED_STATE_NONE
    assert node._blocked_retry_attempt == 0
