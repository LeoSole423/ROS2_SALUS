import math
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion

from navegacion_gps.nav_command_server import (
    NAV_FAILURE_HINT_SUMMARIES,
    NavCommandServerNode,
)


def _pose(x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation = Quaternion(
        z=math.sin(0.5 * yaw_rad),
        w=math.cos(0.5 * yaw_rad),
    )
    return pose


def test_densify_exact_path_reconstructs_circle_without_new_mission_waypoints() -> None:
    dense = NavCommandServerNode._densify_exact_path(
        [_pose(1.0, 0.0, 90.0), _pose(0.0, 1.0, 180.0)],
        spacing_m=0.20,
    )

    assert len(dense) > 2
    assert dense[0].pose.position.x == 1.0
    assert dense[-1].pose.position.y == 1.0
    assert all(
        math.isclose(
            math.hypot(point.pose.position.x, point.pose.position.y),
            1.0,
            abs_tol=1.0e-6,
        )
        for point in dense
    )
    assert max(
        math.dist(
            (start.pose.position.x, start.pose.position.y),
            (end.pose.position.x, end.pose.position.y),
        )
        for start, end in zip(dense, dense[1:])
    ) <= 0.21


def test_densify_exact_path_keeps_straight_lane_straight() -> None:
    dense = NavCommandServerNode._densify_exact_path(
        [_pose(0.0, 2.0, 0.0), _pose(3.0, 2.0, 0.0)],
        spacing_m=0.35,
    )

    assert len(dense) > 2
    assert all(math.isclose(point.pose.position.y, 2.0) for point in dense)
    assert dense[-1].pose.position.x == 3.0


def test_build_loop_segment_poses_for_many_items() -> None:
    poses = [1, 2, 3, 4]
    assert NavCommandServerNode._build_loop_segment_poses(poses, 0) == [1, 2]
    assert NavCommandServerNode._build_loop_segment_poses(poses, 1) == [2, 3]
    assert NavCommandServerNode._build_loop_segment_poses(poses, 3) == [4, 1]


def test_build_loop_segment_poses_for_two_items() -> None:
    poses = [10, 20]
    assert NavCommandServerNode._build_loop_segment_poses(poses, 0) == [10, 20]
    assert NavCommandServerNode._build_loop_segment_poses(poses, 1) == [20, 10]


def test_build_loop_segment_poses_for_zero_or_one() -> None:
    assert NavCommandServerNode._build_loop_segment_poses([], 0) == []
    assert NavCommandServerNode._build_loop_segment_poses([7], 0) == [7]


def test_next_loop_segment_start_index_wraps() -> None:
    poses = [1, 2, 3, 4]
    assert NavCommandServerNode._next_loop_segment_start_index(poses, 0, 2) == 2
    assert NavCommandServerNode._next_loop_segment_start_index(poses, 2, 2) == 0
    assert NavCommandServerNode._next_loop_segment_start_index(poses, 3, 2) == 1


def test_next_loop_segment_start_index_handles_short_loops() -> None:
    poses = [1, 2, 3]
    assert NavCommandServerNode._next_loop_segment_start_index(poses, 0, 2) == 2
    assert NavCommandServerNode._next_loop_segment_start_index(poses, 2, 2) == 1


def test_drop_duplicate_loop_closure_waypoint_removes_repeated_first_point() -> None:
    waypoints = [
        (0.0, 0.0, 0.0),
        (0.0, 0.001, 0.0),
        (0.0, 0.0, 180.0),
    ]

    normalized, dropped = NavCommandServerNode._drop_duplicate_loop_closure_waypoint(
        waypoints,
        closure_tolerance_m=1.2,
    )

    assert dropped is True
    assert normalized == waypoints[:2]


def test_rotate_loop_waypoints_after_reached_first_anchor() -> None:
    waypoints = [
        (0.0, 0.0, 0.0),
        (0.0, 0.001, 0.0),
        (0.001, 0.001, 90.0),
    ]

    rotated, start_index = NavCommandServerNode._rotate_loop_waypoints_after_reached_anchor(
        waypoints,
        robot_pose={"lat": 0.0, "lon": 0.0},
        waypoint_reached_tolerance_m=1.2,
    )

    assert start_index == 1
    assert rotated == [waypoints[1], waypoints[2], waypoints[0]]


def test_trim_reached_waypoint_prefix_keeps_last_point_to_avoid_empty_goal() -> None:
    waypoints = [
        (0.0, 0.0, 0.0),
        (0.0, 0.001, 0.0),
    ]

    trimmed, skipped = NavCommandServerNode._trim_reached_waypoint_prefix(
        waypoints,
        robot_pose={"lat": 0.0, "lon": 0.0},
        waypoint_reached_tolerance_m=1.2,
    )

    assert skipped == 1
    assert trimmed == [waypoints[1]]


class _FakeLogger:
    def __init__(self) -> None:
        self.info_msgs = []
        self.warn_msgs = []
        self.error_msgs = []

    def info(self, msg: str) -> None:
        self.info_msgs.append(str(msg))

    def warning(self, msg: str) -> None:
        self.warn_msgs.append(str(msg))

    def error(self, msg: str) -> None:
        self.error_msgs.append(str(msg))


class _FakeResultFuture:
    def __init__(self, status: int, missed_waypoints=None):
        if missed_waypoints is None:
            missed_waypoints = []
        self._result = SimpleNamespace(
            status=int(status),
            result=SimpleNamespace(missed_waypoints=list(missed_waypoints)),
        )

    def result(self):
        return self._result


class _FakeLoopNode:
    _diag_level_value = staticmethod(NavCommandServerNode._diag_level_value)
    _build_loop_segment_poses = staticmethod(NavCommandServerNode._build_loop_segment_poses)
    _next_loop_segment_start_index = staticmethod(
        NavCommandServerNode._next_loop_segment_start_index
    )

    def __init__(self):
        self._lock = threading.Lock()
        self._current_goal_handle = object()
        self._loop_enabled = True
        self._loop_waypoint_poses = [1, 2]
        self._loop_original_poses = [1, 2, 3]
        self._loop_segment_start_index = 0
        self.loop_segment_size = 2
        self._suppress_success_brake = False
        self._manual_enabled = False
        self._is_navigating = True
        self._auto_mode = "loop"
        self._active_action = "navigate_through_poses"
        self._failure_code = ""
        self._failure_component = ""
        self._nav_action_results_to_ignore = 0
        self.nav_failure_hint_window_s = 25.0
        self._recent_nav_failure_hints = []

        self._send_ok = True
        self._send_err = ""
        self.sent_calls = []
        self.telemetry_forced = []
        self.brake_calls = []
        self.events = []
        self.logger = _FakeLogger()

    def _send_nav_goal_for_poses(
        self,
        poses,
        loop_enabled,
        reason,
        details=None,
        suppress_success_brake=False,
    ):
        self.sent_calls.append(
            (
                list(poses),
                bool(loop_enabled),
                str(reason),
                dict(details or {}),
                bool(suppress_success_brake),
            )
        )
        return bool(self._send_ok), str(self._send_err)

    def _publish_telemetry(self, force=False):
        self.telemetry_forced.append(bool(force))

    def _clear_loop_config_locked(self) -> None:
        self._loop_waypoint_poses = []
        self._loop_original_poses = []
        self._loop_segment_start_index = 0
        self._loop_enabled = False

    def _publish_brake_sequence(self, brake_pct: int) -> None:
        self.brake_calls.append(int(brake_pct))

    def _set_failure_locked(self, code: str = "", component: str = "") -> None:
        self._failure_code = str(code)
        self._failure_component = str(component)

    def _publish_event(self, severity, component, code, message, *, details=None):
        severity_value = self._diag_level_value(severity)
        self.events.append(
            {
                "severity": severity_value,
                "component": str(component),
                "code": str(code),
                "message": str(message),
                "details": dict(details or {}),
            }
        )
        if severity_value >= 2:
            self.logger.error(str(message))
        elif severity_value >= 1:
            self.logger.warning(str(message))
        else:
            self.logger.info(str(message))
        return len(self.events)

    def get_logger(self):
        return self.logger


def test_result_callback_advances_to_next_loop_segment_on_success() -> None:
    node = _FakeLoopNode()
    node._loop_original_poses = [1, 2, 3, 4]

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_SUCCEEDED),
    )

    poses, loop_enabled, reason, details, suppress_success_brake = node.sent_calls[0]
    assert poses == [3, 4]
    assert loop_enabled is True
    assert reason == "loop_segment_advance"
    assert suppress_success_brake is False
    assert details["loop_segment_start_index"] == 2
    assert details["loop_segment_size"] == 2
    assert details["loop_total_waypoints"] == 4
    assert node._loop_segment_start_index == 2
    assert node._loop_waypoint_poses == [3, 4]
    assert node._is_navigating is True
    assert node._auto_mode == "loop"


def test_result_callback_wraps_to_first_waypoint_after_last_segment() -> None:
    node = _FakeLoopNode()
    node._loop_original_poses = [1, 2, 3, 4]
    node._loop_waypoint_poses = [3, 4]
    node._loop_segment_start_index = 2

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_SUCCEEDED),
    )

    poses, loop_enabled, reason, details, suppress_success_brake = node.sent_calls[0]
    assert poses == [1, 2]
    assert loop_enabled is True
    assert reason == "loop_segment_advance"
    assert suppress_success_brake is False
    assert details["loop_segment_start_index"] == 0
    assert node._loop_segment_start_index == 0
    assert node._loop_waypoint_poses == [1, 2]


def test_result_callback_stops_loop_when_status_not_succeeded() -> None:
    node = _FakeLoopNode()

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_ABORTED),
    )
    assert node.sent_calls == []
    assert node.brake_calls == [100]
    assert node._is_navigating is False
    assert node._auto_mode == "idle"
    assert node._loop_enabled is False


def test_classify_nav_failure_hint_no_valid_path() -> None:
    code, summary = NavCommandServerNode._classify_nav_failure_hint(
        "planner_server",
        "GridBased: failed to create plan, no valid path found.",
    )

    assert code == "NO_VALID_PATH"
    assert "ruta válida" in summary


def test_classify_nav_failure_hint_action_server_ack_timeout() -> None:
    code, summary = NavCommandServerNode._classify_nav_failure_hint(
        "bt_navigator_navigate_to_pose_rclcpp_node",
        (
            "Timed out while waiting for action server to acknowledge goal request "
            "for compute_path_to_pose"
        ),
    )

    assert code == "ACTION_SERVER_ACK_TIMEOUT"
    assert "action server interno" in summary


def test_classify_nav_failure_hint_tf_extrapolation() -> None:
    code, summary = NavCommandServerNode._classify_nav_failure_hint(
        "controller_server",
        (
            "Exception in transformPose: Lookup would require extrapolation into the future. "
            "Requested time 78.906000 but the latest data is at time 78.870000"
        ),
    )

    assert code == "TF_EXTRAPOLATION"
    assert "desfase transitorio de TF" in summary


def test_classify_nav_failure_hint_tf_transform_pose_fallback_message() -> None:
    code, _ = NavCommandServerNode._classify_nav_failure_hint(
        "controller_server",
        "Unable to transform robot pose into global plan's frame",
    )

    assert code == "TF_EXTRAPOLATION"


def test_abort_result_includes_recent_nav_failure_hint() -> None:
    node = _FakeLoopNode()
    node._loop_enabled = False
    node._auto_mode = "point_to_point"
    node._recent_nav_failure_hints = [
        (
            time.monotonic(),
            "NO_VALID_PATH",
            NAV_FAILURE_HINT_SUMMARIES["NO_VALID_PATH"],
            "GridBased: failed to create plan, no valid path found.",
        )
    ]

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_ABORTED),
    )

    assert "no se encontró una ruta válida" in node._last_nav_result_text
    result_events = [
        event for event in node.events if event["code"] == "GOAL_RESULT_ABORTED"
    ]
    assert result_events
    assert result_events[0]["details"]["failure_reason_code"] == "NO_VALID_PATH"
    assert "obstáculos" in result_events[0]["message"]


def test_result_callback_stops_loop_when_segment_send_fails() -> None:
    node = _FakeLoopNode()
    node._send_ok = False
    node._send_err = "goal rejected by NavigateThroughPoses"

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_SUCCEEDED),
    )

    assert len(node.sent_calls) == 1
    assert node._loop_enabled is False
    assert node._loop_original_poses == []
    assert node._loop_waypoint_poses == []
    assert node.brake_calls == [100]
    assert node._is_navigating is False
    assert node._auto_mode == "idle"
    assert any("Loop restart failed" in msg for msg in node.logger.warn_msgs)


def test_result_callback_point_to_point_stops_on_success() -> None:
    node = _FakeLoopNode()
    node._loop_enabled = False
    node._auto_mode = "point_to_point"
    node._loop_original_poses = []
    node._loop_waypoint_poses = []

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_SUCCEEDED),
    )
    assert node.sent_calls == []
    assert node.brake_calls == [100]
    assert node._is_navigating is False
    assert node._auto_mode == "idle"


def test_result_callback_point_to_point_suppresses_brake_on_success() -> None:
    node = _FakeLoopNode()
    node._loop_enabled = False
    node._auto_mode = "point_to_point"
    node._suppress_success_brake = True
    node._loop_original_poses = []
    node._loop_waypoint_poses = []

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_SUCCEEDED),
    )
    assert node.sent_calls == []
    assert node.brake_calls == []
    assert node._is_navigating is False
    assert node._auto_mode == "idle"
    assert node._suppress_success_brake is False


def test_result_callback_point_to_point_still_brakes_on_abort_when_suppressed() -> None:
    node = _FakeLoopNode()
    node._loop_enabled = False
    node._auto_mode = "point_to_point"
    node._suppress_success_brake = True
    node._loop_original_poses = []
    node._loop_waypoint_poses = []

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_ABORTED),
    )
    assert node.sent_calls == []
    assert node.brake_calls == [100]
    assert node._is_navigating is False
    assert node._auto_mode == "idle"
    assert node._suppress_success_brake is False


def test_result_callback_point_to_point_manual_mode_does_not_brake() -> None:
    node = _FakeLoopNode()
    node._loop_enabled = False
    node._auto_mode = "point_to_point"
    node._manual_enabled = True
    node._loop_original_poses = []
    node._loop_waypoint_poses = []

    NavCommandServerNode._on_nav_action_result_done(
        node,
        "NavigateThroughPoses",
        _FakeResultFuture(GoalStatus.STATUS_ABORTED),
    )
    assert node.sent_calls == []
    assert node.brake_calls == []
    assert node._is_navigating is False
    assert node._auto_mode == "idle"
