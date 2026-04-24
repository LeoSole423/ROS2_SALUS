import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus

from navegacion_gps.nav_command_server import (
    NAV_FAILURE_HINT_SUMMARIES,
    NavCommandServerNode,
)


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
