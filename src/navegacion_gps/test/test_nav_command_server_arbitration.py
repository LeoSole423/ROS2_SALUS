import threading
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import CollisionMonitorState

from interfaces.msg import CmdVelFinal
from navegacion_gps.nav_command_server import NavCommandServerNode


class _FakeArbNode:
    _build_cmd_vel_final = staticmethod(NavCommandServerNode._build_cmd_vel_final)
    _diag_level_value = staticmethod(NavCommandServerNode._diag_level_value)

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._manual_enabled = False
        self._is_navigating = False
        self.forward_cmd_vel_safe_without_goal = False
        self._auto_mode = "idle"
        self._current_goal_handle = None
        self._loop_waypoint_poses = []
        self._loop_original_poses = []
        self._loop_segment_start_index = 0
        self._loop_enabled = False
        self._suppress_success_brake = False
        self._last_cmd_vel_safe = None
        self._collision_stop_active = False
        self._critical_slow_brake_active = False
        self._collision_backup_active = False
        self._collision_backup_last_start_s = None
        self._last_collision_stop_active = False
        self._last_critical_slow_brake_active = False
        self._last_manual_cmd = CmdVelFinal()
        self._last_manual_cmd_time = None
        self._manual_watchdog_stop_sent = False
        self._last_cmd_vel_raw = None
        self._last_cmd_vel_raw_monotonic = None
        self._failure_code = ""
        self._failure_component = ""
        self._nav_action_results_to_ignore = 0
        self.manual_cmd_timeout_s = 0.1
        self.brake_publish_count = 1
        self.brake_publish_interval_s = 0.0
        self.critical_slow_brake_enabled = True
        self.critical_slow_polygon_name = "critical_slow_zone"
        self.critical_slow_brake_pct = 100
        self.collision_backup_recovery_enabled = False
        self.collision_backup_distance_m = 0.5
        self.collision_backup_speed_mps = 0.25
        self.collision_backup_cooldown_s = 8.0
        self.collision_backup_brake_hold_s = 0.0
        self.collision_backup_publish_hz = 10.0
        self.collision_backup_stop_speed_epsilon_mps = 0.03
        self.collision_backup_min_forward_mps = 0.10
        self.collision_backup_raw_timeout_s = 0.5

        self.published = []
        self.telemetry_forced = []
        self.cancel_calls = 0
        self.events = []
        self.backup_started = 0

    def _publish_cmd_vel_final(self, msg: CmdVelFinal) -> None:
        self.published.append(
            (
                float(msg.twist.linear.x),
                float(msg.twist.angular.z),
                int(msg.brake_pct),
            )
        )

    def _publish_stop(self, brake_pct: int) -> None:
        self._publish_cmd_vel_final(
            NavCommandServerNode._build_cmd_vel_final(0.0, 0.0, int(brake_pct))
        )

    def _publish_brake_sequence(self, brake_pct: int) -> None:
        self._publish_stop(brake_pct=brake_pct)

    def _start_collision_backup_recovery(self) -> bool:
        self.backup_started += 1
        self._collision_backup_active = True
        return True

    def _publish_manual_cmd(self, linear_x: float, angular_z: float, brake_pct: int) -> None:
        self._publish_cmd_vel_final(
            NavCommandServerNode._build_cmd_vel_final(linear_x, angular_z, brake_pct)
        )

    def _publish_manual_stop(self) -> None:
        self._publish_manual_cmd(0.0, 0.0, 0)

    def _publish_telemetry(self, force: bool = False) -> None:
        self.telemetry_forced.append(bool(force))

    def _set_failure_locked(self, code: str = "", component: str = "") -> None:
        self._failure_code = str(code)
        self._failure_component = str(component)

    def _publish_event(self, severity, component, code, message, *, details=None):
        self.events.append(
            {
                "severity": self._diag_level_value(severity),
                "component": str(component),
                "code": str(code),
                "message": str(message),
                "details": dict(details or {}),
            }
        )
        return len(self.events)

    def cancel_current_goal(self):
        self.cancel_calls += 1
        return False, "timeout cancelling goal"

    def _cancel_goal_for_manual_takeover_async(self) -> None:
        self.cancel_current_goal()

    def get_logger(self):
        class _Logger:
            def warning(self, _msg: str) -> None:
                pass

        return _Logger()


class _FakeRecoveryNode(_FakeArbNode):
    _start_collision_backup_recovery = NavCommandServerNode._start_collision_backup_recovery
    _detach_goal_handle_locked = NavCommandServerNode._detach_goal_handle_locked

    def __init__(self) -> None:
        super().__init__()
        self.collision_backup_recovery_enabled = True
        self.collision_backup_brake_hold_s = 0.0
        self._current_goal_handle = object()
        self._loop_waypoint_poses = [PoseStamped()]
        self._loop_enabled = False
        self._is_navigating = True
        self._auto_mode = "point_to_point"
        self._active_action = "follow_waypoints"
        self._last_nav_result_status = 0
        self._last_nav_result_text = "idle"
        self._nav_result_event_id = 0
        self.calls = []

    def _cancel_goal_handle_blocking(self, handle):
        self.calls.append(("cancel", handle is not None))
        return True, "cancelled"

    def _send_nav2_backup_goal(self, distance_m: float, speed_mps: float):
        self.calls.append(("backup", round(float(distance_m), 2), round(float(speed_mps), 2)))
        return True, "BackUp succeeded"

    def _send_nav_goal_for_poses(
        self,
        *,
        poses,
        loop_enabled,
        reason,
        details=None,
        suppress_success_brake=False,
    ):
        self.calls.append(("resume", len(list(poses)), bool(loop_enabled), str(reason)))
        self._current_goal_handle = object()
        self._loop_waypoint_poses = list(poses)
        self._is_navigating = True
        self._auto_mode = "loop" if loop_enabled else "point_to_point"
        self._active_action = "follow_waypoints"
        return True, "goal accepted"


def test_on_cmd_vel_safe_ignores_auto_while_manual() -> None:
    node = _FakeArbNode()
    node._manual_enabled = True
    node._is_navigating = True

    msg = Twist()
    msg.linear.x = 1.2
    msg.angular.z = 0.3
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == []


def test_on_cmd_vel_safe_publishes_auto_when_navigating() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = True
    node._collision_stop_active = False

    msg = Twist()
    msg.linear.x = 0.8
    msg.angular.z = -0.2
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == [(0.8, -0.2, 0)]


def test_on_cmd_vel_safe_publishes_auto_when_passthrough_enabled_without_goal() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = False
    node._collision_stop_active = False
    node.forward_cmd_vel_safe_without_goal = True

    msg = Twist()
    msg.linear.x = 0.6
    msg.angular.z = 0.15
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == [(0.6, 0.15, 0)]


def test_on_cmd_vel_safe_brakes_forward_in_critical_slow_zone() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = True
    node._critical_slow_brake_active = True

    msg = Twist()
    msg.linear.x = 0.7
    msg.angular.z = 0.1
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == [(0.0, 0.0, 100)]


def test_on_cmd_vel_safe_allows_reverse_in_critical_slow_zone_for_backup() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = True
    node._critical_slow_brake_active = True

    msg = Twist()
    msg.linear.x = -1.2
    msg.angular.z = 0.0
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == [(-1.2, 0.0, 0)]


def test_on_cmd_vel_safe_ignores_auto_during_collision_backup() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = True
    node._collision_stop_active = True
    node._collision_backup_active = True

    msg = Twist()
    msg.linear.x = 0.7
    msg.angular.z = 0.1
    NavCommandServerNode._on_cmd_vel_safe(node, msg)

    assert node.published == []


def test_on_cmd_vel_safe_starts_backup_when_collision_monitor_stops_raw_forward() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = True
    node.forward_cmd_vel_safe_without_goal = True
    node.collision_backup_recovery_enabled = True
    node._current_goal_handle = object()
    node._loop_waypoint_poses = [object()]

    raw = Twist()
    raw.linear.x = 0.8
    NavCommandServerNode._on_cmd_vel_raw(node, raw)

    safe = Twist()
    safe.linear.x = 0.0
    safe.angular.z = 0.0
    NavCommandServerNode._on_cmd_vel_safe(node, safe)

    assert node.backup_started == 1
    assert node.published == []


def test_on_cmd_vel_safe_does_not_start_backup_without_tracked_goal() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = False
    node.forward_cmd_vel_safe_without_goal = True
    node.collision_backup_recovery_enabled = True

    raw = Twist()
    raw.linear.x = 0.8
    NavCommandServerNode._on_cmd_vel_raw(node, raw)

    safe = Twist()
    safe.linear.x = 0.0
    safe.angular.z = 0.0
    NavCommandServerNode._on_cmd_vel_safe(node, safe)

    assert node.backup_started == 0
    assert node.published == [(0.0, 0.0, 0)]


def test_on_collision_monitor_state_stop_ignored_in_manual() -> None:
    node = _FakeArbNode()
    node._manual_enabled = True
    node._is_navigating = True
    node.collision_backup_recovery_enabled = True

    msg = CollisionMonitorState()
    msg.action_type = CollisionMonitorState.STOP
    NavCommandServerNode._on_collision_monitor_state(node, msg)

    assert node.published == []
    assert node.backup_started == 0


def test_on_collision_monitor_state_stop_starts_backup_for_tracked_goal() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = True
    node.collision_backup_recovery_enabled = True
    node._current_goal_handle = object()
    node._loop_waypoint_poses = [object()]

    msg = CollisionMonitorState()
    msg.action_type = CollisionMonitorState.STOP
    NavCommandServerNode._on_collision_monitor_state(node, msg)

    assert node.backup_started == 1
    assert node.published == []
    assert [event["code"] for event in node.events] == ["COLLISION_STOP_ACTIVE"]


def test_collision_backup_recovery_cancels_backs_up_and_resumes_goal() -> None:
    node = _FakeRecoveryNode()

    started = NavCommandServerNode._start_collision_backup_recovery(node)

    deadline = time.monotonic() + 1.0
    while node._collision_backup_active and time.monotonic() < deadline:
        time.sleep(0.01)

    assert started is True
    assert node._collision_backup_active is False
    assert node._is_navigating is True
    assert node.calls == [
        ("cancel", True),
        ("backup", 0.5, 0.25),
        ("resume", 1, False, "collision_backup_resume"),
    ]
    assert [event["code"] for event in node.events] == [
        "COLLISION_BACKUP_STARTED",
        "COLLISION_BACKUP_FINISHED",
    ]


def test_on_collision_monitor_state_critical_slow_brakes_once_and_emits_event() -> None:
    node = _FakeArbNode()
    node._manual_enabled = False
    node._is_navigating = True

    msg = CollisionMonitorState()
    msg.action_type = CollisionMonitorState.SLOWDOWN
    msg.polygon_name = "critical_slow_zone"
    NavCommandServerNode._on_collision_monitor_state(node, msg)
    NavCommandServerNode._on_collision_monitor_state(node, msg)

    assert node._critical_slow_brake_active is True
    assert node.published == [(0.0, 0.0, 100)]
    assert [event["code"] for event in node.events] == ["CRITICAL_SLOW_BRAKE_ACTIVE"]


def test_manual_watchdog_sends_single_stop() -> None:
    node = _FakeArbNode()
    node._manual_enabled = True
    node._last_manual_cmd_time = time.monotonic() - 1.0
    node._manual_watchdog_stop_sent = False

    NavCommandServerNode._manual_watchdog_tick(node)
    NavCommandServerNode._manual_watchdog_tick(node)

    assert node.published == [(0.0, 0.0, 0)]


def test_set_manual_mode_enables_even_if_cancel_fails() -> None:
    node = _FakeArbNode()
    node._current_goal_handle = object()
    ok, _err, enabled_after = NavCommandServerNode.set_manual_mode(node, True)

    assert ok is True
    assert enabled_after is True
    assert node._manual_enabled is True
    assert node._is_navigating is False
    assert node.cancel_calls == 1
