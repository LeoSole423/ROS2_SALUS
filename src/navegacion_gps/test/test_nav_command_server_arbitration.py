import threading
import time

from geometry_msgs.msg import Twist
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
        self._last_cmd_vel_safe = None
        self._collision_stop_active = False
        self._critical_slow_brake_active = False
        self._last_collision_stop_active = False
        self._last_critical_slow_brake_active = False
        self._last_manual_cmd = CmdVelFinal()
        self._last_manual_cmd_time = None
        self._manual_watchdog_stop_sent = False
        self._failure_code = ""
        self._failure_component = ""
        self.manual_cmd_timeout_s = 0.1
        self.brake_publish_count = 1
        self.brake_publish_interval_s = 0.0
        self.critical_slow_brake_enabled = True
        self.critical_slow_polygon_name = "critical_slow_zone"
        self.critical_slow_brake_pct = 100

        self.published = []
        self.telemetry_forced = []
        self.cancel_calls = 0
        self.events = []

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


def test_on_collision_monitor_state_stop_ignored_in_manual() -> None:
    node = _FakeArbNode()
    node._manual_enabled = True
    node._is_navigating = True

    msg = CollisionMonitorState()
    msg.action_type = CollisionMonitorState.STOP
    NavCommandServerNode._on_collision_monitor_state(node, msg)

    assert node.published == []


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
