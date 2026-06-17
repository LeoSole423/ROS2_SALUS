from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty

from controller_server.control_logic import command_from_cmd_vel


def translate_desired_command_to_gazebo_twist(
    *,
    speed_mps: float,
    steer_pct: int,
    sim_max_forward_mps: float,
    sim_max_reverse_mps: float,
    sim_max_steering_angle_rad: float,
) -> tuple[float, float]:
    linear_x = max(
        -float(sim_max_reverse_mps),
        min(float(sim_max_forward_mps), float(speed_mps)),
    )
    steering_angle_rad = (
        float(steer_pct) / 100.0 * abs(float(sim_max_steering_angle_rad))
    )
    return (linear_x, steering_angle_rad)


class CmdVelAckermannBridgeV2Node(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_ackermann_bridge_v2")

        self.declare_parameter("input_topic", "/cmd_vel_safe")
        self.declare_parameter("output_topic", "/cmd_vel_gazebo")
        self.declare_parameter("max_speed_mps", 4.0)
        self.declare_parameter("max_reverse_mps", 1.30)
        self.declare_parameter("vx_deadband_mps", 0.01)
        self.declare_parameter("vx_min_effective_mps", 0.5)
        self.declare_parameter("max_abs_angular_z", 0.4)
        self.declare_parameter("wheelbase_m", 0.94)
        self.declare_parameter("steering_limit_rad", 0.5235987756)
        self.declare_parameter("operational_steering_limit_rad", 0.3141592654)
        self.declare_parameter("invert_steer_from_cmd_vel", False)
        self.declare_parameter("auto_drive_enabled", True)
        self.declare_parameter("reverse_brake_pct", 20)
        self.declare_parameter("sim_max_forward_mps", 4.0)
        self.declare_parameter("sim_max_reverse_mps", 1.30)
        self.declare_parameter("sim_max_steering_angle_rad", 0.5235987756)
        self.declare_parameter("input_timeout_s", 0.5)
        self.declare_parameter("watchdog_hz", 20.0)
        self.declare_parameter("stop_hold_topic", "/local_nav_v2/stop_hold")
        self.declare_parameter("stop_hold_duration_s", 1.5)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        stop_hold_topic = str(self.get_parameter("stop_hold_topic").value)
        self._max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self._max_reverse_mps = float(self.get_parameter("max_reverse_mps").value)
        self._vx_deadband_mps = float(self.get_parameter("vx_deadband_mps").value)
        self._vx_min_effective_mps = float(
            self.get_parameter("vx_min_effective_mps").value
        )
        self._max_abs_angular_z = float(
            self.get_parameter("max_abs_angular_z").value
        )
        self._wheelbase_m = max(1.0e-6, float(self.get_parameter("wheelbase_m").value))
        self._steering_limit_rad = abs(
            float(self.get_parameter("steering_limit_rad").value)
        )
        self._operational_steering_limit_rad = abs(
            float(self.get_parameter("operational_steering_limit_rad").value)
        )
        if self._operational_steering_limit_rad < 1.0e-6:
            self._operational_steering_limit_rad = self._steering_limit_rad
        self._invert_steer_from_cmd_vel = bool(
            self.get_parameter("invert_steer_from_cmd_vel").value
        )
        self._auto_drive_enabled = bool(
            self.get_parameter("auto_drive_enabled").value
        )
        self._reverse_brake_pct = int(self.get_parameter("reverse_brake_pct").value)
        self._sim_max_forward_mps = float(
            self.get_parameter("sim_max_forward_mps").value
        )
        self._sim_max_reverse_mps = float(
            self.get_parameter("sim_max_reverse_mps").value
        )
        self._sim_max_steering_angle_rad = abs(
            float(self.get_parameter("sim_max_steering_angle_rad").value)
        )
        self._input_timeout_s = max(
            0.05, float(self.get_parameter("input_timeout_s").value)
        )
        self._stop_hold_duration_s = max(
            0.1, float(self.get_parameter("stop_hold_duration_s").value)
        )
        watchdog_hz = max(1.0, float(self.get_parameter("watchdog_hz").value))
        self._last_cmd_stamp_ns: int | None = None
        self._stop_hold_until_ns: int = 0

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, input_topic, self._on_cmd_vel, 10)
        self.create_subscription(Empty, stop_hold_topic, self._on_stop_hold, 10)
        self.create_timer(1.0 / watchdog_hz, self._on_watchdog_timer)

        self.get_logger().info(
            "cmd_vel_ackermann_bridge_v2 ready "
            f"({input_topic} -> {output_topic})"
        )

    def _publish_zero(self) -> None:
        self._pub.publish(Twist())

    def _stop_hold_active(self) -> bool:
        return self.get_clock().now().nanoseconds < self._stop_hold_until_ns

    def _on_stop_hold(self, _msg: Empty) -> None:
        self._stop_hold_until_ns = self.get_clock().now().nanoseconds + int(
            self._stop_hold_duration_s * 1_000_000_000.0
        )
        self._publish_zero()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_stamp_ns = self.get_clock().now().nanoseconds
        if self._stop_hold_active():
            self._publish_zero()
            return
        desired_command = command_from_cmd_vel(
            linear_x=msg.linear.x,
            angular_z=msg.angular.z,
            brake_pct=0,
            max_speed_mps=self._max_speed_mps,
            max_reverse_mps=self._max_reverse_mps,
            vx_deadband_mps=self._vx_deadband_mps,
            vx_min_effective_mps=self._vx_min_effective_mps,
            max_abs_angular_z=self._max_abs_angular_z,
            wheelbase_m=self._wheelbase_m,
            steering_limit_rad=self._steering_limit_rad,
            invert_steer=self._invert_steer_from_cmd_vel,
            auto_drive_enabled=self._auto_drive_enabled,
            reverse_brake_pct=self._reverse_brake_pct,
            operational_steering_limit_rad=self._operational_steering_limit_rad,
        )

        out = Twist()
        if desired_command.estop or not desired_command.drive_enabled:
            self._publish_zero()
            return

        out.linear.x, out.angular.z = translate_desired_command_to_gazebo_twist(
            speed_mps=float(desired_command.speed_mps),
            steer_pct=int(desired_command.steer_pct),
            sim_max_forward_mps=self._sim_max_forward_mps,
            sim_max_reverse_mps=self._sim_max_reverse_mps,
            sim_max_steering_angle_rad=self._sim_max_steering_angle_rad,
        )
        self._pub.publish(out)

    def _on_watchdog_timer(self) -> None:
        if self._stop_hold_active():
            self._publish_zero()
            return
        if self._last_cmd_stamp_ns is None:
            return

        age_s = (
            float(self.get_clock().now().nanoseconds - self._last_cmd_stamp_ns)
            / 1_000_000_000.0
        )
        if age_s <= self._input_timeout_s:
            return
        self._publish_zero()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelAckermannBridgeV2Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
