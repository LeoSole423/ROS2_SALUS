from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool


class SimBatteryPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_battery_publisher")
        self.declare_parameter("battery_state_topic", "/battery_state")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("initial_percentage", 100.0)
        self.declare_parameter("low_percentage", 15.0)
        self.declare_parameter("drain_per_second", 0.0)

        self.battery_state_topic = str(self.get_parameter("battery_state_topic").value)
        self.publish_hz = max(0.5, float(self.get_parameter("publish_hz").value))
        self.initial_percentage = max(
            0.0, min(100.0, float(self.get_parameter("initial_percentage").value))
        )
        self.low_percentage = max(
            0.0, min(100.0, float(self.get_parameter("low_percentage").value))
        )
        self.drain_per_second = max(0.0, float(self.get_parameter("drain_per_second").value))

        self._current_percentage = float(self.initial_percentage)
        self._force_low = False

        self._pub = self.create_publisher(BatteryState, self.battery_state_topic, 10)
        self._set_low_srv = self.create_service(
            SetBool,
            "/sim_battery/set_low",
            self._on_set_low,
        )
        self._timer = self.create_timer(1.0 / float(self.publish_hz), self._tick)

        self.get_logger().info(
            "Sim battery publisher ready "
            f"(topic={self.battery_state_topic}, initial={self.initial_percentage:.1f}%, "
            f"low={self.low_percentage:.1f}%, drain_per_second={self.drain_per_second:.3f})"
        )

    def _on_set_low(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._force_low = bool(request.data)
        if self._force_low:
            self._current_percentage = float(self.low_percentage)
            response.message = f"sim battery forced low at {self.low_percentage:.1f}%"
        else:
            self._current_percentage = float(self.initial_percentage)
            response.message = f"sim battery restored to {self.initial_percentage:.1f}%"
        response.success = True
        self._publish_battery()
        return response

    def _tick(self) -> None:
        if not self._force_low and self.drain_per_second > 0.0:
            self._current_percentage = max(
                0.0, float(self._current_percentage) - float(self.drain_per_second) / float(self.publish_hz)
            )
        self._publish_battery()

    def _publish_battery(self) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = max(0.0, min(1.0, float(self._current_percentage) / 100.0))
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self._pub.publish(msg)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = SimBatteryPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
