from __future__ import annotations

import json
import math
import time
from dataclasses import asdict

import rclpy
from interfaces.msg import BatteryMissionGuard, CmdVelFinal, DriveTelemetry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from .battery_estimator import (
    BatteryEstimator,
    battery_state_label,
    parse_soc_curve_points,
)
from .control_logic import (
    DesiredCommand,
    command_from_cmd_vel,
    safe_command,
    select_effective_command,
)
from .serial_port_resolver import resolve_serial_port
from .transport_backends import create_transport_backend


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


class ControllerServerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_server")

        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("serial_tx_hz", 50.0)
        self.declare_parameter("max_speed_mps", 4.0)
        self.declare_parameter("max_reverse_mps", 1.30)
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("telemetry_pub_hz", 10.0)
        self.declare_parameter("auto_timeout_s", 0.7)
        self.declare_parameter("max_abs_angular_z", 0.4)
        self.declare_parameter("vx_deadband_mps", 0.10)
        self.declare_parameter("vx_min_effective_mps", 0.75)
        self.declare_parameter("wheelbase_m", 0.94)
        self.declare_parameter("steering_limit_rad", 0.5235987756)
        self.declare_parameter("operational_steering_limit_rad", 0.3141592654)
        self.declare_parameter("manual_operational_steering_limit_rad", 0.5235987756)
        self.declare_parameter("reverse_brake_pct", 20)
        self.declare_parameter("invert_steer_from_cmd_vel", False)
        self.declare_parameter("auto_drive_enabled", True)
        self.declare_parameter("estop_brake_pct", 100)
        self.declare_parameter("telemetry_stale_timeout_s", 0.5)
        self.declare_parameter("battery_state_topic", "/battery_state")
        self.declare_parameter("battery_guard_topic", "/battery_mission_guard")
        self.declare_parameter("battery_full_voltage", 60.0)
        self.declare_parameter("battery_empty_voltage", 55.0)
        self.declare_parameter("battery_low_voltage", 58.0)
        self.declare_parameter("battery_critical_voltage", 56.0)
        self.declare_parameter("battery_telemetry_stale_timeout_s", 3.0)
        self.declare_parameter(
            "battery_soc_curve_points",
            [55.0, 0.0, 57.0, 0.8, 57.5, 0.9, 60.0, 1.0],
        )
        self.declare_parameter("battery_loaded_fast_tau_s", 4.0)
        self.declare_parameter("battery_loaded_slow_tau_s", 45.0)
        self.declare_parameter("battery_recovered_tau_s", 12.0)
        self.declare_parameter("battery_soc_discharge_tau_s", 180.0)
        self.declare_parameter("battery_guard_loaded_low_voltage", 56.0)
        self.declare_parameter("battery_guard_recovered_low_voltage", 57.0)
        self.declare_parameter("battery_guard_loaded_low_persist_s", 90.0)
        self.declare_parameter("battery_guard_recovered_low_persist_s", 20.0)
        self.declare_parameter("transport_backend", "uart")
        self.declare_parameter("sim_cmd_vel_topic", "/cmd_vel_gazebo")
        self.declare_parameter("sim_odom_topic", "/odom_raw")
        self.declare_parameter("sim_joint_states_topic", "/joint_states")
        self.declare_parameter("sim_front_left_steer_joint", "front_left_steer_joint")
        self.declare_parameter("sim_front_right_steer_joint", "front_right_steer_joint")
        self.declare_parameter("sim_wheelbase_m", 0.94)
        self.declare_parameter("sim_track_width_m", 0.75)
        self.declare_parameter("sim_max_steering_angle_rad", 0.5235987756)
        self.declare_parameter("sim_telemetry_timeout_s", 0.5)
        self.declare_parameter("sim_invert_actuation_steer_sign", True)
        self.declare_parameter("sim_invert_measured_steer_sign", True)
        self.declare_parameter("sim_max_joint_odom_steer_delta_deg", 5.0)

        requested_serial_port = self.get_parameter("serial_port").value
        serial_selection = resolve_serial_port(requested_serial_port)
        self._serial_port = serial_selection.port
        self._serial_baud = int(self.get_parameter("serial_baud").value)
        self._serial_tx_hz = float(self.get_parameter("serial_tx_hz").value)
        self._max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self._max_reverse_mps = float(self.get_parameter("max_reverse_mps").value)
        if self._max_reverse_mps < 0.0:
            self.get_logger().warn(
                f"Invalid max_reverse_mps={self._max_reverse_mps:.3f}; clamping to 0.0"
            )
            self._max_reverse_mps = 0.0
        self._control_hz = max(1.0, float(self.get_parameter("control_hz").value))
        self._telemetry_pub_hz = max(1.0, float(self.get_parameter("telemetry_pub_hz").value))
        self._auto_timeout_s = float(self.get_parameter("auto_timeout_s").value)
        self._max_abs_angular_z = float(self.get_parameter("max_abs_angular_z").value)
        self._vx_deadband_mps = float(self.get_parameter("vx_deadband_mps").value)
        if self._vx_deadband_mps < 0.0:
            self.get_logger().warn(
                f"Invalid vx_deadband_mps={self._vx_deadband_mps:.3f}; clamping to 0.0"
            )
            self._vx_deadband_mps = 0.0
        self._vx_min_effective_mps = float(self.get_parameter("vx_min_effective_mps").value)
        if self._vx_min_effective_mps < 0.0:
            self.get_logger().warn(
                f"Invalid vx_min_effective_mps={self._vx_min_effective_mps:.3f}; clamping to 0.0"
            )
            self._vx_min_effective_mps = 0.0
        if self._vx_min_effective_mps > self._max_speed_mps:
            self.get_logger().warn(
                "vx_min_effective_mps greater than max_speed_mps; "
                f"using max_speed_mps={self._max_speed_mps:.3f} as effective minimum"
            )
            self._vx_min_effective_mps = self._max_speed_mps
        self._wheelbase_m = max(1.0e-6, float(self.get_parameter("wheelbase_m").value))
        self._steering_limit_rad = abs(
            float(self.get_parameter("steering_limit_rad").value)
        )
        if self._steering_limit_rad < 1.0e-6:
            self.get_logger().warn(
                "Invalid steering_limit_rad; using default 0.5235987756 rad"
            )
            self._steering_limit_rad = 0.5235987756
        self._operational_steering_limit_rad = abs(
            float(self.get_parameter("operational_steering_limit_rad").value)
        )
        if self._operational_steering_limit_rad < 1.0e-6:
            self.get_logger().warn(
                "Invalid operational_steering_limit_rad; using steering_limit_rad"
            )
            self._operational_steering_limit_rad = self._steering_limit_rad
        self._effective_steering_limit_rad = min(
            self._steering_limit_rad,
            self._operational_steering_limit_rad,
        )
        self._manual_operational_steering_limit_rad = abs(
            float(self.get_parameter("manual_operational_steering_limit_rad").value)
        )
        if self._manual_operational_steering_limit_rad < 1.0e-6:
            self.get_logger().warn(
                "Invalid manual_operational_steering_limit_rad; using steering_limit_rad"
            )
            self._manual_operational_steering_limit_rad = self._steering_limit_rad
        self._manual_effective_steering_limit_rad = min(
            self._steering_limit_rad,
            self._manual_operational_steering_limit_rad,
        )
        self._reverse_brake_pct = int(self.get_parameter("reverse_brake_pct").value)
        self._invert_steer_from_cmd_vel = bool(
            self.get_parameter("invert_steer_from_cmd_vel").value
        )
        self._auto_drive_enabled = bool(self.get_parameter("auto_drive_enabled").value)
        self._estop_brake_pct = int(self.get_parameter("estop_brake_pct").value)
        self._telemetry_stale_timeout_s = max(
            0.05, float(self.get_parameter("telemetry_stale_timeout_s").value)
        )
        self._battery_state_topic = str(self.get_parameter("battery_state_topic").value)
        self._battery_guard_topic = str(self.get_parameter("battery_guard_topic").value)
        self._battery_full_voltage = float(self.get_parameter("battery_full_voltage").value)
        self._battery_empty_voltage = float(self.get_parameter("battery_empty_voltage").value)
        self._battery_low_voltage = float(self.get_parameter("battery_low_voltage").value)
        self._battery_critical_voltage = float(
            self.get_parameter("battery_critical_voltage").value
        )
        self._battery_telemetry_stale_timeout_s = max(
            0.5, float(self.get_parameter("battery_telemetry_stale_timeout_s").value)
        )
        battery_soc_curve_values = self.get_parameter("battery_soc_curve_points").value
        try:
            battery_soc_curve_points = parse_soc_curve_points(battery_soc_curve_values)
        except ValueError as exc:
            self.get_logger().warn(
                f"Invalid battery_soc_curve_points; using defaults ({exc})"
            )
            battery_soc_curve_points = parse_soc_curve_points(None)
        if self._battery_full_voltage <= self._battery_empty_voltage:
            self.get_logger().warn(
                "battery_full_voltage must be greater than battery_empty_voltage; "
                "adjusting to keep a valid range"
            )
            self._battery_full_voltage = self._battery_empty_voltage + 1.0
        self._battery_estimator = BatteryEstimator(
            soc_curve_points=battery_soc_curve_points,
            loaded_fast_tau_s=float(self.get_parameter("battery_loaded_fast_tau_s").value),
            loaded_slow_tau_s=float(self.get_parameter("battery_loaded_slow_tau_s").value),
            recovered_tau_s=float(self.get_parameter("battery_recovered_tau_s").value),
            soc_fast_discharge_tau_s=float(
                self.get_parameter("battery_soc_discharge_tau_s").value
            ),
            loaded_low_threshold_v=float(
                self.get_parameter("battery_guard_loaded_low_voltage").value
            ),
            recovered_low_threshold_v=float(
                self.get_parameter("battery_guard_recovered_low_voltage").value
            ),
            loaded_low_persist_s=float(
                self.get_parameter("battery_guard_loaded_low_persist_s").value
            ),
            recovered_low_persist_s=float(
                self.get_parameter("battery_guard_recovered_low_persist_s").value
            ),
        )
        self._transport_backend = str(self.get_parameter("transport_backend").value)
        self._sim_cmd_vel_topic = str(self.get_parameter("sim_cmd_vel_topic").value)
        self._sim_odom_topic = str(self.get_parameter("sim_odom_topic").value)
        self._sim_joint_states_topic = str(
            self.get_parameter("sim_joint_states_topic").value
        )
        self._sim_front_left_steer_joint = str(
            self.get_parameter("sim_front_left_steer_joint").value
        )
        self._sim_front_right_steer_joint = str(
            self.get_parameter("sim_front_right_steer_joint").value
        )
        self._sim_wheelbase_m = max(1.0e-6, float(self.get_parameter("sim_wheelbase_m").value))
        self._sim_track_width_m = max(
            0.0, float(self.get_parameter("sim_track_width_m").value)
        )
        self._sim_max_steering_angle_rad = abs(
            float(self.get_parameter("sim_max_steering_angle_rad").value)
        )
        self._sim_telemetry_timeout_s = max(
            0.05, float(self.get_parameter("sim_telemetry_timeout_s").value)
        )
        self._sim_invert_actuation_steer_sign = bool(
            self.get_parameter("sim_invert_actuation_steer_sign").value
        )
        self._sim_invert_measured_steer_sign = bool(
            self.get_parameter("sim_invert_measured_steer_sign").value
        )
        self._sim_max_joint_odom_steer_delta_deg = max(
            0.0, float(self.get_parameter("sim_max_joint_odom_steer_delta_deg").value)
        )

        self._auto_cmd = safe_command()
        self._auto_stamp_s = 0.0
        self._last_source = "init"
        self._last_steer_saturated = False

        self._client = create_transport_backend(
            node=self,
            transport_backend=self._transport_backend,
            serial_port=self._serial_port,
            serial_baud=self._serial_baud,
            serial_tx_hz=self._serial_tx_hz,
            max_speed_mps=self._max_speed_mps,
            max_reverse_mps=self._max_reverse_mps,
            sim_cmd_vel_topic=self._sim_cmd_vel_topic,
            sim_odom_topic=self._sim_odom_topic,
            sim_joint_states_topic=self._sim_joint_states_topic,
            sim_front_left_steer_joint=self._sim_front_left_steer_joint,
            sim_front_right_steer_joint=self._sim_front_right_steer_joint,
            sim_wheelbase_m=self._sim_wheelbase_m,
            sim_track_width_m=self._sim_track_width_m,
            sim_max_steering_angle_rad=self._sim_max_steering_angle_rad,
            sim_telemetry_timeout_s=self._sim_telemetry_timeout_s,
            sim_invert_actuation_steer_sign=self._sim_invert_actuation_steer_sign,
            sim_invert_measured_steer_sign=self._sim_invert_measured_steer_sign,
            sim_max_joint_odom_steer_delta_deg=self._sim_max_joint_odom_steer_delta_deg,
        )
        self._client.start()

        self.get_logger().info(
            "controller serial port selected "
            f"(reason={serial_selection.reason}, requested={requested_serial_port}, "
            f"resolved={self._serial_port})"
        )

        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self._on_cmd_vel_final, 10)
        self._status_pub = self.create_publisher(String, "/controller/status", 10)
        self._telemetry_pub = self.create_publisher(String, "/controller/telemetry", 10)
        self._drive_telemetry_pub = self.create_publisher(
            DriveTelemetry, "/controller/drive_telemetry", 10
        )
        self._battery_state_pub = self.create_publisher(
            BatteryState, self._battery_state_topic, 10
        )
        self._battery_guard_pub = self.create_publisher(
            BatteryMissionGuard, self._battery_guard_topic, 10
        )

        self.create_timer(1.0 / self._control_hz, self._control_tick)
        self.create_timer(1.0 / self._telemetry_pub_hz, self._telemetry_tick)

        self.get_logger().info(
            "controller_server ready "
            f"(backend={self._transport_backend}, serial={self._serial_port}@{self._serial_baud}, "
            "source=/cmd_vel_final)"
        )

    @staticmethod
    def _abs_float(value: object) -> float:
        try:
            return abs(float(value))
        except (TypeError, ValueError):
            return 0.0

    def _battery_traction_active(self, telemetry, command_state: dict) -> bool:
        drive_enabled = bool(command_state.get("drive_enabled", False))
        brake_applied_pct = (
            int(telemetry.brake_applied_pct)
            if telemetry is not None and getattr(telemetry, "brake_applied_pct", None) is not None
            else int(command_state.get("brake_pct", 0) or 0)
        )
        requested_speed_abs = self._abs_float(command_state.get("speed_mps", 0.0))
        measured_speed_abs = (
            self._abs_float(telemetry.speed_mps)
            if telemetry is not None and telemetry.speed_mps is not None
            else 0.0
        )
        return bool(
            drive_enabled
            and brake_applied_pct < 5
            and requested_speed_abs > 0.35
            and requested_speed_abs > (measured_speed_abs + 0.15)
        )

    def _on_cmd_vel_final(self, msg: CmdVelFinal) -> None:
        command_source = int(getattr(msg, "source", int(CmdVelFinal.SOURCE_UNKNOWN)))
        cmd = command_from_cmd_vel(
            linear_x=msg.twist.linear.x,
            angular_z=msg.twist.angular.z,
            brake_pct=msg.brake_pct,
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
            manual_operational_steering_limit_rad=self._manual_operational_steering_limit_rad,
            command_source=command_source,
        )
        self._auto_cmd = cmd
        self._auto_stamp_s = time.monotonic()
        if cmd.steer_saturated and (not self._last_steer_saturated):
            self.get_logger().warning(
                "Ackermann steer saturated "
                f"(linear_x={cmd.requested_linear_x_mps:.3f} m/s, "
                f"angular_z={cmd.requested_angular_z_rps:.3f} rad/s, "
                f"requested_curvature={cmd.requested_curvature_inv_m:.3f} 1/m, "
                f"requested_steer={math.degrees(cmd.requested_steer_rad):.2f} deg, "
                f"limit={math.degrees(cmd.steering_limit_used_rad):.2f} deg)"
            )
        elif (not cmd.steer_saturated) and self._last_steer_saturated:
            self.get_logger().info("Ackermann steer saturation cleared")
        self._last_steer_saturated = bool(cmd.steer_saturated)
        self.get_logger().info(
            "cmd_vel_final rx "
            f"linear_x={msg.twist.linear.x:.3f} angular_z={msg.twist.angular.z:.3f} "
            f"brake_pct={int(msg.brake_pct)} source={command_source} -> "
            f"drive={int(cmd.drive_enabled)} estop={int(cmd.estop)} "
            f"speed_mps={cmd.speed_mps:.3f} steer_pct={cmd.steer_pct} "
            f"steer_deg={math.degrees(cmd.applied_steer_rad):.2f} "
            f"curvature={cmd.applied_curvature_inv_m:.3f} brake_pct={cmd.brake_pct}"
        )

    def _apply_to_controller(self, cmd: DesiredCommand) -> None:
        self._client.apply_command(cmd)

    def _control_tick(self) -> None:
        now = time.monotonic()
        auto_cmd = self._auto_cmd
        auto_stamp_s = self._auto_stamp_s

        result = select_effective_command(
            now_s=now,
            auto_cmd=auto_cmd,
            auto_stamp_s=auto_stamp_s,
            auto_timeout_s=self._auto_timeout_s,
        )
        cmd = result.command

        if cmd.estop:
            cmd = DesiredCommand(
                drive_enabled=False,
                estop=True,
                speed_mps=0.0,
                steer_pct=0,
                brake_pct=max(cmd.brake_pct, self._estop_brake_pct),
            )
            source = "estop"
        else:
            source = result.source

        self._apply_to_controller(cmd)
        self._last_source = source

        status = {
            "mode": "auto",
            "source": source,
            "fresh": result.fresh,
            "global_estop": False,
            "command": asdict(cmd),
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=True)
        self._status_pub.publish(msg)

    def _telemetry_tick(self) -> None:
        telemetry = self._client.get_latest_telemetry()
        battery_telemetry = self._client.get_latest_battery_telemetry()
        stats = self._client.get_stats()
        command_state = self._client.get_command_state()
        battery_payload = None
        battery_msg = None
        battery_guard_msg = None
        if battery_telemetry is not None:
            battery_link_age_s = max(
                0.0, time.monotonic() - float(battery_telemetry.rx_monotonic_s)
            )
            battery_link_fresh = (
                battery_link_age_s <= self._battery_telemetry_stale_timeout_s
            )
            traction_active = self._battery_traction_active(telemetry, command_state)
            battery_estimate = self._battery_estimator.update(
                battery_telemetry.battery_voltage_v,
                sample_time_s=float(battery_telemetry.rx_monotonic_s),
                traction_active=traction_active,
            )
            battery_percentage = _clamp01(battery_estimate.filtered_percentage)
            battery_state_text = battery_state_label(
                ready=bool(battery_telemetry.ready),
                fresh=bool(battery_telemetry.fresh),
                link_fresh=battery_link_fresh,
                suspect=bool(battery_telemetry.suspect),
                mission_guard_state=battery_estimate.mission_guard_state,
            )
            battery_payload = battery_telemetry.as_dict()
            battery_payload.update(
                {
                    "raw_voltage_v": float(battery_estimate.raw_voltage_v),
                    "filtered_voltage_v": float(battery_estimate.filtered_voltage_v),
                    "loaded_voltage_fast_v": float(battery_estimate.loaded_voltage_fast_v),
                    "loaded_voltage_slow_v": float(battery_estimate.loaded_voltage_slow_v),
                    "recovered_voltage_v": float(battery_estimate.recovered_voltage_v),
                    "soc_voltage_v": float(battery_estimate.soc_voltage_v),
                    "link_age_s": battery_link_age_s,
                    "link_fresh": bool(battery_link_fresh),
                    "percentage": battery_percentage,
                    "raw_percentage": float(battery_estimate.raw_percentage),
                    "filtered_percentage": float(battery_estimate.filtered_percentage),
                    "operator_soc_pct": float(battery_estimate.operator_soc_pct),
                    "traction_active": bool(battery_estimate.traction_active),
                    "state": battery_state_text,
                    "low_voltage_v": self._battery_low_voltage,
                    "critical_voltage_v": self._battery_critical_voltage,
                    "full_voltage_v": self._battery_full_voltage,
                    "empty_voltage_v": self._battery_empty_voltage,
                    "return_home_recommended": bool(
                        battery_estimate.return_home_recommended
                    ),
                    "mission_guard_state": str(battery_state_text),
                    "loaded_low_threshold_v": float(
                        self._battery_estimator.loaded_low_threshold_v
                    ),
                    "recovered_low_threshold_v": float(
                        self._battery_estimator.recovered_low_threshold_v
                    ),
                    "loaded_low_persist_s": float(
                        battery_estimate.loaded_low_persist_s
                    ),
                    "recovered_low_persist_s": float(
                        battery_estimate.recovered_low_persist_s
                    ),
                    "operator_soc_model": str(
                        battery_estimate.operator_model_name
                    ),
                    "mission_guard_model": str(
                        battery_estimate.mission_guard_model_name
                    ),
                    "soc_model": str(battery_estimate.operator_model_name),
                }
            )

            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.present = bool(battery_telemetry.ready)
            battery_msg.voltage = float(battery_estimate.filtered_voltage_v)
            battery_msg.percentage = battery_percentage
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            battery_msg.power_supply_health = (
                BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
                if bool(battery_telemetry.suspect)
                else BatteryState.POWER_SUPPLY_HEALTH_GOOD
            )
            battery_msg.power_supply_technology = (
                BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
            )

            battery_guard_msg = BatteryMissionGuard()
            battery_guard_msg.stamp = self.get_clock().now().to_msg()
            battery_guard_msg.ready = bool(battery_telemetry.ready)
            battery_guard_msg.fresh = bool(
                battery_telemetry.fresh and battery_link_fresh
            )
            battery_guard_msg.traction_active = bool(
                battery_estimate.traction_active
            )
            battery_guard_msg.return_home_recommended = bool(
                battery_estimate.return_home_recommended
            )
            battery_guard_msg.state = str(battery_state_text)
            battery_guard_msg.raw_voltage_v = float(battery_estimate.raw_voltage_v)
            battery_guard_msg.loaded_voltage_fast_v = float(
                battery_estimate.loaded_voltage_fast_v
            )
            battery_guard_msg.loaded_voltage_slow_v = float(
                battery_estimate.loaded_voltage_slow_v
            )
            battery_guard_msg.recovered_voltage_v = float(
                battery_estimate.recovered_voltage_v
            )
            battery_guard_msg.operator_soc_pct = float(
                battery_estimate.operator_soc_pct
            )
            battery_guard_msg.loaded_low_threshold_v = float(
                self._battery_estimator.loaded_low_threshold_v
            )
            battery_guard_msg.recovered_low_threshold_v = float(
                self._battery_estimator.recovered_low_threshold_v
            )
            battery_guard_msg.loaded_low_persist_s = float(
                battery_estimate.loaded_low_persist_s
            )
            battery_guard_msg.recovered_low_persist_s = float(
                battery_estimate.recovered_low_persist_s
            )
            battery_guard_msg.model_name = str(
                battery_estimate.mission_guard_model_name
            )
        payload = {
            "source": self._last_source,
            "telemetry": telemetry.as_dict() if telemetry is not None else None,
            "battery": battery_payload,
            "stats": asdict(stats),
            "requested_auto_command": asdict(self._auto_cmd),
            "ackermann_limits": {
                "wheelbase_m": self._wheelbase_m,
                "steering_limit_deg": math.degrees(self._steering_limit_rad),
                "operational_steering_limit_deg": math.degrees(
                    self._operational_steering_limit_rad
                ),
                "manual_operational_steering_limit_deg": math.degrees(
                    self._manual_operational_steering_limit_rad
                ),
                "effective_steering_limit_deg": math.degrees(
                    self._effective_steering_limit_rad
                ),
                "manual_effective_steering_limit_deg": math.degrees(
                    self._manual_effective_steering_limit_rad
                ),
            },
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self._telemetry_pub.publish(msg)

        drive_msg = DriveTelemetry()
        drive_msg.stamp = self.get_clock().now().to_msg()
        telemetry_age_s = None
        if telemetry is not None:
            telemetry_age_s = max(0.0, time.monotonic() - float(telemetry.rx_monotonic_s))
        drive_msg.ready = bool(telemetry.ready) if telemetry is not None else False
        drive_msg.fresh = (
            telemetry is not None
            and telemetry_age_s is not None
            and telemetry_age_s <= self._telemetry_stale_timeout_s
        )
        drive_msg.drive_enabled = bool(command_state.get("drive_enabled", False))
        drive_msg.estop = bool(telemetry.estop_active) if telemetry is not None else bool(
            command_state.get("estop", False)
        )
        drive_msg.reverse_requested = float(command_state.get("speed_mps", 0.0)) < 0.0
        drive_msg.speed_valid = telemetry is not None and telemetry.speed_mps is not None
        drive_msg.steer_valid = telemetry is not None and telemetry.steer_deg is not None
        drive_msg.control_source = (
            telemetry.control_source.name if telemetry is not None else "NONE"
        )
        drive_msg.speed_mps_measured = (
            float(telemetry.speed_mps) if drive_msg.speed_valid else 0.0
        )
        drive_msg.steer_deg_measured = (
            float(telemetry.steer_deg) if drive_msg.steer_valid else 0.0
        )
        drive_msg.brake_applied_pct = (
            int(telemetry.brake_applied_pct) if telemetry is not None else 0
        )
        self._drive_telemetry_pub.publish(drive_msg)

        if battery_msg is not None:
            self._battery_state_pub.publish(battery_msg)
        if battery_guard_msg is not None:
            self._battery_guard_pub.publish(battery_guard_msg)

    def destroy_node(self) -> bool:
        self._client.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
