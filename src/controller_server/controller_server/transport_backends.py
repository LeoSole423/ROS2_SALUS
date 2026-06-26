from __future__ import annotations

import math

from rclpy.node import Node

from controller_server.control_logic import DesiredCommand
from controller_server.sim_gazebo_backend import SimGazeboBackend


class UartTransportBackend:
    def __init__(
        self,
        *,
        port: str,
        baud: int,
        tx_hz: float,
        max_speed_mps: float,
        max_reverse_mps: float,
    ) -> None:
        from controller_server.rpy_esp32_comms.transport import CommsClient

        self._client = CommsClient(
            port=port,
            baud=baud,
            tx_hz=tx_hz,
            max_speed_mps=max_speed_mps,
            max_reverse_mps=max_reverse_mps,
        )

    def start(self) -> None:
        self._client.start()

    def stop(self) -> None:
        self._client.stop()

    def apply_command(self, cmd: DesiredCommand) -> None:
        self._client.set_drive_enabled(bool(cmd.drive_enabled))
        self._client.set_estop(bool(cmd.estop))
        self._client.set_speed_mps(float(cmd.speed_mps))
        self._client.set_steer_pct(int(cmd.steer_pct))
        self._client.set_brake_pct(int(cmd.brake_pct))

    def get_latest_telemetry(self):
        return self._client.get_latest_telemetry()

    def get_latest_battery_telemetry(self):
        return self._client.get_latest_battery_telemetry()

    def get_stats(self):
        return self._client.get_stats()

    def get_command_state(self):
        return self._client.get_command_state()


def create_transport_backend(
    *,
    node: Node,
    transport_backend: str,
    serial_port: str,
    serial_baud: int,
    serial_tx_hz: float,
    max_speed_mps: float,
    max_reverse_mps: float,
    sim_cmd_vel_topic: str,
    sim_odom_topic: str,
    sim_joint_states_topic: str,
    sim_front_left_steer_joint: str,
    sim_front_right_steer_joint: str,
    sim_wheelbase_m: float,
    sim_track_width_m: float,
    sim_max_steering_angle_rad: float,
    sim_telemetry_timeout_s: float,
    sim_invert_actuation_steer_sign: bool,
    sim_invert_measured_steer_sign: bool,
    sim_max_joint_odom_steer_delta_deg: float,
):
    backend_name = str(transport_backend).strip().lower()
    if backend_name == "uart":
        return UartTransportBackend(
            port=serial_port,
            baud=serial_baud,
            tx_hz=serial_tx_hz,
            max_speed_mps=max_speed_mps,
            max_reverse_mps=max_reverse_mps,
        )
    if backend_name == "sim_gazebo":
        return SimGazeboBackend(
            node=node,
            tx_hz=serial_tx_hz,
            max_speed_mps=max_speed_mps,
            max_reverse_mps=max_reverse_mps,
            cmd_vel_topic=sim_cmd_vel_topic,
            odom_topic=sim_odom_topic,
            joint_states_topic=sim_joint_states_topic,
            front_left_steer_joint=sim_front_left_steer_joint,
            front_right_steer_joint=sim_front_right_steer_joint,
            wheelbase_m=sim_wheelbase_m,
            track_width_m=sim_track_width_m,
            max_steering_angle_rad=sim_max_steering_angle_rad,
            telemetry_timeout_s=sim_telemetry_timeout_s,
            invert_actuation_steer_sign=sim_invert_actuation_steer_sign,
            invert_measured_steer_sign=sim_invert_measured_steer_sign,
            max_joint_odom_delta_rad=math.radians(sim_max_joint_odom_steer_delta_deg),
        )
    raise ValueError(
        f"Unsupported transport_backend='{transport_backend}'. Expected 'uart' or 'sim_gazebo'."
    )
