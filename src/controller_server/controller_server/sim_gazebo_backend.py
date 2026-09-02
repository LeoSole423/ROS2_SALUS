from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from controller_server.rpy_esp32_comms.controller import CommandState
from controller_server.rpy_esp32_comms.telemetry import (
    BatteryTelemetry,
    ControlSource,
    Telemetry,
)
from controller_server.rpy_esp32_comms.transport import CommsStats
from controller_server.control_logic import DesiredCommand


DEFAULT_SIM_WHEELBASE_M = 0.94
DEFAULT_SIM_TRACK_WIDTH_M = 0.75
DEFAULT_ODOM_STEER_MIN_SPEED_MPS = 0.05
DEFAULT_MAX_JOINT_ODOM_STEER_DELTA_RAD = math.radians(5.0)
DEFAULT_SIM_BATTERY_ADC_DIVISOR = 25.0


@dataclass(frozen=True, slots=True)
class SimBatteryPreset:
    name: str
    recovered_voltage_v: float
    loaded_voltage_v: float
    traction_active: bool
    ready: bool
    fresh: bool
    suspect: bool


@dataclass(slots=True)
class SimBatteryState:
    recovered_voltage_v: float
    loaded_voltage_v: float
    traction_active_override: Optional[bool]
    ready: bool
    fresh: bool
    suspect: bool
    last_update_monotonic_s: float


SIM_BATTERY_PRESETS = {
    "full": SimBatteryPreset(
        name="full",
        recovered_voltage_v=53.5,
        loaded_voltage_v=53.3,
        traction_active=False,
        ready=True,
        fresh=True,
        suspect=False,
    ),
    "under_load": SimBatteryPreset(
        name="under_load",
        recovered_voltage_v=52.0,
        loaded_voltage_v=50.0,
        traction_active=True,
        ready=True,
        fresh=True,
        suspect=False,
    ),
    "watching": SimBatteryPreset(
        name="watching",
        recovered_voltage_v=47.2,
        loaded_voltage_v=47.2,
        traction_active=False,
        ready=True,
        fresh=True,
        suspect=False,
    ),
    "return_home_rest": SimBatteryPreset(
        name="return_home_rest",
        recovered_voltage_v=46.4,
        loaded_voltage_v=46.4,
        traction_active=False,
        ready=True,
        fresh=True,
        suspect=False,
    ),
    "return_home_load": SimBatteryPreset(
        name="return_home_load",
        recovered_voltage_v=46.4,
        loaded_voltage_v=46.4,
        traction_active=True,
        ready=True,
        fresh=True,
        suspect=False,
    ),
    "stale": SimBatteryPreset(
        name="stale",
        recovered_voltage_v=53.5,
        loaded_voltage_v=53.3,
        traction_active=False,
        ready=True,
        fresh=False,
        suspect=False,
    ),
    "suspect": SimBatteryPreset(
        name="suspect",
        recovered_voltage_v=53.5,
        loaded_voltage_v=53.3,
        traction_active=False,
        ready=True,
        fresh=True,
        suspect=True,
    ),
    "unavailable": SimBatteryPreset(
        name="unavailable",
        recovered_voltage_v=53.5,
        loaded_voltage_v=53.3,
        traction_active=False,
        ready=False,
        fresh=False,
        suspect=False,
    ),
}


@dataclass(slots=True)
class OdomSample:
    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float
    rx_monotonic_s: float


def build_battery_status_flags(
    *,
    ready: bool,
    fresh: bool,
    suspect: bool,
    calibrated: bool = True,
) -> int:
    flags = 0
    if bool(ready):
        flags |= 1 << 0
    if bool(fresh):
        flags |= 1 << 1
    if bool(suspect):
        flags |= 1 << 2
    if bool(calibrated):
        flags |= 1 << 3
    return flags


def translate_command_state_to_gazebo_twist(
    *,
    command_state: CommandState,
    max_steering_angle_rad: float,
    invert_actuation_steer_sign: bool,
) -> tuple[float, float]:
    if (not bool(command_state.drive_enabled)) or bool(command_state.estop):
        return (0.0, 0.0)
    if int(command_state.brake_pct) > 0:
        return (0.0, 0.0)

    steering_angle_rad = (
        float(command_state.steer_pct) / 100.0 * abs(float(max_steering_angle_rad))
    )
    if bool(invert_actuation_steer_sign):
        steering_angle_rad = -steering_angle_rad
    return (float(command_state.speed_mps), steering_angle_rad)


def shortest_angular_distance_rad(from_angle_rad: float, to_angle_rad: float) -> float:
    return math.atan2(
        math.sin(float(to_angle_rad) - float(from_angle_rad)),
        math.cos(float(to_angle_rad) - float(from_angle_rad)),
    )


def steering_angle_from_odom(
    *,
    odom_linear_x_mps: Optional[float],
    odom_angular_z_rps: Optional[float],
    wheelbase_m: float = DEFAULT_SIM_WHEELBASE_M,
    min_speed_mps: float = DEFAULT_ODOM_STEER_MIN_SPEED_MPS,
) -> Optional[float]:
    if odom_linear_x_mps is None or odom_angular_z_rps is None:
        return None

    signed_speed_mps = float(odom_linear_x_mps)
    if abs(signed_speed_mps) < max(0.0, float(min_speed_mps)):
        return None
    if abs(float(wheelbase_m)) < 1.0e-6:
        return None
    return math.atan(float(wheelbase_m) * float(odom_angular_z_rps) / signed_speed_mps)


def steering_angle_from_wheel_angles(
    *,
    left_joint_angle_rad: Optional[float],
    right_joint_angle_rad: Optional[float],
    wheelbase_m: float = DEFAULT_SIM_WHEELBASE_M,
    track_width_m: float = DEFAULT_SIM_TRACK_WIDTH_M,
) -> Optional[float]:
    left = (
        float(left_joint_angle_rad)
        if left_joint_angle_rad is not None and math.isfinite(float(left_joint_angle_rad))
        else None
    )
    right = (
        float(right_joint_angle_rad)
        if right_joint_angle_rad is not None and math.isfinite(float(right_joint_angle_rad))
        else None
    )
    if left is None and right is None:
        return None
    if left is not None and right is not None:
        tan_left = math.tan(left)
        tan_right = math.tan(right)
        denom = tan_left + tan_right
        if abs(denom) < 1.0e-6:
            return 0.5 * (left + right)
        center = math.atan(2.0 * tan_left * tan_right / denom)
        if math.isfinite(center):
            return center
        return 0.5 * (left + right)

    if abs(float(wheelbase_m)) < 1.0e-6:
        return left if left is not None else right

    track_half_m = 0.5 * abs(float(track_width_m))
    if left is not None:
        tan_left = math.tan(left)
        if abs(tan_left) < 1.0e-6:
            return 0.0
        center_radius_m = float(wheelbase_m) / tan_left + track_half_m
        return math.atan(float(wheelbase_m) / center_radius_m)

    tan_right = math.tan(right)
    if abs(tan_right) < 1.0e-6:
        return 0.0
    center_radius_m = float(wheelbase_m) / tan_right - track_half_m
    return math.atan(float(wheelbase_m) / center_radius_m)


def select_physical_steering_angle_rad(
    *,
    left_joint_angle_rad: Optional[float],
    right_joint_angle_rad: Optional[float],
    odom_linear_x_mps: Optional[float],
    odom_angular_z_rps: Optional[float],
    wheelbase_m: float = DEFAULT_SIM_WHEELBASE_M,
    track_width_m: float = DEFAULT_SIM_TRACK_WIDTH_M,
    min_speed_mps: float = DEFAULT_ODOM_STEER_MIN_SPEED_MPS,
    max_joint_odom_delta_rad: float = DEFAULT_MAX_JOINT_ODOM_STEER_DELTA_RAD,
) -> Optional[float]:
    joint_angle_rad = steering_angle_from_wheel_angles(
        left_joint_angle_rad=left_joint_angle_rad,
        right_joint_angle_rad=right_joint_angle_rad,
        wheelbase_m=wheelbase_m,
        track_width_m=track_width_m,
    )
    odom_angle_rad = steering_angle_from_odom(
        odom_linear_x_mps=odom_linear_x_mps,
        odom_angular_z_rps=odom_angular_z_rps,
        wheelbase_m=wheelbase_m,
        min_speed_mps=min_speed_mps,
    )
    if joint_angle_rad is None:
        return odom_angle_rad
    if odom_angle_rad is None:
        return joint_angle_rad
    delta_rad = abs(shortest_angular_distance_rad(joint_angle_rad, odom_angle_rad))
    if delta_rad > max(0.0, float(max_joint_odom_delta_rad)):
        return odom_angle_rad
    return joint_angle_rad


def build_status_flags(
    *,
    ready: bool,
    estop_active: bool,
    pi_fresh: bool,
    control_source: ControlSource,
    failsafe_active: bool = False,
    overspeed_active: bool = False,
) -> int:
    flags = 0
    if bool(ready):
        flags |= 1 << 0
    if bool(estop_active):
        flags |= 1 << 1
    if bool(failsafe_active):
        flags |= 1 << 2
    if bool(pi_fresh):
        flags |= 1 << 3
    flags |= (int(control_source) & 0x03) << 4
    if bool(overspeed_active):
        flags |= 1 << 6
    return flags


def synthesize_telemetry(
    *,
    command_state: CommandState,
    odom_sample: Optional[OdomSample],
    left_joint_angle_rad: Optional[float],
    right_joint_angle_rad: Optional[float],
    invert_measured_steer_sign: bool,
    telemetry_timeout_s: float,
    wheelbase_m: float = DEFAULT_SIM_WHEELBASE_M,
    track_width_m: float = DEFAULT_SIM_TRACK_WIDTH_M,
    max_joint_odom_delta_rad: float = DEFAULT_MAX_JOINT_ODOM_STEER_DELTA_RAD,
) -> Optional[Telemetry]:
    rx_monotonic_s = (
        float(odom_sample.rx_monotonic_s) if odom_sample is not None else time.monotonic()
    )
    ready = odom_sample is not None
    now_s = time.monotonic()
    pi_fresh = bool(ready) and ((now_s - rx_monotonic_s) <= max(0.05, float(telemetry_timeout_s)))

    if bool(command_state.drive_enabled) and bool(pi_fresh):
        control_source = ControlSource.PI
    else:
        control_source = ControlSource.NONE

    speed_mps = None
    steering_angle_rad = None
    if odom_sample is not None:
        speed_mps = math.hypot(
            float(odom_sample.linear_x_mps),
            float(odom_sample.linear_y_mps),
        )
        steering_angle_rad = select_physical_steering_angle_rad(
            left_joint_angle_rad=left_joint_angle_rad,
            right_joint_angle_rad=right_joint_angle_rad,
            odom_linear_x_mps=odom_sample.linear_x_mps,
            odom_angular_z_rps=odom_sample.angular_z_rps,
            wheelbase_m=wheelbase_m,
            track_width_m=track_width_m,
            max_joint_odom_delta_rad=max_joint_odom_delta_rad,
        )

    steer_deg = None
    if steering_angle_rad is not None and math.isfinite(float(steering_angle_rad)):
        steer_deg = math.degrees(float(steering_angle_rad))
        if bool(invert_measured_steer_sign):
            steer_deg = -steer_deg

    brake_applied_pct = int(command_state.brake_pct) if int(command_state.brake_pct) > 0 else 0
    status_flags = build_status_flags(
        ready=ready,
        estop_active=bool(command_state.estop) or brake_applied_pct > 0,
        pi_fresh=pi_fresh,
        control_source=control_source,
    )

    return Telemetry(
        status_flags=status_flags,
        speed_mps=float(speed_mps) if speed_mps is not None else None,
        steer_deg=float(steer_deg) if steer_deg is not None else None,
        brake_applied_pct=brake_applied_pct,
        raw_speed_centi_mps=int(round(float(speed_mps) * 100.0)) if speed_mps is not None else 0,
        raw_steer_centi_deg=int(round(float(steer_deg) * 100.0)) if steer_deg is not None else 0,
        rx_monotonic_s=rx_monotonic_s,
    )


class SimGazeboBackend:
    def __init__(
        self,
        *,
        node: Node,
        tx_hz: float,
        max_speed_mps: float,
        max_reverse_mps: float,
        cmd_vel_topic: str,
        odom_topic: str,
        joint_states_topic: str,
        front_left_steer_joint: str,
        front_right_steer_joint: str,
        wheelbase_m: float,
        track_width_m: float,
        max_steering_angle_rad: float,
        telemetry_timeout_s: float,
        invert_actuation_steer_sign: bool,
        invert_measured_steer_sign: bool,
        max_joint_odom_delta_rad: float = DEFAULT_MAX_JOINT_ODOM_STEER_DELTA_RAD,
    ) -> None:
        self._node = node
        self._tx_period_s = 1.0 / max(1.0, float(tx_hz))
        self._telemetry_timeout_s = max(0.05, float(telemetry_timeout_s))
        self._front_left_steer_joint = str(front_left_steer_joint)
        self._front_right_steer_joint = str(front_right_steer_joint)
        self._wheelbase_m = max(1.0e-6, float(wheelbase_m))
        self._track_width_m = max(0.0, float(track_width_m))
        self._max_steering_angle_rad = abs(float(max_steering_angle_rad))
        self._invert_actuation_steer_sign = bool(invert_actuation_steer_sign)
        self._invert_measured_steer_sign = bool(invert_measured_steer_sign)
        self._max_joint_odom_delta_rad = max(0.0, float(max_joint_odom_delta_rad))

        self._state_lock = threading.Lock()
        self._state = CommandState(
            max_speed_mps=float(max_speed_mps),
            max_reverse_mps=float(max_reverse_mps),
        )
        self._latest_odom: Optional[OdomSample] = None
        self._latest_left_joint_angle_rad: Optional[float] = None
        self._latest_right_joint_angle_rad: Optional[float] = None
        now_s = time.monotonic()
        self._battery_state = SimBatteryState(
            recovered_voltage_v=53.5,
            loaded_voltage_v=53.3,
            traction_active_override=None,
            ready=True,
            fresh=True,
            suspect=False,
            last_update_monotonic_s=now_s,
        )
        self._stats = CommsStats()
        self._running = False

        self._pub = self._node.create_publisher(Twist, str(cmd_vel_topic), 10)
        self._node.create_subscription(Odometry, str(odom_topic), self._on_odom, 10)
        self._node.create_subscription(
            JointState, str(joint_states_topic), self._on_joint_states, 10
        )
        self._timer = self._node.create_timer(self._tx_period_s, self._publish_current_command)

    def start(self) -> None:
        with self._state_lock:
            self._state.safe_reset()
        self._running = True

    def stop(self) -> None:
        with self._state_lock:
            self._state.safe_reset()
            self._running = False
        self._pub.publish(Twist())
        self._pub.publish(Twist())

    def apply_command(self, cmd: DesiredCommand) -> None:
        with self._state_lock:
            self._state.set_drive_enabled(bool(cmd.drive_enabled))
            self._state.set_estop(bool(cmd.estop))
            self._state.set_speed_mps(float(cmd.speed_mps))
            self._state.set_steer_pct(int(cmd.steer_pct))
            self._state.set_brake_pct(int(cmd.brake_pct))

    def get_latest_telemetry(self) -> Optional[Telemetry]:
        with self._state_lock:
            command_state = CommandState(**self._state.to_dict())
            odom_sample = self._latest_odom
            left_joint_angle_rad = self._latest_left_joint_angle_rad
            right_joint_angle_rad = self._latest_right_joint_angle_rad
        return synthesize_telemetry(
            command_state=command_state,
            odom_sample=odom_sample,
            left_joint_angle_rad=left_joint_angle_rad,
            right_joint_angle_rad=right_joint_angle_rad,
            invert_measured_steer_sign=self._invert_measured_steer_sign,
            telemetry_timeout_s=self._telemetry_timeout_s,
            wheelbase_m=self._wheelbase_m,
            track_width_m=self._track_width_m,
            max_joint_odom_delta_rad=self._max_joint_odom_delta_rad,
        )

    def get_latest_battery_telemetry(self):
        with self._state_lock:
            battery_state = SimBatteryState(
                recovered_voltage_v=float(self._battery_state.recovered_voltage_v),
                loaded_voltage_v=float(self._battery_state.loaded_voltage_v),
                traction_active_override=self._battery_state.traction_active_override,
                ready=bool(self._battery_state.ready),
                fresh=bool(self._battery_state.fresh),
                suspect=bool(self._battery_state.suspect),
                last_update_monotonic_s=float(self._battery_state.last_update_monotonic_s),
            )
            command_state = CommandState(**self._state.to_dict())

        traction_active = self._resolve_battery_traction_active(
            command_state=command_state,
            override=battery_state.traction_active_override,
        )
        battery_voltage_v = (
            float(battery_state.loaded_voltage_v)
            if traction_active
            else float(battery_state.recovered_voltage_v)
        )
        adc_pin_voltage_v = battery_voltage_v / DEFAULT_SIM_BATTERY_ADC_DIVISOR
        now_s = time.monotonic()
        if battery_state.fresh:
            sample_age_s = 0.0
            rx_monotonic_s = now_s
        else:
            sample_age_s = max(0.0, now_s - float(battery_state.last_update_monotonic_s))
            rx_monotonic_s = float(battery_state.last_update_monotonic_s)
        return BatteryTelemetry(
            status_flags=build_battery_status_flags(
                ready=battery_state.ready,
                fresh=battery_state.fresh,
                suspect=battery_state.suspect,
            ),
            battery_voltage_v=float(battery_voltage_v),
            adc_pin_voltage_v=float(adc_pin_voltage_v),
            sample_age_s=float(sample_age_s),
            raw_battery_centi_volts=int(round(float(battery_voltage_v) * 100.0)),
            raw_adc_pin_mv=int(round(float(adc_pin_voltage_v) * 1000.0)),
            raw_sample_age_ds=int(round(float(sample_age_s) * 10.0)),
            rx_monotonic_s=float(rx_monotonic_s),
        )

    def set_sim_battery_preset(self, preset_name: str) -> SimBatteryPreset:
        normalized_name = str(preset_name).strip().lower()
        preset = SIM_BATTERY_PRESETS.get(normalized_name)
        if preset is None:
            raise ValueError(
                "Unknown sim battery preset "
                f"'{preset_name}'. Available presets: {', '.join(sorted(SIM_BATTERY_PRESETS))}"
            )
        self.set_sim_battery_state(
            recovered_voltage_v=preset.recovered_voltage_v,
            loaded_voltage_v=preset.loaded_voltage_v,
            traction_active_override=preset.traction_active,
            ready=preset.ready,
            fresh=preset.fresh,
            suspect=preset.suspect,
        )
        return preset

    def set_sim_battery_state(
        self,
        *,
        recovered_voltage_v: float,
        loaded_voltage_v: float,
        traction_active_override: Optional[bool],
        ready: bool,
        fresh: bool,
        suspect: bool,
    ) -> SimBatteryState:
        recovered_voltage = float(recovered_voltage_v)
        loaded_voltage = float(loaded_voltage_v)
        if not math.isfinite(recovered_voltage) or not math.isfinite(loaded_voltage):
            raise ValueError("Sim battery voltages must be finite")
        if recovered_voltage <= 0.0 or loaded_voltage <= 0.0:
            raise ValueError("Sim battery voltages must be positive")
        now_s = time.monotonic()
        next_state = SimBatteryState(
            recovered_voltage_v=recovered_voltage,
            loaded_voltage_v=loaded_voltage,
            traction_active_override=(
                bool(traction_active_override)
                if traction_active_override is not None
                else None
            ),
            ready=bool(ready),
            fresh=bool(fresh),
            suspect=bool(suspect),
            last_update_monotonic_s=now_s,
        )
        with self._state_lock:
            self._battery_state = next_state
            self._stats.rx_battery_frames_ok += 1
        return next_state

    def get_command_state(self) -> dict:
        with self._state_lock:
            return self._state.to_dict()

    def get_stats(self) -> CommsStats:
        with self._state_lock:
            return CommsStats(
                tx_frames_ok=self._stats.tx_frames_ok,
                tx_errors=self._stats.tx_errors,
                rx_frames_ok=self._stats.rx_frames_ok,
                rx_control_frames_ok=self._stats.rx_control_frames_ok,
                rx_battery_frames_ok=self._stats.rx_battery_frames_ok,
                rx_crc_errors=self._stats.rx_crc_errors,
                rx_parse_drops=self._stats.rx_parse_drops,
            )

    def _on_odom(self, msg: Odometry) -> None:
        sample = OdomSample(
            linear_x_mps=float(msg.twist.twist.linear.x),
            linear_y_mps=float(msg.twist.twist.linear.y),
            angular_z_rps=float(msg.twist.twist.angular.z),
            rx_monotonic_s=time.monotonic(),
        )
        with self._state_lock:
            self._latest_odom = sample
            self._stats.rx_frames_ok += 1

    def _on_joint_states(self, msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))
        left_joint_angle_rad = positions_by_name.get(self._front_left_steer_joint)
        right_joint_angle_rad = positions_by_name.get(self._front_right_steer_joint)
        if left_joint_angle_rad is None and right_joint_angle_rad is None:
            return
        with self._state_lock:
            if left_joint_angle_rad is not None:
                self._latest_left_joint_angle_rad = float(left_joint_angle_rad)
            if right_joint_angle_rad is not None:
                self._latest_right_joint_angle_rad = float(right_joint_angle_rad)

    def _publish_current_command(self) -> None:
        with self._state_lock:
            if not self._running:
                return
            command_state = CommandState(**self._state.to_dict())
        out = Twist()
        out.linear.x, out.angular.z = translate_command_state_to_gazebo_twist(
            command_state=command_state,
            max_steering_angle_rad=self._max_steering_angle_rad,
            invert_actuation_steer_sign=self._invert_actuation_steer_sign,
        )
        self._pub.publish(out)
        with self._state_lock:
            self._stats.tx_frames_ok += 1

    @staticmethod
    def _resolve_battery_traction_active(
        *,
        command_state: CommandState,
        override: Optional[bool],
    ) -> bool:
        if override is not None:
            return bool(override)
        return bool(
            command_state.drive_enabled
            and (not command_state.estop)
            and int(command_state.brake_pct) < 5
            and abs(float(command_state.speed_mps)) > 0.35
        )
