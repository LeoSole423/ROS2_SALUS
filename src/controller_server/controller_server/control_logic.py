from __future__ import annotations

import math
from dataclasses import dataclass

COMMAND_SOURCE_UNKNOWN = 0
COMMAND_SOURCE_AUTO = 1
COMMAND_SOURCE_MANUAL = 2
COMMAND_SOURCE_SAFETY = 3


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def effective_steering_limit_rad(
    *,
    steering_limit_rad: float,
    operational_steering_limit_rad: float | None = None,
    manual_operational_steering_limit_rad: float | None = None,
    command_source: int = COMMAND_SOURCE_UNKNOWN,
) -> float:
    steering_limit = max(1.0e-6, abs(float(steering_limit_rad)))
    if int(command_source) == COMMAND_SOURCE_MANUAL:
        configured_limit = manual_operational_steering_limit_rad
    else:
        configured_limit = operational_steering_limit_rad
    if configured_limit is None:
        return steering_limit
    return min(steering_limit, max(1.0e-6, abs(float(configured_limit))))


@dataclass(slots=True)
class DesiredCommand:
    drive_enabled: bool = False
    estop: bool = False
    speed_mps: float = 0.0
    steer_pct: int = 0
    brake_pct: int = 0
    requested_linear_x_mps: float = 0.0
    requested_angular_z_rps: float = 0.0
    steering_reference_speed_mps: float = 0.0
    requested_curvature_inv_m: float = 0.0
    applied_curvature_inv_m: float = 0.0
    requested_steer_rad: float = 0.0
    applied_steer_rad: float = 0.0
    steering_limit_used_rad: float = 0.0
    command_source: int = COMMAND_SOURCE_UNKNOWN
    steer_saturated: bool = False
    used_steering_speed_fallback: bool = False
    speed_limited: bool = False
    min_speed_enforced: bool = False


@dataclass(slots=True)
class ArbitrationResult:
    command: DesiredCommand
    source: str
    fresh: bool


def safe_command() -> DesiredCommand:
    return DesiredCommand(
        drive_enabled=False,
        estop=False,
        speed_mps=0.0,
        steer_pct=0,
        brake_pct=0,
    )


def _compute_ackermann_steer_command(
    *,
    linear_x: float,
    angular_z: float,
    vx_deadband_mps: float,
    vx_min_effective_mps: float,
    wheelbase_m: float,
    steering_limit_rad: float,
    operational_steering_limit_rad: float | None = None,
    manual_operational_steering_limit_rad: float | None = None,
    command_source: int = COMMAND_SOURCE_UNKNOWN,
) -> tuple[int, float, float, float, float, float, bool, bool]:
    wheelbase = max(1.0e-6, abs(float(wheelbase_m)))
    steering_limit = max(1.0e-6, abs(float(steering_limit_rad)))
    applied_limit = effective_steering_limit_rad(
        steering_limit_rad=steering_limit,
        operational_steering_limit_rad=operational_steering_limit_rad,
        manual_operational_steering_limit_rad=manual_operational_steering_limit_rad,
        command_source=command_source,
    )
    deadband = max(0.0, float(vx_deadband_mps))
    min_effective = max(0.0, float(vx_min_effective_mps))
    requested_linear = float(linear_x)
    requested_angular = float(angular_z)

    used_speed_fallback = False
    reference_speed = requested_linear
    if abs(reference_speed) <= max(1.0e-6, deadband):
        if abs(requested_angular) <= 1.0e-6:
            return (0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
        reference_speed = max(min_effective, 0.1)
        used_speed_fallback = True

    requested_curvature = requested_angular / reference_speed
    requested_steer_rad = math.atan(wheelbase * requested_curvature)
    applied_steer_rad = clamp(requested_steer_rad, -applied_limit, applied_limit)
    applied_curvature = math.tan(applied_steer_rad) / wheelbase
    steer_saturated = abs(requested_steer_rad - applied_steer_rad) > 1.0e-6
    steer_pct = int(round((applied_steer_rad / steering_limit) * 100.0))
    steer_pct = int(clamp(float(steer_pct), -100.0, 100.0))

    return (
        steer_pct,
        reference_speed,
        requested_curvature,
        applied_curvature,
        requested_steer_rad,
        applied_steer_rad,
        steer_saturated,
        used_speed_fallback,
    )


def command_from_cmd_vel(
    linear_x: float,
    angular_z: float,
    brake_pct: int,
    max_speed_mps: float,
    max_reverse_mps: float,
    vx_deadband_mps: float,
    vx_min_effective_mps: float,
    max_abs_angular_z: float,
    wheelbase_m: float,
    steering_limit_rad: float,
    invert_steer: bool,
    auto_drive_enabled: bool,
    reverse_brake_pct: int,
    operational_steering_limit_rad: float | None = None,
    manual_operational_steering_limit_rad: float | None = None,
    command_source: int = COMMAND_SOURCE_UNKNOWN,
) -> DesiredCommand:
    max_speed = max(0.0, float(max_speed_mps))
    max_reverse = max(0.0, float(max_reverse_mps))
    deadband = max(0.0, float(vx_deadband_mps))
    min_effective = clamp(float(vx_min_effective_mps), 0.0, max_speed)

    linear = float(linear_x)
    angular = float(angular_z)
    _ = float(max_abs_angular_z)
    speed = 0.0
    speed_limited = False
    min_speed_enforced = False
    if linear > 0.0:
        requested_speed = clamp(linear, 0.0, max_speed)
        speed_limited = abs(requested_speed - linear) > 1.0e-6
        speed = requested_speed
        if speed < deadband:
            speed = 0.0
        elif speed < min_effective:
            speed = min_effective
            min_speed_enforced = requested_speed > 1.0e-6
    elif linear < 0.0:
        reverse_speed = clamp(abs(linear), 0.0, max_reverse)
        speed_limited = abs(reverse_speed - abs(linear)) > 1.0e-6
        if reverse_speed < deadband:
            speed = 0.0
        else:
            speed = -reverse_speed

    (
        steer,
        steering_reference_speed_mps,
        requested_curvature,
        applied_curvature,
        requested_steer_rad,
        applied_steer_rad,
        steer_saturated,
        used_steering_speed_fallback,
    ) = _compute_ackermann_steer_command(
        linear_x=linear,
        angular_z=angular,
        vx_deadband_mps=deadband,
        vx_min_effective_mps=min_effective,
        wheelbase_m=wheelbase_m,
        steering_limit_rad=steering_limit_rad,
        operational_steering_limit_rad=operational_steering_limit_rad,
        manual_operational_steering_limit_rad=manual_operational_steering_limit_rad,
        command_source=command_source,
    )
    if bool(invert_steer):
        steer = -steer

    brake = int(clamp(float(brake_pct), 0.0, 100.0))
    estop = brake > 0
    if estop:
        speed = 0.0
        steer = 0

    return DesiredCommand(
        drive_enabled=bool(auto_drive_enabled),
        estop=estop,
        speed_mps=speed,
        steer_pct=steer,
        brake_pct=brake,
        requested_linear_x_mps=linear,
        requested_angular_z_rps=angular,
        steering_reference_speed_mps=steering_reference_speed_mps,
        requested_curvature_inv_m=requested_curvature,
        applied_curvature_inv_m=applied_curvature,
        requested_steer_rad=requested_steer_rad,
        applied_steer_rad=applied_steer_rad,
        steering_limit_used_rad=effective_steering_limit_rad(
            steering_limit_rad=steering_limit_rad,
            operational_steering_limit_rad=operational_steering_limit_rad,
            manual_operational_steering_limit_rad=manual_operational_steering_limit_rad,
            command_source=command_source,
        ),
        command_source=int(command_source),
        steer_saturated=steer_saturated,
        used_steering_speed_fallback=used_steering_speed_fallback,
        speed_limited=speed_limited,
        min_speed_enforced=min_speed_enforced,
    )


def select_effective_command(
    now_s: float,
    auto_cmd: DesiredCommand,
    auto_stamp_s: float,
    auto_timeout_s: float,
) -> ArbitrationResult:
    auto_fresh = (now_s - auto_stamp_s) <= max(0.0, auto_timeout_s)

    if auto_fresh:
        return ArbitrationResult(command=auto_cmd, source="auto", fresh=True)
    return ArbitrationResult(command=safe_command(), source="auto_timeout", fresh=False)
