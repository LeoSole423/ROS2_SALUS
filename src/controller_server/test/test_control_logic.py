import math

import pytest

from controller_server.control_logic import (
    COMMAND_SOURCE_AUTO,
    COMMAND_SOURCE_MANUAL,
    COMMAND_SOURCE_UNKNOWN,
    DesiredCommand,
    command_from_cmd_vel,
    select_effective_command,
)

ACKERMANN_KWARGS = {
    "wheelbase_m": 0.94,
    "steering_limit_rad": 0.5235987756,
    "operational_steering_limit_rad": 0.3141592654,
    "manual_operational_steering_limit_rad": 0.5235987756,
}


def test_command_from_cmd_vel_clamps_and_scales() -> None:
    cmd = command_from_cmd_vel(
        linear_x=9.0,
        angular_z=2.0,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=30,
    )
    assert cmd.drive_enabled is True
    assert cmd.speed_mps == 4.0
    assert cmd.steer_pct == 39
    assert cmd.brake_pct == 0
    assert cmd.estop is False
    assert cmd.speed_limited is True
    assert cmd.steer_saturated is False


def test_command_from_cmd_vel_negative_speed_maps_to_reverse() -> None:
    cmd = command_from_cmd_vel(
        linear_x=-0.5,
        angular_z=0.0,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == -0.5
    assert cmd.brake_pct == 0
    assert cmd.requested_curvature_inv_m == 0.0


def test_command_from_cmd_vel_negative_speed_is_clamped_by_max_reverse() -> None:
    cmd = command_from_cmd_vel(
        linear_x=-9.0,
        angular_z=0.0,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == -1.3
    assert cmd.brake_pct == 0
    assert cmd.speed_limited is True


def test_command_from_cmd_vel_zero_speed_does_not_brake_without_request() -> None:
    cmd = command_from_cmd_vel(
        linear_x=0.0,
        angular_z=0.0,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 0.0
    assert cmd.brake_pct == 0
    assert cmd.estop is False
    assert cmd.steer_pct == 0


def test_command_from_cmd_vel_brake_pct_triggers_estop() -> None:
    cmd = command_from_cmd_vel(
        linear_x=1.0,
        angular_z=0.4,
        brake_pct=30,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 0.0
    assert cmd.steer_pct == 0
    assert cmd.brake_pct == 30
    assert cmd.estop is True


def test_command_from_cmd_vel_brake_pct_is_clamped_low() -> None:
    cmd = command_from_cmd_vel(
        linear_x=1.0,
        angular_z=0.0,
        brake_pct=-5,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.estop is False
    assert cmd.brake_pct == 0
    assert cmd.speed_mps == 1.0


def test_command_from_cmd_vel_brake_pct_is_clamped_high() -> None:
    cmd = command_from_cmd_vel(
        linear_x=1.0,
        angular_z=0.0,
        brake_pct=140,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.estop is True
    assert cmd.brake_pct == 100
    assert cmd.speed_mps == 0.0


def test_command_from_cmd_vel_invert_steer() -> None:
    cmd = command_from_cmd_vel(
        linear_x=1.0,
        angular_z=0.4,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=True,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.steer_pct == -60
    assert math.degrees(cmd.applied_steer_rad) == pytest.approx(18.0)


def test_command_from_cmd_vel_below_deadband_maps_to_zero() -> None:
    cmd = command_from_cmd_vel(
        linear_x=0.05,
        angular_z=0.0,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 0.0


def test_command_from_cmd_vel_between_deadband_and_min_maps_to_min() -> None:
    cmd = command_from_cmd_vel(
        linear_x=0.30,
        angular_z=0.0,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 0.75
    assert cmd.min_speed_enforced is True


def test_command_from_cmd_vel_clamps_to_operational_limit_when_min_is_applied() -> None:
    cmd = command_from_cmd_vel(
        linear_x=0.20,
        angular_z=0.10,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.01,
        vx_min_effective_mps=0.50,
        max_abs_angular_z=0.4,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 0.50
    assert cmd.steer_pct == 60
    assert cmd.requested_curvature_inv_m == pytest.approx(0.5)
    assert cmd.applied_curvature_inv_m == pytest.approx(
        math.tan(math.radians(18.0)) / 0.94,
        abs=1.0e-6,
    )
    assert cmd.steer_saturated is True


def test_command_from_cmd_vel_above_min_keeps_value() -> None:
    cmd = command_from_cmd_vel(
        linear_x=1.20,
        angular_z=0.0,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 1.20


def test_command_from_cmd_vel_min_effective_is_clamped_by_max_speed() -> None:
    cmd = command_from_cmd_vel(
        linear_x=0.30,
        angular_z=0.0,
        brake_pct=0,
        max_speed_mps=0.60,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.75,
        max_abs_angular_z=0.8,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 0.60


def test_command_from_cmd_vel_keeps_legacy_angular_at_patrol_speed_near_18deg() -> None:
    cmd = command_from_cmd_vel(
        linear_x=1.2,
        angular_z=0.4,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.50,
        max_abs_angular_z=0.4,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 1.2
    assert cmd.steer_pct == 58
    assert math.degrees(cmd.applied_steer_rad) == pytest.approx(17.40, abs=0.01)
    assert cmd.steer_saturated is False


def test_command_from_cmd_vel_saturates_at_operational_limit() -> None:
    cmd = command_from_cmd_vel(
        linear_x=1.0,
        angular_z=2.0,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.50,
        max_abs_angular_z=0.4,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 1.0
    assert cmd.steer_pct == 60
    assert cmd.steer_saturated is True
    assert math.degrees(cmd.requested_steer_rad) > 30.0
    assert math.degrees(cmd.applied_steer_rad) == pytest.approx(18.0)


def test_manual_command_can_use_physical_steering_limit() -> None:
    angular_for_30_deg_at_1mps = math.tan(math.radians(30.0)) / 0.94
    auto_cmd = command_from_cmd_vel(
        linear_x=1.0,
        angular_z=angular_for_30_deg_at_1mps,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.50,
        max_abs_angular_z=0.4,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
        command_source=COMMAND_SOURCE_AUTO,
    )
    manual_cmd = command_from_cmd_vel(
        linear_x=1.0,
        angular_z=angular_for_30_deg_at_1mps,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.50,
        max_abs_angular_z=0.4,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
        command_source=COMMAND_SOURCE_MANUAL,
    )
    unknown_cmd = command_from_cmd_vel(
        linear_x=1.0,
        angular_z=angular_for_30_deg_at_1mps,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.50,
        max_abs_angular_z=0.4,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
        command_source=COMMAND_SOURCE_UNKNOWN,
    )

    assert math.degrees(auto_cmd.applied_steer_rad) == pytest.approx(18.0)
    assert auto_cmd.steer_saturated is True
    assert math.degrees(manual_cmd.applied_steer_rad) == pytest.approx(30.0)
    assert manual_cmd.steer_saturated is False
    assert math.degrees(unknown_cmd.applied_steer_rad) == pytest.approx(18.0)


def test_command_from_cmd_vel_never_exceeds_physical_limit() -> None:
    cmd = command_from_cmd_vel(
        linear_x=1.0,
        angular_z=2.0,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.50,
        max_abs_angular_z=0.4,
        wheelbase_m=0.94,
        steering_limit_rad=0.5235987756,
        operational_steering_limit_rad=1.0,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.steer_pct == 100
    assert cmd.steer_saturated is True
    assert math.degrees(cmd.applied_steer_rad) == pytest.approx(30.0)


def test_command_from_cmd_vel_zero_linear_uses_virtual_speed_for_steer_alignment() -> None:
    cmd = command_from_cmd_vel(
        linear_x=0.0,
        angular_z=0.20,
        brake_pct=0,
        max_speed_mps=4.0,
        max_reverse_mps=1.3,
        vx_deadband_mps=0.10,
        vx_min_effective_mps=0.50,
        max_abs_angular_z=0.4,
        **ACKERMANN_KWARGS,
        invert_steer=False,
        auto_drive_enabled=True,
        reverse_brake_pct=25,
    )
    assert cmd.speed_mps == 0.0
    assert cmd.steer_pct == 60
    assert math.degrees(cmd.applied_steer_rad) == pytest.approx(18.0)
    assert cmd.used_steering_speed_fallback is True


def test_select_effective_command_auto_timeout() -> None:
    now_s = 20.0
    auto_cmd = DesiredCommand(drive_enabled=True, speed_mps=2.0)
    result = select_effective_command(
        now_s=now_s,
        auto_cmd=auto_cmd,
        auto_stamp_s=18.0,
        auto_timeout_s=0.5,
    )
    assert result.source == "auto_timeout"
    assert result.command.drive_enabled is False
    assert result.command.speed_mps == 0.0


def test_select_effective_command_auto_fresh() -> None:
    now_s = 20.0
    auto_cmd = DesiredCommand(drive_enabled=True, speed_mps=2.0)
    result = select_effective_command(
        now_s=now_s,
        auto_cmd=auto_cmd,
        auto_stamp_s=19.9,
        auto_timeout_s=0.5,
    )
    assert result.source == "auto"
    assert result.command.speed_mps == 2.0
