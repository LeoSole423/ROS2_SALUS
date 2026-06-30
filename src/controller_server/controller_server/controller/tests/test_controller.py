from controller_server.rpy_esp32_comms.controller import CommandState


def test_state_clamps() -> None:
    state = CommandState(max_speed_mps=4.0, max_reverse_mps=1.3)

    assert state.set_speed_mps(-0.5) == -0.5
    assert state.set_speed_mps(-9.0) == -1.3
    assert state.set_speed_mps(10.0) == 4.0
    assert state.set_steer_pct(-999) == -100
    assert state.set_steer_pct(999) == 100
    assert state.set_brake_pct(-3) == 0
    assert state.set_brake_pct(500) == 100
    state.set_hazard_enabled(True)
    assert state.hazard_enabled is True
    state.set_hazard_enabled(False)
    assert state.hazard_enabled is False


def test_safe_reset() -> None:
    state = CommandState(
        drive_enabled=True,
        estop=True,
        hazard_enabled=True,
        steer_pct=20,
        speed_mps=2.0,
        brake_pct=40,
    )
    state.safe_reset()

    assert state.drive_enabled is False
    assert state.estop is False
    assert state.hazard_enabled is False
    assert state.steer_pct == 0
    assert state.speed_mps == 0.0
    assert state.brake_pct == 0
