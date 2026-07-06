import pytest

from controller_server.battery_estimator import (
    BatteryEstimator,
    battery_state_label,
    parse_soc_curve_points,
    piecewise_soc_from_voltage,
)


def test_parse_soc_curve_points_accepts_custom_pairs() -> None:
    curve = parse_soc_curve_points([55.0, 0.0, 57.0, 0.8, 60.0, 1.0])

    assert curve == ((55.0, 0.0), (57.0, 0.8), (60.0, 1.0))


def test_piecewise_soc_curve_hits_named_points() -> None:
    curve = parse_soc_curve_points([55.0, 0.0, 57.0, 0.8, 57.5, 0.9, 60.0, 1.0])

    assert piecewise_soc_from_voltage(55.0, curve) == pytest.approx(0.0)
    assert piecewise_soc_from_voltage(57.0, curve) == pytest.approx(0.8)
    assert piecewise_soc_from_voltage(57.5, curve) == pytest.approx(0.9)
    assert piecewise_soc_from_voltage(60.0, curve) == pytest.approx(1.0)


def test_piecewise_soc_curve_is_monotonic() -> None:
    curve = parse_soc_curve_points([55.0, 0.0, 57.0, 0.8, 57.5, 0.9, 60.0, 1.0])
    voltages = [55.0 + 0.2 * i for i in range(26)]
    percentages = [piecewise_soc_from_voltage(voltage, curve) for voltage in voltages]

    assert all(next_pct >= pct for pct, next_pct in zip(percentages, percentages[1:]))


def test_battery_estimator_initializes_operator_soc_from_first_sample() -> None:
    estimator = BatteryEstimator()

    estimate = estimator.update(60.0, sample_time_s=10.0, traction_active=False)

    assert estimate.raw_voltage_v == pytest.approx(60.0)
    assert estimate.filtered_voltage_v == pytest.approx(60.0)
    assert estimate.operator_soc_pct == pytest.approx(100.0)
    assert estimate.mission_guard_state == "OK"


def test_battery_estimator_detects_traction_active_sag_without_triggering_immediately() -> None:
    estimator = BatteryEstimator()
    estimator.update(60.0, sample_time_s=0.0, traction_active=False)

    estimate = estimator.update(55.2, sample_time_s=5.0, traction_active=True)

    assert estimate.loaded_voltage_fast_v < 60.0
    assert estimate.loaded_voltage_slow_v > estimate.loaded_voltage_fast_v
    assert estimate.return_home_recommended is False
    assert estimate.loaded_low_persist_s == pytest.approx(0.0)


def test_battery_estimator_triggers_after_sustained_loaded_low_voltage() -> None:
    estimator = BatteryEstimator()
    estimator.update(56.0, sample_time_s=0.0, traction_active=False)

    estimate = estimator.update(56.0, sample_time_s=91.0, traction_active=True)

    assert estimate.loaded_voltage_slow_v == pytest.approx(56.0)
    assert estimate.loaded_low_persist_s == pytest.approx(91.0)
    assert estimate.return_home_recommended is True
    assert estimate.mission_guard_state == "LOW_ENERGY_GO_HOME"


def test_battery_estimator_triggers_after_recovered_low_voltage_window() -> None:
    estimator = BatteryEstimator()
    estimator.update(56.8, sample_time_s=0.0, traction_active=False)

    estimate = estimator.update(56.8, sample_time_s=21.0, traction_active=False)

    assert estimate.recovered_voltage_v == pytest.approx(56.8)
    assert estimate.recovered_low_persist_s == pytest.approx(21.0)
    assert estimate.return_home_recommended is True


def test_battery_estimator_clears_guard_after_voltage_recovers_with_hysteresis() -> None:
    estimator = BatteryEstimator()
    estimator.update(56.8, sample_time_s=0.0, traction_active=False)
    tripped = estimator.update(56.8, sample_time_s=21.0, traction_active=False)
    cleared = estimator.update(57.6, sample_time_s=60.0, traction_active=False)

    assert tripped.return_home_recommended is True
    assert tripped.mission_guard_state == "LOW_ENERGY_GO_HOME"
    assert cleared.return_home_recommended is False
    assert cleared.mission_guard_state == "OK"
    assert cleared.loaded_low_persist_s == pytest.approx(0.0)
    assert cleared.recovered_low_persist_s == pytest.approx(0.0)


def test_battery_state_label_prioritizes_sensor_validity() -> None:
    assert (
        battery_state_label(
            ready=False,
            fresh=True,
            link_fresh=True,
            suspect=False,
            mission_guard_state="OK",
        )
        == "UNAVAILABLE"
    )
    assert (
        battery_state_label(
            ready=True,
            fresh=False,
            link_fresh=True,
            suspect=False,
            mission_guard_state="LOW_ENERGY_GO_HOME",
        )
        == "STALE"
    )
    assert (
        battery_state_label(
            ready=True,
            fresh=True,
            link_fresh=True,
            suspect=True,
            mission_guard_state="LOW_ENERGY_GO_HOME",
        )
        == "SUSPECT"
    )
    assert (
        battery_state_label(
            ready=True,
            fresh=True,
            link_fresh=True,
            suspect=False,
            mission_guard_state="LOW_ENERGY_GO_HOME",
        )
        == "LOW_ENERGY_GO_HOME"
    )
