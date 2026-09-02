import pytest

from controller_server.battery_estimator import (
    BatteryEstimator,
    battery_state_label,
    parse_soc_curve_points,
    piecewise_soc_from_voltage,
)


def test_default_soc_curve_matches_48v_operator_bands() -> None:
    curve = parse_soc_curve_points(None)

    assert piecewise_soc_from_voltage(44.5, curve) == pytest.approx(0.0)
    assert piecewise_soc_from_voltage(46.5, curve) == pytest.approx(0.15)
    assert piecewise_soc_from_voltage(48.0, curve) == pytest.approx(0.35)
    assert piecewise_soc_from_voltage(50.0, curve) == pytest.approx(0.60)
    assert piecewise_soc_from_voltage(52.0, curve) == pytest.approx(0.85)
    assert piecewise_soc_from_voltage(53.5, curve) == pytest.approx(1.0)


def test_piecewise_soc_curve_is_monotonic() -> None:
    curve = parse_soc_curve_points(None)
    voltages = [44.5 + 0.2 * i for i in range(46)]
    percentages = [piecewise_soc_from_voltage(voltage, curve) for voltage in voltages]

    assert all(next_pct >= pct for pct, next_pct in zip(percentages, percentages[1:]))


def test_estimator_uses_the_calibrated_sample_without_lead_acid_filters() -> None:
    estimator = BatteryEstimator()

    estimate = estimator.update(50.0, sample_time_s=10.0, traction_active=True)

    assert estimate.raw_voltage_v == pytest.approx(50.0)
    assert estimate.filtered_voltage_v == pytest.approx(50.0)
    assert estimate.loaded_voltage_fast_v == pytest.approx(50.0)
    assert estimate.loaded_voltage_slow_v == pytest.approx(50.0)
    assert estimate.recovered_voltage_v == pytest.approx(50.0)
    assert estimate.operator_soc_pct == pytest.approx(60.0)
    assert estimate.mission_guard_state == "OK"


def test_normal_48v_operating_range_never_triggers_return_home() -> None:
    estimator = BatteryEstimator()
    estimator.update(53.5, sample_time_s=0.0, traction_active=False)

    estimate = estimator.update(48.0, sample_time_s=120.0, traction_active=True)

    assert estimate.return_home_recommended is False
    assert estimate.loaded_low_persist_s == pytest.approx(0.0)


def test_return_home_requires_30_continuous_seconds_below_46_5v() -> None:
    estimator = BatteryEstimator()
    estimator.update(46.4, sample_time_s=0.0, traction_active=True)

    before = estimator.update(46.4, sample_time_s=29.0, traction_active=True)
    tripped = estimator.update(46.4, sample_time_s=30.0, traction_active=True)

    assert before.return_home_recommended is False
    assert before.loaded_low_persist_s == pytest.approx(29.0)
    assert tripped.return_home_recommended is True
    assert tripped.mission_guard_state == "LOW_ENERGY_GO_HOME"


def test_short_voltage_recovery_resets_return_home_timer() -> None:
    estimator = BatteryEstimator()
    estimator.update(46.4, sample_time_s=0.0, traction_active=False)
    estimator.update(46.4, sample_time_s=20.0, traction_active=False)

    recovered = estimator.update(46.6, sample_time_s=21.0, traction_active=False)
    later = estimator.update(46.4, sample_time_s=50.0, traction_active=False)

    assert recovered.loaded_low_persist_s == pytest.approx(0.0)
    assert later.return_home_recommended is False
    assert later.loaded_low_persist_s == pytest.approx(29.0)


def test_guard_requires_48v_for_30_seconds_before_clearing() -> None:
    estimator = BatteryEstimator()
    estimator.update(46.4, sample_time_s=0.0, traction_active=True)
    tripped = estimator.update(46.4, sample_time_s=30.0, traction_active=True)
    not_enough = estimator.update(47.9, sample_time_s=90.0, traction_active=False)
    before = estimator.update(48.0, sample_time_s=119.0, traction_active=False)
    cleared = estimator.update(48.0, sample_time_s=120.0, traction_active=False)

    assert tripped.return_home_recommended is True
    assert not_enough.return_home_recommended is True
    assert before.return_home_recommended is True
    assert cleared.return_home_recommended is False
    assert cleared.mission_guard_state == "OK"


@pytest.mark.parametrize(
    ("voltage_v", "expected"),
    [
        (48.0, "OK"),
        (47.0, "LOW"),
        (45.0, "CRITICAL"),
        (44.4, "BELOW_MINIMUM"),
    ],
)
def test_battery_state_label_exposes_48v_voltage_bands(
    voltage_v: float, expected: str
) -> None:
    assert (
        battery_state_label(
            ready=True,
            fresh=True,
            link_fresh=True,
            suspect=False,
            mission_guard_state="OK",
            voltage_v=voltage_v,
            low_voltage_v=47.0,
            critical_voltage_v=45.0,
            minimum_voltage_v=44.5,
        )
        == expected
    )


def test_battery_state_label_prioritizes_sensor_validity() -> None:
    assert (
        battery_state_label(
            ready=False,
            fresh=True,
            link_fresh=True,
            suspect=False,
            mission_guard_state="LOW_ENERGY_GO_HOME",
            voltage_v=44.0,
            low_voltage_v=47.0,
            critical_voltage_v=45.0,
            minimum_voltage_v=44.5,
        )
        == "UNAVAILABLE"
    )
