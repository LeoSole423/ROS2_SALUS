import math

import pytest

from controller_server.battery_estimator import (
    BatteryEstimator,
    battery_state_label,
    piecewise_soc_from_voltage,
)


def test_piecewise_soc_curve_hits_endpoints_and_named_points() -> None:
    assert piecewise_soc_from_voltage(55.0, 55.0, 62.4) == pytest.approx(0.0)
    assert piecewise_soc_from_voltage(62.4, 55.0, 62.4) == pytest.approx(1.0)
    assert piecewise_soc_from_voltage(59.64, 55.0, 62.4) == pytest.approx(0.5)


def test_piecewise_soc_curve_is_monotonic() -> None:
    voltages = [55.0 + 0.2 * i for i in range(38)]
    percentages = [piecewise_soc_from_voltage(voltage, 55.0, 62.4) for voltage in voltages]

    assert all(next_pct >= pct for pct, next_pct in zip(percentages, percentages[1:]))


def test_battery_estimator_initializes_from_first_sample() -> None:
    estimator = BatteryEstimator(
        empty_voltage_v=55.0,
        full_voltage_v=62.4,
        low_voltage_v=58.0,
        critical_voltage_v=56.0,
    )

    estimate = estimator.update(62.4, sample_time_s=10.0)

    assert estimate.raw_voltage_v == pytest.approx(62.4)
    assert estimate.filtered_voltage_v == pytest.approx(62.4)
    assert estimate.filtered_percentage == pytest.approx(1.0)
    assert estimate.severity == "OK"


def test_battery_estimator_ema_converges_using_sample_dt() -> None:
    estimator = BatteryEstimator(
        empty_voltage_v=55.0,
        full_voltage_v=62.4,
        low_voltage_v=58.0,
        critical_voltage_v=56.0,
    )
    estimator.update(62.4, sample_time_s=0.0)

    estimate = estimator.update(55.0, sample_time_s=1.0)
    expected_alpha = 1.0 - math.exp(-1.0 / 20.0)
    expected_voltage = 62.4 + expected_alpha * (55.0 - 62.4)

    assert estimate.filtered_voltage_v == pytest.approx(expected_voltage)
    assert estimate.raw_voltage_v == pytest.approx(55.0)
    assert estimate.filtered_voltage_v > estimate.raw_voltage_v


def test_battery_estimator_hysteresis_keeps_low_until_release_voltage() -> None:
    estimator = BatteryEstimator(
        empty_voltage_v=55.0,
        full_voltage_v=62.4,
        low_voltage_v=58.0,
        critical_voltage_v=56.0,
        filter_tau_s=0.01,
    )

    assert estimator.update(57.9, sample_time_s=0.0).severity == "LOW"
    assert estimator.update(58.2, sample_time_s=1.0).severity == "LOW"
    assert estimator.update(58.45, sample_time_s=2.0).severity == "OK"


def test_battery_estimator_hysteresis_keeps_critical_until_release_voltage() -> None:
    estimator = BatteryEstimator(
        empty_voltage_v=55.0,
        full_voltage_v=62.4,
        low_voltage_v=58.0,
        critical_voltage_v=56.0,
        filter_tau_s=0.01,
    )

    assert estimator.update(55.8, sample_time_s=0.0).severity == "CRITICAL"
    assert estimator.update(56.2, sample_time_s=1.0).severity == "CRITICAL"
    assert estimator.update(56.5, sample_time_s=2.0).severity == "LOW"


def test_battery_state_label_prioritizes_link_and_sensor_flags() -> None:
    assert (
        battery_state_label(
            ready=True,
            fresh=True,
            link_fresh=False,
            suspect=False,
            severity="OK",
        )
        == "LINK_STALE"
    )
    assert (
        battery_state_label(
            ready=True,
            fresh=True,
            link_fresh=True,
            suspect=True,
            severity="CRITICAL",
        )
        == "SUSPECT"
    )
    assert (
        battery_state_label(
            ready=True,
            fresh=True,
            link_fresh=True,
            suspect=False,
            severity="CRITICAL",
        )
        == "CRITICAL"
    )
