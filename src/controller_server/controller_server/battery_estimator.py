from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

DEFAULT_EMPTY_VOLTAGE_V = 55.0
DEFAULT_FULL_VOLTAGE_V = 62.4
DEFAULT_FILTER_TAU_S = 20.0
DEFAULT_LOW_RELEASE_HYSTERESIS_V = 0.4
DEFAULT_CRITICAL_RELEASE_HYSTERESIS_V = 0.4
SOC_MODEL_NAME = "lead_acid_curve_v1"

_DEFAULT_SOC_CURVE_V: Tuple[Tuple[float, float], ...] = (
    (55.00, 0.00),
    (56.41, 0.10),
    (57.29, 0.20),
    (58.07, 0.30),
    (58.91, 0.40),
    (59.64, 0.50),
    (60.26, 0.60),
    (60.78, 0.70),
    (61.20, 0.80),
    (61.82, 0.90),
    (62.40, 1.00),
)


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _rescaled_soc_curve(
    empty_voltage_v: float,
    full_voltage_v: float,
) -> Tuple[Tuple[float, float], ...]:
    span_default = max(1.0e-6, DEFAULT_FULL_VOLTAGE_V - DEFAULT_EMPTY_VOLTAGE_V)
    span = max(1.0e-6, float(full_voltage_v) - float(empty_voltage_v))
    points = []
    for voltage_v, pct in _DEFAULT_SOC_CURVE_V:
        normalized = (float(voltage_v) - DEFAULT_EMPTY_VOLTAGE_V) / span_default
        scaled_voltage = float(empty_voltage_v) + normalized * span
        points.append((scaled_voltage, float(pct)))
    return tuple(points)


def piecewise_soc_from_voltage(
    voltage_v: float,
    empty_voltage_v: float,
    full_voltage_v: float,
) -> float:
    if not math.isfinite(voltage_v):
        return 0.0
    curve = _rescaled_soc_curve(empty_voltage_v, full_voltage_v)
    if voltage_v <= curve[0][0]:
        return 0.0
    if voltage_v >= curve[-1][0]:
        return 1.0

    for (v0, p0), (v1, p1) in zip(curve, curve[1:]):
        if voltage_v <= v1:
            span_v = max(1.0e-6, v1 - v0)
            ratio = (float(voltage_v) - v0) / span_v
            return clamp01(p0 + ratio * (p1 - p0))
    return 1.0


@dataclass(frozen=True, slots=True)
class BatteryEstimate:
    raw_voltage_v: float
    filtered_voltage_v: float
    raw_percentage: float
    filtered_percentage: float
    severity: str
    model_name: str = SOC_MODEL_NAME


class BatteryEstimator:
    def __init__(
        self,
        *,
        empty_voltage_v: float,
        full_voltage_v: float,
        low_voltage_v: float,
        critical_voltage_v: float,
        filter_tau_s: float = DEFAULT_FILTER_TAU_S,
        low_release_hysteresis_v: float = DEFAULT_LOW_RELEASE_HYSTERESIS_V,
        critical_release_hysteresis_v: float = DEFAULT_CRITICAL_RELEASE_HYSTERESIS_V,
    ) -> None:
        self._empty_voltage_v = float(empty_voltage_v)
        self._full_voltage_v = float(full_voltage_v)
        self._low_voltage_v = float(low_voltage_v)
        self._critical_voltage_v = float(critical_voltage_v)
        self._filter_tau_s = max(1.0e-6, float(filter_tau_s))
        self._low_release_hysteresis_v = max(0.0, float(low_release_hysteresis_v))
        self._critical_release_hysteresis_v = max(0.0, float(critical_release_hysteresis_v))
        self._filtered_voltage_v: Optional[float] = None
        self._last_sample_time_s: Optional[float] = None
        self._severity = "OK"

    def update(self, raw_voltage_v: float, *, sample_time_s: float) -> BatteryEstimate:
        raw_voltage_v = float(raw_voltage_v)
        if self._filtered_voltage_v is None or self._last_sample_time_s is None:
            filtered_voltage_v = raw_voltage_v
        else:
            dt_s = max(0.0, float(sample_time_s) - float(self._last_sample_time_s))
            if dt_s <= 0.0:
                filtered_voltage_v = self._filtered_voltage_v
            else:
                alpha = 1.0 - math.exp(-dt_s / self._filter_tau_s)
                filtered_voltage_v = self._filtered_voltage_v + alpha * (
                    raw_voltage_v - self._filtered_voltage_v
                )

        self._filtered_voltage_v = filtered_voltage_v
        self._last_sample_time_s = float(sample_time_s)
        self._severity = self._next_severity(filtered_voltage_v)

        return BatteryEstimate(
            raw_voltage_v=raw_voltage_v,
            filtered_voltage_v=filtered_voltage_v,
            raw_percentage=piecewise_soc_from_voltage(
                raw_voltage_v, self._empty_voltage_v, self._full_voltage_v
            ),
            filtered_percentage=piecewise_soc_from_voltage(
                filtered_voltage_v, self._empty_voltage_v, self._full_voltage_v
            ),
            severity=self._severity,
        )

    def _next_severity(self, filtered_voltage_v: float) -> str:
        current = str(self._severity)
        low_enter_v = self._low_voltage_v
        low_exit_v = self._low_voltage_v + self._low_release_hysteresis_v
        critical_enter_v = self._critical_voltage_v
        critical_exit_v = self._critical_voltage_v + self._critical_release_hysteresis_v

        if current == "CRITICAL":
            if filtered_voltage_v < critical_exit_v:
                return "CRITICAL"
            return "LOW" if filtered_voltage_v <= low_enter_v else "OK"

        if current == "LOW":
            if filtered_voltage_v <= critical_enter_v:
                return "CRITICAL"
            if filtered_voltage_v < low_exit_v:
                return "LOW"
            return "OK"

        if filtered_voltage_v <= critical_enter_v:
            return "CRITICAL"
        if filtered_voltage_v <= low_enter_v:
            return "LOW"
        return "OK"


def battery_state_label(
    *,
    ready: bool,
    fresh: bool,
    link_fresh: bool,
    suspect: bool,
    severity: str,
) -> str:
    if not ready:
        return "UNAVAILABLE"
    if not link_fresh:
        return "LINK_STALE"
    if not fresh:
        return "STALE"
    if suspect:
        return "SUSPECT"
    if severity == "CRITICAL":
        return "CRITICAL"
    if severity == "LOW":
        return "LOW"
    return "OK"
