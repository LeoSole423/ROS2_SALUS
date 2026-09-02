from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple


DEFAULT_EMPTY_VOLTAGE_V = 44.5
DEFAULT_FULL_VOLTAGE_V = 53.5
DEFAULT_RETURN_HOME_VOLTAGE_V = 46.5
DEFAULT_RETURN_HOME_PERSIST_S = 30.0
DEFAULT_GUARD_CLEAR_VOLTAGE_V = 48.0
DEFAULT_GUARD_CLEAR_PERSIST_S = 30.0

OPERATOR_SOC_MODEL_NAME = "pylontech_48v_voltage_estimate_v1"
MISSION_GUARD_MODEL_NAME = "pylontech_48v_voltage_guard_v1"

# LiFePO4 voltage does not give an exact SOC. This curve is operator guidance;
# navigation protection is based on voltage and persistence, not percentage.
DEFAULT_OPERATOR_SOC_CURVE: Tuple[Tuple[float, float], ...] = (
    (44.5, 0.0),
    (46.5, 0.15),
    (48.0, 0.35),
    (50.0, 0.60),
    (52.0, 0.85),
    (53.5, 1.0),
)


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _ensure_monotonic_curve(
    points: Iterable[Tuple[float, float]],
) -> Tuple[Tuple[float, float], ...]:
    curve = tuple((float(voltage_v), clamp01(pct)) for voltage_v, pct in points)
    if len(curve) < 2:
        raise ValueError("SOC curve requires at least two points")
    last_voltage = None
    last_pct = None
    for voltage_v, pct in curve:
        if not math.isfinite(voltage_v):
            raise ValueError("SOC curve voltage points must be finite")
        if last_voltage is not None and voltage_v <= last_voltage:
            raise ValueError("SOC curve voltages must be strictly increasing")
        if last_pct is not None and pct < last_pct:
            raise ValueError("SOC curve percentages must be monotonic non-decreasing")
        last_voltage = voltage_v
        last_pct = pct
    return curve


def parse_soc_curve_points(
    values: Sequence[float] | Sequence[int] | None,
    *,
    default_curve: Sequence[Tuple[float, float]] = DEFAULT_OPERATOR_SOC_CURVE,
) -> Tuple[Tuple[float, float], ...]:
    if not values:
        return _ensure_monotonic_curve(default_curve)
    raw = [float(value) for value in values]
    if len(raw) < 4 or (len(raw) % 2) != 0:
        raise ValueError("battery_soc_curve_points must contain voltage/pct pairs")
    return _ensure_monotonic_curve(
        [(raw[idx], raw[idx + 1]) for idx in range(0, len(raw), 2)]
    )


def piecewise_soc_from_voltage(
    voltage_v: float,
    curve_points: Sequence[Tuple[float, float]] | None = None,
) -> float:
    if not math.isfinite(voltage_v):
        return 0.0
    curve = _ensure_monotonic_curve(curve_points or DEFAULT_OPERATOR_SOC_CURVE)
    if voltage_v <= curve[0][0]:
        return 0.0
    if voltage_v >= curve[-1][0]:
        return 1.0
    for (v0, p0), (v1, p1) in zip(curve, curve[1:]):
        if voltage_v <= v1:
            ratio = (float(voltage_v) - v0) / max(1.0e-6, v1 - v0)
            return clamp01(p0 + ratio * (p1 - p0))
    return 1.0


@dataclass(frozen=True, slots=True)
class BatteryEstimate:
    raw_voltage_v: float
    filtered_voltage_v: float
    loaded_voltage_fast_v: float
    loaded_voltage_slow_v: float
    recovered_voltage_v: float
    soc_voltage_v: float
    raw_percentage: float
    filtered_percentage: float
    operator_soc_pct: float
    traction_active: bool
    return_home_recommended: bool
    mission_guard_state: str
    loaded_low_persist_s: float
    recovered_low_persist_s: float
    operator_model_name: str = OPERATOR_SOC_MODEL_NAME
    mission_guard_model_name: str = MISSION_GUARD_MODEL_NAME


class BatteryEstimator:
    """48 V LiFePO4 state from the ESP32's already-stabilized sample."""

    def __init__(
        self,
        *,
        soc_curve_points: Sequence[Tuple[float, float]] | None = None,
        return_home_voltage_v: float = DEFAULT_RETURN_HOME_VOLTAGE_V,
        return_home_persist_s: float = DEFAULT_RETURN_HOME_PERSIST_S,
        guard_clear_voltage_v: float = DEFAULT_GUARD_CLEAR_VOLTAGE_V,
        guard_clear_persist_s: float = DEFAULT_GUARD_CLEAR_PERSIST_S,
    ) -> None:
        self._soc_curve_points = _ensure_monotonic_curve(
            soc_curve_points or DEFAULT_OPERATOR_SOC_CURVE
        )
        self._return_home_voltage_v = float(return_home_voltage_v)
        self._return_home_persist_s = max(0.0, float(return_home_persist_s))
        self._guard_clear_voltage_v = max(
            self._return_home_voltage_v, float(guard_clear_voltage_v)
        )
        self._guard_clear_persist_s = max(0.0, float(guard_clear_persist_s))
        self._last_sample_time_s: float | None = None
        self._low_elapsed_s = 0.0
        self._clear_elapsed_s = 0.0
        self._mission_guard_latched = False

    @property
    def loaded_low_threshold_v(self) -> float:
        """Compatibility name for the return-home voltage threshold."""
        return self._return_home_voltage_v

    @property
    def recovered_low_threshold_v(self) -> float:
        """Compatibility name for the guard-clear voltage threshold."""
        return self._guard_clear_voltage_v

    @property
    def loaded_low_persist_required_s(self) -> float:
        return self._return_home_persist_s

    @property
    def recovered_low_persist_required_s(self) -> float:
        return self._guard_clear_persist_s

    def update(
        self,
        raw_voltage_v: float,
        *,
        sample_time_s: float,
        traction_active: bool,
    ) -> BatteryEstimate:
        voltage_v = float(raw_voltage_v)
        dt_s = (
            0.0
            if self._last_sample_time_s is None
            else max(0.0, float(sample_time_s) - self._last_sample_time_s)
        )

        if not self._mission_guard_latched:
            self._clear_elapsed_s = 0.0
            if voltage_v <= self._return_home_voltage_v:
                self._low_elapsed_s += dt_s
            else:
                self._low_elapsed_s = 0.0
            if self._low_elapsed_s >= self._return_home_persist_s:
                self._mission_guard_latched = True
        else:
            if voltage_v >= self._guard_clear_voltage_v:
                self._clear_elapsed_s += dt_s
            else:
                self._clear_elapsed_s = 0.0
            if self._clear_elapsed_s >= self._guard_clear_persist_s:
                self._mission_guard_latched = False
                self._low_elapsed_s = 0.0
                self._clear_elapsed_s = 0.0

        self._last_sample_time_s = float(sample_time_s)
        percentage = piecewise_soc_from_voltage(voltage_v, self._soc_curve_points)
        mission_guard_state = (
            "LOW_ENERGY_GO_HOME" if self._mission_guard_latched else "OK"
        )

        return BatteryEstimate(
            raw_voltage_v=voltage_v,
            filtered_voltage_v=voltage_v,
            # Legacy fields mirror the calibrated ESP32 sample.
            loaded_voltage_fast_v=voltage_v,
            loaded_voltage_slow_v=voltage_v,
            recovered_voltage_v=voltage_v,
            soc_voltage_v=voltage_v,
            raw_percentage=percentage,
            filtered_percentage=percentage,
            operator_soc_pct=100.0 * percentage,
            traction_active=bool(traction_active),
            return_home_recommended=bool(self._mission_guard_latched),
            mission_guard_state=mission_guard_state,
            loaded_low_persist_s=self._low_elapsed_s,
            recovered_low_persist_s=self._clear_elapsed_s,
        )


def battery_state_label(
    *,
    ready: bool,
    fresh: bool,
    link_fresh: bool,
    suspect: bool,
    mission_guard_state: str,
    voltage_v: float,
    low_voltage_v: float,
    critical_voltage_v: float,
    minimum_voltage_v: float,
) -> str:
    if not ready:
        return "UNAVAILABLE"
    if not link_fresh or not fresh:
        return "STALE"
    if suspect:
        return "SUSPECT"
    if mission_guard_state == "LOW_ENERGY_GO_HOME":
        return "LOW_ENERGY_GO_HOME"
    if voltage_v < minimum_voltage_v:
        return "BELOW_MINIMUM"
    if voltage_v <= critical_voltage_v:
        return "CRITICAL"
    if voltage_v <= low_voltage_v:
        return "LOW"
    return "OK"
