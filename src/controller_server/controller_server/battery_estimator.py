from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

DEFAULT_EMPTY_VOLTAGE_V = 55.0
DEFAULT_FULL_VOLTAGE_V = 60.0
DEFAULT_SOC_FAST_DISCHARGE_TAU_S = 180.0
DEFAULT_LOADED_FAST_TAU_S = 4.0
DEFAULT_LOADED_SLOW_TAU_S = 45.0
DEFAULT_RECOVERED_TAU_S = 12.0
DEFAULT_LOADED_LOW_THRESHOLD_V = 56.0
DEFAULT_RECOVERED_LOW_THRESHOLD_V = 57.0
DEFAULT_LOADED_LOW_PERSIST_S = 90.0
DEFAULT_RECOVERED_LOW_PERSIST_S = 20.0
DEFAULT_GUARD_CLEAR_HYSTERESIS_V = 0.4

OPERATOR_SOC_MODEL_NAME = "lead_acid_empirical_operator_v1"
MISSION_GUARD_MODEL_NAME = "lead_acid_voltage_guard_v1"

DEFAULT_OPERATOR_SOC_CURVE: Tuple[Tuple[float, float], ...] = (
    (55.0, 0.0),
    (57.0, 0.8),
    (57.5, 0.9),
    (60.0, 1.0),
)


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _ema(previous: float, sample: float, dt_s: float, tau_s: float) -> float:
    if dt_s <= 0.0:
        return float(previous)
    alpha = 1.0 - math.exp(-dt_s / max(1.0e-6, float(tau_s)))
    return float(previous) + alpha * (float(sample) - float(previous))


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
    pairs = [(raw[idx], raw[idx + 1]) for idx in range(0, len(raw), 2)]
    return _ensure_monotonic_curve(pairs)


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
            span_v = max(1.0e-6, v1 - v0)
            ratio = (float(voltage_v) - v0) / span_v
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
    def __init__(
        self,
        *,
        soc_curve_points: Sequence[Tuple[float, float]] | None = None,
        loaded_fast_tau_s: float = DEFAULT_LOADED_FAST_TAU_S,
        loaded_slow_tau_s: float = DEFAULT_LOADED_SLOW_TAU_S,
        recovered_tau_s: float = DEFAULT_RECOVERED_TAU_S,
        soc_fast_discharge_tau_s: float = DEFAULT_SOC_FAST_DISCHARGE_TAU_S,
        loaded_low_threshold_v: float = DEFAULT_LOADED_LOW_THRESHOLD_V,
        recovered_low_threshold_v: float = DEFAULT_RECOVERED_LOW_THRESHOLD_V,
        loaded_low_persist_s: float = DEFAULT_LOADED_LOW_PERSIST_S,
        recovered_low_persist_s: float = DEFAULT_RECOVERED_LOW_PERSIST_S,
        guard_clear_hysteresis_v: float = DEFAULT_GUARD_CLEAR_HYSTERESIS_V,
    ) -> None:
        self._soc_curve_points = _ensure_monotonic_curve(
            soc_curve_points or DEFAULT_OPERATOR_SOC_CURVE
        )
        self._loaded_fast_tau_s = max(1.0e-6, float(loaded_fast_tau_s))
        self._loaded_slow_tau_s = max(1.0e-6, float(loaded_slow_tau_s))
        self._recovered_tau_s = max(1.0e-6, float(recovered_tau_s))
        self._soc_fast_discharge_tau_s = max(1.0e-6, float(soc_fast_discharge_tau_s))
        self._loaded_low_threshold_v = float(loaded_low_threshold_v)
        self._recovered_low_threshold_v = float(recovered_low_threshold_v)
        self._loaded_low_persist_required_s = max(0.0, float(loaded_low_persist_s))
        self._recovered_low_persist_required_s = max(0.0, float(recovered_low_persist_s))
        self._guard_clear_hysteresis_v = max(0.0, float(guard_clear_hysteresis_v))

        self._loaded_voltage_fast_v: Optional[float] = None
        self._loaded_voltage_slow_v: Optional[float] = None
        self._recovered_voltage_v: Optional[float] = None
        self._soc_voltage_v: Optional[float] = None
        self._last_sample_time_s: Optional[float] = None
        self._loaded_low_elapsed_s = 0.0
        self._recovered_low_elapsed_s = 0.0
        self._mission_guard_latched = False

    @property
    def loaded_low_threshold_v(self) -> float:
        return self._loaded_low_threshold_v

    @property
    def recovered_low_threshold_v(self) -> float:
        return self._recovered_low_threshold_v

    @property
    def loaded_low_persist_required_s(self) -> float:
        return self._loaded_low_persist_required_s

    @property
    def recovered_low_persist_required_s(self) -> float:
        return self._recovered_low_persist_required_s

    def update(
        self,
        raw_voltage_v: float,
        *,
        sample_time_s: float,
        traction_active: bool,
    ) -> BatteryEstimate:
        raw_voltage_v = float(raw_voltage_v)
        traction_active = bool(traction_active)
        if (
            self._loaded_voltage_fast_v is None
            or self._loaded_voltage_slow_v is None
            or self._recovered_voltage_v is None
            or self._soc_voltage_v is None
            or self._last_sample_time_s is None
        ):
            loaded_voltage_fast_v = raw_voltage_v
            loaded_voltage_slow_v = raw_voltage_v
            recovered_voltage_v = raw_voltage_v
            soc_voltage_v = raw_voltage_v
            dt_s = 0.0
        else:
            dt_s = max(0.0, float(sample_time_s) - float(self._last_sample_time_s))
            loaded_voltage_fast_v = _ema(
                self._loaded_voltage_fast_v,
                raw_voltage_v,
                dt_s,
                self._loaded_fast_tau_s,
            )
            loaded_voltage_slow_v = _ema(
                self._loaded_voltage_slow_v,
                raw_voltage_v,
                dt_s,
                self._loaded_slow_tau_s,
            )
            if traction_active:
                recovered_voltage_v = float(self._recovered_voltage_v)
            else:
                recovered_voltage_v = _ema(
                    self._recovered_voltage_v,
                    raw_voltage_v,
                    dt_s,
                    self._recovered_tau_s,
                )

            if traction_active:
                soc_target_v = min(float(self._soc_voltage_v), loaded_voltage_slow_v)
                soc_voltage_v = _ema(
                    self._soc_voltage_v,
                    soc_target_v,
                    dt_s,
                    self._soc_fast_discharge_tau_s,
                )
            else:
                soc_voltage_v = _ema(
                    self._soc_voltage_v,
                    recovered_voltage_v,
                    dt_s,
                    self._recovered_tau_s,
                )

        if traction_active and loaded_voltage_slow_v <= self._loaded_low_threshold_v:
            self._loaded_low_elapsed_s += dt_s
        else:
            self._loaded_low_elapsed_s = 0.0

        if (not traction_active) and recovered_voltage_v <= self._recovered_low_threshold_v:
            self._recovered_low_elapsed_s += dt_s
        else:
            self._recovered_low_elapsed_s = 0.0

        if not self._mission_guard_latched:
            if self._loaded_low_elapsed_s >= self._loaded_low_persist_required_s:
                self._mission_guard_latched = True
            if self._recovered_low_elapsed_s >= self._recovered_low_persist_required_s:
                self._mission_guard_latched = True
        else:
            loaded_clear_v = self._loaded_low_threshold_v + self._guard_clear_hysteresis_v
            recovered_clear_v = (
                self._recovered_low_threshold_v + self._guard_clear_hysteresis_v
            )
            if (
                loaded_voltage_slow_v >= loaded_clear_v
                and recovered_voltage_v >= recovered_clear_v
            ):
                self._mission_guard_latched = False
                self._loaded_low_elapsed_s = 0.0
                self._recovered_low_elapsed_s = 0.0

        self._loaded_voltage_fast_v = loaded_voltage_fast_v
        self._loaded_voltage_slow_v = loaded_voltage_slow_v
        self._recovered_voltage_v = recovered_voltage_v
        self._soc_voltage_v = soc_voltage_v
        self._last_sample_time_s = float(sample_time_s)

        raw_percentage = piecewise_soc_from_voltage(raw_voltage_v, self._soc_curve_points)
        filtered_percentage = piecewise_soc_from_voltage(
            soc_voltage_v, self._soc_curve_points
        )
        mission_guard_state = (
            "LOW_ENERGY_GO_HOME" if self._mission_guard_latched else "OK"
        )

        return BatteryEstimate(
            raw_voltage_v=raw_voltage_v,
            filtered_voltage_v=loaded_voltage_slow_v,
            loaded_voltage_fast_v=loaded_voltage_fast_v,
            loaded_voltage_slow_v=loaded_voltage_slow_v,
            recovered_voltage_v=recovered_voltage_v,
            soc_voltage_v=soc_voltage_v,
            raw_percentage=raw_percentage,
            filtered_percentage=filtered_percentage,
            operator_soc_pct=100.0 * filtered_percentage,
            traction_active=traction_active,
            return_home_recommended=bool(self._mission_guard_latched),
            mission_guard_state=mission_guard_state,
            loaded_low_persist_s=self._loaded_low_elapsed_s,
            recovered_low_persist_s=self._recovered_low_elapsed_s,
        )


def battery_state_label(
    *,
    ready: bool,
    fresh: bool,
    link_fresh: bool,
    suspect: bool,
    mission_guard_state: str,
) -> str:
    if not ready:
        return "UNAVAILABLE"
    if not link_fresh or not fresh:
        return "STALE"
    if suspect:
        return "SUSPECT"
    return "LOW_ENERGY_GO_HOME" if mission_guard_state == "LOW_ENERGY_GO_HOME" else "OK"
