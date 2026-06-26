from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


class ControlSource(IntEnum):
    NONE = 0
    PI = 1
    RC = 2
    TEL = 3


@dataclass(slots=True)
class Telemetry:
    status_flags: int
    speed_mps: Optional[float]
    steer_deg: Optional[float]
    brake_applied_pct: int
    raw_speed_centi_mps: int
    raw_steer_centi_deg: int
    rx_monotonic_s: float

    @property
    def ready(self) -> bool:
        return bool(self.status_flags & (1 << 0))

    @property
    def estop_active(self) -> bool:
        return bool(self.status_flags & (1 << 1))

    @property
    def failsafe_active(self) -> bool:
        return bool(self.status_flags & (1 << 2))

    @property
    def pi_fresh(self) -> bool:
        return bool(self.status_flags & (1 << 3))

    @property
    def control_source(self) -> ControlSource:
        return ControlSource((self.status_flags >> 4) & 0x03)

    @property
    def overspeed_active(self) -> bool:
        return bool(self.status_flags & (1 << 6))

    def as_dict(self) -> dict:
        return {
            "status_flags": self.status_flags,
            "ready": self.ready,
            "estop_active": self.estop_active,
            "failsafe_active": self.failsafe_active,
            "pi_fresh": self.pi_fresh,
            "control_source": self.control_source.name,
            "overspeed_active": self.overspeed_active,
            "speed_mps": self.speed_mps,
            "steer_deg": self.steer_deg,
            "brake_applied_pct": self.brake_applied_pct,
            "raw_speed_centi_mps": self.raw_speed_centi_mps,
            "raw_steer_centi_deg": self.raw_steer_centi_deg,
            "rx_monotonic_s": self.rx_monotonic_s,
        }


@dataclass(slots=True)
class BatteryTelemetry:
    status_flags: int
    battery_voltage_v: float
    adc_pin_voltage_v: float
    sample_age_s: float
    raw_battery_centi_volts: int
    raw_adc_pin_mv: int
    raw_sample_age_ds: int
    rx_monotonic_s: float

    @property
    def ready(self) -> bool:
        return bool(self.status_flags & (1 << 0))

    @property
    def fresh(self) -> bool:
        return bool(self.status_flags & (1 << 1))

    @property
    def suspect(self) -> bool:
        return bool(self.status_flags & (1 << 2))

    @property
    def calibrated(self) -> bool:
        return bool(self.status_flags & (1 << 3))

    def as_dict(self) -> dict:
        return {
            "status_flags": self.status_flags,
            "ready": self.ready,
            "fresh": self.fresh,
            "suspect": self.suspect,
            "calibrated": self.calibrated,
            "battery_voltage_v": self.battery_voltage_v,
            "adc_pin_voltage_v": self.adc_pin_voltage_v,
            "sample_age_s": self.sample_age_s,
            "raw_battery_centi_volts": self.raw_battery_centi_volts,
            "raw_adc_pin_mv": self.raw_adc_pin_mv,
            "raw_sample_age_ds": self.raw_sample_age_ds,
            "rx_monotonic_s": self.rx_monotonic_s,
        }
