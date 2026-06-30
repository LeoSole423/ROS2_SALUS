from __future__ import annotations

from dataclasses import dataclass


DEFAULT_MAX_SPEED_MPS = 12.0
DEFAULT_MAX_REVERSE_MPS = 1.30


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(slots=True)
class CommandState:
    drive_enabled: bool = False
    estop: bool = False
    hazard_enabled: bool = False
    steer_pct: int = 0
    speed_mps: float = 0.0
    brake_pct: int = 0
    max_speed_mps: float = DEFAULT_MAX_SPEED_MPS
    max_reverse_mps: float = DEFAULT_MAX_REVERSE_MPS

    def set_speed_mps(self, value: float) -> float:
        clamped = _clamp(float(value), -self.max_reverse_mps, self.max_speed_mps)
        self.speed_mps = clamped
        return clamped

    def set_steer_pct(self, value: int) -> int:
        clamped = int(_clamp(int(value), -100, 100))
        self.steer_pct = clamped
        return clamped

    def set_brake_pct(self, value: int) -> int:
        clamped = int(_clamp(int(value), 0, 100))
        self.brake_pct = clamped
        return clamped

    def set_drive_enabled(self, enabled: bool) -> None:
        self.drive_enabled = bool(enabled)

    def set_estop(self, enabled: bool) -> None:
        self.estop = bool(enabled)

    def set_hazard_enabled(self, enabled: bool) -> None:
        self.hazard_enabled = bool(enabled)

    def safe_reset(self) -> None:
        self.drive_enabled = False
        self.estop = False
        self.hazard_enabled = False
        self.speed_mps = 0.0
        self.steer_pct = 0
        self.brake_pct = 0

    def to_dict(self) -> dict:
        return {
            "drive_enabled": self.drive_enabled,
            "estop": self.estop,
            "hazard_enabled": self.hazard_enabled,
            "steer_pct": self.steer_pct,
            "speed_mps": self.speed_mps,
            "brake_pct": self.brake_pct,
            "max_speed_mps": self.max_speed_mps,
            "max_reverse_mps": self.max_reverse_mps,
        }
