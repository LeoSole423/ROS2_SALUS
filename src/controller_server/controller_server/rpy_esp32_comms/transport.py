from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial

from .controller import CommandState, DEFAULT_MAX_SPEED_MPS, DEFAULT_MAX_REVERSE_MPS
from .protocol import EspFrameParser, decode_transport_frame, encode_pi_frame
from .telemetry import BatteryTelemetry, Telemetry


@dataclass(slots=True)
class CommsStats:
    tx_frames_ok: int = 0
    tx_errors: int = 0
    rx_frames_ok: int = 0
    rx_control_frames_ok: int = 0
    rx_battery_frames_ok: int = 0
    rx_crc_errors: int = 0
    rx_parse_drops: int = 0


class CommsClient:
    def __init__(
        self,
        port: str = "/dev/serial0",
        baud: int = 115200,
        tx_hz: float = 50.0,
        max_speed_mps: float = DEFAULT_MAX_SPEED_MPS,
        max_reverse_mps: float = DEFAULT_MAX_REVERSE_MPS,
    ) -> None:
        if tx_hz <= 0:
            raise ValueError("tx_hz must be > 0")

        self.port = port
        self.baud = int(baud)
        self.tx_hz = float(tx_hz)
        self.tx_period_s = 1.0 / self.tx_hz

        self._state = CommandState(
            max_speed_mps=float(max_speed_mps),
            max_reverse_mps=float(max_reverse_mps),
        )
        self._state_lock = threading.Lock()

        self._latest_telemetry: Optional[Telemetry] = None
        self._telemetry_lock = threading.Lock()
        self._latest_battery_telemetry: Optional[BatteryTelemetry] = None
        self._battery_telemetry_lock = threading.Lock()

        self._stats = CommsStats()
        self._stats_lock = threading.Lock()

        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None

        self._running = False
        self._log_enabled = False

    def start(self) -> None:
        if self._running:
            return

        with self._state_lock:
            self._state.safe_reset()

        ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=0.05,
            write_timeout=0.05,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        with self._serial_lock:
            self._serial = ser

        self._stop_event.clear()
        self._tx_thread = threading.Thread(target=self._tx_loop, name="rpy-uart-tx", daemon=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, name="rpy-uart-rx", daemon=True)
        self._tx_thread.start()
        self._rx_thread.start()
        self._running = True

    def stop(self) -> None:
        if not self._running:
            return

        self._set_safe_state()
        self._stop_event.set()

        if self._tx_thread is not None:
            self._tx_thread.join(timeout=1.5)
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.5)

        self._send_safe_frames(3)

        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                finally:
                    self._serial = None

        self._running = False

    def set_speed_mps(self, value: float) -> float:
        with self._state_lock:
            return self._state.set_speed_mps(float(value))

    def set_steer_pct(self, value: int) -> int:
        with self._state_lock:
            return self._state.set_steer_pct(int(value))

    def set_brake_pct(self, value: int) -> int:
        with self._state_lock:
            return self._state.set_brake_pct(int(value))

    def set_drive_enabled(self, value: bool) -> None:
        with self._state_lock:
            self._state.set_drive_enabled(bool(value))

    def set_estop(self, value: bool) -> None:
        with self._state_lock:
            self._state.set_estop(bool(value))

    def set_hazard_enabled(self, value: bool) -> None:
        with self._state_lock:
            self._state.set_hazard_enabled(bool(value))

    def set_log_enabled(self, enabled: bool) -> None:
        self._log_enabled = bool(enabled)

    def get_latest_telemetry(self) -> Optional[Telemetry]:
        with self._telemetry_lock:
            return self._latest_telemetry

    def get_latest_battery_telemetry(self) -> Optional[BatteryTelemetry]:
        with self._battery_telemetry_lock:
            return self._latest_battery_telemetry

    def get_command_state(self) -> dict:
        with self._state_lock:
            return self._state.to_dict()

    def get_stats(self) -> CommsStats:
        with self._stats_lock:
            return CommsStats(
                tx_frames_ok=self._stats.tx_frames_ok,
                tx_errors=self._stats.tx_errors,
                rx_frames_ok=self._stats.rx_frames_ok,
                rx_control_frames_ok=self._stats.rx_control_frames_ok,
                rx_battery_frames_ok=self._stats.rx_battery_frames_ok,
                rx_crc_errors=self._stats.rx_crc_errors,
                rx_parse_drops=self._stats.rx_parse_drops,
            )

    def _set_safe_state(self) -> None:
        with self._state_lock:
            self._state.safe_reset()

    def _state_snapshot(self) -> CommandState:
        with self._state_lock:
            return CommandState(**self._state.to_dict())

    def _serial_write(self, payload: bytes) -> None:
        with self._serial_lock:
            if self._serial is None or not self._serial.is_open:
                raise RuntimeError("serial not open")
            self._serial.write(payload)

    def _write_current_frame(self) -> None:
        state = self._state_snapshot()
        frame = encode_pi_frame(state)
        try:
            self._serial_write(frame)
            with self._stats_lock:
                self._stats.tx_frames_ok += 1
            if self._log_enabled:
                print(f"[TX] {frame.hex(' ')}")
        except Exception:
            with self._stats_lock:
                self._stats.tx_errors += 1

    def _send_safe_frames(self, count: int) -> None:
        self._set_safe_state()
        for _ in range(max(0, int(count))):
            try:
                self._write_current_frame()
            except Exception:
                pass
            time.sleep(self.tx_period_s)

    def _tx_loop(self) -> None:
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            self._write_current_frame()
            next_tick += self.tx_period_s
            wait_s = max(0.0, next_tick - time.monotonic())
            self._stop_event.wait(wait_s)

    def _rx_loop(self) -> None:
        parser = EspFrameParser()
        while not self._stop_event.is_set():
            with self._serial_lock:
                ser = self._serial

            if ser is None or not ser.is_open:
                self._stop_event.wait(0.02)
                continue

            try:
                chunk = ser.read(128)
            except Exception:
                self._stop_event.wait(0.02)
                continue

            if not chunk:
                continue

            frames = parser.feed(chunk)
            with self._stats_lock:
                self._stats.rx_parse_drops = parser.dropped_partial_frames
                self._stats.rx_crc_errors = parser.crc_error_frames

            for frame in frames:
                try:
                    frame_kind, payload = decode_transport_frame(frame)
                except ValueError:
                    with self._stats_lock:
                        self._stats.rx_crc_errors += 1
                    continue

                with self._stats_lock:
                    self._stats.rx_frames_ok += 1
                    if frame_kind == "telemetry":
                        self._stats.rx_control_frames_ok += 1
                    elif frame_kind == "battery":
                        self._stats.rx_battery_frames_ok += 1

                if frame_kind == "telemetry":
                    telemetry = payload
                    with self._telemetry_lock:
                        self._latest_telemetry = telemetry

                    if self._log_enabled:
                        speed = "N/A" if telemetry.speed_mps is None else f"{telemetry.speed_mps:.2f}"
                        steer = "N/A" if telemetry.steer_deg is None else f"{telemetry.steer_deg:.2f}"
                        print(
                            "[RX]"
                            f" src={telemetry.control_source.name}"
                            f" speed={speed}m/s"
                            f" steer={steer}deg"
                            f" brake={telemetry.brake_applied_pct}%"
                            f" flags=0x{telemetry.status_flags:02X}"
                        )
                elif frame_kind == "battery":
                    battery = payload
                    with self._battery_telemetry_lock:
                        self._latest_battery_telemetry = battery

                    if self._log_enabled:
                        print(
                            "[RX][BAT]"
                            f" voltage={battery.battery_voltage_v:.2f}V"
                            f" adc={battery.adc_pin_voltage_v:.3f}V"
                            f" age={battery.sample_age_s:.1f}s"
                            f" ready={int(battery.ready)}"
                            f" fresh={int(battery.fresh)}"
                            f" suspect={int(battery.suspect)}"
                            f" flags=0x{battery.status_flags:02X}"
                        )
