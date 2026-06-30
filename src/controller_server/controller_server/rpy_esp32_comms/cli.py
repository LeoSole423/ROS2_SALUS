from __future__ import annotations

import argparse
import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from .transport import CommsClient


HELP_TEXT = """Comandos:
  help
  status
  drive on|off
  estop on|off
  hazard on|off
  speed <mps signed>
  steer <int -100..100>
  brake <0..100>
  watch on|off
  log on|off
  quit
"""


class SessionLogger:
    def __init__(self, path: Optional[str]) -> None:
        self._path = Path(path).expanduser() if path else None
        self._file = None
        if self._path is not None:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self._path.open("a", encoding="utf-8")

    def write(
        self,
        event: str,
        command: str,
        telemetry: Optional[dict],
        extra: Optional[dict] = None,
    ) -> None:
        if self._file is None:
            return
        payload = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "event": event,
            "command": command,
            "telemetry": telemetry,
            "extra": extra or {},
        }
        self._file.write(json.dumps(payload, ensure_ascii=True) + "\n")
        self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None


def _parse_on_off(raw: str) -> bool:
    if raw == "on":
        return True
    if raw == "off":
        return False
    raise ValueError("expected 'on' or 'off'")


def _format_telemetry(telemetry) -> str:
    if telemetry is None:
        return "telemetry: N/A"
    speed = "N/A" if telemetry.speed_mps is None else f"{telemetry.speed_mps:.2f} m/s"
    steer = "N/A" if telemetry.steer_deg is None else f"{telemetry.steer_deg:.2f} deg"
    return (
        "telemetry: "
        f"src={telemetry.control_source.name} "
        f"speed={speed} "
        f"steer={steer} "
        f"brake={telemetry.brake_applied_pct}% "
        f"ready={int(telemetry.ready)} "
        f"estop={int(telemetry.estop_active)} "
        f"failsafe={int(telemetry.failsafe_active)} "
        f"pi_fresh={int(telemetry.pi_fresh)} "
        f"overspeed={int(telemetry.overspeed_active)} "
        f"flags=0x{telemetry.status_flags:02X}"
    )


def _format_battery_telemetry(battery) -> str:
    if battery is None:
        return "battery: N/A"
    return (
        "battery: "
        f"voltage={battery.battery_voltage_v:.2f} V "
        f"adc={battery.adc_pin_voltage_v:.3f} V "
        f"sample_age={battery.sample_age_s:.1f} s "
        f"ready={int(battery.ready)} "
        f"fresh={int(battery.fresh)} "
        f"suspect={int(battery.suspect)} "
        f"cal={int(battery.calibrated)} "
        f"flags=0x{battery.status_flags:02X}"
    )


def _watch_loop(
    client: CommsClient,
    stop_event: threading.Event,
    enabled_ref: dict,
    period_s: float,
) -> None:
    while not stop_event.is_set():
        if enabled_ref.get("watch", False):
            print(_format_telemetry(client.get_latest_telemetry()))
            print(_format_battery_telemetry(client.get_latest_battery_telemetry()))
        stop_event.wait(period_s)


def run_cli(args: argparse.Namespace) -> int:
    client = CommsClient(
        port=args.port,
        baud=args.baud,
        tx_hz=args.tx_hz,
        max_reverse_mps=args.max_reverse_mps,
    )
    logger = SessionLogger(args.log_file)

    watch_state = {"watch": False}
    watch_stop = threading.Event()
    watch_thread = threading.Thread(
        target=_watch_loop,
        args=(client, watch_stop, watch_state, 1.0 / max(0.1, float(args.telemetry_print_hz))),
        daemon=True,
        name="rpy-watch",
    )

    try:
        client.start()
        watch_thread.start()
        print("UART v2 listo. Escribe 'help' para ver comandos.")

        while True:
            try:
                raw = input("rpy> ").strip()
            except EOFError:
                raw = "quit"

            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()

            try:
                if cmd == "help":
                    print(HELP_TEXT, end="")

                elif cmd == "status":
                    desired = client.get_command_state()
                    stats = client.get_stats()
                    print(f"desired: {desired}")
                    print(_format_telemetry(client.get_latest_telemetry()))
                    print(_format_battery_telemetry(client.get_latest_battery_telemetry()))
                    print(
                        "stats: "
                        f"tx_ok={stats.tx_frames_ok} tx_err={stats.tx_errors} "
                        f"rx_ok={stats.rx_frames_ok} "
                        f"rx_ctrl={stats.rx_control_frames_ok} "
                        f"rx_bat={stats.rx_battery_frames_ok} "
                        f"rx_crc={stats.rx_crc_errors} "
                        f"rx_drop={stats.rx_parse_drops}"
                    )

                elif cmd == "drive":
                    if len(parts) != 2:
                        raise ValueError("uso: drive on|off")
                    enabled = _parse_on_off(parts[1].lower())
                    client.set_drive_enabled(enabled)
                    print(f"drive={'on' if enabled else 'off'}")

                elif cmd == "estop":
                    if len(parts) != 2:
                        raise ValueError("uso: estop on|off")
                    enabled = _parse_on_off(parts[1].lower())
                    client.set_estop(enabled)
                    print(f"estop={'on' if enabled else 'off'}")

                elif cmd == "hazard":
                    if len(parts) != 2:
                        raise ValueError("uso: hazard on|off")
                    enabled = _parse_on_off(parts[1].lower())
                    client.set_hazard_enabled(enabled)
                    print(f"hazard={'on' if enabled else 'off'}")

                elif cmd == "speed":
                    if len(parts) != 2:
                        raise ValueError("uso: speed <mps signed>")
                    value = float(parts[1])
                    applied = client.set_speed_mps(value)
                    print(f"speed={applied:.2f} m/s")

                elif cmd == "steer":
                    if len(parts) != 2:
                        raise ValueError("uso: steer <-100..100>")
                    value = int(parts[1])
                    applied = client.set_steer_pct(value)
                    print(f"steer={applied}")

                elif cmd == "brake":
                    if len(parts) != 2:
                        raise ValueError("uso: brake <0..100>")
                    value = int(parts[1])
                    applied = client.set_brake_pct(value)
                    print(f"brake={applied}%")

                elif cmd == "watch":
                    if len(parts) != 2:
                        raise ValueError("uso: watch on|off")
                    watch_state["watch"] = _parse_on_off(parts[1].lower())
                    print(f"watch={'on' if watch_state['watch'] else 'off'}")

                elif cmd == "log":
                    if len(parts) != 2:
                        raise ValueError("uso: log on|off")
                    enabled = _parse_on_off(parts[1].lower())
                    client.set_log_enabled(enabled)
                    print(f"log={'on' if enabled else 'off'}")

                elif cmd == "quit":
                    print("saliendo...")
                    logger.write(
                        event="command",
                        command=raw,
                        telemetry=(
                            client.get_latest_telemetry().as_dict()
                            if client.get_latest_telemetry() is not None
                            else None
                        ),
                        extra={
                            "battery": (
                                client.get_latest_battery_telemetry().as_dict()
                                if client.get_latest_battery_telemetry() is not None
                                else None
                            )
                        },
                    )
                    break

                else:
                    print("comando no reconocido. usa: help")

                telemetry = client.get_latest_telemetry()
                logger.write(
                    event="command",
                    command=raw,
                    telemetry=telemetry.as_dict() if telemetry else None,
                    extra={
                        "desired": client.get_command_state(),
                        "battery": (
                            client.get_latest_battery_telemetry().as_dict()
                            if client.get_latest_battery_telemetry() is not None
                            else None
                        ),
                    },
                )

            except ValueError as exc:
                print(f"error: {exc}")

    except KeyboardInterrupt:
        print("\ninterrumpido por usuario")

    finally:
        watch_stop.set()
        watch_thread.join(timeout=1.0)
        client.stop()
        logger.close()

    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Cliente UART v2 Raspberry <-> ESP32")
    parser.add_argument(
        "--port",
        default="/dev/serial0",
        help="Puerto serial (default: /dev/serial0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baudrate UART (default: 115200)",
    )
    parser.add_argument(
        "--tx-hz",
        type=float,
        default=50.0,
        help="Frecuencia de TX Pi->ESP32 (default: 50)",
    )
    parser.add_argument(
        "--telemetry-print-hz",
        type=float,
        default=10.0,
        help="Frecuencia de impresión de telemetría cuando watch=on (default: 10)",
    )
    parser.add_argument(
        "--max-reverse-mps",
        type=float,
        default=1.30,
        help="Magnitud máxima permitida en reversa para speed<0 (default: 1.30)",
    )
    parser.add_argument(
        "--log-file",
        default=None,
        help="Ruta opcional para log JSONL de sesión",
    )
    return parser
