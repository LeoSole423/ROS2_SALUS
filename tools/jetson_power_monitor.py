#!/usr/bin/env python3
"""Host-side Jetson power monitor using INA3221 and tegrastats."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

from jetson_power_lib import (
    AGGRESSIVE_PROFILE,
    CONTEXT_PREFIX,
    FAST_PREFIX,
    MonitorProfile,
    SegmentedJsonlWriter,
    TegrastatsPoller,
    VoltageWindow,
    append_json_record,
    build_event,
    default_log_dir_from_tools_dir,
    discover_ina3221_channels,
    events_file_path,
    load_boot_data,
    previous_boot_id,
    profile_from_name,
    read_boot_id,
    read_float,
    read_loadavg,
    read_thermal_zones,
    read_uptime_s,
    summarize_boot,
)


SCRIPT_DIR = Path(__file__).resolve().parent


def env_default(name: str, default: Any, cast: type) -> Any:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    if cast is Path:
        return Path(raw)
    return cast(raw)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=env_default(
            "JETSON_POWER_MONITOR_LOG_DIR",
            default_log_dir_from_tools_dir(SCRIPT_DIR),
            Path,
        ),
        help="Directory where JSONL files are written.",
    )
    parser.add_argument(
        "--profile",
        type=str,
        default=env_default("JETSON_POWER_MONITOR_PROFILE", AGGRESSIVE_PROFILE.name, str),
        help="Capture profile: aggressive or moderate.",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=env_default("JETSON_POWER_MONITOR_MAX_SAMPLES", 0, int),
        help="Exit cleanly after this many fast samples. Zero means run forever.",
    )
    return parser


def profile_payload(profile: MonitorProfile) -> dict[str, Any]:
    return {
        "profile": profile.name,
        "fast_sample_hz": profile.fast_sample_hz,
        "context_hz": profile.context_hz,
        "segment_seconds": profile.segment_seconds,
        "fast_sync_interval_ms": profile.fast_sync_interval_ms,
        "context_sync_interval_ms": profile.context_sync_interval_ms,
        "low_vdd_in_mv": profile.thresholds.low_vdd_in_mv,
        "critical_vdd_in_mv": profile.thresholds.critical_vdd_in_mv,
        "stable_vdd_in_mv": profile.thresholds.stable_vdd_in_mv,
        "fast_drop_window_ms": profile.thresholds.fast_drop_window_ms,
        "fast_drop_delta_mv": profile.thresholds.fast_drop_delta_mv,
        "low_persist_ms": profile.thresholds.low_persist_ms,
        "window_seconds": profile.thresholds.window_seconds,
    }


def main() -> int:
    args = build_parser().parse_args()
    profile = profile_from_name(str(args.profile))
    thresholds = profile.thresholds

    try:
        channels = discover_ina3221_channels()
    except Exception as exc:
        print(f"jetson_power_monitor: failed to discover INA3221 channels: {exc}", file=sys.stderr)
        return 2

    current_boot_id = read_boot_id()
    log_dir = Path(args.log_dir)
    events_path = events_file_path(log_dir, current_boot_id)
    stop_requested = threading.Event()
    clean_exit = {"value": False}
    event_lock = threading.Lock()

    def emit_event(event_type: str, *, sync: bool = False, **payload: Any) -> None:
        record = build_event(event_type, boot_id=current_boot_id, **payload)
        with event_lock:
            append_json_record(events_path, record, sync=sync)

    def _request_stop(_signum: int, _frame: Any) -> None:
        clean_exit["value"] = True
        stop_requested.set()

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    previous_id = previous_boot_id(log_dir, current_boot_id)
    previous_analysis: Optional[dict[str, Any]] = None
    if previous_id is not None:
        previous_analysis = summarize_boot(log_dir, previous_id, persist=True)
        previous_events = list(load_boot_data(log_dir, previous_id).events)
        previous_event_names = {str(event.get("event")) for event in previous_events}
        if bool(previous_analysis.get("corrupt_tail_detected")) and "corrupt_tail_detected" not in previous_event_names:
            append_json_record(
                events_file_path(log_dir, previous_id),
                build_event(
                    "corrupt_tail_detected",
                    boot_id=previous_id,
                    corrupt_paths=previous_analysis.get("corrupt_paths"),
                ),
                sync=True,
            )
        if "abrupt_session_end_detected" not in previous_event_names:
            append_json_record(
                events_file_path(log_dir, previous_id),
                build_event(
                    "abrupt_session_end_detected",
                    boot_id=previous_id,
                    classification=previous_analysis.get("classification"),
                    detail=previous_analysis.get("detail"),
                ),
                sync=True,
            )
        if "post_reboot_analysis" not in previous_event_names:
            append_json_record(
                events_file_path(log_dir, previous_id),
                build_event(
                    "post_reboot_analysis",
                    boot_id=previous_id,
                    **{key: value for key, value in previous_analysis.items() if key != "boot_id"},
                ),
                sync=True,
            )

    def on_tegrastats_error(event_type: str, payload: dict[str, Any]) -> None:
        emit_event(event_type, sync=True, **payload)

    poller = TegrastatsPoller(interval_hz=profile.context_hz, on_error=on_tegrastats_error)
    poller.start()

    fast_writer = SegmentedJsonlWriter(
        log_dir=log_dir,
        prefix=FAST_PREFIX,
        boot_id=current_boot_id,
        segment_seconds=profile.segment_seconds,
        sync_interval_ms=profile.fast_sync_interval_ms,
    )
    context_writer = SegmentedJsonlWriter(
        log_dir=log_dir,
        prefix=CONTEXT_PREFIX,
        boot_id=current_boot_id,
        segment_seconds=profile.segment_seconds,
        sync_interval_ms=profile.context_sync_interval_ms,
    )
    voltage_window = VoltageWindow(thresholds=thresholds)

    emit_event(
        f"monitor_profile_{profile.name}",
        sync=True,
        config=profile_payload(profile),
    )
    emit_event(
        "monitor_started",
        sync=True,
        config=profile_payload(profile),
        channels={
            name: {
                "voltage_path": str(channel.voltage_path),
                "current_path": str(channel.current_path) if channel.current_path else None,
            }
            for name, channel in channels.items()
        },
        previous_analysis=previous_analysis,
    )
    if previous_analysis is not None:
        emit_event(
            "previous_session_unclean",
            sync=True,
            previous_boot_id=previous_id,
            previous_classification=previous_analysis.get("classification"),
            previous_label=previous_analysis.get("label"),
            previous_detail=previous_analysis.get("detail"),
        )

    fast_period_s = 1.0 / max(0.1, profile.fast_sample_hz)
    context_period_s = 1.0 / max(0.1, profile.context_hz)
    next_context_monotonic_s = time.monotonic()
    fast_seq = 0
    context_seq = 0
    last_fast_monotonic_s: Optional[float] = None

    def _read_rail(name: str, suffix: str) -> Optional[float]:
        channel = channels.get(name)
        if channel is None:
            return None
        if suffix == "voltage":
            return read_float(channel.voltage_path)
        if channel.current_path is None:
            return None
        return read_float(channel.current_path)

    try:
        while not stop_requested.is_set():
            loop_started_s = time.monotonic()
            wall_time_s = time.time()
            uptime_s = read_uptime_s()
            vdd_in_mv = _read_rail("VDD_IN", "voltage")
            vdd_in_ma = _read_rail("VDD_IN", "current")
            if vdd_in_mv is None:
                raise RuntimeError("VDD_IN channel disappeared during monitoring")
            vdd_in_w = None
            if vdd_in_ma is not None:
                vdd_in_w = (float(vdd_in_mv) / 1000.0) * (float(vdd_in_ma) / 1000.0)

            metrics = voltage_window.update(monotonic_s=loop_started_s, vdd_in_mv=float(vdd_in_mv))
            fast_record = {
                "seq": fast_seq,
                "boot_id": current_boot_id,
                "timestamp": build_event("sample", boot_id=current_boot_id, wall_time_s=wall_time_s)["timestamp"],
                "wall_time_s": wall_time_s,
                "monotonic_ns": time.monotonic_ns(),
                "uptime_s": uptime_s,
                "vdd_in_mv": float(vdd_in_mv),
                "vdd_in_ma": float(vdd_in_ma) if vdd_in_ma is not None else None,
                "vdd_in_w": float(vdd_in_w) if vdd_in_w is not None else None,
                "vdd_cpu_gpu_cv_mv": _read_rail("VDD_CPU_GPU_CV", "voltage"),
                "vdd_cpu_gpu_cv_ma": _read_rail("VDD_CPU_GPU_CV", "current"),
                "vdd_soc_mv": _read_rail("VDD_SOC", "voltage"),
                "vdd_soc_ma": _read_rail("VDD_SOC", "current"),
                "dv_dt_mv_per_s": float(metrics["dv_dt_mv_per_s"]),
            }
            fast_writer.write(fast_record, monotonic_s=loop_started_s)
            last_fast_monotonic_s = loop_started_s

            if bool(metrics["low_triggered"]):
                emit_event(
                    "vdd_in_low",
                    sync=True,
                    vdd_in_mv=float(vdd_in_mv),
                    low_persist_s=float(metrics["low_persist_s"]),
                    threshold_mv=float(thresholds.low_vdd_in_mv),
                )
            if bool(metrics["critical_triggered"]):
                emit_event(
                    "vdd_in_critical",
                    sync=True,
                    vdd_in_mv=float(vdd_in_mv),
                    threshold_mv=float(thresholds.critical_vdd_in_mv),
                )
            if bool(metrics["fast_drop_triggered"]):
                emit_event(
                    "fast_voltage_drop",
                    sync=True,
                    vdd_in_mv=float(vdd_in_mv),
                    dv_dt_mv_per_s=float(metrics["dv_dt_mv_per_s"]),
                    threshold_mv_per_s=float(-thresholds.fast_drop_rate_mv_per_s),
                )

            if loop_started_s >= next_context_monotonic_s:
                context_record = {
                    "seq": context_seq,
                    "boot_id": current_boot_id,
                    "timestamp": build_event(
                        "context",
                        boot_id=current_boot_id,
                        wall_time_s=wall_time_s,
                    )["timestamp"],
                    "wall_time_s": wall_time_s,
                    "monotonic_ns": time.monotonic_ns(),
                    "uptime_s": uptime_s,
                    "loadavg": read_loadavg(),
                    "thermal_zones_c": read_thermal_zones(),
                    "tegrastats": poller.latest_snapshot(),
                    "last_fast_sample_age_ms": (
                        1000.0 * max(0.0, loop_started_s - last_fast_monotonic_s)
                        if last_fast_monotonic_s is not None
                        else None
                    ),
                    "last_fast_seq": fast_seq,
                }
                context_writer.write(context_record, monotonic_s=loop_started_s)
                context_seq += 1
                next_context_monotonic_s = loop_started_s + context_period_s

            fast_seq += 1
            if int(args.max_samples) > 0 and fast_seq >= int(args.max_samples):
                clean_exit["value"] = True
                break

            elapsed_s = max(0.0, time.monotonic() - loop_started_s)
            sleep_s = max(0.0, fast_period_s - elapsed_s)
            if stop_requested.wait(sleep_s):
                break
    except Exception as exc:
        emit_event("monitor_exception", sync=True, error=str(exc), fast_seq=fast_seq)
        raise
    finally:
        poller.stop()
        fast_writer.close()
        context_writer.close()
        if clean_exit["value"]:
            emit_event("monitor_stopped_cleanly", sync=True, fast_seq=fast_seq)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
