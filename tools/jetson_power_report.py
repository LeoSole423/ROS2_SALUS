#!/usr/bin/env python3
"""Summarize Jetson host-side power monitor logs."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from jetson_power_lib import (
    AGGRESSIVE_PROFILE,
    default_log_dir_from_tools_dir,
    events_file_path,
    list_monitored_boots,
    load_boot_data,
    read_boot_id,
    recent_fast_samples,
    summarize_boot,
)


SCRIPT_DIR = Path(__file__).resolve().parent


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=default_log_dir_from_tools_dir(SCRIPT_DIR),
        help="Directory containing fast/context/events logs.",
    )
    parser.add_argument("--boot-id", type=str, default="", help="Boot ID to inspect.")
    parser.add_argument(
        "--list-boots",
        action="store_true",
        help="List monitored boots and their current classification.",
    )
    parser.add_argument(
        "--limit-events",
        type=int,
        default=8,
        help="How many trailing events to show.",
    )
    parser.add_argument(
        "--limit-samples",
        type=int,
        default=5,
        help="How many trailing fast samples to show.",
    )
    return parser


def fmt_optional_float(value: object, suffix: str) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float) and math.isnan(value):
        return "n/a"
    return f"{float(value):.2f}{suffix}"


def boot_status(boot_id: str, summary: dict[str, object], events: list[dict[str, object]]) -> str:
    current_boot_id = read_boot_id()
    if boot_id == current_boot_id and not bool(summary.get("session_closed_cleanly")):
        return "active"
    return str(summary.get("classification", "unknown"))


def print_boot_summary(log_dir: Path, boot_id: str, *, limit_events: int, limit_samples: int) -> None:
    data = load_boot_data(log_dir, boot_id)
    summary = summarize_boot(log_dir, boot_id, fallback_profile=AGGRESSIVE_PROFILE, persist=False)
    status = boot_status(boot_id, summary, list(data.events))

    print(f"Boot ID: {boot_id}")
    print(f"Status: {status}")
    if status == "active":
        print("Diagnosis: active session (no reboot analysis yet)")
    else:
        print(f"Diagnosis: {summary.get('label', 'n/a')} ({summary.get('classification', 'n/a')})")
    if summary.get("detail"):
        print(f"Detail: {summary['detail']}")
    if summary.get("upstream_note"):
        print(f"Note: {summary['upstream_note']}")
    print(f"Fast samples: {summary.get('sample_total', 0)}")
    print(f"Context samples: {summary.get('context_total', 0)}")
    print(f"Events: {summary.get('event_total', 0)}")
    print(f"Window min VDD_IN: {fmt_optional_float(summary.get('min_vdd_in_mv_window'), ' mV')}")
    print(f"Window max current: {fmt_optional_float(summary.get('max_vdd_in_ma_window'), ' mA')}")
    print(f"Window min dV/dt: {fmt_optional_float(summary.get('min_dv_dt_mv_per_s_window'), ' mV/s')}")
    print(f"Corrupt tail: {'yes' if summary.get('corrupt_tail_detected') else 'no'}")
    markers = summary.get("markers") or []
    print("Markers near reboot: " + (", ".join(markers) if markers else "none"))

    print("Recent events:")
    for event in list(data.events)[-max(0, limit_events):]:
        parts = [str(event.get("timestamp", "")), str(event.get("event", ""))]
        if event.get("label"):
            parts.append(f"label={event['label']}")
        if event.get("vdd_in_mv") is not None:
            parts.append(f"vdd={float(event['vdd_in_mv']):.1f}mV")
        if event.get("dv_dt_mv_per_s") is not None:
            parts.append(f"dvdt={float(event['dv_dt_mv_per_s']):.1f}mV/s")
        print("  - " + " | ".join(parts))

    print("Last valid fast samples:")
    for sample in recent_fast_samples(data.fast_samples, limit=max(0, limit_samples)):
        print(
            "  - "
            + " | ".join(
                [
                    str(sample.get("timestamp", "")),
                    f"seq={sample.get('seq')}",
                    f"vdd={fmt_optional_float(sample.get('vdd_in_mv'), ' mV')}",
                    f"i={fmt_optional_float(sample.get('vdd_in_ma'), ' mA')}",
                    f"p={fmt_optional_float(sample.get('vdd_in_w'), ' W')}",
                    f"dvdt={fmt_optional_float(sample.get('dv_dt_mv_per_s'), ' mV/s')}",
                ]
            )
        )


def main() -> int:
    args = build_parser().parse_args()
    log_dir = Path(args.log_dir)
    boot_ids = list_monitored_boots(log_dir)

    if not boot_ids:
        print(f"No monitor logs found in {log_dir}")
        return 1

    if args.list_boots:
        print("Monitored boots:")
        for boot_id in boot_ids:
            data = load_boot_data(log_dir, boot_id)
            summary = summarize_boot(log_dir, boot_id, fallback_profile=AGGRESSIVE_PROFILE, persist=False)
            status = boot_status(boot_id, summary, list(data.events))
            label = "active session" if status == "active" else str(summary.get("label", "n/a"))
            print(f"- {boot_id} | {status} | {label}")
        return 0

    boot_id = args.boot_id or boot_ids[-1]
    print_boot_summary(
        log_dir,
        boot_id,
        limit_events=int(args.limit_events),
        limit_samples=int(args.limit_samples),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
