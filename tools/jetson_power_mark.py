#!/usr/bin/env python3
"""Write operator markers into the current Jetson power monitor event log."""

from __future__ import annotations

import argparse
from pathlib import Path

from jetson_power_lib import (
    append_json_record,
    build_event,
    default_log_dir_from_tools_dir,
    events_file_path,
    read_boot_id,
    read_uptime_s,
)


SCRIPT_DIR = Path(__file__).resolve().parent


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--label", required=True, help="Marker label, for example steering_test_start.")
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=default_log_dir_from_tools_dir(SCRIPT_DIR),
        help="Directory containing the Jetson power monitor logs.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    boot_id = read_boot_id()
    record = build_event(
        "operator_marker",
        boot_id=boot_id,
        label=str(args.label),
        uptime_s=read_uptime_s(),
    )
    append_json_record(events_file_path(Path(args.log_dir), boot_id), record, sync=True)
    print(f"Recorded marker '{args.label}' for boot {boot_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
