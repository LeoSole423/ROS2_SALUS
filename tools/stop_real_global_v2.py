#!/usr/bin/env python3
"""Stop only the real Global V2 launch inside the dedicated ROS container.

systemd stopping the host's `docker exec` client does not stop its ROS children.
Send SIGINT to the launch supervisor, then wait for it and its descendants.
The service's ExecStop falls back to stopping the dedicated container on error.
"""
import argparse
import os
from pathlib import Path
import signal
import sys
import time

LAUNCHES = {"real_global_v2.launch.py", "real_global_v2_wifi.launch.py"}


def is_real_launch(args):
    if "--show-args" in args:
        return False
    for index, arg in enumerate(args[:-2]):
        if Path(arg).name != "ros2" or args[index + 1] != "launch":
            continue
        target = args[index + 2]
        if target == "navegacion_gps" and index + 3 < len(args):
            target = args[index + 3]
        return Path(target).name in LAUNCHES
    return False


def processes():
    result = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            # comm can contain spaces or parentheses; fields after its last ')'
            # start at state (field 3), with starttime at field 22.
            fields = entry.joinpath("stat").read_text().rsplit(")", 1)[1].split()
            args = entry.joinpath("cmdline").read_bytes().decode(errors="replace").rstrip("\0").split("\0")
            result[int(entry.name)] = (int(fields[1]), fields[19], fields[0], args)
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-stopped", action="store_true")
    args = parser.parse_args()
    initial = processes()
    roots = {pid for pid, info in initial.items() if is_real_launch(info[3])}
    if args.check_stopped:
        if roots:
            print("Refusing duplicate real Global V2 launch", file=sys.stderr)
            return 1
        return 0
    if not roots:
        print("No real Global V2 launch to stop")
        return 0
    owned = set(roots)
    while True:
        expanded = owned | {pid for pid, info in initial.items() if info[0] in owned}
        if expanded == owned:
            break
        owned = expanded
    for pid in roots:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 40
    while time.monotonic() < deadline:
        current = processes()
        alive = {pid for pid in owned if pid in current and current[pid][1] == initial[pid][1]
                 and current[pid][2] != "Z"}
        if not alive:
            print("Real Global V2 launch and children stopped")
            return 0
        time.sleep(0.2)
    print("ROS shutdown timed out; dedicated container stop required", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
