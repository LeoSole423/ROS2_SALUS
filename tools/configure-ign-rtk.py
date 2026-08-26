#!/usr/bin/env python3
"""Provision local IGN credentials interactively; never accept secrets in argv."""
import getpass
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "sensores"))
from sensores.rtk_source_config import RtkSource, load_sources, save_sources, validate_source


def main():
    path = ROOT / "src/sensores/config/rtk_sources.local.yaml"
    original = path if path.exists() else path.with_name("rtk_sources.yaml")
    sources, _ = load_sources(original)
    username = getpass.getpass("IGN username (hidden): ").strip()
    password = getpass.getpass("IGN password (hidden): ")
    if not username or not password:
        raise ValueError("missing_credentials")
    ign = RtkSource("ign_ucor", "IGN UCOR", "ntrip.ign.gob.ar", 2101, "UCOR-v3.3", username, password)
    validate_source(ign)
    save_sources(path, [ign] + [s for s in sources if s.id != ign.id], ign.id)
    print("IGN UCOR configured in owner-only local file. No ROS process started/restarted.")


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt):
        print("RTK configuration not saved; check permissions/configuration and retry.", file=sys.stderr)
        sys.exit(1)
