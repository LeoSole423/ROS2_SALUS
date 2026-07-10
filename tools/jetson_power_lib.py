#!/usr/bin/env python3
"""Helpers for host-side Jetson power monitoring."""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import socket
import subprocess
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Optional


DEFAULT_LOG_PREFIX = "jetson_power_monitor"
DEFAULT_FAST_SAMPLE_HZ = 100.0
DEFAULT_CONTEXT_HZ = 2.0
DEFAULT_SEGMENT_SECONDS = 10.0
DEFAULT_WINDOW_SECONDS = 15.0
DEFAULT_FAST_SYNC_INTERVAL_MS = 100.0
DEFAULT_CONTEXT_SYNC_INTERVAL_MS = 500.0
DEFAULT_LOW_VDD_IN_MV = 4750.0
DEFAULT_CRITICAL_VDD_IN_MV = 4650.0
DEFAULT_STABLE_VDD_IN_MV = 4950.0
DEFAULT_FAST_DROP_WINDOW_MS = 250.0
DEFAULT_FAST_DROP_DELTA_MV = 180.0
DEFAULT_LOW_PERSIST_MS = 150.0

FAST_PREFIX = "fast"
CONTEXT_PREFIX = "context"
EVENTS_PREFIX = "events"
ANALYSIS_PREFIX = "analysis"
LEGACY_SAMPLES_PREFIX = "samples"

SEGMENT_RE = re.compile(r"^(fast|context)-([0-9a-fA-F-]+)-(\d{6})\.jsonl$")
LEGACY_RE = re.compile(r"^(samples|events|analysis)-([0-9a-fA-F-]+)\.(jsonl|json)$")

TEGRASTATS_TEMP_RE = re.compile(r"tj@([0-9.]+)C")
TEGRASTATS_POWER_RE = re.compile(r"(VDD_[A-Z_]+)\s+([0-9]+)mW/([0-9]+)mW/([0-9]+)mW")
TEGRASTATS_RAM_RE = re.compile(r"RAM\s+([0-9]+)/([0-9]+)MB")


@dataclass(frozen=True, slots=True)
class Thresholds:
    low_vdd_in_mv: float = DEFAULT_LOW_VDD_IN_MV
    critical_vdd_in_mv: float = DEFAULT_CRITICAL_VDD_IN_MV
    stable_vdd_in_mv: float = DEFAULT_STABLE_VDD_IN_MV
    fast_drop_window_ms: float = DEFAULT_FAST_DROP_WINDOW_MS
    fast_drop_delta_mv: float = DEFAULT_FAST_DROP_DELTA_MV
    low_persist_ms: float = DEFAULT_LOW_PERSIST_MS
    window_seconds: float = DEFAULT_WINDOW_SECONDS

    @property
    def fast_drop_rate_mv_per_s(self) -> float:
        return float(self.fast_drop_delta_mv) / max(
            1.0e-6,
            float(self.fast_drop_window_ms) / 1000.0,
        )


@dataclass(frozen=True, slots=True)
class MonitorProfile:
    name: str
    fast_sample_hz: float
    context_hz: float
    segment_seconds: float
    fast_sync_interval_ms: float
    context_sync_interval_ms: float
    thresholds: Thresholds


@dataclass(frozen=True, slots=True)
class ChannelPaths:
    name: str
    voltage_path: Path
    current_path: Optional[Path]


@dataclass(frozen=True, slots=True)
class JsonlLoadResult:
    records: tuple[dict[str, Any], ...]
    corrupt_tail_detected: bool
    corrupt_reason: Optional[str]
    corrupt_path: Optional[str]


@dataclass(frozen=True, slots=True)
class BootData:
    fast_samples: tuple[dict[str, Any], ...]
    context_samples: tuple[dict[str, Any], ...]
    events: tuple[dict[str, Any], ...]
    corrupt_tail_detected: bool
    corrupt_paths: tuple[str, ...]


AGGRESSIVE_PROFILE = MonitorProfile(
    name="aggressive",
    fast_sample_hz=DEFAULT_FAST_SAMPLE_HZ,
    context_hz=DEFAULT_CONTEXT_HZ,
    segment_seconds=DEFAULT_SEGMENT_SECONDS,
    fast_sync_interval_ms=DEFAULT_FAST_SYNC_INTERVAL_MS,
    context_sync_interval_ms=DEFAULT_CONTEXT_SYNC_INTERVAL_MS,
    thresholds=Thresholds(),
)

MODERATE_PROFILE = MonitorProfile(
    name="moderate",
    fast_sample_hz=40.0,
    context_hz=1.0,
    segment_seconds=15.0,
    fast_sync_interval_ms=200.0,
    context_sync_interval_ms=1000.0,
    thresholds=Thresholds(
        low_vdd_in_mv=DEFAULT_LOW_VDD_IN_MV,
        critical_vdd_in_mv=DEFAULT_CRITICAL_VDD_IN_MV,
        stable_vdd_in_mv=DEFAULT_STABLE_VDD_IN_MV,
        fast_drop_window_ms=350.0,
        fast_drop_delta_mv=DEFAULT_FAST_DROP_DELTA_MV,
        low_persist_ms=200.0,
        window_seconds=DEFAULT_WINDOW_SECONDS,
    ),
)

PROFILE_MAP = {
    AGGRESSIVE_PROFILE.name: AGGRESSIVE_PROFILE,
    MODERATE_PROFILE.name: MODERATE_PROFILE,
}


def profile_from_name(name: str) -> MonitorProfile:
    normalized = str(name).strip().lower()
    profile = PROFILE_MAP.get(normalized)
    if profile is None:
        raise ValueError(
            f"Unsupported profile '{name}'. Expected one of: {', '.join(sorted(PROFILE_MAP))}"
        )
    return profile


def thresholds_from_mapping(payload: dict[str, Any], *, fallback: Thresholds) -> Thresholds:
    return Thresholds(
        low_vdd_in_mv=float(payload.get("low_vdd_in_mv", fallback.low_vdd_in_mv)),
        critical_vdd_in_mv=float(
            payload.get("critical_vdd_in_mv", fallback.critical_vdd_in_mv)
        ),
        stable_vdd_in_mv=float(payload.get("stable_vdd_in_mv", fallback.stable_vdd_in_mv)),
        fast_drop_window_ms=float(
            payload.get("fast_drop_window_ms", fallback.fast_drop_window_ms)
        ),
        fast_drop_delta_mv=float(
            payload.get("fast_drop_delta_mv", fallback.fast_drop_delta_mv)
        ),
        low_persist_ms=float(payload.get("low_persist_ms", fallback.low_persist_ms)),
        window_seconds=float(payload.get("window_seconds", fallback.window_seconds)),
    )


def profile_from_monitor_started_event(event: dict[str, Any]) -> MonitorProfile:
    config = event.get("config")
    config = config if isinstance(config, dict) else {}
    profile_name = str(config.get("profile", AGGRESSIVE_PROFILE.name))
    base = PROFILE_MAP.get(profile_name, AGGRESSIVE_PROFILE)
    return MonitorProfile(
        name=profile_name,
        fast_sample_hz=float(config.get("fast_sample_hz", base.fast_sample_hz)),
        context_hz=float(config.get("context_hz", base.context_hz)),
        segment_seconds=float(config.get("segment_seconds", base.segment_seconds)),
        fast_sync_interval_ms=float(
            config.get("fast_sync_interval_ms", base.fast_sync_interval_ms)
        ),
        context_sync_interval_ms=float(
            config.get("context_sync_interval_ms", base.context_sync_interval_ms)
        ),
        thresholds=thresholds_from_mapping(config, fallback=base.thresholds),
    )


def profile_from_events(events: Iterable[dict[str, Any]], *, fallback: MonitorProfile) -> MonitorProfile:
    started_events = [event for event in events if event.get("event") == "monitor_started"]
    if not started_events:
        return fallback
    return profile_from_monitor_started_event(started_events[-1])


def repo_root_from_tools_dir(tools_dir: Path) -> Path:
    return tools_dir.resolve().parent


def default_log_dir_from_tools_dir(tools_dir: Path) -> Path:
    return repo_root_from_tools_dir(tools_dir) / "log" / DEFAULT_LOG_PREFIX


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8").strip()


def read_float(path: Path, *, scale: float = 1.0) -> float:
    return float(read_text(path)) * float(scale)


def read_boot_id() -> str:
    return read_text(Path("/proc/sys/kernel/random/boot_id"))


def read_uptime_s() -> float:
    return float(read_text(Path("/proc/uptime")).split()[0])


def read_loadavg() -> dict[str, float]:
    parts = read_text(Path("/proc/loadavg")).split()
    return {
        "load1": float(parts[0]),
        "load5": float(parts[1]),
        "load15": float(parts[2]),
    }


def iso_timestamp(now_s: Optional[float] = None) -> str:
    now_s = time.time() if now_s is None else float(now_s)
    wall = time.localtime(now_s)
    return time.strftime("%Y-%m-%dT%H:%M:%S", wall) + f".{int((now_s % 1.0) * 1000):03d}"


def hostname() -> str:
    return socket.gethostname()


def events_file_path(log_dir: Path, boot_id: str) -> Path:
    return log_dir.resolve() / f"{EVENTS_PREFIX}-{boot_id}.jsonl"


def analysis_file_path(log_dir: Path, boot_id: str) -> Path:
    return log_dir.resolve() / f"{ANALYSIS_PREFIX}-{boot_id}.json"


def legacy_samples_path(log_dir: Path, boot_id: str) -> Path:
    return log_dir.resolve() / f"{LEGACY_SAMPLES_PREFIX}-{boot_id}.jsonl"


def segment_file_path(log_dir: Path, prefix: str, boot_id: str, index: int) -> Path:
    return log_dir.resolve() / f"{prefix}-{boot_id}-{int(index):06d}.jsonl"


def list_segment_paths(log_dir: Path, prefix: str, boot_id: str) -> list[Path]:
    return sorted(log_dir.resolve().glob(f"{prefix}-{boot_id}-*.jsonl"))


def list_monitored_boots(log_dir: Path) -> list[str]:
    if not log_dir.exists():
        return []
    boot_mtimes: dict[str, float] = {}
    for path in log_dir.iterdir():
        if not path.is_file():
            continue
        boot_id: Optional[str] = None
        segment_match = SEGMENT_RE.match(path.name)
        legacy_match = LEGACY_RE.match(path.name)
        if segment_match is not None:
            boot_id = segment_match.group(2)
        elif legacy_match is not None:
            boot_id = legacy_match.group(2)
        if boot_id is None:
            continue
        boot_mtimes[boot_id] = max(boot_mtimes.get(boot_id, 0.0), path.stat().st_mtime)
    return [boot_id for boot_id, _mtime in sorted(boot_mtimes.items(), key=lambda item: item[1])]


def previous_boot_id(log_dir: Path, current_boot_id: str) -> Optional[str]:
    boot_ids = [boot_id for boot_id in list_monitored_boots(log_dir) if boot_id != current_boot_id]
    return boot_ids[-1] if boot_ids else None


def discover_ina3221_channels(
    *,
    hwmon_root: Path = Path("/sys/class/hwmon"),
) -> dict[str, ChannelPaths]:
    channels: dict[str, ChannelPaths] = {}
    for hwmon_dir in sorted(hwmon_root.glob("hwmon*")):
        name_path = hwmon_dir / "name"
        if not name_path.exists():
            continue
        if read_text(name_path) != "ina3221":
            continue
        for label_path in sorted(hwmon_dir.glob("in*_label")):
            match = re.fullmatch(r"in(\d+)_label", label_path.name)
            if match is None:
                continue
            index = match.group(1)
            label = read_text(label_path)
            voltage_path = hwmon_dir / f"in{index}_input"
            current_path = hwmon_dir / f"curr{index}_input"
            channels[label] = ChannelPaths(
                name=label,
                voltage_path=voltage_path,
                current_path=current_path if current_path.exists() else None,
            )
    if "VDD_IN" not in channels:
        raise FileNotFoundError(
            "Could not discover VDD_IN channel under /sys/class/hwmon/*/ina3221"
        )
    return channels


def read_thermal_zones(
    *,
    thermal_root: Path = Path("/sys/devices/virtual/thermal"),
) -> dict[str, float]:
    temperatures_c: dict[str, float] = {}
    for zone_dir in sorted(thermal_root.glob("thermal_zone*")):
        type_path = zone_dir / "type"
        temp_path = zone_dir / "temp"
        if not type_path.exists() or not temp_path.exists():
            continue
        try:
            zone_name = read_text(type_path)
            raw_temp = float(read_text(temp_path))
        except (OSError, ValueError):
            continue
        if raw_temp > 1000.0:
            raw_temp /= 1000.0
        temperatures_c[zone_name] = raw_temp
    return temperatures_c


def parse_tegrastats_line(line: str) -> dict[str, Any]:
    snapshot: dict[str, Any] = {"raw": line.strip()}
    temp_match = TEGRASTATS_TEMP_RE.search(line)
    if temp_match is not None:
        snapshot["tj_c"] = float(temp_match.group(1))
    ram_match = TEGRASTATS_RAM_RE.search(line)
    if ram_match is not None:
        snapshot["ram_used_mb"] = int(ram_match.group(1))
        snapshot["ram_total_mb"] = int(ram_match.group(2))
    power: dict[str, dict[str, int]] = {}
    for rail, current_mw, avg_mw, max_mw in TEGRASTATS_POWER_RE.findall(line):
        power[rail] = {
            "current_mw": int(current_mw),
            "avg_mw": int(avg_mw),
            "max_mw": int(max_mw),
        }
    if power:
        snapshot["power"] = power
    return snapshot


class TegrastatsPoller:
    """Continuously read tegrastats in the background."""

    def __init__(
        self,
        *,
        interval_hz: float,
        on_error: Callable[[str, dict[str, Any]], None],
    ) -> None:
        self._interval_ms = max(100, int(round(1000.0 / max(0.1, float(interval_hz)))))
        self._on_error = on_error
        self._binary = shutil.which("tegrastats")
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._process: Optional[subprocess.Popen[str]] = None
        self._latest_lock = threading.Lock()
        self._latest_snapshot: Optional[dict[str, Any]] = None

    def start(self) -> None:
        if self._binary is None:
            self._on_error("tegrastats_missing", {"reason": "binary_not_found"})
            return
        try:
            self._process = subprocess.Popen(
                [self._binary, "--interval", str(self._interval_ms)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            self._process = None
            self._on_error("tegrastats_missing", {"reason": "spawn_failed", "error": str(exc)})
            return
        self._thread = threading.Thread(target=self._run, name="tegrastats-poller", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        process = self._process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def latest_snapshot(self) -> Optional[dict[str, Any]]:
        with self._latest_lock:
            return None if self._latest_snapshot is None else dict(self._latest_snapshot)

    def _run(self) -> None:
        assert self._process is not None
        stdout = self._process.stdout
        if stdout is None:
            self._on_error("tegrastats_missing", {"reason": "stdout_unavailable"})
            return
        while (not self._stop_event.is_set()) and self._process.poll() is None:
            line = stdout.readline()
            if not line:
                time.sleep(0.05)
                continue
            snapshot = parse_tegrastats_line(line)
            snapshot["monotonic_s"] = time.monotonic()
            snapshot["timestamp"] = iso_timestamp()
            with self._latest_lock:
                self._latest_snapshot = snapshot
        if not self._stop_event.is_set():
            self._on_error(
                "tegrastats_missing",
                {"reason": "process_exited", "returncode": self._process.returncode},
            )


class SegmentedJsonlWriter:
    """Low-overhead JSONL writer with time-based segment rotation."""

    def __init__(
        self,
        *,
        log_dir: Path,
        prefix: str,
        boot_id: str,
        segment_seconds: float,
        sync_interval_ms: float,
    ) -> None:
        self._log_dir = log_dir.resolve()
        self._prefix = str(prefix)
        self._boot_id = str(boot_id)
        self._segment_seconds = max(1.0, float(segment_seconds))
        self._sync_interval_s = max(0.0, float(sync_interval_ms) / 1000.0)
        self._segment_index = 0
        self._segment_started_monotonic_s: Optional[float] = None
        self._fd: Optional[int] = None
        self._path: Optional[Path] = None
        self._last_sync_monotonic_s: Optional[float] = None
        self._log_dir.mkdir(parents=True, exist_ok=True)

    @property
    def current_path(self) -> Optional[Path]:
        return self._path

    def write(
        self,
        record: dict[str, Any],
        *,
        monotonic_s: float,
        force_sync: bool = False,
    ) -> None:
        self._rotate_if_needed(monotonic_s=monotonic_s)
        assert self._fd is not None
        payload = (json.dumps(record, ensure_ascii=True, sort_keys=True) + "\n").encode("utf-8")
        os.write(self._fd, payload)
        now_s = float(monotonic_s)
        if force_sync or self._should_sync(now_s):
            os.fdatasync(self._fd)
            self._last_sync_monotonic_s = now_s

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.fdatasync(self._fd)
            except OSError:
                pass
            os.close(self._fd)
            self._fd = None
            self._path = None

    def _rotate_if_needed(self, *, monotonic_s: float) -> None:
        if self._fd is None:
            self._open_next_segment(monotonic_s=monotonic_s)
            return
        assert self._segment_started_monotonic_s is not None
        if float(monotonic_s) - self._segment_started_monotonic_s < self._segment_seconds:
            return
        self.close()
        self._open_next_segment(monotonic_s=monotonic_s)

    def _open_next_segment(self, *, monotonic_s: float) -> None:
        self._segment_index += 1
        self._segment_started_monotonic_s = float(monotonic_s)
        self._path = segment_file_path(
            self._log_dir,
            self._prefix,
            self._boot_id,
            self._segment_index,
        )
        self._fd = os.open(
            self._path,
            os.O_CREAT | os.O_WRONLY | os.O_APPEND,
            0o664,
        )
        self._last_sync_monotonic_s = None

    def _should_sync(self, now_s: float) -> bool:
        if self._sync_interval_s <= 0.0:
            return False
        if self._last_sync_monotonic_s is None:
            return True
        return (now_s - self._last_sync_monotonic_s) >= self._sync_interval_s


def append_json_record(path: Path, record: dict[str, Any], *, sync: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(path, os.O_CREAT | os.O_WRONLY | os.O_APPEND, 0o664)
    try:
        payload = (json.dumps(record, ensure_ascii=True, sort_keys=True) + "\n").encode("utf-8")
        os.write(fd, payload)
        if sync:
            os.fdatasync(fd)
    finally:
        os.close(fd)


def read_analysis_file(log_dir: Path, boot_id: str) -> Optional[dict[str, Any]]:
    path = analysis_file_path(log_dir, boot_id)
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def write_analysis_file(log_dir: Path, boot_id: str, analysis: dict[str, Any]) -> Path:
    path = analysis_file_path(log_dir, boot_id)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(analysis, ensure_ascii=True, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    return path


def _decode_json_bytes(raw_line: bytes) -> Optional[dict[str, Any]]:
    try:
        return json.loads(raw_line.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None


def load_jsonl(path: Path) -> JsonlLoadResult:
    if not path.exists():
        return JsonlLoadResult(records=(), corrupt_tail_detected=False, corrupt_reason=None, corrupt_path=None)
    records: list[dict[str, Any]] = []
    with path.open("rb") as handle:
        for raw_line in handle:
            stripped = raw_line.strip()
            if not stripped:
                continue
            if b"\x00" in stripped:
                return JsonlLoadResult(
                    records=tuple(records),
                    corrupt_tail_detected=True,
                    corrupt_reason="nul_tail",
                    corrupt_path=str(path),
                )
            decoded = _decode_json_bytes(stripped)
            if decoded is None:
                return JsonlLoadResult(
                    records=tuple(records),
                    corrupt_tail_detected=True,
                    corrupt_reason="invalid_json_tail",
                    corrupt_path=str(path),
                )
            records.append(decoded)
    return JsonlLoadResult(records=tuple(records), corrupt_tail_detected=False, corrupt_reason=None, corrupt_path=None)


def load_jsonl_many(paths: Iterable[Path]) -> JsonlLoadResult:
    records: list[dict[str, Any]] = []
    corrupt_tail_detected = False
    corrupt_reason = None
    corrupt_path = None
    for path in paths:
        result = load_jsonl(path)
        records.extend(result.records)
        if result.corrupt_tail_detected and not corrupt_tail_detected:
            corrupt_tail_detected = True
            corrupt_reason = result.corrupt_reason
            corrupt_path = result.corrupt_path
            break
    return JsonlLoadResult(
        records=tuple(records),
        corrupt_tail_detected=corrupt_tail_detected,
        corrupt_reason=corrupt_reason,
        corrupt_path=corrupt_path,
    )


def load_boot_data(log_dir: Path, boot_id: str) -> BootData:
    fast_paths = list_segment_paths(log_dir, FAST_PREFIX, boot_id)
    context_paths = list_segment_paths(log_dir, CONTEXT_PREFIX, boot_id)
    events_path = events_file_path(log_dir, boot_id)

    if fast_paths:
        fast_result = load_jsonl_many(fast_paths)
    else:
        fast_result = load_jsonl(legacy_samples_path(log_dir, boot_id))
    context_result = load_jsonl_many(context_paths)
    event_result = load_jsonl(events_path)

    corrupt_paths: list[str] = []
    for result in (fast_result, context_result, event_result):
        if result.corrupt_tail_detected and result.corrupt_path:
            corrupt_paths.append(result.corrupt_path)

    return BootData(
        fast_samples=fast_result.records,
        context_samples=context_result.records,
        events=event_result.records,
        corrupt_tail_detected=bool(corrupt_paths),
        corrupt_paths=tuple(corrupt_paths),
    )


def build_event(
    event_type: str,
    *,
    boot_id: str,
    monotonic_s: Optional[float] = None,
    wall_time_s: Optional[float] = None,
    **payload: Any,
) -> dict[str, Any]:
    now_s = time.time() if wall_time_s is None else float(wall_time_s)
    monotonic_now_s = time.monotonic() if monotonic_s is None else float(monotonic_s)
    return {
        "event": event_type,
        "boot_id": boot_id,
        "hostname": hostname(),
        "timestamp": iso_timestamp(now_s),
        "wall_time_s": now_s,
        "monotonic_s": monotonic_now_s,
        **payload,
    }


def monotonic_from_record(record: dict[str, Any]) -> float:
    if record.get("monotonic_ns") is not None:
        return float(record["monotonic_ns"]) / 1.0e9
    if record.get("monotonic_s") is not None:
        return float(record["monotonic_s"])
    if record.get("uptime_s") is not None:
        return float(record["uptime_s"])
    return 0.0


def last_window(records: Iterable[dict[str, Any]], *, window_seconds: float) -> list[dict[str, Any]]:
    rows = list(records)
    if not rows:
        return []
    last_monotonic_s = monotonic_from_record(rows[-1])
    threshold = last_monotonic_s - float(window_seconds)
    return [row for row in rows if monotonic_from_record(row) >= threshold]


def latest_operator_markers(events: Iterable[dict[str, Any]], *, limit: int = 5) -> list[str]:
    labels = [
        str(event.get("label", ""))
        for event in events
        if event.get("event") == "operator_marker" and event.get("label")
    ]
    return labels[-max(0, limit):]


def recent_fast_samples(records: Iterable[dict[str, Any]], *, limit: int = 5) -> list[dict[str, Any]]:
    rows = list(records)
    return rows[-max(0, limit):]


def classify_boot(data: BootData, *, profile: MonitorProfile) -> dict[str, Any]:
    fast_samples = list(data.fast_samples)
    context_samples = list(data.context_samples)
    events = list(data.events)

    started_events = [event for event in events if event.get("event") == "monitor_started"]
    clean_stop_events = [event for event in events if event.get("event") == "monitor_stopped_cleanly"]
    last_started_monotonic_s = (
        max(monotonic_from_record(event) for event in started_events) if started_events else None
    )
    last_clean_stop_monotonic_s = (
        max(monotonic_from_record(event) for event in clean_stop_events)
        if clean_stop_events
        else None
    )
    session_closed_cleanly = bool(
        clean_stop_events
        and last_clean_stop_monotonic_s is not None
        and (
            last_started_monotonic_s is None
            or last_clean_stop_monotonic_s >= last_started_monotonic_s
        )
    )

    if session_closed_cleanly:
        return {
            "classification": "clean_shutdown",
            "label": "reinicio limpio",
            "detail": "Monitor stopped cleanly before reboot.",
            "window_seconds": profile.thresholds.window_seconds,
            "sample_total": len(fast_samples),
            "context_total": len(context_samples),
            "event_total": len(events),
            "corrupt_tail_detected": bool(data.corrupt_tail_detected),
            "corrupt_paths": list(data.corrupt_paths),
            "markers": latest_operator_markers(events),
            "upstream_note": None,
            "session_closed_cleanly": True,
        }

    final_fast_window = last_window(fast_samples, window_seconds=profile.thresholds.window_seconds)
    final_event_window = last_window(events, window_seconds=profile.thresholds.window_seconds)

    min_vdd_in_mv = min(
        (
            float(sample["vdd_in_mv"])
            for sample in final_fast_window
            if sample.get("vdd_in_mv") is not None
        ),
        default=float("nan"),
    )
    max_vdd_in_ma = max(
        (
            float(sample["vdd_in_ma"])
            for sample in final_fast_window
            if sample.get("vdd_in_ma") is not None
        ),
        default=float("nan"),
    )
    min_dv_dt_mv_per_s = min(
        (
            float(sample["dv_dt_mv_per_s"])
            for sample in final_fast_window
            if sample.get("dv_dt_mv_per_s") is not None
        ),
        default=0.0,
    )

    saw_low = any(event.get("event") == "vdd_in_low" for event in final_event_window)
    saw_critical = any(
        event.get("event") == "vdd_in_critical" for event in final_event_window
    )
    saw_fast_drop = any(
        event.get("event") == "fast_voltage_drop" for event in final_event_window
    )

    if (min_vdd_in_mv == min_vdd_in_mv) and min_vdd_in_mv <= profile.thresholds.low_vdd_in_mv:
        saw_low = True
    if (min_vdd_in_mv == min_vdd_in_mv) and min_vdd_in_mv <= profile.thresholds.critical_vdd_in_mv:
        saw_critical = True
    if min_dv_dt_mv_per_s <= -profile.thresholds.fast_drop_rate_mv_per_s:
        saw_fast_drop = True

    stable_internal_rail = (
        (min_vdd_in_mv == min_vdd_in_mv)
        and min_vdd_in_mv >= profile.thresholds.stable_vdd_in_mv
        and (not saw_fast_drop)
    )

    insufficient_samples = len(fast_samples) < 2
    monitor_gap_unknown = (not final_fast_window) or (
        insufficient_samples and (not saw_low) and (not saw_critical) and (not saw_fast_drop)
    ) or ((not started_events) and (not saw_low) and (not saw_critical) and (not saw_fast_drop))

    if monitor_gap_unknown:
        classification = "monitor_gap_unknown"
        label = "monitor gap unknown"
        detail = (
            "Insufficient fast samples or monitor did not start early enough to classify the reboot."
        )
        upstream_note = None
    elif saw_low or saw_critical or saw_fast_drop:
        classification = "internal_rail_drop_suspected"
        label = "internal rail drop suspected"
        detail = "Visible instability was detected on the monitored internal rail before the abrupt reboot."
        upstream_note = None
    elif stable_internal_rail:
        classification = "abrupt_reset_internal_rail_stable"
        label = "abrupt reset with internal rail stable"
        detail = "Abrupt reboot detected while the monitored internal rail remained stable in the last valid samples."
        upstream_note = (
            "No visible collapse on internal rail; upstream 19 V / regulator path remains unobserved."
        )
    else:
        classification = "monitor_gap_unknown"
        label = "monitor gap unknown"
        detail = "Abrupt reboot detected, but the available last-window data is not conclusive."
        upstream_note = None

    return {
        "classification": classification,
        "label": label,
        "detail": detail,
        "window_seconds": profile.thresholds.window_seconds,
        "sample_total": len(fast_samples),
        "context_total": len(context_samples),
        "event_total": len(events),
        "last_valid_timestamp": (
            str(final_fast_window[-1].get("timestamp")) if final_fast_window else None
        ),
        "last_valid_uptime_s": (
            float(final_fast_window[-1].get("uptime_s"))
            if final_fast_window and final_fast_window[-1].get("uptime_s") is not None
            else None
        ),
        "min_vdd_in_mv_window": min_vdd_in_mv,
        "max_vdd_in_ma_window": max_vdd_in_ma,
        "min_dv_dt_mv_per_s_window": min_dv_dt_mv_per_s,
        "stable_internal_rail": bool(stable_internal_rail),
        "saw_low": bool(saw_low),
        "saw_critical": bool(saw_critical),
        "saw_fast_drop": bool(saw_fast_drop),
        "monitor_gap_unknown": bool(monitor_gap_unknown),
        "corrupt_tail_detected": bool(data.corrupt_tail_detected),
        "corrupt_paths": list(data.corrupt_paths),
        "markers": latest_operator_markers(events),
        "recent_fast_samples": recent_fast_samples(final_fast_window, limit=5),
        "upstream_note": upstream_note,
        "session_closed_cleanly": False,
        "profile": {
            "name": profile.name,
            "fast_sample_hz": profile.fast_sample_hz,
            "context_hz": profile.context_hz,
            "segment_seconds": profile.segment_seconds,
            "fast_sync_interval_ms": profile.fast_sync_interval_ms,
            "context_sync_interval_ms": profile.context_sync_interval_ms,
            "thresholds": asdict(profile.thresholds),
        },
    }


def summarize_boot(
    log_dir: Path,
    boot_id: str,
    *,
    fallback_profile: MonitorProfile = AGGRESSIVE_PROFILE,
    persist: bool = False,
) -> dict[str, Any]:
    analysis = read_analysis_file(log_dir, boot_id)
    current_boot_id = read_boot_id()
    if analysis is not None and boot_id != current_boot_id:
        return analysis

    data = load_boot_data(log_dir, boot_id)
    profile = profile_from_events(data.events, fallback=fallback_profile)
    summary = classify_boot(data, profile=profile)
    summary.update(
        {
            "boot_id": boot_id,
            "fast_paths": [str(path) for path in list_segment_paths(log_dir, FAST_PREFIX, boot_id)],
            "context_paths": [
                str(path) for path in list_segment_paths(log_dir, CONTEXT_PREFIX, boot_id)
            ],
            "events_path": str(events_file_path(log_dir, boot_id)),
            "analysis_path": str(analysis_file_path(log_dir, boot_id)),
            "generated_at": iso_timestamp(),
        }
    )
    if persist:
        write_analysis_file(log_dir, boot_id, summary)
    return summary


class VoltageWindow:
    """Track short-term power metrics used by the monitor."""

    def __init__(self, *, thresholds: Thresholds) -> None:
        self._thresholds = thresholds
        self._samples: deque[tuple[float, float]] = deque()
        self._low_since_monotonic: Optional[float] = None
        self._low_event_active = False
        self._critical_event_active = False
        self._fast_drop_event_active = False

    def update(self, *, monotonic_s: float, vdd_in_mv: float) -> dict[str, Any]:
        self._samples.append((float(monotonic_s), float(vdd_in_mv)))
        trim_before = float(monotonic_s) - max(1.0, float(self._thresholds.window_seconds))
        while len(self._samples) > 1 and self._samples[0][0] < trim_before:
            self._samples.popleft()

        voltages = [item[1] for item in self._samples]
        comparison = self._samples[0]
        fast_window_s = max(1.0e-3, float(self._thresholds.fast_drop_window_ms) / 1000.0)
        for candidate in reversed(self._samples):
            if (float(monotonic_s) - candidate[0]) >= fast_window_s:
                comparison = candidate
                break
        dt_s = max(1.0e-6, float(monotonic_s) - comparison[0])
        dv_dt_mv_per_s = (float(vdd_in_mv) - comparison[1]) / dt_s

        if float(vdd_in_mv) <= self._thresholds.low_vdd_in_mv:
            if self._low_since_monotonic is None:
                self._low_since_monotonic = float(monotonic_s)
        else:
            self._low_since_monotonic = None
            self._low_event_active = False

        low_persist_s = (
            0.0
            if self._low_since_monotonic is None
            else max(0.0, float(monotonic_s) - self._low_since_monotonic)
        )
        low_triggered = (
            low_persist_s >= float(self._thresholds.low_persist_ms) / 1000.0
            and (not self._low_event_active)
        )
        if low_triggered:
            self._low_event_active = True

        critical_triggered = (
            float(vdd_in_mv) <= self._thresholds.critical_vdd_in_mv
            and (not self._critical_event_active)
        )
        if float(vdd_in_mv) > self._thresholds.critical_vdd_in_mv:
            self._critical_event_active = False
        elif critical_triggered:
            self._critical_event_active = True

        fast_drop_triggered = (
            dv_dt_mv_per_s <= -self._thresholds.fast_drop_rate_mv_per_s
            and (not self._fast_drop_event_active)
        )
        if dv_dt_mv_per_s > -0.5 * self._thresholds.fast_drop_rate_mv_per_s:
            self._fast_drop_event_active = False
        elif fast_drop_triggered:
            self._fast_drop_event_active = True

        return {
            "dv_dt_mv_per_s": dv_dt_mv_per_s,
            "low_persist_s": low_persist_s,
            "low_triggered": low_triggered,
            "critical_triggered": critical_triggered,
            "fast_drop_triggered": fast_drop_triggered,
        }
