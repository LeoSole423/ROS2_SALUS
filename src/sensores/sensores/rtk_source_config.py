"""RTK source configuration; secrets must never appear in errors/telemetry."""
from __future__ import annotations

import os
import tempfile
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml


@dataclass(frozen=True)
class RtkSource:
    id: str
    label: str
    host: str
    port: int
    mountpoint: str
    username: str = field(repr=False)
    password: str = field(repr=False)


def load_sources(path: Path) -> tuple[list[RtkSource], str]:
    try:
        doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        sources = []
        for raw in doc.get("sources", []):
            source = RtkSource(
                id=str(raw["id"]).strip(), label=str(raw.get("label") or raw["id"]).strip(),
                host=str(raw["host"]).strip(), port=int(raw.get("port", 2101)),
                mountpoint=str(raw["mountpoint"]).strip(),
                username=str(raw.get("username", "")).strip(),
                password=str(raw.get("password", "")),
            )
            validate_source(source)
            sources.append(source)
        if not sources or len({s.id for s in sources}) != len(sources):
            raise ValueError
        active = str(doc.get("active_source_id") or "").strip()
        if active and active not in {s.id for s in sources}:
            raise ValueError
        return sources, active
    except (OSError, ValueError, TypeError, KeyError, AttributeError, yaml.YAMLError):
        raise ValueError("invalid_or_unreadable_rtk_sources_config") from None


def validate_source(source: RtkSource) -> None:
    if not source.id or not source.host or not source.mountpoint or not 1 <= source.port <= 65535:
        raise ValueError("invalid_rtk_source")
    if any(c in value for value in (source.id, source.label, source.host, source.mountpoint,
                                    source.username, source.password) for c in ("\r", "\n")):
        raise ValueError("invalid_rtk_source")
    if any(c in source.host for c in ("/", "@", " ")) or any(c in source.mountpoint for c in (" ", "?", "#")):
        raise ValueError("invalid_rtk_endpoint")


def save_sources(path: Path, sources: list[RtkSource], active: str) -> None:
    """Atomic, owner-only replacement; no partial writes or world-readable secrets."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".rtk-", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump({"active_source_id": active, "sources": [asdict(s) for s in sources]},
                           stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
