from datetime import datetime
from pathlib import Path
import math
import re
from typing import Any, Dict, List, Optional, Tuple

import yaml


DatumProfile = Dict[str, Any]


def utc_now_iso() -> str:
    return datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _to_finite_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except Exception:
        return None
    if not math.isfinite(out):
        return None
    return out


def normalize_yaw_deg(value: float) -> float:
    yaw = float(value)
    while yaw <= -180.0:
        yaw += 360.0
    while yaw > 180.0:
        yaw -= 360.0
    return float(yaw)


def normalize_datum_name(value: Any) -> Tuple[Optional[str], str]:
    name = str(value or "").strip()
    if not name:
        return None, "name is required"
    if len(name) > 64:
        return None, "name must be at most 64 characters"
    return name, ""


def slugify_datum_id(name: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9]+", "-", str(name).strip().lower()).strip("-")
    return slug or "datum"


def unique_datum_id(name: str, existing_ids: set[str], requested_id: Any = None) -> str:
    base = slugify_datum_id(str(requested_id or name))
    candidate = base
    index = 2
    while candidate in existing_ids:
        candidate = f"{base}-{index}"
        index += 1
    return candidate


def normalize_datum_profile(
    item: Any,
    *,
    index: int = 0,
    existing_ids: Optional[set[str]] = None,
    allow_existing_id: bool = True,
) -> Tuple[Optional[DatumProfile], str]:
    if not isinstance(item, dict):
        return None, f"datum[{index}] must be an object"

    name, err = normalize_datum_name(item.get("name"))
    if name is None:
        return None, f"datum[{index}] {err}"

    lat = _to_finite_float(item.get("lat", item.get("latitude")))
    lon = _to_finite_float(item.get("lon", item.get("longitude")))
    yaw = _to_finite_float(item.get("yaw_deg", item.get("yaw", 0.0)))
    if lat is None:
        return None, f"datum[{index}] lat must be a finite number"
    if lon is None:
        return None, f"datum[{index}] lon must be a finite number"
    if yaw is None:
        return None, f"datum[{index}] yaw_deg must be a finite number"
    if lat < -90.0 or lat > 90.0:
        return None, f"datum[{index}] lat must be between -90 and 90"
    if lon < -180.0 or lon > 180.0:
        return None, f"datum[{index}] lon must be between -180 and 180"

    ids = existing_ids if existing_ids is not None else set()
    requested_id = str(item.get("id") or "").strip()
    if requested_id and allow_existing_id:
        profile_id = slugify_datum_id(requested_id)
    else:
        profile_id = unique_datum_id(name, ids, requested_id=requested_id)

    now = utc_now_iso()
    profile: DatumProfile = {
        "id": profile_id,
        "name": name,
        "lat": float(lat),
        "lon": float(lon),
        "yaw_deg": normalize_yaw_deg(float(yaw)),
        "created_at": str(item.get("created_at") or now),
        "updated_at": str(item.get("updated_at") or now),
    }

    source = str(item.get("source") or "").strip()
    if source:
        profile["source"] = source
    notes = str(item.get("notes") or "").strip()
    if notes:
        profile["notes"] = notes[:256]
    metadata = item.get("metadata")
    if isinstance(metadata, dict):
        profile["metadata"] = dict(metadata)
    return profile, ""


def build_datums_doc(datums: List[DatumProfile], selected_id: str = "") -> Dict[str, Any]:
    return {
        "version": 1,
        "selected_id": str(selected_id or ""),
        "datums": datums,
    }


def parse_datums_yaml_text(yaml_text: str) -> Tuple[Optional[Dict[str, Any]], str]:
    try:
        raw = yaml.safe_load(yaml_text) if yaml_text.strip() else {}
    except Exception as exc:
        return None, f"invalid yaml: {exc}"
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        return None, "yaml root must be a map/object"

    selected_id = str(raw.get("selected_id") or "").strip()
    raw_datums = raw.get("datums", [])
    if raw_datums is None:
        raw_datums = []
    if not isinstance(raw_datums, list):
        return None, "datums must be a list"

    datums: List[DatumProfile] = []
    seen_ids: set[str] = set()
    for idx, item in enumerate(raw_datums):
        profile, err = normalize_datum_profile(
            item,
            index=idx,
            existing_ids=seen_ids,
            allow_existing_id=True,
        )
        if profile is None:
            return None, err
        if profile["id"] in seen_ids:
            profile["id"] = unique_datum_id(profile["name"], seen_ids, profile["id"])
        seen_ids.add(str(profile["id"]))
        datums.append(profile)

    if selected_id and selected_id not in seen_ids:
        selected_id = ""
    return build_datums_doc(datums, selected_id), ""


def load_datums_yaml_file(file_path: Path) -> Tuple[bool, str, Dict[str, Any]]:
    if not file_path.exists():
        return True, "", build_datums_doc([], "")
    try:
        raw_text = file_path.read_text(encoding="utf-8")
    except Exception as exc:
        return False, f"failed reading datums file: {exc}", build_datums_doc([], "")
    doc, err = parse_datums_yaml_text(raw_text)
    if doc is None:
        return False, err, build_datums_doc([], "")
    return True, "", doc


def save_datums_yaml_file(file_path: Path, doc: Dict[str, Any]) -> Tuple[bool, str]:
    normalized, err = parse_datums_yaml_text(yaml.safe_dump(doc, sort_keys=False))
    if normalized is None:
        return False, err
    try:
        file_path.parent.mkdir(parents=True, exist_ok=True)
        file_path.write_text(yaml.safe_dump(normalized, sort_keys=False), encoding="utf-8")
    except Exception as exc:
        return False, f"failed writing datums file: {exc}"
    return True, ""


def find_selected_datum(doc: Dict[str, Any]) -> Optional[DatumProfile]:
    selected_id = str(doc.get("selected_id") or "").strip()
    if not selected_id:
        return None
    for profile in doc.get("datums", []) or []:
        if isinstance(profile, dict) and str(profile.get("id") or "") == selected_id:
            return dict(profile)
    return None
