from pathlib import Path
import math
from typing import Tuple

import yaml


DEFAULT_DATUM_LAT = -31.4858037
DEFAULT_DATUM_LON = -64.2410570
DEFAULT_DATUM_YAW_DEG = 0.0


def resolve_config_file_path(package_share_dir: str, filename: str) -> str:
    package_share_path = Path(package_share_dir)
    default_path = package_share_path / "config" / filename
    try:
        workspace_root = package_share_path.parents[3]
        source_path = workspace_root / "src" / "navegacion_gps" / "config" / filename
        if source_path.exists():
            return str(source_path)
    except IndexError:
        pass
    return str(default_path)


def resolve_selected_datum(package_share_dir: str) -> Tuple[float, float, float, str]:
    datums_file = resolve_config_file_path(package_share_dir, "datums.yaml")
    fallback = (DEFAULT_DATUM_LAT, DEFAULT_DATUM_LON, DEFAULT_DATUM_YAW_DEG, datums_file)
    try:
        with open(datums_file, "r", encoding="utf-8") as handle:
            doc = yaml.safe_load(handle) or {}
    except Exception:
        return fallback
    if not isinstance(doc, dict):
        return fallback

    selected_id = str(doc.get("selected_id") or "").strip()
    datums = doc.get("datums") or []
    if not selected_id or not isinstance(datums, list):
        return fallback

    for item in datums:
        if not isinstance(item, dict) or str(item.get("id") or "") != selected_id:
            continue
        try:
            lat = float(item.get("lat"))
            lon = float(item.get("lon"))
            yaw_deg = float(item.get("yaw_deg", 0.0))
        except (TypeError, ValueError):
            return fallback
        if (
            math.isfinite(lat)
            and -90.0 <= lat <= 90.0
            and math.isfinite(lon)
            and -180.0 <= lon <= 180.0
            and math.isfinite(yaw_deg)
        ):
            return (lat, lon, yaw_deg, datums_file)
        return fallback

    return fallback
