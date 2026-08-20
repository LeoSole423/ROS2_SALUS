from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import statistics
from typing import Any, Iterable, Mapping, Optional, Sequence

import yaml

from navegacion_gps.gps_course_heading_core import ros_yaw_deg_from_north_east
from navegacion_gps.heading_math import AngleSeries
from navegacion_gps.heading_math import normalize_yaw_deg
from navegacion_gps.heading_math import shortest_angular_distance_deg


DEFAULT_JUMP_THRESHOLD_DEG = 12.0


@dataclass(frozen=True)
class BenchmarkScenario:
    scenario_id: str
    scenario_type: str
    order: int
    difficulty: int
    description: str
    purpose: str
    tags: tuple[str, ...]
    pre_idle_s: float
    post_idle_s: float
    run_timeout_s: float
    sample_hz: float
    hold_s: float
    forward_m: float
    left_m: float
    north_m: float
    east_m: float
    yaw_mode: str
    yaw_deg: Optional[float]
    yaw_delta_deg: Optional[float]

    @classmethod
    def from_mapping(cls, scenario_id: str, raw: Mapping[str, Any]) -> "BenchmarkScenario":
        scenario_type = str(raw.get("type", "body_relative_goal"))
        if scenario_type not in {"hold", "body_relative_goal", "north_east_goal"}:
            raise ValueError(
                f"Scenario '{scenario_id}' has unsupported type '{scenario_type}'"
            )
        yaw_mode = str(raw.get("yaw_mode", "path"))
        if yaw_mode not in {"path", "hold_start", "explicit", "relative"}:
            raise ValueError(
                f"Scenario '{scenario_id}' has unsupported yaw_mode '{yaw_mode}'"
            )
        tags = tuple(str(item) for item in raw.get("tags", []))
        return cls(
            scenario_id=str(scenario_id),
            scenario_type=scenario_type,
            order=int(raw.get("order", 0)),
            difficulty=int(raw.get("difficulty", 0)),
            description=str(raw.get("description", "")),
            purpose=str(raw.get("purpose", "")),
            tags=tags,
            pre_idle_s=max(0.0, float(raw.get("pre_idle_s", 5.0))),
            post_idle_s=max(0.0, float(raw.get("post_idle_s", 5.0))),
            run_timeout_s=max(0.0, float(raw.get("run_timeout_s", 90.0))),
            sample_hz=max(1.0, float(raw.get("sample_hz", 5.0))),
            hold_s=max(0.0, float(raw.get("hold_s", 0.0))),
            forward_m=float(raw.get("forward_m", 0.0)),
            left_m=float(raw.get("left_m", 0.0)),
            north_m=float(raw.get("north_m", 0.0)),
            east_m=float(raw.get("east_m", 0.0)),
            yaw_mode=yaw_mode,
            yaw_deg=(
                float(raw["yaw_deg"])
                if raw.get("yaw_deg") is not None and math.isfinite(float(raw["yaw_deg"]))
                else None
            ),
            yaw_delta_deg=(
                float(raw["yaw_delta_deg"])
                if raw.get("yaw_delta_deg") is not None
                and math.isfinite(float(raw["yaw_delta_deg"]))
                else None
            ),
        )


@dataclass(frozen=True)
class BenchmarkProfile:
    profile_id: str
    description: str
    scenario_ids: tuple[str, ...]


@dataclass(frozen=True)
class BenchmarkCatalog:
    version: int
    default_profile: str
    profiles: dict[str, BenchmarkProfile]
    scenarios: dict[str, BenchmarkScenario]


def default_benchmark_catalog_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    share_dir = Path(get_package_share_directory("navegacion_gps"))
    return share_dir / "config" / "nav_benchmark_scenarios.yaml"


def load_benchmark_catalog(path: Path | str) -> BenchmarkCatalog:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    scenarios_raw = raw.get("scenarios", {})
    if not isinstance(scenarios_raw, Mapping):
        raise ValueError("Benchmark catalog 'scenarios' must be a mapping")
    scenarios = {
        str(scenario_id): BenchmarkScenario.from_mapping(str(scenario_id), data or {})
        for scenario_id, data in scenarios_raw.items()
    }

    profiles_raw = raw.get("profiles", {})
    if not isinstance(profiles_raw, Mapping):
        raise ValueError("Benchmark catalog 'profiles' must be a mapping")
    profiles: dict[str, BenchmarkProfile] = {}
    for profile_id, data in profiles_raw.items():
        data = data or {}
        scenario_ids = tuple(str(item) for item in data.get("scenarios", []))
        for scenario_id in scenario_ids:
            if scenario_id not in scenarios:
                raise ValueError(
                    f"Profile '{profile_id}' references unknown scenario '{scenario_id}'"
                )
        profiles[str(profile_id)] = BenchmarkProfile(
            profile_id=str(profile_id),
            description=str(data.get("description", "")),
            scenario_ids=scenario_ids,
        )

    default_profile = str(raw.get("default_profile", "")).strip()
    if not default_profile:
        default_profile = next(iter(profiles.keys()), "")
    if default_profile and default_profile not in profiles:
        raise ValueError(f"Unknown default_profile '{default_profile}' in benchmark catalog")
    return BenchmarkCatalog(
        version=int(raw.get("version", 1)),
        default_profile=default_profile,
        profiles=profiles,
        scenarios=scenarios,
    )


def select_benchmark_scenarios(
    catalog: BenchmarkCatalog,
    *,
    profile: str = "",
    scenario_ids: Optional[Sequence[str]] = None,
    max_difficulty: Optional[int] = None,
) -> list[BenchmarkScenario]:
    selected_ids: list[str]
    if scenario_ids:
        selected_ids = [str(item) for item in scenario_ids if str(item).strip()]
    else:
        resolved_profile = str(profile or catalog.default_profile)
        if not resolved_profile:
            return []
        if resolved_profile not in catalog.profiles:
            raise ValueError(f"Unknown benchmark profile '{resolved_profile}'")
        selected_ids = list(catalog.profiles[resolved_profile].scenario_ids)

    selected: list[BenchmarkScenario] = []
    seen: set[str] = set()
    for scenario_id in selected_ids:
        if scenario_id in seen:
            continue
        if scenario_id not in catalog.scenarios:
            raise ValueError(f"Unknown benchmark scenario '{scenario_id}'")
        scenario = catalog.scenarios[scenario_id]
        if max_difficulty is not None and scenario.difficulty > int(max_difficulty):
            continue
        seen.add(scenario_id)
        selected.append(scenario)
    selected.sort(key=lambda item: (item.order, item.difficulty, item.scenario_id))
    return selected


def body_relative_offsets_to_north_east(
    start_yaw_deg: float,
    forward_m: float,
    left_m: float,
) -> tuple[float, float]:
    yaw_rad = math.radians(float(start_yaw_deg))
    east_m = (float(forward_m) * math.cos(yaw_rad)) - (float(left_m) * math.sin(yaw_rad))
    north_m = (float(forward_m) * math.sin(yaw_rad)) + (float(left_m) * math.cos(yaw_rad))
    return float(north_m), float(east_m)


def resolve_goal_yaw_deg(
    scenario: BenchmarkScenario,
    *,
    start_yaw_deg: float,
    north_m: float,
    east_m: float,
) -> float:
    if scenario.yaw_mode == "hold_start":
        return float(normalize_yaw_deg(start_yaw_deg))
    if scenario.yaw_mode == "explicit":
        if scenario.yaw_deg is None:
            raise ValueError(f"Scenario '{scenario.scenario_id}' is missing yaw_deg")
        return float(normalize_yaw_deg(scenario.yaw_deg))
    if scenario.yaw_mode == "relative":
        delta_deg = float(scenario.yaw_delta_deg or 0.0)
        return float(normalize_yaw_deg(float(start_yaw_deg) + delta_deg))
    return float(ros_yaw_deg_from_north_east(north_m=north_m, east_m=east_m))


def distance_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def meters_per_deg_lon(lat_deg: float) -> float:
    return 111_320.0 * max(1.0e-6, abs(math.cos(math.radians(float(lat_deg)))))


def offset_lat_lon(
    lat_deg: float,
    lon_deg: float,
    *,
    north_m: float,
    east_m: float,
) -> tuple[float, float]:
    lat = float(lat_deg) + float(north_m) / 111_320.0
    lon = float(lon_deg) + float(east_m) / meters_per_deg_lon(lat_deg)
    # Mantener la representacion geografica canonica al cruzar el
    # antimeridiano. La inversa (ll_to_body) toma el arco corto.
    lon = ((lon + 180.0) % 360.0) - 180.0
    return float(lat), float(lon)


def line_lateral_error_m(
    point_xy: tuple[float, float],
    line_start_xy: tuple[float, float],
    line_end_xy: tuple[float, float],
) -> float:
    line_dx = float(line_end_xy[0]) - float(line_start_xy[0])
    line_dy = float(line_end_xy[1]) - float(line_start_xy[1])
    line_norm = math.hypot(line_dx, line_dy)
    if line_norm < 1.0e-6:
        return 0.0
    rel_x = float(point_xy[0]) - float(line_start_xy[0])
    rel_y = float(point_xy[1]) - float(line_start_xy[1])
    return ((rel_x * line_dy) - (rel_y * line_dx)) / line_norm


def line_progress_m(
    point_xy: tuple[float, float],
    line_start_xy: tuple[float, float],
    line_end_xy: tuple[float, float],
) -> float:
    line_dx = float(line_end_xy[0]) - float(line_start_xy[0])
    line_dy = float(line_end_xy[1]) - float(line_start_xy[1])
    line_norm = math.hypot(line_dx, line_dy)
    if line_norm < 1.0e-6:
        return 0.0
    rel_x = float(point_xy[0]) - float(line_start_xy[0])
    rel_y = float(point_xy[1]) - float(line_start_xy[1])
    return ((rel_x * line_dx) + (rel_y * line_dy)) / line_norm


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    finite = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite:
        return None
    if len(finite) == 1:
        return float(finite[0])
    rank = max(0.0, min(100.0, float(q))) / 100.0 * float(len(finite) - 1)
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return float(finite[low])
    fraction = rank - float(low)
    return float(finite[low] + ((finite[high] - finite[low]) * fraction))


def summarize_scalar(values: Iterable[float]) -> dict[str, Any]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {
            "count": 0,
            "mean": None,
            "std": None,
            "min": None,
            "max": None,
            "p95": None,
            "max_abs": None,
        }
    return {
        "count": len(finite),
        "mean": float(statistics.fmean(finite)),
        "std": float(statistics.pstdev(finite)) if len(finite) >= 2 else 0.0,
        "min": float(min(finite)),
        "max": float(max(finite)),
        "p95": percentile(finite, 95.0),
        "max_abs": float(max(abs(value) for value in finite)),
    }


def summarize_angle(values_deg: Iterable[float]) -> dict[str, Any]:
    series = AngleSeries()
    for value in values_deg:
        if math.isfinite(float(value)):
            series.add(float(value))
    return series.summary()


def summarize_angle_jumps(
    values_deg: Sequence[float],
    *,
    jump_threshold_deg: float = DEFAULT_JUMP_THRESHOLD_DEG,
) -> dict[str, Any]:
    finite = [float(value) for value in values_deg if math.isfinite(float(value))]
    deltas = [
        abs(shortest_angular_distance_deg(prev_value, value))
        for prev_value, value in zip(finite[:-1], finite[1:])
    ]
    summary = summarize_scalar(deltas)
    jump_count = sum(1 for value in deltas if value >= float(jump_threshold_deg))
    summary.update(
        {
            "jump_threshold_deg": float(jump_threshold_deg),
            "jump_count": int(jump_count),
            "jump_ratio": float(jump_count) / float(len(deltas)) if deltas else 0.0,
        }
    )
    return summary


def event_code_counts(events: Iterable[Mapping[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for event in events:
        code = str(event.get("code", "")).strip()
        if not code:
            continue
        counts[code] = counts.get(code, 0) + 1
    return dict(sorted(counts.items()))


def json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    if isinstance(value, float):
        return None if not math.isfinite(value) else float(value)
    return value


def _nested_get(mapping: Mapping[str, Any], *keys: str) -> Any:
    current: Any = mapping
    for key in keys:
        if not isinstance(current, Mapping) or key not in current:
            return None
        current = current[key]
    return current


def extract_key_metrics(run: Mapping[str, Any]) -> dict[str, Any]:
    summary = run.get("summary", {})
    return {
        "success": _nested_get(summary, "outcome", "success"),
        "timeout": _nested_get(summary, "outcome", "timeout"),
        "duration_s": _nested_get(summary, "outcome", "duration_s"),
        "final_goal_error_m": _nested_get(summary, "outcome", "final_goal_error_m"),
        "goal_distance_m": _nested_get(summary, "outcome", "goal_distance_m"),
        "path_progress_ratio": _nested_get(summary, "path_tracking", "progress", "ratio"),
        "map_base_lateral_max_abs_m": _nested_get(
            summary, "path_tracking", "map_base_lateral_error_m", "absolute", "max"
        ),
        "map_base_lateral_p95_abs_m": _nested_get(
            summary, "path_tracking", "map_base_lateral_error_m", "absolute", "p95"
        ),
        "odom_global_lateral_max_abs_m": _nested_get(
            summary, "path_tracking", "odom_global_lateral_error_m", "absolute", "max"
        ),
        "map_odom_jump_count": _nested_get(
            summary, "heading_stability", "map_odom_yaw", "jumps", "jump_count"
        ),
        "map_odom_jump_max_abs_deg": _nested_get(
            summary, "heading_stability", "map_odom_yaw", "jumps", "max"
        ),
        "map_base_jump_count": _nested_get(
            summary, "heading_stability", "map_base_yaw", "jumps", "jump_count"
        ),
        "map_base_jump_max_abs_deg": _nested_get(
            summary, "heading_stability", "map_base_yaw", "jumps", "max"
        ),
        "gps_heading_valid_ratio": _nested_get(
            summary, "heading_stability", "gps_course_heading", "valid_ratio"
        ),
    }


def comparison_direction(metric_name: str) -> str:
    higher_better = {"success", "path_progress_ratio", "gps_heading_valid_ratio"}
    return "higher" if metric_name in higher_better else "lower"
