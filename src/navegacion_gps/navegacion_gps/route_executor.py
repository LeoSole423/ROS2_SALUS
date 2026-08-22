import json
import math
import threading
import time
import uuid
from dataclasses import dataclass
from dataclasses import replace
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav2_msgs.action import BackUp
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rcl_interfaces.srv import SetParameters
from robot_localization.srv import FromLL
from sensor_msgs.msg import BatteryState
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener

from interfaces.msg import BatteryMissionGuard, NavEvent, NavTelemetry
from interfaces.srv import (
    BrakeNav,
    CancelPatrolMission,
    CancelNavGoal,
    CancelRouteMission,
    GenerateCoveragePlanLL,
    GetPatrolMissionState,
    GetRouteMissionState,
    GetZonesState,
    RequestReturnHome,
    SetPatrolMissionLL,
    SetNavGoalLL,
    SetNavigationProfile,
    SetRouteMissionLL,
)
from navegacion_gps.nav_benchmarking import body_relative_offsets_to_north_east
from navegacion_gps.nav_benchmarking import offset_lat_lon


BLOCKED_STATE_NONE = ""
BLOCKED_STATE_WAITING = "BLOCKED_WAITING"
BLOCKED_STATE_RETRYING = "BLOCKED_RETRYING"
BLOCKED_STATE_NEEDS_OPERATOR = "BLOCKED_NEEDS_OPERATOR"
WAYPOINT_ROLE_NORMAL = "normal"
WAYPOINT_ROLE_HOME = "home"
WAYPOINT_ROLE_COVERAGE = "coverage"
WAYPOINT_ROLE_COVERAGE_CURVE = "coverage_curve"
WAYPOINT_ROLE_COVERAGE_TRANSIT = "coverage_transit"
PATROL_PHASE_IDLE = "idle"
PATROL_PHASE_DEPART_HOME = "depart_home"
PATROL_PHASE_LOOP_MAIN = "loop_main"
PATROL_PHASE_RETURN_PENDING = "return_pending"
PATROL_PHASE_RETURN_CONNECTOR = "return_connector"
PATROL_PHASE_PARKED_HOME = "parked_home"
NAVIGATION_PROFILE_URBAN = "urban"
NAVIGATION_PROFILE_RURAL = "rural"
NAVIGATION_PROFILES = {NAVIGATION_PROFILE_URBAN, NAVIGATION_PROFILE_RURAL}
# Se mantiene local para no importar el modulo agricola al arrancar Ruta,
# Patrol o goals. Es el mismo vocabulario que emite Fields2Cover.
WORK_PHASE = "row"


def coverage_auto_waypoint_spacing_m(
    field_length_m: float,
    field_width_m: float,
    *,
    ratio: float = 0.10,
    minimum_m: float = 1.0,
    maximum_m: float = 4.0,
) -> float:
    """Calcular el espaciado de muestreo a partir del tamano real del lote.

    Se usa la media geometrica de las dimensiones, ``sqrt(largo * ancho)``,
    para que el detalle crezca con el area sin crecer linealmente con un solo
    lado. El valor queda acotado: ``clamp(0.10 * sqrt(area), 1.0, 4.0)``.
    Por ejemplo, 20x15 m da ``0.10*sqrt(300)=1.73 m``, 36x23 m da
    ``0.10*sqrt(828)=2.88 m`` y 80x50 m da ``6.32 m``, limitado a 4.0 m.
    ``coverage_full_row_leg_spacing_m`` sigue gobernando el espaciado de una
    fila completa y no se usa para insertar metas dentro de la pasada.
    """
    largo = max(1.0e-6, float(field_length_m))
    ancho = max(1.0e-6, float(field_width_m))
    piso = max(1.0e-3, float(minimum_m))
    techo = max(piso, float(maximum_m))
    factor = max(0.0, float(ratio))
    return float(min(techo, max(piso, factor * math.sqrt(largo * ancho))))


def _coverage_polygon_dimensions_m(
    polygon_ll: Sequence[Tuple[float, float]],
    fallback_length_m: float,
    fallback_width_m: float,
) -> Tuple[float, float]:
    """Obtener dimensiones metricas del anillo geografico cargado.

    Se mide el rectangulo de AREA MINIMA que envuelve al anillo, no el bounding
    box alineado a lat/lon. El bounding box dependia de como estuviera orientado
    el lote respecto del norte: el mismo lote de 36x23 m daba 828 m2 apuntando
    al este, 920 m2 a 87 grados y 1740 m2 girado 45 grados. Como de aca sale el
    espaciado automatico, girar el campo sin cambiar nada del trabajo movia la
    densidad de muestreo de 2.88 m a 4.00 m.

    Un anillo invalido, vacio o degenerado conserva las dimensiones explicitas
    del pedido rectangular.
    """
    if not polygon_ll or len(polygon_ll) < 3:
        return float(fallback_length_m), float(fallback_width_m)
    latitudes = [float(point[0]) for point in polygon_ll]
    longitudes = [float(point[1]) for point in polygon_ll]
    latitud_media = math.radians(sum(latitudes) / len(latitudes))
    metros_lat = 111_320.0
    metros_lon = metros_lat * max(1.0e-6, abs(math.cos(latitud_media)))
    puntos = [
        (
            (float(lon) - longitudes[0]) * metros_lon,
            (float(lat) - latitudes[0]) * metros_lat,
        )
        for lat, lon in zip(latitudes, longitudes)
    ]
    largo, ancho = _minimum_area_rectangle_m(puntos)
    if largo <= 1.0e-6 or ancho <= 1.0e-6:
        return float(fallback_length_m), float(fallback_width_m)
    return float(largo), float(ancho)


def _convex_hull_m(points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Envolvente convexa (monotone chain), en metros."""
    unicos = sorted(set((float(x), float(y)) for x, y in points))
    if len(unicos) < 3:
        return unicos

    def cruz(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])) - ((a[1] - o[1]) * (b[0] - o[0]))

    inferior: List[Tuple[float, float]] = []
    for punto in unicos:
        while len(inferior) >= 2 and cruz(inferior[-2], inferior[-1], punto) <= 0.0:
            inferior.pop()
        inferior.append(punto)
    superior: List[Tuple[float, float]] = []
    for punto in reversed(unicos):
        while len(superior) >= 2 and cruz(superior[-2], superior[-1], punto) <= 0.0:
            superior.pop()
        superior.append(punto)
    return inferior[:-1] + superior[:-1]


def _minimum_area_rectangle_m(
    points: Sequence[Tuple[float, float]],
) -> Tuple[float, float]:
    """Lados del rectangulo de area minima que envuelve a los puntos.

    Para un poligono convexo ese rectangulo apoya uno de sus lados sobre una
    arista, asi que alcanza con probar cada arista de la envolvente. El
    resultado no depende de como este rotado el lote, que es justamente lo que
    se necesita para que el espaciado automatico sea estable.
    """
    hull = _convex_hull_m(points)
    if len(hull) < 3:
        return 0.0, 0.0
    mejor: Optional[Tuple[float, float, float]] = None
    for indice in range(len(hull)):
        a = hull[indice]
        b = hull[(indice + 1) % len(hull)]
        eje_x = b[0] - a[0]
        eje_y = b[1] - a[1]
        largo_arista = math.hypot(eje_x, eje_y)
        if largo_arista <= 1.0e-9:
            continue
        ux = eje_x / largo_arista
        uy = eje_y / largo_arista
        proyecciones = [(p[0] * ux) + (p[1] * uy) for p in hull]
        normales = [(-p[0] * uy) + (p[1] * ux) for p in hull]
        ancho = max(proyecciones) - min(proyecciones)
        alto = max(normales) - min(normales)
        area = ancho * alto
        if mejor is None or area < mejor[0]:
            mejor = (area, ancho, alto)
    if mejor is None:
        return 0.0, 0.0
    return max(mejor[1], mejor[2]), min(mejor[1], mejor[2])

BLOCKING_FAILURE_CODES = {
    "NO_VALID_PATH",
    "ACTION_SERVER_ACK_TIMEOUT",
    "TF_EXTRAPOLATION",
    "CONTROLLER_COLLISION",
    "COLLISION_STOP_ACTIVE",
    "SMOOTHED_PATH_COLLISION",
    "RECOVERY_FAILED",
    "RECOVERY_OFF_GRID",
    "COSTMAP_CLEAR_TIMEOUT",
}

BLOCKING_REASON_TEXT = {
    "NO_VALID_PATH": "no valid path found",
    "ACTION_SERVER_ACK_TIMEOUT": "internal Nav2 action server acknowledgement timed out",
    "TF_EXTRAPOLATION": "transient TF extrapolation while transforming robot pose",
    "CONTROLLER_COLLISION": "controller predicted or detected collision",
    "COLLISION_STOP_ACTIVE": "collision monitor stop persisted",
    "SMOOTHED_PATH_COLLISION": "smoothed path leads to collision",
    "RECOVERY_FAILED": "recovery behavior failed",
    "RECOVERY_OFF_GRID": "recovery would leave costmap",
    "COSTMAP_CLEAR_TIMEOUT": "costmap clear timed out",
}


@dataclass(frozen=True)
class RouteWaypoint:
    lat: float
    lon: float
    yaw_deg: float


@dataclass(frozen=True)
class RouteAction:
    action_type: str
    duration_s: float = 0.0
    brake_pct: int = 100
    profile: str = ""
    label: str = ""
    # Metros de marcha atras de coverage_backup. Solo lo usa Campo.
    distance_m: float = 0.0


@dataclass(frozen=True)
class PreparedRouteMission:
    waypoints: List[RouteWaypoint]
    action_jsons: List[str]
    waypoint_roles: List[str]
    start_index: int
    skipped_waypoints: int
    rotated: bool
    note: str
    input_indices: Optional[List[int]] = None


@dataclass(frozen=True)
class RouteSegmentMatch:
    segment_index: int
    next_index: int
    distance_m: float
    ratio: float


@dataclass(frozen=True)
class RouteProgress:
    expanded_index: int
    ratio: float
    cross_track_error_m: float
    distance_to_target_m: float


@dataclass(frozen=True)
class BlockedRetryStartResolution:
    requested_start_index: int
    resolved_start_index: int
    reanchored: bool
    reason: str
    match_distance_m: float


@dataclass(frozen=True)
class RouteMissionHome:
    waypoint: RouteWaypoint
    input_index: int


@dataclass(frozen=True)
class ReturnHomeExitSelection:
    route_index: int
    input_index: int
    waypoint: RouteWaypoint


@dataclass(frozen=True)
class PatrolMissionProfile:
    loop_waypoints: List[RouteWaypoint]
    loop_action_jsons: List[str]
    home_waypoint: RouteWaypoint
    return_waypoints: List[RouteWaypoint]
    return_action_jsons: List[str]
    depart_waypoints: List[RouteWaypoint]
    depart_action_jsons: List[str]
    depart_entry_loop_index: int
    leg_spacing_m: float
    chunk_span_m: float
    chunk_max_waypoints: int


def _normalize_yaw_deg(yaw_deg: float) -> float:
    yaw = float(yaw_deg)
    while yaw <= -180.0:
        yaw += 360.0
    while yaw > 180.0:
        yaw -= 360.0
    return float(yaw)


def _yaw_to_quaternion(yaw_deg: float) -> Quaternion:
    yaw_rad = math.radians(float(yaw_deg))
    half_yaw = yaw_rad / 2.0
    return Quaternion(x=0.0, y=0.0, z=math.sin(half_yaw), w=math.cos(half_yaw))


def _serialize_route_actions(actions: Sequence[RouteAction]) -> str:
    if not actions:
        return ""
    payload = []
    for action in actions:
        item: Dict[str, Any] = {"type": str(action.action_type)}
        if action.action_type == "brake_hold":
            item["duration_s"] = float(action.duration_s)
            item["brake_pct"] = int(action.brake_pct)
        elif action.action_type == "set_navigation_profile":
            item["profile"] = str(action.profile)
        elif action.action_type == "coverage_backup":
            item["distance_m"] = float(action.distance_m)
        if action.label:
            item["label"] = str(action.label)
        payload.append(item)
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def _parse_route_action_json(action_json: str, index: int) -> Tuple[Optional[str], str]:
    raw_text = str(action_json or "").strip()
    if not raw_text:
        return "", ""
    try:
        raw = json.loads(raw_text)
    except Exception as exc:
        return None, f"waypoint_action_jsons[{index}] invalid json: {exc}"
    if raw in (None, "", []):
        return "", ""
    if not isinstance(raw, list):
        return None, f"waypoint_action_jsons[{index}] must be a JSON array"

    actions: List[RouteAction] = []
    for action_idx, item in enumerate(raw):
        if not isinstance(item, dict):
            return None, f"waypoint_action_jsons[{index}][{action_idx}] must be an object"
        action_type = str(item.get("type", "")).strip()
        label = str(item.get("label", "") or "").strip()
        if action_type == "brake_hold":
            try:
                duration_s = float(item.get("duration_s", 0.0))
                brake_pct = int(float(item.get("brake_pct", 100)))
            except (TypeError, ValueError):
                return None, f"invalid brake_hold action at waypoint {index}"
            if (not np.isfinite(duration_s)) or duration_s <= 0.0 or duration_s > 600.0:
                return None, f"brake_hold duration_s at waypoint {index} must be > 0 and <= 600"
            actions.append(
                RouteAction(
                    action_type="brake_hold",
                    duration_s=float(duration_s),
                    brake_pct=max(0, min(100, brake_pct)),
                    label=label[:80],
                )
            )
            continue
        if action_type == "set_navigation_profile":
            profile = str(item.get("profile", "") or "").strip().lower()
            if profile not in NAVIGATION_PROFILES:
                return None, (
                    f"set_navigation_profile at waypoint {index} must use "
                    f"one of {sorted(NAVIGATION_PROFILES)}"
                )
            actions.append(
                RouteAction(
                    action_type="set_navigation_profile",
                    profile=profile,
                    label=label[:80],
                )
            )
            continue
        if action_type == "coverage_backup":
            # La marcha atras de la cabecera de tres puntos de Campo. El tope de
            # 20 m es holgado: la maniobra pide 2R menos la separacion entre
            # pasadas, o sea menos de dos diametros de giro.
            try:
                distance_m = float(item.get("distance_m", 0.0))
            except (TypeError, ValueError):
                return None, f"invalid coverage_backup action at waypoint {index}"
            if (not np.isfinite(distance_m)) or distance_m <= 0.0 or distance_m > 20.0:
                return None, (
                    f"coverage_backup distance_m at waypoint {index} must be > 0 "
                    "and <= 20"
                )
            actions.append(
                RouteAction(
                    action_type="coverage_backup",
                    distance_m=float(distance_m),
                    label=label[:80],
                )
            )
            continue
        return None, f"unsupported waypoint action type: {action_type or '<empty>'}"
    return _serialize_route_actions(actions), ""


def _parse_route_action_jsons(
    action_jsons: Sequence[str],
    waypoint_count: int,
) -> Tuple[Optional[List[str]], str]:
    if not action_jsons:
        return ["" for _ in range(int(waypoint_count))], ""
    if len(action_jsons) != int(waypoint_count):
        return None, "waypoint_action_jsons length must match lats/lons when provided"
    normalized: List[str] = []
    for idx, raw in enumerate(action_jsons):
        parsed, err = _parse_route_action_json(str(raw), idx)
        if parsed is None:
            return None, err
        normalized.append(parsed)
    return normalized, ""


def _actions_from_json(action_json: str) -> List[RouteAction]:
    normalized, err = _parse_route_action_json(action_json, 0)
    if err or normalized is None or not normalized:
        return []
    raw = json.loads(normalized)
    actions: List[RouteAction] = []
    for item in raw:
        actions.append(
            RouteAction(
                action_type=str(item.get("type", "")),
                duration_s=float(item.get("duration_s", 0.0)),
                brake_pct=int(item.get("brake_pct", 100)),
                profile=str(item.get("profile", "") or ""),
                label=str(item.get("label", "") or ""),
                distance_m=float(item.get("distance_m", 0.0)),
            )
        )
    return actions


def _parse_waypoint_roles(
    waypoint_roles: Sequence[str],
    waypoint_count: int,
) -> Tuple[Optional[List[str]], str]:
    if not waypoint_roles:
        return [WAYPOINT_ROLE_NORMAL for _ in range(int(waypoint_count))], ""
    if len(waypoint_roles) != int(waypoint_count):
        return None, "waypoint_roles length must match lats/lons when provided"
    normalized: List[str] = []
    home_count = 0
    for idx, raw in enumerate(waypoint_roles):
        role = str(raw or WAYPOINT_ROLE_NORMAL).strip().lower()
        if role not in (
            WAYPOINT_ROLE_NORMAL,
            WAYPOINT_ROLE_HOME,
            WAYPOINT_ROLE_COVERAGE,
            WAYPOINT_ROLE_COVERAGE_CURVE,
            WAYPOINT_ROLE_COVERAGE_TRANSIT,
        ):
            return None, f"unsupported waypoint role at index {idx}: {role or '<empty>'}"
        if role == WAYPOINT_ROLE_HOME:
            home_count += 1
            if home_count > 1:
                return None, "only one HOME waypoint is allowed"
        normalized.append(role)
    return normalized, ""


def _poses_to_debug_path(
    poses: Sequence[PoseStamped],
    *,
    frame_id: str,
    stamp: Any,
) -> Path:
    path = Path()
    path.header.frame_id = str(frame_id)
    path.header.stamp = stamp
    path.poses = list(poses)
    for pose in path.poses:
        pose.header.frame_id = str(frame_id)
        pose.header.stamp = stamp
    return path


def _distance_m(start_lat: float, start_lon: float, end_lat: float, end_lon: float) -> float:
    meters_per_deg_lat = 111_320.0
    avg_lat_rad = math.radians((float(start_lat) + float(end_lat)) * 0.5)
    meters_per_deg_lon = meters_per_deg_lat * max(1.0e-6, abs(math.cos(avg_lat_rad)))
    north_m = (float(end_lat) - float(start_lat)) * meters_per_deg_lat
    east_m = (float(end_lon) - float(start_lon)) * meters_per_deg_lon
    return float(math.hypot(north_m, east_m))


def _select_return_home_exit_waypoint(
    route: Sequence[RouteWaypoint],
    input_indices: Sequence[int],
    home_waypoint: Optional[RouteWaypoint],
) -> Optional[ReturnHomeExitSelection]:
    route_list = list(route)
    source_indices = list(input_indices)
    if not route_list or home_waypoint is None:
        return None

    best: Optional[ReturnHomeExitSelection] = None
    best_distance_m = math.inf
    best_input_index = math.inf
    for route_index, waypoint in enumerate(route_list):
        input_index = (
            int(source_indices[route_index]) if route_index < len(source_indices) else int(route_index)
        )
        distance_m = _distance_m(
            waypoint.lat,
            waypoint.lon,
            float(home_waypoint.lat),
            float(home_waypoint.lon),
        )
        candidate = ReturnHomeExitSelection(
            route_index=int(route_index),
            input_index=int(input_index),
            waypoint=waypoint,
        )
        if (distance_m, input_index) < (best_distance_m, best_input_index):
            best = candidate
            best_distance_m = float(distance_m)
            best_input_index = int(input_index)
    return best


def _rotate_list(values: Sequence[Any], start_index: int) -> List[Any]:
    items = list(values)
    if not items:
        return []
    start = int(start_index) % len(items)
    if start == 0:
        return list(items)
    return items[start:] + items[:start]


def _map_prepared_input_indices(
    prepared: PreparedRouteMission,
    source_indices: Sequence[int],
) -> PreparedRouteMission:
    if prepared.input_indices is None:
        return prepared
    mapped = [
        int(source_indices[index]) if 0 <= int(index) < len(source_indices) else int(index)
        for index in prepared.input_indices
    ]
    return PreparedRouteMission(
        waypoints=list(prepared.waypoints),
        action_jsons=list(prepared.action_jsons),
        waypoint_roles=list(prepared.waypoint_roles),
        start_index=int(prepared.start_index),
        skipped_waypoints=int(prepared.skipped_waypoints),
        rotated=bool(prepared.rotated),
        note=str(prepared.note),
        input_indices=mapped,
    )


def _bearing_deg(start_lat: float, start_lon: float, end_lat: float, end_lon: float) -> float:
    meters_per_deg_lat = 111_320.0
    avg_lat_rad = math.radians((float(start_lat) + float(end_lat)) * 0.5)
    meters_per_deg_lon = meters_per_deg_lat * max(1.0e-6, abs(math.cos(avg_lat_rad)))
    north_m = (float(end_lat) - float(start_lat)) * meters_per_deg_lat
    east_m = (float(end_lon) - float(start_lon)) * meters_per_deg_lon
    if math.hypot(north_m, east_m) <= 1.0e-6:
        return 0.0
    return _normalize_yaw_deg(math.degrees(math.atan2(north_m, east_m)))


def _interpolate_waypoint(start: RouteWaypoint, end: RouteWaypoint, ratio: float) -> RouteWaypoint:
    return RouteWaypoint(
        lat=float(start.lat + ((end.lat - start.lat) * ratio)),
        lon=float(start.lon + ((end.lon - start.lon) * ratio)),
        yaw_deg=_bearing_deg(start.lat, start.lon, end.lat, end.lon),
    )


def _local_xy_m(
    lat: float,
    lon: float,
    *,
    origin_lat: float,
    origin_lon: float,
) -> Tuple[float, float]:
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = meters_per_deg_lat * max(
        1.0e-6, abs(math.cos(math.radians(float(origin_lat))))
    )
    x = (float(lon) - float(origin_lon)) * meters_per_deg_lon
    y = (float(lat) - float(origin_lat)) * meters_per_deg_lat
    return float(x), float(y)


def _closest_route_segment(
    route: Sequence[RouteWaypoint],
    *,
    robot_lat: float,
    robot_lon: float,
    loop: bool,
) -> Optional[RouteSegmentMatch]:
    route_list = list(route)
    if len(route_list) <= 1:
        return None

    robot_x, robot_y = _local_xy_m(
        robot_lat,
        robot_lon,
        origin_lat=robot_lat,
        origin_lon=robot_lon,
    )
    segment_count = len(route_list) if loop else len(route_list) - 1
    best: Optional[RouteSegmentMatch] = None

    for idx in range(segment_count):
        start = route_list[idx]
        next_index = (idx + 1) % len(route_list)
        end = route_list[next_index]
        start_x, start_y = _local_xy_m(
            start.lat,
            start.lon,
            origin_lat=robot_lat,
            origin_lon=robot_lon,
        )
        end_x, end_y = _local_xy_m(
            end.lat,
            end.lon,
            origin_lat=robot_lat,
            origin_lon=robot_lon,
        )
        seg_x = end_x - start_x
        seg_y = end_y - start_y
        seg_len_sq = (seg_x * seg_x) + (seg_y * seg_y)
        if seg_len_sq <= 1.0e-6:
            continue

        raw_ratio = (
            ((robot_x - start_x) * seg_x) + ((robot_y - start_y) * seg_y)
        ) / seg_len_sq
        ratio = min(1.0, max(0.0, float(raw_ratio)))
        proj_x = start_x + (seg_x * ratio)
        proj_y = start_y + (seg_y * ratio)
        distance_m = math.hypot(robot_x - proj_x, robot_y - proj_y)
        candidate = RouteSegmentMatch(
            segment_index=int(idx),
            next_index=int(next_index),
            distance_m=float(distance_m),
            ratio=float(ratio),
        )
        if best is None or (candidate.distance_m, candidate.segment_index) < (
            best.distance_m,
            best.segment_index,
        ):
            best = candidate

    return best


def _is_usable_segment_match(
    match: Optional[RouteSegmentMatch],
    *,
    segment_tolerance_m: float,
) -> bool:
    return bool(
        match is not None
        and match.distance_m <= segment_tolerance_m
        and match.ratio > 0.0
        and match.ratio < 1.0
    )


def expanded_input_indices(
    key_flags: Sequence[bool], key_input_indices: Optional[Sequence[int]] = None
) -> List[int]:
    input_index = -1
    source_indices = list(key_input_indices or [])
    result: List[int] = []
    for is_key in key_flags:
        if bool(is_key):
            input_index += 1
            result.append(
                int(source_indices[input_index])
                if input_index < len(source_indices)
                else input_index
            )
        else:
            result.append(-1)
    return result


def route_progress(
    route: Sequence[RouteWaypoint],
    *,
    start_index: int,
    target_index: int,
    loop: bool,
    robot_lat: Optional[float],
    robot_lon: Optional[float],
) -> Optional[RouteProgress]:
    route_list = list(route)
    if not route_list or robot_lat is None or robot_lon is None:
        return None
    if not np.isfinite(float(robot_lat)) or not np.isfinite(float(robot_lon)):
        return None

    total = len(route_list)
    start = max(0, int(start_index))
    target = max(0, int(target_index))
    if loop:
        start %= total
        target %= total
    elif start >= total or target >= total:
        return None

    distance_to_target_m = _distance_m(
        float(robot_lat),
        float(robot_lon),
        route_list[target].lat,
        route_list[target].lon,
    )
    if start == target:
        return RouteProgress(target, 1.0, distance_to_target_m, distance_to_target_m)

    indices = [start]
    while indices[-1] != target and len(indices) <= total:
        next_index = indices[-1] + 1
        if loop:
            next_index %= total
        elif next_index >= total:
            break
        if next_index in indices and next_index != target:
            break
        indices.append(next_index)
    if len(indices) <= 1 or indices[-1] != target:
        return None

    robot_x, robot_y = 0.0, 0.0
    best: Optional[Tuple[float, int, float]] = None
    for order, segment_index in enumerate(indices[:-1]):
        next_index = indices[order + 1]
        start_wp = route_list[segment_index]
        end_wp = route_list[next_index]
        start_x, start_y = _local_xy_m(
            start_wp.lat,
            start_wp.lon,
            origin_lat=float(robot_lat),
            origin_lon=float(robot_lon),
        )
        end_x, end_y = _local_xy_m(
            end_wp.lat,
            end_wp.lon,
            origin_lat=float(robot_lat),
            origin_lon=float(robot_lon),
        )
        seg_x = end_x - start_x
        seg_y = end_y - start_y
        seg_len_sq = (seg_x * seg_x) + (seg_y * seg_y)
        if seg_len_sq <= 1.0e-6:
            continue
        raw_ratio = (((robot_x - start_x) * seg_x) + ((robot_y - start_y) * seg_y)) / seg_len_sq
        ratio = min(1.0, max(0.0, float(raw_ratio)))
        proj_x = start_x + (seg_x * ratio)
        proj_y = start_y + (seg_y * ratio)
        cross_track_m = math.hypot(robot_x - proj_x, robot_y - proj_y)
        candidate = (cross_track_m, -order, ratio)
        if best is None or candidate < best:
            best = candidate

    if best is None:
        return None
    cross_track_m, negative_order, ratio = best
    order = -negative_order
    return RouteProgress(
        expanded_index=int(indices[order]),
        ratio=float(ratio),
        cross_track_error_m=float(cross_track_m),
        distance_to_target_m=float(distance_to_target_m),
    )


def _resolve_input_waypoints(
    lats: Sequence[float],
    lons: Sequence[float],
    yaws_deg: Sequence[float],
    loop: bool,
) -> List[RouteWaypoint]:
    resolved: List[RouteWaypoint] = []
    use_explicit_yaws = len(yaws_deg) == len(lats)
    for idx, (lat, lon) in enumerate(zip(lats, lons)):
        if use_explicit_yaws:
            yaw_deg = _normalize_yaw_deg(float(yaws_deg[idx]))
        elif idx + 1 < len(lats):
            yaw_deg = _bearing_deg(lat, lon, float(lats[idx + 1]), float(lons[idx + 1]))
        elif loop and len(lats) > 1:
            yaw_deg = _bearing_deg(lat, lon, float(lats[0]), float(lons[0]))
        elif idx > 0:
            yaw_deg = _bearing_deg(float(lats[idx - 1]), float(lons[idx - 1]), lat, lon)
        else:
            yaw_deg = 0.0
        resolved.append(RouteWaypoint(lat=float(lat), lon=float(lon), yaw_deg=float(yaw_deg)))
    return resolved


def _split_home_waypoint(
    base_waypoints: Sequence[RouteWaypoint],
    action_jsons: Sequence[str],
    waypoint_roles: Sequence[str],
) -> Tuple[List[RouteWaypoint], List[str], List[str], Optional[RouteMissionHome], str]:
    route = list(base_waypoints)
    actions = list(action_jsons)
    roles = list(waypoint_roles)
    if len(actions) != len(route):
        actions = ["" for _ in route]
    if len(roles) != len(route):
        roles = [WAYPOINT_ROLE_NORMAL for _ in route]

    patrol_waypoints: List[RouteWaypoint] = []
    patrol_actions: List[str] = []
    patrol_roles: List[str] = []
    home_waypoint: Optional[RouteMissionHome] = None

    for idx, waypoint in enumerate(route):
        role = str(roles[idx] or WAYPOINT_ROLE_NORMAL).strip().lower()
        action_json = str(actions[idx] or "")
        if role == WAYPOINT_ROLE_HOME:
            if action_json:
                return [], [], [], None, "HOME waypoint cannot include actions"
            if home_waypoint is not None:
                return [], [], [], None, "only one HOME waypoint is allowed"
            home_waypoint = RouteMissionHome(waypoint=waypoint, input_index=int(idx))
            continue
        patrol_waypoints.append(waypoint)
        patrol_actions.append(action_json)
        patrol_roles.append(role or WAYPOINT_ROLE_NORMAL)

    if not patrol_waypoints:
        return [], [], [], home_waypoint, "at least one non-HOME waypoint is required"
    return patrol_waypoints, patrol_actions, patrol_roles, home_waypoint, ""


def expand_route_waypoints(
    base_waypoints: Sequence[RouteWaypoint],
    *,
    leg_spacing_m: float,
    loop: bool,
) -> List[RouteWaypoint]:
    base = list(base_waypoints)
    if len(base) <= 1:
        return base

    spacing = max(1.0, float(leg_spacing_m))
    expanded: List[RouteWaypoint] = [base[0]]
    segment_count = len(base) if loop else len(base) - 1

    for idx in range(segment_count):
        start = base[idx]
        end = base[(idx + 1) % len(base)]
        distance_m = _distance_m(start.lat, start.lon, end.lat, end.lon)
        split_count = max(1, int(math.ceil(distance_m / spacing)))
        for split_idx in range(1, split_count):
            expanded.append(_interpolate_waypoint(start, end, float(split_idx) / float(split_count)))
        if idx + 1 < len(base):
            expanded.append(base[idx + 1])

    return expanded


def expand_route_waypoints_with_actions(
    base_waypoints: Sequence[RouteWaypoint],
    action_jsons: Sequence[str],
    *,
    leg_spacing_m: float,
    loop: bool,
    base_key_flags: Optional[Sequence[bool]] = None,
) -> Tuple[List[RouteWaypoint], List[str], List[bool]]:
    base = list(base_waypoints)
    base_actions = list(action_jsons)
    base_flags = list(base_key_flags or [True for _ in base])
    if len(base_actions) != len(base):
        base_actions = ["" for _ in base]
    if len(base_flags) != len(base):
        base_flags = [True for _ in base]
    if len(base) <= 1:
        return base, base_actions, [bool(value) for value in base_flags]

    spacing = max(1.0, float(leg_spacing_m))
    expanded: List[RouteWaypoint] = [base[0]]
    expanded_actions: List[str] = [str(base_actions[0] or "")]
    expanded_key_flags: List[bool] = [bool(base_flags[0])]
    segment_count = len(base) if loop else len(base) - 1

    for idx in range(segment_count):
        start = base[idx]
        next_base_index = (idx + 1) % len(base)
        end = base[next_base_index]
        distance_m = _distance_m(start.lat, start.lon, end.lat, end.lon)
        split_count = max(1, int(math.ceil(distance_m / spacing)))
        for split_idx in range(1, split_count):
            expanded.append(_interpolate_waypoint(start, end, float(split_idx) / float(split_count)))
            expanded_actions.append("")
            expanded_key_flags.append(False)
        if idx + 1 < len(base):
            expanded.append(base[idx + 1])
            expanded_actions.append(str(base_actions[idx + 1] or ""))
            expanded_key_flags.append(bool(base_flags[idx + 1]))

    return expanded, expanded_actions, expanded_key_flags


def expand_route_waypoint_roles(
    base_waypoints: Sequence[RouteWaypoint],
    base_roles: Sequence[str],
    *,
    leg_spacing_m: float,
    loop: bool,
) -> List[str]:
    """Propaga roles sin convertir las cabeceras en seguimiento estricto."""
    base = list(base_waypoints)
    roles = [str(role or WAYPOINT_ROLE_NORMAL).strip().lower() for role in base_roles]
    if len(roles) != len(base):
        roles = [WAYPOINT_ROLE_NORMAL for _ in base]
    if len(base) <= 1:
        return roles

    spacing = max(1.0, float(leg_spacing_m))
    expanded_roles = [roles[0]]
    segment_count = len(base) if loop else len(base) - 1
    for idx in range(segment_count):
        next_index = (idx + 1) % len(base)
        start = base[idx]
        end = base[next_index]
        distance_m = _distance_m(start.lat, start.lon, end.lat, end.lon)
        split_count = max(1, int(math.ceil(distance_m / spacing)))
        segment_role = (
            WAYPOINT_ROLE_COVERAGE
            if (
                roles[idx] == WAYPOINT_ROLE_COVERAGE
                and roles[next_index] == WAYPOINT_ROLE_COVERAGE
            )
            else (
                WAYPOINT_ROLE_COVERAGE_CURVE
                if (
                    roles[idx] == WAYPOINT_ROLE_COVERAGE_CURVE
                    and roles[next_index] == WAYPOINT_ROLE_COVERAGE_CURVE
                )
                else WAYPOINT_ROLE_COVERAGE_TRANSIT
            )
        )
        expanded_roles.extend([segment_role] * max(0, split_count - 1))
        if idx + 1 < len(base):
            expanded_roles.append(roles[idx + 1])
    return expanded_roles


def coverage_full_row_leg_spacing_m(
    minimum_m: float,
    field_length_m: float,
    max_turn_separation_m: float = 0.0,
) -> float:
    """Espaciado que evita metas sinteticas dentro de una fila completa."""
    return float(
        max(
            float(minimum_m),
            float(field_length_m),
            float(max_turn_separation_m),
        )
        + 1.0
    )


def should_follow_exact_coverage_chunk(
    mission_follow_exact_path: bool,
    chunk_roles: Sequence[str],
) -> bool:
    """Identifica una pasada de trabajo completa, sin cabecera mezclada."""
    return bool(
        mission_follow_exact_path
        and len(chunk_roles) > 1
        and all(str(role).strip().lower() == WAYPOINT_ROLE_COVERAGE for role in chunk_roles)
    )


def should_follow_exact_precision_anchor_chunk(
    mission_follow_exact_path: bool,
    chunk_roles: Sequence[str],
) -> bool:
    """Seguir exactas las dos mitades separadas por el ancla de una curva."""
    normalized = [str(role).strip().lower() for role in chunk_roles]
    return bool(
        mission_follow_exact_path
        and len(normalized) > 1
        and WAYPOINT_ROLE_COVERAGE_CURVE in normalized
        and WAYPOINT_ROLE_COVERAGE_TRANSIT in normalized
        and all(
            role
            in (
                WAYPOINT_ROLE_COVERAGE,
                WAYPOINT_ROLE_COVERAGE_CURVE,
                WAYPOINT_ROLE_COVERAGE_TRANSIT,
            )
            for role in normalized
        )
    )


def coverage_flexible_chunk_pass_indices(
    waypoint_roles: Sequence[str],
    key_flags: Sequence[bool],
) -> Set[int]:
    """Keys flexibles que NO pueden cerrar un bloque de cobertura.

    Una key ``coverage_transit`` es, por definicion, una guia de entrada o de
    salida: se acepta con la tolerancia floja (metros) y sin exigir rumbo. Si un
    chunk terminara ahi, la accion se daria por cumplida con el vehiculo hasta
    varios metros afuera y sin alinear, y el tramo siguiente arrancaria un
    ``FollowPath`` nuevo desde esa pose torcida. Eso es lo que hacia que la
    salida de la curva y la fila siguiente se pelearan: Nav2 intentaba cortar
    camino y devolvia ``CONTROLLER_COLLISION`` y despues ``NO_VALID_PATH``.

    Marcandolas como paso, la curva y la fila siguiente viajan en un unico
    camino continuo (apex preciso -> tangente floja -> alineacion floja ->
    inicio de fila -> fila entera) y el bloque solo cierra donde la pose si
    importa: el apex de una curva o una key de trabajo. No se agrega ningun
    waypoint: solo se dejan de cortar los que ya existian.
    """
    roles = [str(role).strip().lower() for role in waypoint_roles]
    flags = list(key_flags)
    return {
        index
        for index in range(min(len(roles), len(flags)))
        if bool(flags[index]) and roles[index] == WAYPOINT_ROLE_COVERAGE_TRANSIT
    }


def block_target_already_passed(
    route: Sequence[RouteWaypoint],
    *,
    end_index: int,
    robot_lat: Optional[float],
    robot_lon: Optional[float],
    corridor_tolerance_m: float,
) -> bool:
    """True cuando el vehiculo ya paso de largo el cierre del bloque.

    Nav2 poda del path las poses que quedaron atras; si TODAS quedaron atras el
    controlador aborta con ``Resulting plan has 0 poses in it`` y se cae la
    mision entera. Eso pasa cuando el bloque cierra en una meta que el vehiculo
    ya cruzo sin entrar en su tolerancia -tipico en el apex de una S, que se
    barre de largo en vez de pisarse-.

    No es saltear cobertura: el punto ya se recorrio. Lo unico que se evita es
    despachar un camino que el controlador no puede seguir.
    """
    route_list = list(route)
    index = int(end_index)
    if index <= 0 or index >= len(route_list):
        return False
    if (
        robot_lat is None
        or robot_lon is None
        or (not np.isfinite(float(robot_lat)))
        or (not np.isfinite(float(robot_lon)))
    ):
        return False

    start = route_list[index - 1]
    end = route_list[index]
    robot_x, robot_y = _local_xy_m(
        float(robot_lat),
        float(robot_lon),
        origin_lat=float(robot_lat),
        origin_lon=float(robot_lon),
    )
    start_x, start_y = _local_xy_m(
        start.lat, start.lon, origin_lat=float(robot_lat), origin_lon=float(robot_lon)
    )
    end_x, end_y = _local_xy_m(
        end.lat, end.lon, origin_lat=float(robot_lat), origin_lon=float(robot_lon)
    )
    seg_x = end_x - start_x
    seg_y = end_y - start_y
    seg_len_sq = (seg_x * seg_x) + (seg_y * seg_y)
    if seg_len_sq <= 1.0e-6:
        return False

    ratio = (((robot_x - start_x) * seg_x) + ((robot_y - start_y) * seg_y)) / seg_len_sq
    if ratio <= 1.0:
        return False
    # Fuera del pasillo del tramo no se puede afirmar que se paso de largo:
    # puede ser otra fila que quedo cerca.
    perpendicular_m = abs(
        ((robot_x - start_x) * seg_y) - ((robot_y - start_y) * seg_x)
    ) / math.sqrt(seg_len_sq)
    return perpendicular_m <= max(0.05, float(corridor_tolerance_m))


def coverage_block_anchor_index(
    key_flags: Sequence[bool],
    key_pass_indices: Set[int],
    start_index: int,
) -> int:
    """Indice preciso donde abre el bloque que se va a despachar.

    Las guias y las keys flojas son puntos internos del camino, no anclas: el
    ancla real es la key precisa anterior (el apex de una curva o un extremo de
    fila). Rearmar siempre desde ahi hace que el tramo despachado no dependa de
    cuantos puntos se dieron por alcanzados antes de armarlo.
    """
    anchor = min(int(start_index) - 1, len(key_flags) - 1)
    passes = {int(index) for index in (key_pass_indices or set())}
    while anchor > 0 and ((not bool(key_flags[anchor])) or anchor in passes):
        anchor -= 1
    return max(0, anchor)


def needs_exact_path_approach(
    *,
    follow_exact_path: bool,
    resume_exact_at_start: bool,
    robot_pose: Optional[Tuple[float, float]],
    first_waypoint: RouteWaypoint,
    reached_tolerance_m: float,
) -> bool:
    """True when a coverage row must be approached before being executed.

    This deliberately applies to every row, not just the route's first row:
    the preceding headland transition can leave the robot farther than the
    controller's local path search window from the next row start.
    """
    return bool(
        follow_exact_path
        and not resume_exact_at_start
        and robot_pose is not None
        and _distance_m(
            float(robot_pose[0]),
            float(robot_pose[1]),
            float(first_waypoint.lat),
            float(first_waypoint.lon),
        ) > float(reached_tolerance_m)
    )


def drop_duplicate_loop_closure(
    base_waypoints: Sequence[RouteWaypoint],
    *,
    loop: bool,
    closure_tolerance_m: float,
) -> Tuple[List[RouteWaypoint], bool]:
    route = list(base_waypoints)
    if (not loop) or len(route) <= 2:
        return route, False

    tolerance_m = max(0.05, float(closure_tolerance_m))
    if _distance_m(route[0].lat, route[0].lon, route[-1].lat, route[-1].lon) > tolerance_m:
        return route, False

    return route[:-1], True


def drop_duplicate_loop_closure_with_actions(
    base_waypoints: Sequence[RouteWaypoint],
    action_jsons: Sequence[str],
    *,
    loop: bool,
    closure_tolerance_m: float,
) -> Tuple[List[RouteWaypoint], List[str], bool]:
    route, dropped = drop_duplicate_loop_closure(
        base_waypoints,
        loop=loop,
        closure_tolerance_m=closure_tolerance_m,
    )
    actions = list(action_jsons)
    if dropped:
        actions = actions[: len(route)]
    return route, actions, dropped


def prepare_route_waypoints(
    base_waypoints: Sequence[RouteWaypoint],
    *,
    loop: bool,
    robot_lat: Optional[float],
    robot_lon: Optional[float],
    waypoint_reached_tolerance_m: float,
    segment_start_tolerance_m: float = 5.0,
    action_jsons: Optional[Sequence[str]] = None,
    waypoint_roles: Optional[Sequence[str]] = None,
    allow_segment_join: bool = True,
    force_first_waypoint: bool = False,
) -> Tuple[Optional[PreparedRouteMission], str]:
    route = list(base_waypoints)
    source_indices = list(range(len(route)))
    actions = list(action_jsons or ["" for _ in route])
    roles = list(waypoint_roles or [WAYPOINT_ROLE_NORMAL for _ in route])
    if len(actions) != len(route):
        actions = ["" for _ in route]
    if len(roles) != len(route):
        roles = [WAYPOINT_ROLE_NORMAL for _ in route]
    if not route:
        return PreparedRouteMission([], [], [], 0, 0, False, ""), ""

    if (
        robot_lat is None
        or robot_lon is None
        or (not np.isfinite(float(robot_lat)))
        or (not np.isfinite(float(robot_lon)))
    ):
        return PreparedRouteMission(route, actions, roles, 0, 0, False, ""), ""

    tolerance_m = max(0.05, float(waypoint_reached_tolerance_m))
    segment_tolerance_raw = float(segment_start_tolerance_m)
    segment_tolerance_m = (
        max(tolerance_m, segment_tolerance_raw)
        if np.isfinite(segment_tolerance_raw)
        else tolerance_m
    )
    distances_m = [
        _distance_m(float(robot_lat), float(robot_lon), waypoint.lat, waypoint.lon)
        for waypoint in route
    ]

    if loop:
        if all(distance_m <= tolerance_m for distance_m in distances_m):
            return None, (
                f"route already within {tolerance_m:.2f} m of robot; "
                "refine the patrol route before sending it"
            )

        if len(route) <= 1:
            return PreparedRouteMission(route, actions, roles, 0, 0, False, ""), ""

        reached_indexes = [
            idx for idx, distance_m in enumerate(distances_m) if distance_m <= tolerance_m
        ]
        if not reached_indexes:
            closest_segment = _closest_route_segment(
                route,
                robot_lat=float(robot_lat),
                robot_lon=float(robot_lon),
                loop=True,
            )
            if not _is_usable_segment_match(
                closest_segment,
                segment_tolerance_m=segment_tolerance_m,
            ):
                return PreparedRouteMission(route, actions, roles, 0, 0, False, ""), ""

            match = closest_segment
            if match is None:
                return PreparedRouteMission(route, actions, roles, 0, 0, False, ""), ""
            start_index = int(match.next_index)
            if start_index == 0:
                return PreparedRouteMission(route, actions, roles, 0, 0, False, ""), ""
            return (
                PreparedRouteMission(
                    route[start_index:] + route[:start_index],
                    actions[start_index:] + actions[:start_index],
                    roles[start_index:] + roles[:start_index],
                    int(start_index),
                    0,
                    True,
                    (
                        "loop joined nearest segment "
                        f"{match.segment_index + 1}->{match.next_index + 1}"
                    ),
                    source_indices[start_index:] + source_indices[:start_index],
                ),
                "",
            )

        anchor_index = min(reached_indexes, key=lambda idx: (distances_m[idx], idx))
        start_index = anchor_index
        for _ in range(len(route)):
            if distances_m[start_index] > tolerance_m:
                break
            start_index = (start_index + 1) % len(route)

        if start_index == 0:
            return PreparedRouteMission(route, actions, roles, 0, 0, False, ""), ""

        return (
            PreparedRouteMission(
                route[start_index:] + route[:start_index],
                actions[start_index:] + actions[:start_index],
                roles[start_index:] + roles[:start_index],
                int(start_index),
                0,
                True,
                f"loop rotated to waypoint {start_index + 1}",
                source_indices[start_index:] + source_indices[:start_index],
            ),
            "",
        )

    if distances_m[-1] <= tolerance_m:
        return None, (
            f"final waypoint already within {tolerance_m:.2f} m of robot; "
            "reorder the non-loop route or enable loop mode"
        )

    start_index = 0
    if not force_first_waypoint:
        while start_index < (len(route) - 1) and distances_m[start_index] <= tolerance_m:
            start_index += 1

    segment_note = ""
    # El enganche a un tramo del medio descarta las metas anteriores. Sirve para
    # retomar una ruta que el robot ya venia haciendo, y hay que poder apagarlo:
    # en cobertura la ruta se recorre completa desde su primera meta, y los lazos
    # de cabecera pasan cerca del vehiculo sin que eso signifique que ese tramo ya
    # se hizo.
    if allow_segment_join and start_index == 0 and len(route) > 1:
        closest_segment = _closest_route_segment(
            route,
            robot_lat=float(robot_lat),
            robot_lon=float(robot_lon),
            loop=False,
        )
        if _is_usable_segment_match(
            closest_segment,
            segment_tolerance_m=segment_tolerance_m,
        ):
            match = closest_segment
            if match is None:
                return PreparedRouteMission(route, actions, roles, 0, 0, False, ""), ""
            start_index = max(start_index, int(match.next_index))
            segment_note = (
                "joined nearest segment "
                f"{match.segment_index + 1}->{match.next_index + 1}"
            )

    note = ""
    if segment_note:
        note = segment_note
    elif start_index > 0:
        suffix = "s" if start_index != 1 else ""
        note = f"skipped {start_index} reached waypoint{suffix}"

    return (
        PreparedRouteMission(
            route[start_index:],
            actions[start_index:],
            roles[start_index:],
            int(start_index),
            int(start_index),
            False,
            note,
            source_indices[start_index:],
        ),
        "",
    )


def skip_reached_chunk_start(
    route: Sequence[RouteWaypoint],
    *,
    start_index: int,
    loop: bool,
    robot_lat: Optional[float],
    robot_lon: Optional[float],
    waypoint_reached_tolerance_m: float,
    protected_indices: Optional[Set[int]] = None,
    loose_indices: Optional[Set[int]] = None,
    loose_tolerance_m: Optional[float] = None,
    index_tolerances_m: Optional[Dict[int, float]] = None,
) -> Tuple[Optional[int], int]:
    route_list = list(route)
    if not route_list:
        return None, 0

    total = len(route_list)
    protected = {int(index) for index in (protected_indices or set())}
    # Las guias de cabecera son transito afuera del lote: darlas por alcanzadas
    # antes evita que el vehiculo frene y corrija donde la precision no importa.
    # Los surcos no entran aca y conservan su tolerancia.
    loose = {int(index) for index in (loose_indices or set())}
    custom_tolerances = {
        int(index): max(0.05, float(value))
        for index, value in (index_tolerances_m or {}).items()
    }
    requested_start = max(0, int(start_index))
    if (not loop) and requested_start >= total:
        return None, 0

    start = requested_start % total if loop else requested_start
    if (
        robot_lat is None
        or robot_lon is None
        or (not np.isfinite(float(robot_lat)))
        or (not np.isfinite(float(robot_lon)))
    ):
        return start, 0

    tolerance_m = max(0.05, float(waypoint_reached_tolerance_m))
    max_skips = max(0, total - 1)
    skipped = 0
    current = start
    while skipped < max_skips:
        if current in protected:
            break
        distance_m = _distance_m(
            float(robot_lat),
            float(robot_lon),
            route_list[current].lat,
            route_list[current].lon,
        )
        limite_m = tolerance_m
        if current in loose and loose_tolerance_m is not None:
            limite_m = max(tolerance_m, float(loose_tolerance_m))
        if current in custom_tolerances:
            limite_m = max(limite_m, custom_tolerances[current])
        if distance_m > limite_m:
            break
        next_index = current + 1
        if loop:
            next_index %= total
        elif next_index >= total:
            break
        current = next_index
        skipped += 1

    return current, skipped


def skip_passed_synthetic_chunk_start(
    route: Sequence[RouteWaypoint],
    *,
    start_index: int,
    loop: bool,
    robot_lat: Optional[float],
    robot_lon: Optional[float],
    segment_tolerance_m: float,
    skippable_indices: Optional[Set[int]] = None,
    protected_indices: Optional[Set[int]] = None,
) -> Tuple[Optional[int], int]:
    route_list = list(route)
    if not route_list:
        return None, 0

    total = len(route_list)
    requested_start = max(0, int(start_index))
    if (not loop) and requested_start >= total:
        return None, 0
    current = requested_start % total if loop else requested_start
    if total <= 1:
        return current, 0
    if (
        robot_lat is None
        or robot_lon is None
        or (not np.isfinite(float(robot_lat)))
        or (not np.isfinite(float(robot_lon)))
    ):
        return current, 0

    skippable = {int(index) for index in (skippable_indices or set())}
    protected = {int(index) for index in (protected_indices or set())}
    tolerance_m = max(0.05, float(segment_tolerance_m))
    skipped = 0
    max_skips = max(0, total - 1)

    while skipped < max_skips and current in skippable and current not in protected:
        next_index = current + 1
        if loop:
            next_index %= total
        elif next_index >= total:
            break
        if next_index == current:
            break

        start = route_list[current]
        end = route_list[next_index]
        robot_x, robot_y = _local_xy_m(
            float(robot_lat),
            float(robot_lon),
            origin_lat=float(robot_lat),
            origin_lon=float(robot_lon),
        )
        start_x, start_y = _local_xy_m(
            start.lat,
            start.lon,
            origin_lat=float(robot_lat),
            origin_lon=float(robot_lon),
        )
        end_x, end_y = _local_xy_m(
            end.lat,
            end.lon,
            origin_lat=float(robot_lat),
            origin_lon=float(robot_lon),
        )
        seg_x = end_x - start_x
        seg_y = end_y - start_y
        seg_len_sq = (seg_x * seg_x) + (seg_y * seg_y)
        if seg_len_sq <= 1.0e-6:
            break

        raw_ratio = (((robot_x - start_x) * seg_x) + ((robot_y - start_y) * seg_y)) / seg_len_sq
        if raw_ratio <= 0.0:
            break
        ratio = min(1.0, float(raw_ratio))
        proj_x = start_x + (seg_x * ratio)
        proj_y = start_y + (seg_y * ratio)
        distance_m = math.hypot(robot_x - proj_x, robot_y - proj_y)
        if distance_m > tolerance_m:
            break

        current = next_index
        skipped += 1

    return current, skipped


def resolve_blocked_retry_start(
    route: Sequence[RouteWaypoint],
    *,
    current_start_index: int,
    current_target_index: Optional[int] = None,
    loop: bool,
    robot_lat: Optional[float],
    robot_lon: Optional[float],
    waypoint_reached_tolerance_m: float,
    segment_start_tolerance_m: float,
    reanchor_enabled: bool,
) -> BlockedRetryStartResolution:
    route_list = list(route)
    total = len(route_list)
    requested_start = max(0, int(current_start_index))
    if total <= 0:
        return BlockedRetryStartResolution(
            requested_start,
            requested_start,
            False,
            "empty_route",
            math.inf,
        )

    current = requested_start % total if loop else min(requested_start, total - 1)
    if not reanchor_enabled:
        return BlockedRetryStartResolution(current, current, False, "disabled", math.inf)
    if (
        robot_lat is None
        or robot_lon is None
        or (not np.isfinite(float(robot_lat)))
        or (not np.isfinite(float(robot_lon)))
    ):
        return BlockedRetryStartResolution(current, current, False, "pose_unavailable", math.inf)

    tolerance_m = max(0.05, float(waypoint_reached_tolerance_m))
    segment_tolerance_m = max(tolerance_m, float(segment_start_tolerance_m))
    distances_m = [
        _distance_m(float(robot_lat), float(robot_lon), waypoint.lat, waypoint.lon)
        for waypoint in route_list
    ]
    nearest_index = min(range(total), key=lambda idx: (distances_m[idx], idx))
    nearest_distance_m = float(distances_m[nearest_index])

    candidate: Optional[int] = None
    reason = ""
    match_distance_m = math.inf
    if nearest_distance_m <= tolerance_m:
        candidate = int(nearest_index)
        max_steps = max(0, total - 1)
        steps = 0
        while steps < max_steps and distances_m[candidate] <= tolerance_m:
            next_index = candidate + 1
            if loop:
                next_index %= total
            elif next_index >= total:
                break
            candidate = next_index
            steps += 1
        reason = "nearest_reached_waypoint"
        match_distance_m = nearest_distance_m
    else:
        match = _closest_route_segment(
            route_list,
            robot_lat=float(robot_lat),
            robot_lon=float(robot_lon),
            loop=loop,
        )
        if _is_usable_segment_match(match, segment_tolerance_m=segment_tolerance_m):
            if match is not None:
                candidate = int(match.next_index)
                reason = "nearest_segment"
                match_distance_m = float(match.distance_m)

    if candidate is None:
        return BlockedRetryStartResolution(current, current, False, "no_match", nearest_distance_m)

    if loop:
        target = current if current_target_index is None else int(current_target_index) % total
        candidate_mod = int(candidate) % total
        if target >= current:
            candidate_is_forward = current <= candidate_mod <= target
        else:
            candidate_is_forward = candidate_mod >= current or candidate_mod <= target
        if not candidate_is_forward:
            return BlockedRetryStartResolution(
                current,
                current,
                False,
                "no_forward_reanchor_loop",
                match_distance_m,
            )
        resolved = candidate_mod
    else:
        resolved = max(current, min(candidate, total - 1))
    if resolved == current:
        return BlockedRetryStartResolution(
            current,
            current,
            False,
            "no_forward_reanchor" if (not loop) and candidate < current else reason,
            match_distance_m,
        )
    return BlockedRetryStartResolution(current, resolved, True, reason, match_distance_m)


def build_chunk_waypoints(
    route: Sequence[RouteWaypoint],
    *,
    start_index: int,
    loop: bool,
    chunk_span_m: float,
    chunk_max_waypoints: int,
    action_stop_indices: Optional[Set[int]] = None,
    key_stop_indices: Optional[Set[int]] = None,
    key_pass_indices: Optional[Set[int]] = None,
) -> Tuple[List[RouteWaypoint], int]:
    route_list = list(route)
    if not route_list:
        return [], 0

    total = len(route_list)
    requested_start = max(0, int(start_index))
    if not loop and requested_start >= total:
        return [], total
    start = requested_start % total if loop else requested_start
    max_points = max(1, int(chunk_max_waypoints))
    stop_indices = {int(index) for index in (action_stop_indices or set())}
    key_indices = {int(index) for index in (key_stop_indices or set())}
    # Una key floja no cierra el bloque: se cruza como punto interno del path.
    pass_indices = {int(index) for index in (key_pass_indices or set())}
    use_key_boundaries = bool(key_indices)
    if loop and total > 1 and not use_key_boundaries:
        # Do not send a full loop in one NavigateThroughPoses chunk. If the robot is
        # already at the closure waypoint, Nav2 can report success immediately because
        # the final pose is within goal tolerance.
        max_points = min(max_points, max(1, total - 1))
    max_span_m = max(1.0, float(chunk_span_m))

    chunk = [route_list[start]]
    end_index = start
    if start in stop_indices:
        return chunk, end_index

    visited_steps = 0
    cumulative_distance_m = 0.0

    while True:
        if not use_key_boundaries and len(chunk) >= max_points:
            break
        next_index = end_index + 1
        if loop:
            next_index %= total
            if next_index == start:
                break
        elif next_index >= total:
            break

        step_distance_m = _distance_m(
            route_list[end_index].lat,
            route_list[end_index].lon,
            route_list[next_index].lat,
            route_list[next_index].lon,
        )
        should_stop = (
            not use_key_boundaries
            and len(chunk) > 1
            and (cumulative_distance_m + step_distance_m) > max_span_m
        )
        if should_stop:
            break

        chunk.append(route_list[next_index])
        cumulative_distance_m += step_distance_m
        end_index = next_index
        if next_index in stop_indices or (
            use_key_boundaries
            and next_index in key_indices
            and next_index not in pass_indices
        ):
            break
        visited_steps += 1
        if visited_steps >= max(0, total - 1):
            break

    if (
        len(chunk) == 1
        and start not in stop_indices
        and total > 1
        and ((not loop) or max_points > 1)
    ):
        next_index = (start + 1) % total if loop else start + 1
        if next_index < total and next_index != start:
            chunk.append(route_list[next_index])
            end_index = next_index

    return chunk, end_index


def next_chunk_start_index(
    *,
    current_target_index: int,
    route_size: int,
    loop: bool,
) -> int:
    total = max(0, int(route_size))
    if total <= 0:
        return 0

    target_index = max(0, int(current_target_index))
    if loop and total > 1:
        return (target_index + 1) % total
    return min(target_index + 1, total)


def should_suppress_chunk_success_brake(
    *,
    current_target_index: int,
    route_size: int,
    loop: bool,
) -> bool:
    total = max(0, int(route_size))
    if bool(loop):
        return total > 1
    return int(current_target_index) < max(0, total - 1)


# Claves de `values` en la validacion de cobertura que NO son numeros. Se listan
# aparte porque el resto del dict se convierte a float en bloque: sumar una clave
# no numerica sin agregarla aca revienta la validacion.
_NON_NUMERIC_COVERAGE_KEYS = frozenset(
    {"side", "start_is_field_corner", "coverage_polygon", "coverage_exclusions",
     "field_mode"}
)


class RouteExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("route_executor")

        self.declare_parameter("nav_set_goal_service", "/nav_command_server/set_goal_ll")
        self.declare_parameter("nav_cancel_goal_service", "/nav_command_server/cancel_goal")
        self.declare_parameter("nav_telemetry_topic", "/nav_command_server/telemetry")
        self.declare_parameter("set_route_service", "/route_executor/set_route_ll")
        self.declare_parameter(
            "generate_coverage_plan_service",
            "/route_executor/generate_coverage_plan_ll",
        )
        self.declare_parameter("cancel_route_service", "/route_executor/cancel_route")
        self.declare_parameter("get_state_service", "/route_executor/get_state")
        self.declare_parameter("set_patrol_service", "/route_executor/set_patrol_ll")
        self.declare_parameter("cancel_patrol_service", "/route_executor/cancel_patrol")
        self.declare_parameter("get_patrol_state_service", "/route_executor/get_patrol_state")
        self.declare_parameter("request_return_home_service", "/route_executor/request_return_home")
        self.declare_parameter("set_navigation_profile_service", "/route_executor/set_navigation_profile")
        self.declare_parameter("mission_path_topic", "/route_executor/mission_path")
        self.declare_parameter("active_chunk_path_topic", "/route_executor/active_chunk_path")
        self.declare_parameter("path_frame", "map")
        self.declare_parameter("fromll_service", "/fromLL")
        self.declare_parameter("fromll_service_fallback", "/navsat_transform/fromLL")
        self.declare_parameter("fromll_frame", "odom")
        self.declare_parameter("fromll_wait_timeout_s", 0.2)
        self.declare_parameter("fromll_call_timeout_s", 1.0)
        self.declare_parameter("tf_lookup_timeout_s", 0.5)
        self.declare_parameter("request_timeout_s", 8.0)
        self.declare_parameter("default_leg_spacing_m", 35.0)
        self.declare_parameter("default_chunk_span_m", 120.0)
        self.declare_parameter("default_chunk_max_waypoints", 5)
        self.declare_parameter("min_leg_spacing_m", 5.0)
        self.declare_parameter("min_chunk_span_m", 20.0)
        self.declare_parameter("min_chunk_max_waypoints", 2)
        # Campo conserva las guias de cada arco de cabecera. Con 200 puntos se
        # truncaban globalmente: en un lote de 40 m quedaban unas pocas guias
        # por giro y la polilinea pasaba a tener radios de 2 m aunque
        # Fields2Cover hubiese calculado 4 m. Este limite admite la curva
        # muestreada a 1 m sin afectar la semantica de Ruta o Patrol.
        self.declare_parameter("max_route_input_waypoints", 800)
        self.declare_parameter("coverage_max_rows", 100)
        self.declare_parameter("coverage_max_key_waypoints", 200)
        self.declare_parameter("coverage_max_sampled_waypoints", 2000)
        self.declare_parameter("coverage_max_field_dimension_m", 500.0)
        self.declare_parameter("coverage_max_turning_radius_m", 100.0)
        # 4.0 y no 2.9: es el radio que pide el operador y el que traza el Smac
        # de los perfiles rolling. El piso del plan de cobertura tiene que
        # acompanarlo. El vehiculo hace 2.02 m con el limite operativo de
        # direccion, o sea que 4.0 deja margen de correccion.
        self.declare_parameter("coverage_planner_min_turning_radius_m", 4.0)
        self.declare_parameter("coverage_allow_headland_conflicts", False)
        # Apagado: la cobertura recorre las pasadas de a una, desde la del
        # vehiculo hacia el otro borde. Prenderlo deja que el planificador
        # saltee pasadas para agrandar cada giro y achicar la cabecera, a costa
        # de cubrir el lote en dos bloques intercalados.
        self.declare_parameter("coverage_allow_row_skipping", False)
        self.declare_parameter("coverage_min_waypoint_spacing_m", 0.5)
        # 0.0 significa auto: se calcula como 0.10*sqrt(largo*ancho) y se
        # acota entre los dos parametros siguientes. Un valor positivo fuerza
        # el espaciado para una instalacion que necesite una densidad fija.
        self.declare_parameter("coverage_waypoint_spacing_m", 0.0)
        self.declare_parameter("coverage_auto_waypoint_spacing_ratio", 0.10)
        self.declare_parameter("coverage_auto_waypoint_spacing_min_m", 1.0)
        self.declare_parameter("coverage_auto_waypoint_spacing_max_m", 4.0)
        self.declare_parameter("coverage_topology_audit_spacing_m", 0.5)
        # Desactivado por ahora: el rodeo poligonal genera quiebres rigidos que
        # un Ackermann no puede seguir con precision. El editor y el codigo de
        # zonas se conservan para rehacerlo con curvas factibles, pero CAMPO no
        # debe deformar sus filas con esta implementacion.
        self.declare_parameter("coverage_nogo_enabled", False)
        self.declare_parameter("zones_get_state_service", "/zones_manager/get_state")
        self.declare_parameter("zones_refresh_period_s", 2.0)
        self.declare_parameter("zones_max_age_s", 15.0)
        # Colchon sobre el medio ancho de corte: la ruta pasa por el centro del
        # implemento, asi que sin margen el ala de afuera pisaria la zona.
        self.declare_parameter("coverage_nogo_extra_margin_m", 0.5)
        # Cuanto radio de giro se suma al margen, ademas del medio implemento.
        #
        # Va en 0 por decision de operacion: el margen infla la zona, y el
        # trazado empieza a desviarse donde termina el poligono inflado, no
        # donde esta la zona. Con una zona de 5.2 x 3.2 m y radio 2.9 m, medido:
        #
        #   ratio 0.0 (margen 1.50 m): se desvia 1.5 m antes, 3 pasadas tocadas
        #   ratio 1.0 (margen 4.40 m): se desvia 4.4 m antes, 7 pasadas tocadas
        #
        # Subirlo hace que el rodeo tenga espacio para que el vehiculo maniobre
        # sin pisar la zona —un Ackermann no puede doblar en escuadra—, pero a
        # costa de agrandar el hueco y de empezar a esquivar mucho antes.
        # Medido con margen 1.5 m: el robot llega a meterse 1.14 m adentro.
        # Lo que corrige eso de verdad es prender el filtro keepout del costmap,
        # que es lo unico que hace que Nav2 respete la zona por si mismo.
        self.declare_parameter("coverage_nogo_turning_margin_ratio", 0.0)
        # Que planificador atiende el modo Campo.
        #
        #   legacy       : el generador propio, rectangulo pose+largo+ancho.
        #   fields2cover : el planificador agricola, acepta poligono con huecos.
        #
        # Solo Campo mira este parametro. Ruta, patrulla y goals no pasan por
        # aca en ningun caso.
        self.declare_parameter("coverage_planner", "legacy")
        # Solo se leen con coverage_planner=fields2cover.
        self.declare_parameter("coverage_f2c_action", "compute_coverage_path")
        self.declare_parameter(
            "coverage_f2c_parameter_service", "/coverage_server/set_parameters"
        )
        self.declare_parameter(
            "coverage_f2c_change_state_service", "/coverage_server/change_state"
        )
        self.declare_parameter("coverage_f2c_timeout_s", 30.0)
        # BOUSTROPHEDON: pasada por pasada, en orden, que es como se trabaja un
        # lote. Estuvo en SNAKE mientras los giros los resolvia Fields2Cover:
        # medido sobre un octogono de 50 m, BOUSTROPHEDON dejaba giros de 0.32 m
        # de radio —inejecutables— porque a 1.65 m de separacion no entra
        # ninguna curva hacia adelante, y saltear pasadas era la unica forma de
        # agrandar el giro. Ese motivo ya no existe: la cabecera de tres puntos
        # descarta los giros de Fields2Cover y arma la transicion retrocediendo,
        # asi que el route_type ahora solo elige el ORDEN de las pasadas. Se lee
        # fresco en cada pedido: se puede volver a SNAKE con `ros2 param set`
        # sin recompilar ni relanzar.
        self.declare_parameter("coverage_f2c_route_type", "BOUSTROPHEDON")
        # DUBIN y no REEDS_SHEPP: Reeds-Shepp usa marcha atras y mete cuspides
        # en las cabeceras. Para una cortadora eso no se quiere.
        self.declare_parameter("coverage_f2c_path_type", "DUBIN")
        self.declare_parameter("coverage_f2c_path_continuity", "CONTINUOUS")
        # Ancho fisico del vehiculo, distinto del ancho de corte.
        self.declare_parameter("coverage_f2c_robot_width_m", 1.0)
        # Cabecera interior. En 0 el lote entero es superficie de trabajo y los
        # giros salen del poligono, que es lo que se quiere en Campo.
        self.declare_parameter("coverage_f2c_headland_width_m", 0.0)
        # Paso de muestreo de los giros. El default de Fields2Cover es
        # 0.1 m y solo infla el conteo de waypoints.
        # Con R=4 m, 1 m entre puntos de arco da una flecha menor a 3.2 cm:
        # suficiente para conservar visual y geometricamente la cabecera, sin
        # convertir cada giro en cientos de metas.
        self.declare_parameter("coverage_f2c_turn_point_distance_m", 1.0)
        # Cabecera flexible. Con la separacion entre pasadas por debajo de dos
        # radios minimos —1.7 m contra 4 m en el caso tipico de Campo— la unica
        # curva Dubins posible es una omega, y el lote termina rodeado de
        # petalos enormes. Con esto prendido los surcos quedan intactos y cada
        # giro se reemplaza por salir derecho, correrse afuera y reentrar
        # derecho. Se apaga para volver a los arcos crudos de Fields2Cover.
        self.declare_parameter("coverage_f2c_flexible_turns", True)
        # Radio de la S con la que se esquiva una zona no-go interna. Es una
        # maniobra suave y adentro del cultivo, asi que no necesita el radio
        # conservador de la cabecera: con el de cabecera el rodeo arranca
        # metros antes de la zona y se lleva puestas filas que no hacia falta
        # tocar. Con 0.0 vuelve a derivarse del radio de cabecera.
        self.declare_parameter("coverage_nogo_lane_change_radius_m", 3.0)
        # Piso al que puede achicarse la S cuando el radio preferido no entra
        # entre la zona y la cabecera. Sin esto el plan se rechazaba entero. El
        # piso geometrico es la mitad de la separacion entre pasadas (2.0 m con
        # 4 m de corte), donde la S degenera en dos cuartos de circulo; 2.5 m
        # deja margen sobre el limite operativo de direccion, que da un radio
        # efectivo de 2.02 m.
        self.declare_parameter("coverage_nogo_lane_change_min_radius_m", 2.5)
        # Que hacer con una fila que una zona no-go interna parte al medio.
        #   headland    : no se recorre esa fila; el resto se enlaza con el
        #                 giro de cabecera normal, que pide 13 grados de
        #                 direccion. Nada de maniobras adentro del cultivo.
        #   lane_change : S hacia la fila vecina, adentro del cultivo. Pide
        #                 17 a 20 grados y solo cierra si la separacion entre
        #                 pasadas supera el alcance de la exclusion.
        self.declare_parameter("coverage_nogo_internal_strategy", "lane_change")
        # Tramo recto antes y despues de los arcos de la cabecera de tres
        # puntos. Con 0.5 m y radio 4 m, el pivote queda 4.5 m mas alla del
        # extremo del surco y 4 m al costado; no altera ningun punto de trabajo.
        self.declare_parameter("coverage_f2c_turn_lead_m", 0.5)
        # Politica forward-only. En False la cobertura no puede planificar ni
        # ejecutar nada que retroceda: la cabecera de tres puntos queda
        # descartada, ningun waypoint sale con backup_m, no se emite la accion
        # coverage_backup y, si llegara una desde otra fuente, la ejecucion la
        # rechaza. La separacion menor que el diametro de giro se resuelve con
        # una omega hacia adelante que sale del lote por la cabecera. La
        # simulacion la apaga; el perfil real la deja prendida porque ahi la
        # marcha atras esta permitida y la cabecera de tres puntos es la
        # maniobra mas corta. Se lee fresca en cada pedido para poder probar el
        # cambio con `ros2 param set`.
        self.declare_parameter("coverage_f2c_allow_reverse", True)
        # Orientacion de las pasadas, en grados del marco del lote. NaN deja que
        # Fields2Cover busque el angulo que minimiza el objetivo (BRUTE_FORCE).
        # Fijarlo importa porque la guarda de aproximacion compara el rumbo del
        # vehiculo contra el de la primera meta: con el angulo libre, el
        # planificador puede elegir uno que el vehiculo no esta mirando.
        self.declare_parameter("coverage_f2c_swath_angle_deg", float("nan"))
        # Tope de waypoints muestreados del planner agricola, aparte del que usa
        # el planificador propio.
        #
        # Los sampled_waypoints son para dibujar el preview, no metas: las metas
        # son las key, y esas siguen con su tope estricto. Fields2Cover muestrea
        # bastante mas denso —cada giro es un Path propio, y una exclusion parte
        # las pasadas y suma giros—, asi que con el tope de 2000 del zigzag un
        # lote normal con una exclusion se rechazaba de entrada.
        self.declare_parameter("coverage_f2c_max_sampled_waypoints", 10000)
        self.declare_parameter("route_waypoint_reached_tolerance_m", 1.2)
        # Los extremos de surco necesitan bastante mas precision que una ruta
        # normal. Con 1.2 m de tolerancia y pasadas separadas 1.7 m, Nav2 puede
        # terminar una fila casi encima de la siguiente y arrancar el giro
        # antes de llegar al borde. La tolerancia de Ruta/Patrol sigue intacta.
        self.declare_parameter("coverage_row_reached_tolerance_m", 0.35)
        # El ancla central de una curva impide recortarla, pero no es un surco:
        # tolera el error normal de Ackermann sin ordenar un circulo correctivo.
        self.declare_parameter("coverage_curve_reached_tolerance_m", 1.25)
        # Las guias de cabecera son transito afuera del lote: no son metas de
        # trabajo y exigirles precision hace que el vehiculo frene y corrija
        # donde no importa. Con esta tolerancia el ejecutor las da por
        # alcanzadas antes y sigue. No toca la tolerancia de los surcos, que
        # es lo unico que tiene que salir exacto.
        self.declare_parameter("coverage_transit_reached_tolerance_m", 3.0)
        # Marcha atras de la cabecera de tres puntos, via el behavior server
        # de Nav2. Solo la usa Campo: ni Ruta ni Patrol emiten esta accion.
        self.declare_parameter("coverage_backup_action", "backup")
        self.declare_parameter("coverage_backup_speed_mps", 0.3)
        self.declare_parameter("coverage_backup_timeout_s", 45.0)
        self.declare_parameter("route_segment_start_tolerance_m", 5.0)
        self.declare_parameter("blocked_retry_max_attempts", 3)
        self.declare_parameter("blocked_retry_wait_s", 5.0)
        self.declare_parameter("blocked_retry_reanchor_on_current_pose", True)
        self.declare_parameter("blocked_retry_reanchor_tolerance_m", 8.0)
        self.declare_parameter("collision_stop_persistent_s", 3.0)
        self.declare_parameter("clear_costmaps_before_blocked_retry", True)
        self.declare_parameter("blocked_retry_tick_hz", 2.0)
        self.declare_parameter("nav_event_topic", "/nav_command_server/events")
        self.declare_parameter("nav_brake_service", "/nav_command_server/brake")
        self.declare_parameter("battery_state_topic", "/battery_state")
        self.declare_parameter("battery_guard_topic", "/battery_mission_guard")
        self.declare_parameter("low_battery_threshold_pct", 25.0)
        self.declare_parameter("default_home_lat", float("nan"))
        self.declare_parameter("default_home_lon", float("nan"))
        self.declare_parameter("default_home_yaw_deg", 0.0)
        self.declare_parameter(
            "clear_local_costmap_service",
            "/local_costmap/clear_entirely_local_costmap",
        )
        self.declare_parameter(
            "clear_global_costmap_service",
            "/global_costmap/clear_entirely_global_costmap",
        )
        self.declare_parameter("local_costmap_node", "/local_costmap/local_costmap")
        self.declare_parameter("global_costmap_node", "/global_costmap/global_costmap")
        self.declare_parameter("controller_server_node", "/controller_server")
        self.declare_parameter("scan_ground_filter_node", "/scan_ground_filter")
        self.declare_parameter("urban_local_inflation_radius", 1.4)
        self.declare_parameter("urban_global_inflation_radius", 1.5)
        self.declare_parameter("urban_local_cost_scaling_factor", 1.3)
        self.declare_parameter("urban_global_cost_scaling_factor", 1.4)
        self.declare_parameter("rural_local_inflation_radius", 0.8)
        self.declare_parameter("rural_global_inflation_radius", 0.8)
        self.declare_parameter("rural_local_cost_scaling_factor", 3.0)
        self.declare_parameter("rural_global_cost_scaling_factor", 3.0)
        self.declare_parameter("urban_ground_global_slope_max_angle_deg", 10.0)
        self.declare_parameter("urban_ground_local_slope_max_angle_deg", 13.0)
        self.declare_parameter("urban_ground_split_height_distance", 0.20)
        self.declare_parameter("rural_ground_global_slope_max_angle_deg", 15.0)
        self.declare_parameter("rural_ground_local_slope_max_angle_deg", 18.0)
        self.declare_parameter("rural_ground_split_height_distance", 0.25)
        self.declare_parameter("navigation_profile_timeout_s", 3.0)

        self.nav_set_goal_service = str(self.get_parameter("nav_set_goal_service").value)
        self.nav_cancel_goal_service = str(self.get_parameter("nav_cancel_goal_service").value)
        self.nav_telemetry_topic = str(self.get_parameter("nav_telemetry_topic").value)
        self.set_route_service = str(self.get_parameter("set_route_service").value)
        self.generate_coverage_plan_service = str(
            self.get_parameter("generate_coverage_plan_service").value
        )
        self.cancel_route_service = str(self.get_parameter("cancel_route_service").value)
        self.get_state_service = str(self.get_parameter("get_state_service").value)
        self.set_patrol_service = str(self.get_parameter("set_patrol_service").value)
        self.cancel_patrol_service = str(self.get_parameter("cancel_patrol_service").value)
        self.get_patrol_state_service = str(
            self.get_parameter("get_patrol_state_service").value
        )
        self.request_return_home_service = str(
            self.get_parameter("request_return_home_service").value
        )
        self.set_navigation_profile_service = str(
            self.get_parameter("set_navigation_profile_service").value
        )
        self.mission_path_topic = str(self.get_parameter("mission_path_topic").value)
        self.active_chunk_path_topic = str(self.get_parameter("active_chunk_path_topic").value)
        self.path_frame = str(self.get_parameter("path_frame").value).strip() or "map"
        self.fromll_service = str(self.get_parameter("fromll_service").value)
        self.fromll_service_fallback = str(self.get_parameter("fromll_service_fallback").value)
        self.fromll_frame = str(self.get_parameter("fromll_frame").value).strip() or "odom"
        self.fromll_wait_timeout_s = max(
            0.01, float(self.get_parameter("fromll_wait_timeout_s").value)
        )
        self.fromll_call_timeout_s = max(
            0.05, float(self.get_parameter("fromll_call_timeout_s").value)
        )
        self.tf_lookup_timeout_s = max(0.0, float(self.get_parameter("tf_lookup_timeout_s").value))
        self.request_timeout_s = max(0.5, float(self.get_parameter("request_timeout_s").value))
        self.default_leg_spacing_m = max(1.0, float(self.get_parameter("default_leg_spacing_m").value))
        self.default_chunk_span_m = max(5.0, float(self.get_parameter("default_chunk_span_m").value))
        self.default_chunk_max_waypoints = max(
            2, int(self.get_parameter("default_chunk_max_waypoints").value)
        )
        self.min_leg_spacing_m = max(1.0, float(self.get_parameter("min_leg_spacing_m").value))
        self.min_chunk_span_m = max(5.0, float(self.get_parameter("min_chunk_span_m").value))
        self.min_chunk_max_waypoints = max(
            2, int(self.get_parameter("min_chunk_max_waypoints").value)
        )
        self.max_route_input_waypoints = max(
            1, int(self.get_parameter("max_route_input_waypoints").value)
        )
        self.coverage_max_rows = max(
            1, int(self.get_parameter("coverage_max_rows").value)
        )
        self.coverage_max_key_waypoints = max(
            2, int(self.get_parameter("coverage_max_key_waypoints").value)
        )
        self.coverage_max_sampled_waypoints = max(
            2, int(self.get_parameter("coverage_max_sampled_waypoints").value)
        )
        self.coverage_max_field_dimension_m = max(
            1.0, float(self.get_parameter("coverage_max_field_dimension_m").value)
        )
        self.coverage_max_turning_radius_m = max(
            0.1, float(self.get_parameter("coverage_max_turning_radius_m").value)
        )
        self.coverage_planner_min_turning_radius_m = max(
            0.1,
            float(
                self.get_parameter("coverage_planner_min_turning_radius_m").value
            ),
        )
        self.coverage_allow_headland_conflicts = bool(
            self.get_parameter("coverage_allow_headland_conflicts").value
        )
        self.coverage_allow_row_skipping = bool(
            self.get_parameter("coverage_allow_row_skipping").value
        )
        self.coverage_min_waypoint_spacing_m = max(
            0.05, float(self.get_parameter("coverage_min_waypoint_spacing_m").value)
        )
        self.coverage_waypoint_spacing_override_m = max(
            0.0, float(self.get_parameter("coverage_waypoint_spacing_m").value)
        )
        self.coverage_auto_waypoint_spacing_ratio = max(
            0.0,
            float(
                self.get_parameter("coverage_auto_waypoint_spacing_ratio").value
            ),
        )
        self.coverage_auto_waypoint_spacing_min_m = max(
            self.coverage_min_waypoint_spacing_m,
            float(
                self.get_parameter("coverage_auto_waypoint_spacing_min_m").value
            ),
        )
        self.coverage_auto_waypoint_spacing_max_m = max(
            self.coverage_auto_waypoint_spacing_min_m,
            float(
                self.get_parameter("coverage_auto_waypoint_spacing_max_m").value
            ),
        )
        self.coverage_nogo_enabled = bool(
            self.get_parameter("coverage_nogo_enabled").value
        )
        self.zones_get_state_service = str(
            self.get_parameter("zones_get_state_service").value
        )
        self.zones_refresh_period_s = max(
            0.2, float(self.get_parameter("zones_refresh_period_s").value)
        )
        self.zones_max_age_s = max(
            self.zones_refresh_period_s * 2.0,
            float(self.get_parameter("zones_max_age_s").value),
        )
        self.coverage_nogo_extra_margin_m = max(
            0.0, float(self.get_parameter("coverage_nogo_extra_margin_m").value)
        )
        self.coverage_nogo_turning_margin_ratio = max(
            0.0,
            float(self.get_parameter("coverage_nogo_turning_margin_ratio").value),
        )
        self.coverage_planner = str(
            self.get_parameter("coverage_planner").value or "legacy"
        ).strip().lower()
        self.coverage_f2c_action = str(
            self.get_parameter("coverage_f2c_action").value
        )
        self.coverage_f2c_parameter_service = str(
            self.get_parameter("coverage_f2c_parameter_service").value
        )
        self.coverage_f2c_change_state_service = str(
            self.get_parameter("coverage_f2c_change_state_service").value
        )
        self.coverage_f2c_timeout_s = max(
            1.0, float(self.get_parameter("coverage_f2c_timeout_s").value)
        )
        self.coverage_f2c_route_type = str(
            self.get_parameter("coverage_f2c_route_type").value
        )
        self.coverage_f2c_path_type = str(
            self.get_parameter("coverage_f2c_path_type").value
        )
        self.coverage_f2c_path_continuity = str(
            self.get_parameter("coverage_f2c_path_continuity").value
        )
        self.coverage_f2c_robot_width_m = max(
            0.01, float(self.get_parameter("coverage_f2c_robot_width_m").value)
        )
        self.coverage_f2c_headland_width_m = max(
            0.0, float(self.get_parameter("coverage_f2c_headland_width_m").value)
        )
        self.coverage_f2c_turn_point_distance_m = max(
            0.05,
            float(self.get_parameter("coverage_f2c_turn_point_distance_m").value),
        )
        self.coverage_f2c_max_sampled_waypoints = max(
            int(self.coverage_max_sampled_waypoints),
            int(self.get_parameter("coverage_f2c_max_sampled_waypoints").value),
        )
        angulo = float(self.get_parameter("coverage_f2c_swath_angle_deg").value)
        self.coverage_f2c_swath_angle_deg = (
            None if not math.isfinite(angulo) else angulo
        )
        if self.coverage_planner not in {"legacy", "fields2cover"}:
            raise ValueError(
                "coverage_planner debe ser 'legacy' o 'fields2cover', "
                f"llego '{self.coverage_planner}'"
            )
        self.coverage_topology_audit_spacing_m = min(
            0.5,
            max(
                0.05,
                float(
                    self.get_parameter("coverage_topology_audit_spacing_m").value
                ),
            ),
        )
        self.route_waypoint_reached_tolerance_m = max(
            0.05, float(self.get_parameter("route_waypoint_reached_tolerance_m").value)
        )
        self.coverage_row_reached_tolerance_m = min(
            self.route_waypoint_reached_tolerance_m,
            max(
                0.05,
                float(
                    self.get_parameter("coverage_row_reached_tolerance_m").value
                ),
            ),
        )
        self.coverage_curve_reached_tolerance_m = max(
            self.coverage_row_reached_tolerance_m,
            float(self.get_parameter("coverage_curve_reached_tolerance_m").value),
        )
        self.coverage_transit_reached_tolerance_m = max(
            self.route_waypoint_reached_tolerance_m,
            float(self.get_parameter("coverage_transit_reached_tolerance_m").value),
        )
        self.coverage_backup_action = str(
            self.get_parameter("coverage_backup_action").value
        )
        self.coverage_backup_speed_mps = max(
            0.05, float(self.get_parameter("coverage_backup_speed_mps").value)
        )
        self.coverage_backup_timeout_s = max(
            1.0, float(self.get_parameter("coverage_backup_timeout_s").value)
        )
        self._backup_client: Optional[ActionClient] = None
        self.route_segment_start_tolerance_m = max(
            self.route_waypoint_reached_tolerance_m,
            float(self.get_parameter("route_segment_start_tolerance_m").value),
        )
        self.blocked_retry_max_attempts = max(
            0, int(self.get_parameter("blocked_retry_max_attempts").value)
        )
        self.blocked_retry_wait_s = max(
            0.0, float(self.get_parameter("blocked_retry_wait_s").value)
        )
        self.blocked_retry_reanchor_on_current_pose = bool(
            self.get_parameter("blocked_retry_reanchor_on_current_pose").value
        )
        self.blocked_retry_reanchor_tolerance_m = max(
            self.route_waypoint_reached_tolerance_m,
            float(self.get_parameter("blocked_retry_reanchor_tolerance_m").value),
        )
        self.collision_stop_persistent_s = max(
            0.0, float(self.get_parameter("collision_stop_persistent_s").value)
        )
        self.clear_costmaps_before_blocked_retry = bool(
            self.get_parameter("clear_costmaps_before_blocked_retry").value
        )
        self.blocked_retry_tick_hz = max(
            0.5, float(self.get_parameter("blocked_retry_tick_hz").value)
        )
        self.nav_event_topic = str(self.get_parameter("nav_event_topic").value)
        self.nav_brake_service = str(self.get_parameter("nav_brake_service").value)
        self.battery_state_topic = str(self.get_parameter("battery_state_topic").value)
        self.battery_guard_topic = str(self.get_parameter("battery_guard_topic").value)
        self.low_battery_threshold_pct = max(
            0.0, min(100.0, float(self.get_parameter("low_battery_threshold_pct").value))
        )
        self.default_home_lat = float(self.get_parameter("default_home_lat").value)
        self.default_home_lon = float(self.get_parameter("default_home_lon").value)
        self.default_home_yaw_deg = _normalize_yaw_deg(
            float(self.get_parameter("default_home_yaw_deg").value)
        )
        self.clear_local_costmap_service = str(
            self.get_parameter("clear_local_costmap_service").value
        )
        self.clear_global_costmap_service = str(
            self.get_parameter("clear_global_costmap_service").value
        )
        self.local_costmap_node = str(self.get_parameter("local_costmap_node").value)
        self.global_costmap_node = str(self.get_parameter("global_costmap_node").value)
        self.controller_server_node = str(
            self.get_parameter("controller_server_node").value
        )
        self.scan_ground_filter_node = str(
            self.get_parameter("scan_ground_filter_node").value
        )
        self.navigation_profile_timeout_s = max(
            0.5, float(self.get_parameter("navigation_profile_timeout_s").value)
        )
        self._navigation_profiles = {
            NAVIGATION_PROFILE_URBAN: {
                "local_radius": float(self.get_parameter("urban_local_inflation_radius").value),
                "global_radius": float(self.get_parameter("urban_global_inflation_radius").value),
                "local_scaling": float(
                    self.get_parameter("urban_local_cost_scaling_factor").value
                ),
                "global_scaling": float(
                    self.get_parameter("urban_global_cost_scaling_factor").value
                ),
                "ground_global_slope": float(
                    self.get_parameter("urban_ground_global_slope_max_angle_deg").value
                ),
                "ground_local_slope": float(
                    self.get_parameter("urban_ground_local_slope_max_angle_deg").value
                ),
                "ground_split_height": float(
                    self.get_parameter("urban_ground_split_height_distance").value
                ),
            },
            NAVIGATION_PROFILE_RURAL: {
                "local_radius": float(self.get_parameter("rural_local_inflation_radius").value),
                "global_radius": float(self.get_parameter("rural_global_inflation_radius").value),
                "local_scaling": float(
                    self.get_parameter("rural_local_cost_scaling_factor").value
                ),
                "global_scaling": float(
                    self.get_parameter("rural_global_cost_scaling_factor").value
                ),
                "ground_global_slope": float(
                    self.get_parameter("rural_ground_global_slope_max_angle_deg").value
                ),
                "ground_local_slope": float(
                    self.get_parameter("rural_ground_local_slope_max_angle_deg").value
                ),
                "ground_split_height": float(
                    self.get_parameter("rural_ground_split_height_distance").value
                ),
            },
        }

        self._lock = threading.Lock()
        self._route_input: List[RouteWaypoint] = []
        self._route_input_source_indices: List[int] = []
        self._route_expanded: List[RouteWaypoint] = []
        self._route_action_jsons: List[str] = []
        self._route_waypoint_roles: List[str] = []
        self._route_key_waypoint_flags: List[bool] = []
        self._route_input_indices: List[int] = []
        self._home_waypoint: Optional[RouteWaypoint] = None
        self._home_input_index: int = -1
        self._active_chunk: List[RouteWaypoint] = []
        self._mission_id = ""
        self._chunk_id = 0
        self._loop_iteration = 0
        self._reached_checkpoint_count = 0
        self._mission_active = False
        self._mission_paused = False
        self._mission_loop = False
        self._mission_follow_exact_path = False
        # La primera pasada CAMPO puede estar lejos del robot. Primero se llega
        # a su inicio con Nav2; luego se reenvia el mismo chunk al seguidor
        # exacto, sin que el recorte de metas alcanzadas elimine ese inicio.
        self._coverage_approach_pending = False
        self._coverage_exact_resume_index = -1
        self._mission_status = "idle"
        self._leg_spacing_m = float(self.default_leg_spacing_m)
        self._chunk_span_m = float(self.default_chunk_span_m)
        self._chunk_max_waypoints = int(self.default_chunk_max_waypoints)
        self._last_robot_pose: Optional[Tuple[float, float]] = None
        self._mission_note = ""
        self._current_start_index = 0
        self._current_target_index = 0
        self._awaiting_chunk_result = False
        self._last_nav_goal_active = False
        self._last_nav_result_status = int(GoalStatus.STATUS_UNKNOWN)
        self._last_nav_result_event_id = 0
        self._last_handled_nav_result_event_id = 0
        self._blocked_state = BLOCKED_STATE_NONE
        self._blocked_reason_code = ""
        self._blocked_reason_text = ""
        self._blocked_retry_attempt = 0
        self._blocked_wait_until: Optional[float] = None
        self._blocked_retry_inflight = False
        self._last_blocking_nav_event_code = ""
        self._last_blocking_nav_event_text = ""
        self._last_collision_stop_started: Optional[float] = None
        self._last_collision_stop_handled = False
        self._action_active = False
        self._action_waypoint_index = 0
        self._action_type = ""
        self._action_until: Optional[float] = None
        # Ultima tolerancia que este nodo aplico al goal checker de Nav2.
        # ``None`` obliga a fijar el valor estricto al iniciar el primer CAMPO.
        self._coverage_goal_tolerance_applied_m: Optional[float] = None
        self._battery_pct: Optional[float] = None
        self._battery_guard_seen = False
        self._low_battery_active = False
        self._return_home_requested = False
        self._return_home_active = False
        self._return_home_exit_route_index = -1
        self._return_home_exit_input_index = -1
        # The launch configuration is urban by default. This value tracks only
        # successful profile changes made while this executor is alive.
        self._active_navigation_profile = NAVIGATION_PROFILE_URBAN
        self._patrol_mission_profile: Optional[PatrolMissionProfile] = None
        self._patrol_mission_id = ""
        self._patrol_phase = PATROL_PHASE_IDLE
        self._patrol_active = False
        self._patrol_return_exit_loop_index = -1
        self._event_seq = 0

        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        # Las zonas se refrescan de fondo y se guardan aca. El planificado NO
        # puede pedirlas en el momento: `_on_generate_coverage_plan` corre en un
        # callback de servicio y el executor tiene dos hilos, asi que bloquear
        # esperando la respuesta compite con el resto del trafico del nodo y se
        # va a timeout justo cuando el cockpit esta conectado (que es cuando hay
        # trafico). Con el cache la planificacion no espera a nadie.
        self._zones_geojson = ""
        self._zones_updated_at = 0.0
        self._zones_note = "zones state not received yet"
        self._zones_request_pending = False
        self._zones_get_state_client = (
            self.create_client(
                GetZonesState,
                self.zones_get_state_service,
                callback_group=self._client_group,
            )
            if self.coverage_nogo_enabled and self.zones_get_state_service
            else None
        )
        # Solo con el planner agricola: un perfil legacy no levanta un nodo ni
        # un hilo de mas. Con fields2cover se crea en el arranque y no al primer
        # pedido, porque al crearse deja activo el Coverage Server —que viene
        # del lanzamiento sin configurar— y eso tarda mas que el timeout del
        # cockpit. Hacerlo aca es la diferencia entre que el primer preview de
        # la sesion ande o falle.
        self._fields2cover: Optional[Any] = None
        # Se agenda, no se hace: el arranque del nodo no puede depender de que
        # un servidor de cobertura conteste, y asi el planificador agricola
        # sigue apareciendo solo dentro de metodos de Campo.
        self._coverage_warmup_timer = (
            self.create_timer(
                0.5,
                self._coverage_warmup,
                callback_group=self._client_group,
            )
            if self.coverage_planner == "fields2cover"
            else None
        )
        self._zones_refresh_timer = (
            self.create_timer(
                self.zones_refresh_period_s,
                self._refresh_no_go_zones,
                callback_group=self._client_group,
            )
            if self._zones_get_state_client is not None
            else None
        )
        self._nav_set_goal_client = self.create_client(
            SetNavGoalLL,
            self.nav_set_goal_service,
            callback_group=self._client_group,
        )
        self._nav_cancel_goal_client = self.create_client(
            CancelNavGoal,
            self.nav_cancel_goal_service,
            callback_group=self._client_group,
        )
        self._nav_brake_client = self.create_client(
            BrakeNav,
            self.nav_brake_service,
            callback_group=self._client_group,
        )
        self._clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.clear_local_costmap_service,
            callback_group=self._client_group,
        )
        self._clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.clear_global_costmap_service,
            callback_group=self._client_group,
        )
        self._local_costmap_parameters = self.create_client(
            SetParameters,
            f"{self.local_costmap_node.rstrip('/')}/set_parameters",
            callback_group=self._client_group,
        )
        self._global_costmap_parameters = self.create_client(
            SetParameters,
            f"{self.global_costmap_node.rstrip('/')}/set_parameters",
            callback_group=self._client_group,
        )
        self._controller_server_parameters = self.create_client(
            SetParameters,
            f"{self.controller_server_node.rstrip('/')}/set_parameters",
            callback_group=self._client_group,
        )
        self._scan_ground_filter_parameters = self.create_client(
            SetParameters,
            f"{self.scan_ground_filter_node.rstrip('/')}/set_parameters",
            callback_group=self._client_group,
        )
        self._fromll_client = self.create_client(
            FromLL, self.fromll_service, callback_group=self._client_group
        )
        self._fromll_fallback_client = None
        if self.fromll_service_fallback and (
            self.fromll_service_fallback != self.fromll_service
        ):
            self._fromll_fallback_client = self.create_client(
                FromLL,
                self.fromll_service_fallback,
                callback_group=self._client_group,
            )
        self._active_fromll_name: Optional[str] = None
        self._active_fromll_client: Optional[Any] = None
        self._last_fromll_error: Optional[str] = None
        self._fromll_unavailable_until = 0.0
        self._tf_unavailable_until = 0.0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._mission_path_pub = self.create_publisher(Path, self.mission_path_topic, 10)
        self._active_chunk_path_pub = self.create_publisher(
            Path, self.active_chunk_path_topic, 10
        )
        self._nav_event_pub = self.create_publisher(NavEvent, self.nav_event_topic, 10)

        self._set_route_srv = self.create_service(
            SetRouteMissionLL,
            self.set_route_service,
            self._on_set_route,
            callback_group=self._service_group,
        )
        self._generate_coverage_plan_srv = self.create_service(
            GenerateCoveragePlanLL,
            self.generate_coverage_plan_service,
            self._on_generate_coverage_plan,
            callback_group=self._service_group,
        )
        self._cancel_route_srv = self.create_service(
            CancelRouteMission,
            self.cancel_route_service,
            self._on_cancel_route,
            callback_group=self._service_group,
        )
        self._get_state_srv = self.create_service(
            GetRouteMissionState,
            self.get_state_service,
            self._on_get_state,
            callback_group=self._service_group,
        )
        self._set_patrol_srv = self.create_service(
            SetPatrolMissionLL,
            self.set_patrol_service,
            self._on_set_patrol,
            callback_group=self._service_group,
        )
        self._cancel_patrol_srv = self.create_service(
            CancelPatrolMission,
            self.cancel_patrol_service,
            self._on_cancel_patrol,
            callback_group=self._service_group,
        )
        self._get_patrol_state_srv = self.create_service(
            GetPatrolMissionState,
            self.get_patrol_state_service,
            self._on_get_patrol_state,
            callback_group=self._service_group,
        )
        self._request_return_home_srv = self.create_service(
            RequestReturnHome,
            self.request_return_home_service,
            self._on_request_return_home,
            callback_group=self._service_group,
        )
        self._set_navigation_profile_srv = self.create_service(
            SetNavigationProfile,
            self.set_navigation_profile_service,
            self._on_set_navigation_profile,
            callback_group=self._service_group,
        )
        self._nav_telemetry_sub = self.create_subscription(
            NavTelemetry,
            self.nav_telemetry_topic,
            self._on_nav_telemetry,
            10,
            callback_group=self._client_group,
        )
        self._nav_event_sub = self.create_subscription(
            NavEvent,
            self.nav_event_topic,
            self._on_nav_event,
            10,
            callback_group=self._client_group,
        )
        self._battery_state_sub = self.create_subscription(
            BatteryState,
            self.battery_state_topic,
            self._on_battery_state,
            10,
            callback_group=self._client_group,
        )
        self._battery_guard_sub = self.create_subscription(
            BatteryMissionGuard,
            self.battery_guard_topic,
            self._on_battery_mission_guard,
            10,
            callback_group=self._client_group,
        )
        self._blocked_retry_timer = self.create_timer(
            1.0 / float(self.blocked_retry_tick_hz),
            self._blocked_retry_tick,
            callback_group=self._client_group,
        )

        self.get_logger().info(
            "Route executor ready "
            f"(set_route={self.set_route_service}, cancel={self.cancel_route_service}, "
            f"get_state={self.get_state_service}, nav_goal={self.nav_set_goal_service}, "
            f"mission_path={self.mission_path_topic}, "
            f"active_chunk_path={self.active_chunk_path_topic}, "
            f"blocked_retry_max_attempts={self.blocked_retry_max_attempts}, "
            f"blocked_retry_wait_s={self.blocked_retry_wait_s:.1f})"
        )

    @staticmethod
    def _wait_for_future(future: Any, timeout_s: float) -> Optional[Any]:
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                return future.result()
            if (time.monotonic() - start) >= timeout_s:
                return None
            time.sleep(0.01)
        return None

    def _call_service(self, client: Any, request: Any, timeout_s: float) -> Optional[Any]:
        if not client.wait_for_service(timeout_sec=min(timeout_s, 2.0)):
            self.get_logger().warning(f"Service unavailable: {getattr(client, 'srv_name', '<unknown>')}")
            return None
        future = client.call_async(request)
        return self._wait_for_future(future, timeout_s)

    def _set_costmap_inflation_parameters(
        self,
        client: Any,
        *,
        radius: float,
        scaling: float,
        label: str,
    ) -> Tuple[bool, str]:
        if radius <= 0.0 or scaling <= 0.0:
            return False, f"invalid {label} inflation values"
        if not client.wait_for_service(timeout_sec=min(self.navigation_profile_timeout_s, 2.0)):
            return False, f"{label} costmap parameter service unavailable"
        try:
            request = SetParameters.Request()
            request.parameters = [
                parameter.to_parameter_msg()
                for parameter in (
                    Parameter(
                        name="inflation_layer.inflation_radius",
                        value=float(radius),
                    ),
                    Parameter(
                        name="inflation_layer.cost_scaling_factor",
                        value=float(scaling),
                    ),
                )
            ]
            response = self._wait_for_future(
                client.call_async(request), self.navigation_profile_timeout_s
            )
        except Exception as exc:
            return False, f"{label} costmap parameter update failed: {exc}"
        if response is None:
            return False, f"{label} costmap parameter update timed out"
        results = list(response.results)
        failures = [
            str(getattr(result, "reason", "parameter rejected"))
            for result in results
            if not bool(getattr(result, "successful", False))
        ]
        if failures:
            return False, f"{label} costmap rejected profile: {'; '.join(failures)}"
        return True, ""

    def _set_coverage_goal_tolerance(self, tolerance_m: float) -> Tuple[bool, str]:
        """Cambiar la tolerancia real de Nav2 solo al cruzar fila/cabecera."""
        desired_m = max(0.05, float(tolerance_m))
        applied_m = getattr(self, "_coverage_goal_tolerance_applied_m", None)
        if applied_m is not None and math.isclose(
            float(applied_m), desired_m, rel_tol=0.0, abs_tol=1.0e-9
        ):
            return True, ""

        client = self._controller_server_parameters
        if not client.wait_for_service(timeout_sec=min(self.request_timeout_s, 2.0)):
            return False, "controller_server parameter service unavailable"
        try:
            request = SetParameters.Request()
            request.parameters = [
                Parameter(
                    name="general_goal_checker.xy_goal_tolerance",
                    value=desired_m,
                ).to_parameter_msg(),
            ]
            response = self._wait_for_future(
                client.call_async(request), self.request_timeout_s
            )
        except Exception as exc:
            return False, f"goal checker tolerance update failed: {exc}"
        if response is None:
            return False, "goal checker tolerance update timed out"
        failures = [
            str(getattr(result, "reason", "parameter rejected"))
            for result in list(response.results)
            if not bool(getattr(result, "successful", False))
        ]
        if failures:
            return False, f"controller_server rejected goal tolerance: {'; '.join(failures)}"
        self._coverage_goal_tolerance_applied_m = desired_m
        self.get_logger().info(
            "Coverage goal tolerance applied "
            f"(general_goal_checker.xy_goal_tolerance={desired_m:.2f}m)"
        )
        return True, ""

    def _set_scan_ground_filter_parameters(
        self,
        *,
        global_slope: float,
        local_slope: float,
        split_height: float,
        label: str,
    ) -> Tuple[bool, str]:
        if not (0.0 < global_slope <= 35.0 and 0.0 < local_slope <= 35.0):
            return False, f"invalid {label} ground slope values"
        if not (0.0 < split_height <= 0.50):
            return False, f"invalid {label} ground split height"
        client = self._scan_ground_filter_parameters
        if not client.wait_for_service(timeout_sec=min(self.navigation_profile_timeout_s, 2.0)):
            return False, "scan ground filter parameter service unavailable"
        try:
            request = SetParameters.Request()
            request.parameters = [
                parameter.to_parameter_msg()
                for parameter in (
                    Parameter(
                        name="global_slope_max_angle_deg",
                        value=float(global_slope),
                    ),
                    Parameter(
                        name="local_slope_max_angle_deg",
                        value=float(local_slope),
                    ),
                    Parameter(
                        name="split_height_distance",
                        value=float(split_height),
                    ),
                )
            ]
            response = self._wait_for_future(
                client.call_async(request), self.navigation_profile_timeout_s
            )
        except Exception as exc:
            return False, f"scan ground filter parameter update failed: {exc}"
        if response is None:
            return False, "scan ground filter parameter update timed out"
        failures = [
            str(getattr(result, "reason", "parameter rejected"))
            for result in list(response.results)
            if not bool(getattr(result, "successful", False))
        ]
        if failures:
            return False, f"scan ground filter rejected profile: {'; '.join(failures)}"
        return True, ""

    def _clear_costmaps(self) -> Tuple[bool, str]:
        errors: List[str] = []
        for label, client in (
            ("local", self._clear_local_costmap_client),
            ("global", self._clear_global_costmap_client),
        ):
            try:
                res = self._call_service(
                    client,
                    ClearEntireCostmap.Request(),
                    min(self.request_timeout_s, self.navigation_profile_timeout_s),
                )
            except Exception as exc:
                errors.append(f"{label}: {exc}")
                continue
            if res is None:
                errors.append(f"{label} clear timeout")
        return (not errors), "; ".join(errors)

    def _apply_navigation_profile(self, profile: str) -> Tuple[bool, str]:
        target = str(profile or "").strip().lower()
        if target not in NAVIGATION_PROFILES:
            return False, f"unknown navigation profile: {profile}"
        with self._lock:
            current = str(self._active_navigation_profile)
        if target == current:
            return True, ""

        values = self._navigation_profiles[target]
        previous_values = self._navigation_profiles.get(
            current,
            self._navigation_profiles[NAVIGATION_PROFILE_URBAN],
        )
        ground_ok, ground_error = self._set_scan_ground_filter_parameters(
            global_slope=float(values["ground_global_slope"]),
            local_slope=float(values["ground_local_slope"]),
            split_height=float(values["ground_split_height"]),
            label="target",
        )
        if not ground_ok:
            return False, ground_error
        local_ok, local_error = self._set_costmap_inflation_parameters(
            self._local_costmap_parameters,
            radius=float(values["local_radius"]),
            scaling=float(values["local_scaling"]),
            label="local",
        )
        if not local_ok:
            rollback_ok, rollback_error = self._set_scan_ground_filter_parameters(
                global_slope=float(previous_values["ground_global_slope"]),
                local_slope=float(previous_values["ground_local_slope"]),
                split_height=float(previous_values["ground_split_height"]),
                label="rollback",
            )
            details = [local_error]
            if not rollback_ok:
                details.append(rollback_error)
            return False, "; ".join(part for part in details if part)
        global_ok, global_error = self._set_costmap_inflation_parameters(
            self._global_costmap_parameters,
            radius=float(values["global_radius"]),
            scaling=float(values["global_scaling"]),
            label="global",
        )
        if not global_ok:
            rollback_ok, rollback_error = self._set_costmap_inflation_parameters(
                self._local_costmap_parameters,
                radius=float(previous_values["local_radius"]),
                scaling=float(previous_values["local_scaling"]),
                label="local rollback",
            )
            ground_rollback_ok, ground_rollback_error = self._set_scan_ground_filter_parameters(
                global_slope=float(previous_values["ground_global_slope"]),
                local_slope=float(previous_values["ground_local_slope"]),
                split_height=float(previous_values["ground_split_height"]),
                label="rollback",
            )
            details = [global_error]
            if not rollback_ok:
                details.append(rollback_error)
            if not ground_rollback_ok:
                details.append(ground_rollback_error)
            return False, "; ".join(part for part in details if part)

        clear_ok, clear_error = self._clear_costmaps()
        with self._lock:
            self._active_navigation_profile = target
        if not clear_ok:
            self.get_logger().warning(
                f"Navigation profile {target} applied but costmap clear failed: {clear_error}"
            )
        return True, ""

    def _restore_urban_navigation_profile(self) -> None:
        with self._lock:
            current = str(getattr(self, "_active_navigation_profile", NAVIGATION_PROFILE_URBAN))
        if current == NAVIGATION_PROFILE_URBAN:
            return
        ok, error = self._apply_navigation_profile(NAVIGATION_PROFILE_URBAN)
        if ok:
            self._publish_route_event(
                DiagnosticStatus.OK,
                "NAVIGATION_PROFILE_RESTORED",
                "Navigation profile restored to urban",
                details={"profile": NAVIGATION_PROFILE_URBAN},
            )
        else:
            self.get_logger().error(f"Failed restoring urban navigation profile: {error}")

    @staticmethod
    def _diag_level_value(value: Any) -> int:
        if isinstance(value, (bytes, bytearray)):
            return int.from_bytes(value, byteorder="little", signed=False)
        return int(value)

    @staticmethod
    def _details_to_key_values(details: Optional[Dict[str, Any]]) -> List[KeyValue]:
        if not details:
            return []
        values: List[KeyValue] = []
        for key, value in details.items():
            item = KeyValue()
            item.key = str(key)
            item.value = str(value)
            values.append(item)
        return values

    @staticmethod
    def _nav_event_details_to_dict(msg: NavEvent) -> Dict[str, str]:
        details: Dict[str, str] = {}
        for item in getattr(msg, "details", []) or []:
            key = str(getattr(item, "key", "") or "")
            if key:
                details[key] = str(getattr(item, "value", "") or "")
        return details

    @staticmethod
    def _blocking_reason_text(code: str, fallback: str = "") -> str:
        normalized = str(code or "")
        return str(fallback or BLOCKING_REASON_TEXT.get(normalized) or normalized)

    @staticmethod
    def _is_blocking_reason(code: str) -> bool:
        return str(code or "") in BLOCKING_FAILURE_CODES

    def _publish_route_event(
        self,
        severity: int,
        code: str,
        message: str,
        *,
        details: Optional[Dict[str, Any]] = None,
    ) -> int:
        with self._lock:
            self._event_seq += 1
            event_id = int(self._event_seq)
            context = {
                "mission_id": str(self._mission_id),
                "chunk_id": int(self._chunk_id),
                "loop_iteration": int(self._loop_iteration),
                "current_start_index": int(self._current_start_index),
                "current_target_index": int(self._current_target_index),
            }
        event_details = dict(context)
        event_details.update(dict(details or {}))
        event = NavEvent()
        event.stamp = self.get_clock().now().to_msg()
        event.severity = self._diag_level_value(severity)
        event.component = "route_executor"
        event.code = str(code)
        event.message = str(message)
        event.event_id = event_id
        event.details = self._details_to_key_values(event_details)
        self._nav_event_pub.publish(event)

        if event.severity >= self._diag_level_value(DiagnosticStatus.ERROR):
            self.get_logger().error(f"{message} code={code} details={event_details}")
        elif event.severity >= self._diag_level_value(DiagnosticStatus.WARN):
            self.get_logger().warning(f"{message} code={code} details={event_details}")
        else:
            self.get_logger().info(f"{message} code={code} details={event_details}")
        return event_id

    def _clear_blocked_state_locked(self) -> bool:
        was_blocked = bool(self._blocked_state)
        self._blocked_state = BLOCKED_STATE_NONE
        self._blocked_reason_code = ""
        self._blocked_reason_text = ""
        self._blocked_retry_attempt = 0
        self._blocked_wait_until = None
        self._blocked_retry_inflight = False
        self._last_blocking_nav_event_code = ""
        self._last_blocking_nav_event_text = ""
        self._last_collision_stop_started = None
        self._last_collision_stop_handled = False
        return was_blocked

    def _blocked_wait_remaining_s_locked(self) -> float:
        if self._blocked_state != BLOCKED_STATE_WAITING or self._blocked_wait_until is None:
            return 0.0
        return max(0.0, float(self._blocked_wait_until) - time.monotonic())

    def _blocked_status_locked(self) -> str:
        if not self._blocked_state:
            return self._mission_status
        remaining_s = self._blocked_wait_remaining_s_locked()
        if self._blocked_state == BLOCKED_STATE_WAITING:
            return (
                "route blocked: waiting "
                f"({self._blocked_reason_code}, retry "
                f"{self._blocked_retry_attempt + 1}/{self.blocked_retry_max_attempts}, "
                f"{remaining_s:.1f}s)"
            )
        if self._blocked_state == BLOCKED_STATE_RETRYING:
            return (
                "route blocked: retrying "
                f"({self._blocked_reason_code}, attempt "
                f"{self._blocked_retry_attempt}/{self.blocked_retry_max_attempts})"
            )
        if self._blocked_state == BLOCKED_STATE_NEEDS_OPERATOR:
            return f"route blocked: needs operator ({self._blocked_reason_code})"
        return self._mission_status

    def _apply_brake(self, *, duration_s: float = 0.0, brake_pct: int = 100) -> None:
        request = BrakeNav.Request()
        if hasattr(request, "duration_s"):
            request.duration_s = float(duration_s)
        if hasattr(request, "brake_pct"):
            request.brake_pct = int(max(0, min(100, int(brake_pct))))
        res = self._call_service(
            self._nav_brake_client,
            request,
            min(self.request_timeout_s, 3.0),
        )
        if res is None:
            self.get_logger().warning("Brake service timeout while handling blocked route")
        elif not bool(getattr(res, "ok", False)):
            self.get_logger().warning(
                f"Brake service failed while handling blocked route: {getattr(res, 'error', '')}"
            )

    def _clear_costmaps_for_retry(self) -> None:
        if not self.clear_costmaps_before_blocked_retry:
            return
        for label, client in (
            ("local", self._clear_local_costmap_client),
            ("global", self._clear_global_costmap_client),
        ):
            res = self._call_service(
                client,
                ClearEntireCostmap.Request(),
                min(self.request_timeout_s, 3.0),
            )
            if res is None:
                self.get_logger().warning(f"{label} costmap clear timeout before blocked retry")

    def _enter_blocked_waiting(
        self,
        reason_code: str,
        reason_text: str = "",
        *,
        cancel_goal: bool = False,
        brake: bool = True,
    ) -> None:
        if brake:
            self._apply_brake()
        if cancel_goal:
            self._cancel_nav_goal()

        normalized_reason = str(reason_code or "")
        normalized_text = self._blocking_reason_text(normalized_reason, reason_text)
        needs_operator = False
        with self._lock:
            if not self._mission_active:
                return
            if self._blocked_state == BLOCKED_STATE_NEEDS_OPERATOR:
                return
            if self._blocked_retry_attempt >= self.blocked_retry_max_attempts:
                self._blocked_state = BLOCKED_STATE_NEEDS_OPERATOR
                self._blocked_reason_code = normalized_reason
                self._blocked_reason_text = normalized_text
                self._blocked_wait_until = None
                self._awaiting_chunk_result = False
                self._mission_status = self._blocked_status_locked()
                needs_operator = True
            else:
                self._blocked_state = BLOCKED_STATE_WAITING
                self._blocked_reason_code = normalized_reason
                self._blocked_reason_text = normalized_text
                self._blocked_wait_until = time.monotonic() + float(self.blocked_retry_wait_s)
                self._blocked_retry_inflight = False
                self._awaiting_chunk_result = False
                self._mission_status = self._blocked_status_locked()

        if needs_operator:
            self._publish_route_event(
                DiagnosticStatus.ERROR,
                "ROUTE_BLOCKED_NEEDS_OPERATOR",
                "Route blocked and needs operator intervention",
                details={
                    "blocked_reason_code": normalized_reason,
                    "blocked_reason_text": normalized_text,
                    "retry_attempts": self.blocked_retry_max_attempts,
                },
            )
        else:
            self._publish_route_event(
                DiagnosticStatus.WARN,
                "ROUTE_BLOCKED_WAITING",
                "Route blocked; waiting before retry",
                details={
                    "blocked_reason_code": normalized_reason,
                    "blocked_reason_text": normalized_text,
                    "retry_attempt": self._blocked_retry_attempt + 1,
                    "retry_max_attempts": self.blocked_retry_max_attempts,
                    "wait_s": self.blocked_retry_wait_s,
                },
            )

    def _blocked_retry_tick(self) -> None:
        should_retry = False
        with self._lock:
            if (
                self._mission_active
                and self._blocked_state == BLOCKED_STATE_WAITING
                and not self._blocked_retry_inflight
                and self._blocked_wait_until is not None
                and time.monotonic() >= float(self._blocked_wait_until)
            ):
                self._blocked_state = BLOCKED_STATE_RETRYING
                self._blocked_retry_inflight = True
                self._blocked_retry_attempt += 1
                self._mission_status = self._blocked_status_locked()
                should_retry = True

        if not should_retry:
            return

        thread = threading.Thread(
            target=self._run_blocked_retry,
            daemon=True,
            name="route_blocked_retry",
        )
        thread.start()

    def _run_blocked_retry(self) -> None:
        with self._lock:
            requested_start_index = int(self._current_start_index)
            requested_target_index = int(self._current_target_index)
            route = list(self._route_expanded)
            loop_enabled = bool(self._mission_loop)
            robot_pose = self._last_robot_pose
            reason_code = str(self._blocked_reason_code)
            reason_text = str(self._blocked_reason_text)
            retry_attempt = int(self._blocked_retry_attempt)

        resolution = resolve_blocked_retry_start(
            route,
            current_start_index=requested_start_index,
            current_target_index=requested_target_index,
            loop=loop_enabled,
            robot_lat=None if robot_pose is None else float(robot_pose[0]),
            robot_lon=None if robot_pose is None else float(robot_pose[1]),
            waypoint_reached_tolerance_m=self.route_waypoint_reached_tolerance_m,
            segment_start_tolerance_m=self.blocked_retry_reanchor_tolerance_m,
            reanchor_enabled=self.blocked_retry_reanchor_on_current_pose,
        )
        start_index = int(resolution.resolved_start_index)

        self._publish_route_event(
            DiagnosticStatus.WARN,
            "ROUTE_BLOCKED_RETRYING",
            "Retrying blocked route chunk",
            details={
                "blocked_reason_code": reason_code,
                "blocked_reason_text": reason_text,
                "retry_attempt": retry_attempt,
                "retry_max_attempts": self.blocked_retry_max_attempts,
                "requested_start_index": requested_start_index,
                "requested_target_index": requested_target_index,
                "resolved_start_index": start_index,
                "reanchored": bool(resolution.reanchored),
                "reanchor_reason": resolution.reason,
                "reanchor_match_distance_m": f"{resolution.match_distance_m:.3f}",
            },
        )
        # Un timeout de confirmacion entre actions internos no describe un mapa
        # obstruido. Limpiar ambos costmaps agrega carga justo cuando Nav2 viene
        # de perder el deadline; se reenvia el mismo tramo sin esa operacion.
        if reason_code != "ACTION_SERVER_ACK_TIMEOUT":
            self._clear_costmaps_for_retry()
        ok, err = self._send_chunk(start_index=start_index)

        with self._lock:
            if ok:
                self._blocked_state = BLOCKED_STATE_NONE
                self._blocked_reason_code = ""
                self._blocked_reason_text = ""
                self._blocked_wait_until = None
                self._blocked_retry_inflight = False
                self._last_collision_stop_started = None
                self._last_collision_stop_handled = False
                self._mission_status = self._status_with_note_locked(
                    f"route active ({self._current_start_index + 1}->{self._current_target_index + 1})"
                )
            else:
                self._blocked_retry_inflight = False

        if ok:
            self._publish_route_event(
                DiagnosticStatus.OK,
                "ROUTE_BLOCKED_CLEARED",
                "Blocked route retry accepted",
                details={
                    "retry_attempt": retry_attempt,
                    "requested_start_index": requested_start_index,
                    "resolved_start_index": start_index,
                    "reanchored": bool(resolution.reanchored),
                },
            )
            return

        self.get_logger().warning(f"Blocked route retry failed to dispatch: {err}")
        self._enter_blocked_waiting(reason_code, reason_text, brake=True)

    def _resolve_fromll_client(self) -> Optional[Any]:
        if time.monotonic() < self._fromll_unavailable_until:
            self._last_fromll_error = "fromLL service unavailable"
            return None

        candidates: List[Tuple[Any, str, float]] = []
        if self._active_fromll_client is not None and self._active_fromll_name is not None:
            candidates.append((self._active_fromll_client, self._active_fromll_name, 0.01))

        candidates.append((self._fromll_client, self.fromll_service, self.fromll_wait_timeout_s))
        if self._fromll_fallback_client is not None:
            candidates.append(
                (
                    self._fromll_fallback_client,
                    self.fromll_service_fallback,
                    self.fromll_wait_timeout_s,
                )
            )

        seen = set()
        for client, service_name, wait_s in candidates:
            key = (id(client), service_name)
            if key in seen:
                continue
            seen.add(key)
            if client.wait_for_service(timeout_sec=wait_s):
                if self._active_fromll_name != service_name:
                    self.get_logger().info(
                        f"Using fromLL service for route debug paths: {service_name}"
                    )
                self._active_fromll_client = client
                self._active_fromll_name = service_name
                self._last_fromll_error = None
                self._fromll_unavailable_until = 0.0
                return client

        self._last_fromll_error = "fromLL service unavailable"
        self._fromll_unavailable_until = time.monotonic() + 1.0
        return None

    def _call_from_ll(self, lat: float, lon: float) -> Optional[Tuple[float, float, float]]:
        fromll_client = self._resolve_fromll_client()
        if fromll_client is None:
            return None

        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=float(lat), longitude=float(lon), altitude=0.0)
        future = fromll_client.call_async(req)
        try:
            res = self._wait_for_future(future, self.fromll_call_timeout_s)
        except Exception as exc:
            self._last_fromll_error = str(exc)
            self._active_fromll_client = None
            self._active_fromll_name = None
            self._fromll_unavailable_until = time.monotonic() + 1.0
            return None
        if res is None:
            self._last_fromll_error = "timeout waiting fromLL response"
            self._active_fromll_client = None
            self._active_fromll_name = None
            self._fromll_unavailable_until = time.monotonic() + 1.0
            return None
        self._last_fromll_error = None
        return (
            float(res.map_point.x),
            float(res.map_point.y),
            float(res.map_point.z),
        )

    @staticmethod
    def _north_east_m_to_ll(
        lat: float,
        lon: float,
        north_m: float,
        east_m: float,
    ) -> Tuple[float, float]:
        meters_per_deg_lat = 111_320.0
        cos_lat = max(1.0e-6, abs(math.cos(math.radians(float(lat)))))
        meters_per_deg_lon = meters_per_deg_lat * cos_lat
        out_lat = float(lat) + float(north_m) / meters_per_deg_lat
        out_lon = float(lon) + float(east_m) / meters_per_deg_lon
        out_lon = ((out_lon + 180.0) % 360.0) - 180.0
        return out_lat, out_lon

    def _project_geographic_yaw_to_fromll(
        self,
        lat: float,
        lon: float,
        yaw_deg: float,
        origin_xyz: Tuple[float, float, float],
        projection_distance_m: float = 1.0,
    ) -> float:
        heading_rad = math.radians(float(yaw_deg))
        north_m = float(projection_distance_m) * math.sin(heading_rad)
        east_m = float(projection_distance_m) * math.cos(heading_rad)
        tip_lat, tip_lon = self._north_east_m_to_ll(lat, lon, north_m, east_m)
        tip_xyz = self._call_from_ll(tip_lat, tip_lon)
        if tip_xyz is None:
            return _normalize_yaw_deg(yaw_deg)

        dx = float(tip_xyz[0]) - float(origin_xyz[0])
        dy = float(tip_xyz[1]) - float(origin_xyz[1])
        if math.hypot(dx, dy) <= 1.0e-6:
            return _normalize_yaw_deg(yaw_deg)
        return _normalize_yaw_deg(math.degrees(math.atan2(dy, dx)))

    def _transform_pose_to_path_frame(self, pose: PoseStamped) -> Optional[PoseStamped]:
        if pose.header.frame_id == self.path_frame:
            pose.header.stamp = self.get_clock().now().to_msg()
            return pose
        if time.monotonic() < self._tf_unavailable_until:
            self._last_fromll_error = (
                f"tf transform unavailable ({pose.header.frame_id}->{self.path_frame})"
            )
            return None

        try:
            transformed = self._tf_buffer.transform(
                pose,
                self.path_frame,
                timeout=Duration(seconds=self.tf_lookup_timeout_s),
            )
        except TransformException as exc:
            self._last_fromll_error = (
                f"tf transform failed ({pose.header.frame_id}->{self.path_frame}): {exc}"
            )
            self._tf_unavailable_until = time.monotonic() + 1.0
            return None

        self._tf_unavailable_until = 0.0
        transformed.header.stamp = self.get_clock().now().to_msg()
        return transformed

    def _build_debug_pose_from_ll(self, waypoint: RouteWaypoint) -> Optional[PoseStamped]:
        converted = self._call_from_ll(float(waypoint.lat), float(waypoint.lon))
        if converted is None:
            return None

        yaw_deg = self._project_geographic_yaw_to_fromll(
            float(waypoint.lat),
            float(waypoint.lon),
            float(waypoint.yaw_deg),
            converted,
        )

        pose = PoseStamped()
        pose.header.frame_id = self.fromll_frame
        pose.header.stamp = Time().to_msg()
        pose.pose.position.x = float(converted[0])
        pose.pose.position.y = float(converted[1])
        pose.pose.position.z = float(converted[2])
        pose.pose.orientation = _yaw_to_quaternion(yaw_deg)
        return self._transform_pose_to_path_frame(pose)

    def _build_debug_path_from_route(
        self,
        route: Sequence[RouteWaypoint],
        *,
        label: str,
    ) -> Path:
        stamp = self.get_clock().now().to_msg()
        poses: List[PoseStamped] = []
        warned = False
        for idx, waypoint in enumerate(route):
            pose = self._build_debug_pose_from_ll(waypoint)
            if pose is None:
                if not warned:
                    self.get_logger().warning(
                        f"Could not convert {label} debug path waypoint {idx + 1}; "
                        f"publishing partial path "
                        f"(reason={self._last_fromll_error or 'unknown'})"
                    )
                    warned = True
                continue
            poses.append(pose)
        return _poses_to_debug_path(poses, frame_id=self.path_frame, stamp=stamp)

    def _publish_route_debug_path(
        self,
        publisher: Any,
        route: Sequence[RouteWaypoint],
        *,
        label: str,
    ) -> None:
        publisher.publish(self._build_debug_path_from_route(route, label=label))

    def _publish_empty_path(self, publisher: Any) -> None:
        publisher.publish(
            _poses_to_debug_path(
                [],
                frame_id=self.path_frame,
                stamp=self.get_clock().now().to_msg(),
            )
        )

    def _publish_empty_route_paths(self) -> None:
        self._publish_empty_path(self._mission_path_pub)
        self._publish_empty_path(self._active_chunk_path_pub)

    def _publish_empty_active_chunk_path(self) -> None:
        self._publish_empty_path(self._active_chunk_path_pub)

    def _configured_home_waypoint(self) -> Optional[RouteWaypoint]:
        if not (
            np.isfinite(float(self.default_home_lat)) and np.isfinite(float(self.default_home_lon))
        ):
            return None
        return RouteWaypoint(
            lat=float(self.default_home_lat),
            lon=float(self.default_home_lon),
            yaw_deg=float(self.default_home_yaw_deg),
        )

    def _effective_home_waypoint_locked(self) -> Optional[RouteWaypoint]:
        if self._home_waypoint is not None:
            return self._home_waypoint
        return self._configured_home_waypoint()

    def _return_home_phase_locked(self) -> str:
        status_text = str(self._mission_status or "").lower()
        if self._patrol_phase == PATROL_PHASE_RETURN_CONNECTOR:
            return "active"
        if self._patrol_phase == PATROL_PHASE_RETURN_PENDING:
            return "waiting_exit"
        if self._patrol_phase == PATROL_PHASE_PARKED_HOME:
            return "completed"
        if self._return_home_active:
            return "active"
        if self._return_home_requested:
            if self._return_home_exit_input_index >= 0:
                return "waiting_exit"
            return "requested"
        if "return home completed" in status_text:
            return "completed"
        if "return home unavailable" in status_text:
            return "unavailable"
        return "idle"

    def _patrol_return_reference_waypoint_locked(self) -> Optional[RouteWaypoint]:
        profile = self._patrol_mission_profile
        if profile is None:
            return self._effective_home_waypoint_locked()
        if profile.return_waypoints:
            return profile.return_waypoints[0]
        return profile.home_waypoint

    def _patrol_needs_depart_from_home_locked(self) -> bool:
        profile = self._patrol_mission_profile
        if profile is None:
            return False
        if self._last_robot_pose is None:
            return False
        return (
            _distance_m(
                float(self._last_robot_pose[0]),
                float(self._last_robot_pose[1]),
                float(profile.home_waypoint.lat),
                float(profile.home_waypoint.lon),
            )
            <= float(self.route_segment_start_tolerance_m)
        )

    def _set_patrol_state_locked(
        self,
        *,
        profile: Optional[PatrolMissionProfile],
        mission_id: str,
        phase: str,
        active: bool,
        return_exit_loop_index: int = -1,
    ) -> None:
        self._patrol_mission_profile = profile
        self._patrol_mission_id = str(mission_id)
        self._patrol_phase = str(phase)
        self._patrol_active = bool(active)
        self._patrol_return_exit_loop_index = int(return_exit_loop_index)

    def _clear_patrol_state_locked(self, phase: str = PATROL_PHASE_IDLE) -> None:
        self._set_patrol_state_locked(
            profile=None,
            mission_id="",
            phase=phase,
            active=False,
            return_exit_loop_index=-1,
        )

    def _prepare_route_with_source_indices(
        self,
        *,
        route_input: Sequence[RouteWaypoint],
        action_jsons: Sequence[str],
        loop_enabled: bool,
        source_indices: Optional[Sequence[int]] = None,
    ) -> Tuple[Optional[PreparedRouteMission], bool, str]:
        route_source_indices = list(source_indices or range(len(route_input)))
        route_input, action_jsons, dropped_loop_closure = drop_duplicate_loop_closure_with_actions(
            route_input,
            action_jsons,
            loop=loop_enabled,
            closure_tolerance_m=self.route_waypoint_reached_tolerance_m,
        )
        if len(route_source_indices) > len(route_input):
            route_source_indices = route_source_indices[: len(route_input)]
        with self._lock:
            robot_pose = self._last_robot_pose
        prepared, prepare_error = prepare_route_waypoints(
            route_input,
            loop=loop_enabled,
            robot_lat=None if robot_pose is None else float(robot_pose[0]),
            robot_lon=None if robot_pose is None else float(robot_pose[1]),
            waypoint_reached_tolerance_m=self.route_waypoint_reached_tolerance_m,
            segment_start_tolerance_m=self.route_segment_start_tolerance_m,
            action_jsons=action_jsons,
            waypoint_roles=[WAYPOINT_ROLE_NORMAL for _ in route_input],
        )
        if prepared is None:
            return None, dropped_loop_closure, prepare_error
        prepared = _map_prepared_input_indices(prepared, route_source_indices)
        return prepared, dropped_loop_closure, ""

    def _activate_prepared_route(
        self,
        *,
        prepared: PreparedRouteMission,
        loop_enabled: bool,
        leg_spacing_m: float,
        chunk_span_m: float,
        chunk_max_waypoints: int,
        home_waypoint: Optional[RouteMissionHome],
        mission_id: str,
        preserve_energy_state: bool = False,
    ) -> Tuple[bool, str, int, int]:
        mission_note = str(prepared.note)
        expanded, expanded_action_jsons, expanded_key_flags = expand_route_waypoints_with_actions(
            prepared.waypoints,
            prepared.action_jsons,
            leg_spacing_m=leg_spacing_m,
            loop=loop_enabled,
        )

        with self._lock:
            self._route_input = list(prepared.waypoints)
            self._route_input_source_indices = list(
                prepared.input_indices
                if prepared.input_indices is not None
                else range(len(prepared.waypoints))
            )
            self._route_expanded = list(expanded)
            self._route_action_jsons = list(expanded_action_jsons)
            self._route_waypoint_roles = [WAYPOINT_ROLE_NORMAL for _ in expanded]
            self._route_key_waypoint_flags = list(expanded_key_flags)
            self._route_input_indices = expanded_input_indices(
                expanded_key_flags,
                prepared.input_indices,
            )
            self._home_waypoint = (
                home_waypoint.waypoint
                if home_waypoint is not None
                else self._configured_home_waypoint()
            )
            self._home_input_index = int(home_waypoint.input_index) if home_waypoint is not None else -1
            self._active_chunk = []
            self._mission_id = str(mission_id)
            self._chunk_id = 0
            self._loop_iteration = 0
            self._reached_checkpoint_count = 0
            self._mission_active = True
            self._mission_paused = False
            self._mission_loop = bool(loop_enabled)
            self._mission_note = mission_note
            self._mission_status = self._status_with_note_locked("route starting")
            self._leg_spacing_m = float(leg_spacing_m)
            self._chunk_span_m = float(chunk_span_m)
            self._chunk_max_waypoints = int(chunk_max_waypoints)
            self._current_start_index = 0
            self._current_target_index = 0
            self._awaiting_chunk_result = False
            self._action_active = False
            self._action_waypoint_index = 0
            self._action_type = ""
            self._action_until = None
            if not preserve_energy_state:
                self._low_battery_active = False
                self._return_home_requested = False
                self._return_home_active = False
                self._return_home_exit_route_index = -1
                self._return_home_exit_input_index = -1
            self._last_handled_nav_result_event_id = self._last_nav_result_event_id
            self._clear_blocked_state_locked()

        ok, err = self._send_chunk(start_index=0)
        if ok:
            self._publish_route_debug_path(self._mission_path_pub, expanded, label="mission")
        return ok, err, int(len(prepared.waypoints)), int(len(expanded))

    def _start_patrol_phase(self, phase: str) -> Tuple[bool, str, int, int]:
        route_input, action_jsons, loop_enabled, source_indices, home_waypoint, note = (
            self._build_patrol_route_phase(phase=phase)
        )
        if not route_input:
            return False, note or "empty patrol route", 0, 0
        prepared, dropped_loop_closure, prepare_error = self._prepare_route_with_source_indices(
            route_input=route_input,
            action_jsons=action_jsons,
            loop_enabled=loop_enabled,
            source_indices=source_indices,
        )
        if prepared is None:
            return False, prepare_error, 0, 0
        if note:
            prepared = PreparedRouteMission(
                waypoints=list(prepared.waypoints),
                action_jsons=list(prepared.action_jsons),
                waypoint_roles=list(prepared.waypoint_roles),
                start_index=int(prepared.start_index),
                skipped_waypoints=int(prepared.skipped_waypoints),
                rotated=bool(prepared.rotated),
                note=f"{prepared.note}; {note}".strip("; ") if prepared.note else note,
                input_indices=list(prepared.input_indices) if prepared.input_indices is not None else None,
            )
        if dropped_loop_closure:
            prepared = PreparedRouteMission(
                waypoints=list(prepared.waypoints),
                action_jsons=list(prepared.action_jsons),
                waypoint_roles=list(prepared.waypoint_roles),
                start_index=int(prepared.start_index),
                skipped_waypoints=int(prepared.skipped_waypoints),
                rotated=bool(prepared.rotated),
                note=(
                    f"{prepared.note}; dropped duplicate loop closure"
                    if prepared.note
                    else "dropped duplicate loop closure"
                ),
                input_indices=list(prepared.input_indices) if prepared.input_indices is not None else None,
            )
        with self._lock:
            mission_id = self._patrol_mission_id or uuid.uuid4().hex[:12]
            profile = self._patrol_mission_profile
            if profile is None:
                return False, "patrol mission unavailable", 0, 0
            self._patrol_mission_id = mission_id
            self._patrol_phase = str(phase)
            self._patrol_active = phase != PATROL_PHASE_PARKED_HOME
            home_ref = RouteMissionHome(profile.home_waypoint, -1)
            if phase == PATROL_PHASE_RETURN_CONNECTOR:
                self._return_home_active = True
                self._return_home_requested = False
            elif phase == PATROL_PHASE_LOOP_MAIN:
                self._return_home_active = False
            self._home_waypoint = profile.home_waypoint
        return self._activate_prepared_route(
            prepared=prepared,
            loop_enabled=loop_enabled,
            leg_spacing_m=float(profile.leg_spacing_m),
            chunk_span_m=float(profile.chunk_span_m),
            chunk_max_waypoints=int(profile.chunk_max_waypoints),
            home_waypoint=home_ref,
            mission_id=mission_id,
            preserve_energy_state=(phase == PATROL_PHASE_RETURN_CONNECTOR),
        )

    def _build_patrol_route_phase(
        self,
        *,
        phase: str,
    ) -> Tuple[List[RouteWaypoint], List[str], bool, List[int], Optional[RouteMissionHome], str]:
        profile = self._patrol_mission_profile
        if profile is None:
            return [], [], False, [], None, "patrol mission unavailable"
        if phase == PATROL_PHASE_LOOP_MAIN:
            route = list(profile.loop_waypoints)
            actions = list(profile.loop_action_jsons)
            source_indices = list(range(len(route)))
            note = "patrol loop main"
            if self._patrol_needs_depart_from_home_locked():
                entry_index = (int(profile.depart_entry_loop_index) + 1) % len(route)
                route = _rotate_list(route, entry_index)
                actions = _rotate_list(actions, entry_index)
                source_indices = _rotate_list(source_indices, entry_index)
                note = f"patrol loop resumed after HOME via waypoint {entry_index + 1}"
            return (
                route,
                actions,
                True,
                source_indices,
                RouteMissionHome(profile.home_waypoint, -1),
                note,
            )
        if phase == PATROL_PHASE_DEPART_HOME:
            entry_index = int(profile.depart_entry_loop_index)
            if not (0 <= entry_index < len(profile.loop_waypoints)):
                return [], [], False, [], None, "invalid patrol depart entry loop index"
            route = list(profile.depart_waypoints) + [profile.loop_waypoints[entry_index]]
            actions = list(profile.depart_action_jsons) + [""]
            source_indices = list(range(len(route)))
            return route, actions, False, source_indices, RouteMissionHome(profile.home_waypoint, -1), "patrol depart from HOME"
        if phase == PATROL_PHASE_RETURN_CONNECTOR:
            route = list(profile.return_waypoints) + [profile.home_waypoint]
            actions = list(profile.return_action_jsons) + [""]
            source_indices = list(range(len(route)))
            return route, actions, False, source_indices, RouteMissionHome(profile.home_waypoint, -1), "patrol return connector"
        return [], [], False, [], None, f"unsupported patrol phase: {phase}"

    def _reset_mission_locked(self, status: str = "idle") -> None:
        self._route_input = []
        self._route_input_source_indices = []
        self._route_expanded = []
        self._route_action_jsons = []
        self._route_waypoint_roles = []
        self._route_key_waypoint_flags = []
        self._route_input_indices = []
        self._home_waypoint = None
        self._home_input_index = -1
        self._active_chunk = []
        self._mission_id = ""
        self._chunk_id = 0
        self._loop_iteration = 0
        self._reached_checkpoint_count = 0
        self._mission_active = False
        self._mission_paused = False
        self._mission_loop = False
        self._mission_follow_exact_path = False
        self._coverage_approach_pending = False
        self._coverage_exact_resume_index = -1
        self._mission_status = str(status)
        self._mission_note = ""
        self._current_start_index = 0
        self._current_target_index = 0
        self._awaiting_chunk_result = False
        self._action_active = False
        self._action_waypoint_index = 0
        self._action_type = ""
        self._action_until = None
        self._low_battery_active = False
        self._return_home_requested = False
        self._return_home_active = False
        self._return_home_exit_route_index = -1
        self._return_home_exit_input_index = -1
        self._clear_patrol_state_locked()
        self._last_handled_nav_result_event_id = self._last_nav_result_event_id
        self._clear_blocked_state_locked()

    def _action_remaining_s_locked(self) -> float:
        if not self._action_active or self._action_until is None:
            return 0.0
        return max(0.0, float(self._action_until) - time.monotonic())

    def _status_with_note_locked(self, status: str) -> str:
        note = str(self._mission_note).strip()
        if not note:
            return str(status)
        return f"{status} [{note}]"

    def _blocking_reason_from_text(self, text: str) -> Tuple[str, str]:
        normalized = str(text or "").lower()
        if (
            "no valid path" in normalized
            or "no se encontró una ruta válida" in normalized
            or "failed to create plan" in normalized
            or "failed to generate a valid path" in normalized
        ):
            return "NO_VALID_PATH", self._blocking_reason_text("NO_VALID_PATH", text)
        if (
            "timed out while waiting for action server to acknowledge goal request" in normalized
            or "no recibió a tiempo la confirmación de un action server interno" in normalized
        ):
            return (
                "ACTION_SERVER_ACK_TIMEOUT",
                self._blocking_reason_text("ACTION_SERVER_ACK_TIMEOUT", text),
            )
        if (
            "lookup would require extrapolation" in normalized
            or "extrapolation into the future" in normalized
            or "unable to transform robot pose into global plan's frame" in normalized
            or "exception in transformpose" in normalized
            or "desfase transitorio de tf" in normalized
        ):
            return (
                "TF_EXTRAPOLATION",
                self._blocking_reason_text("TF_EXTRAPOLATION", text),
            )
        if "smoothed path" in normalized and "collision" in normalized:
            return (
                "SMOOTHED_PATH_COLLISION",
                self._blocking_reason_text("SMOOTHED_PATH_COLLISION", text),
            )
        if "collision" in normalized or "colisión" in normalized:
            return "CONTROLLER_COLLISION", self._blocking_reason_text("CONTROLLER_COLLISION", text)
        if "backup failed" in normalized or "recovery" in normalized:
            return "RECOVERY_FAILED", self._blocking_reason_text("RECOVERY_FAILED", text)
        if "off grid" in normalized or "pose goes off grid" in normalized:
            return "RECOVERY_OFF_GRID", self._blocking_reason_text("RECOVERY_OFF_GRID", text)
        return "", ""

    def _blocking_reason_from_nav_telemetry_locked(
        self, msg: NavTelemetry
    ) -> Tuple[str, str]:
        if self._is_blocking_reason(self._last_blocking_nav_event_code):
            return self._last_blocking_nav_event_code, self._last_blocking_nav_event_text
        failure_code = str(getattr(msg, "failure_code", "") or "")
        if self._is_blocking_reason(failure_code):
            return failure_code, self._blocking_reason_text(failure_code, msg.nav_result_text)
        return self._blocking_reason_from_text(str(getattr(msg, "nav_result_text", "") or ""))

    def _on_nav_event(self, msg: NavEvent) -> None:
        component = str(getattr(msg, "component", "") or "")
        if component == "route_executor":
            return
        details = self._nav_event_details_to_dict(msg)
        event_code = str(getattr(msg, "code", "") or "")
        reason_code = str(details.get("failure_reason_code") or "")
        reason_text = str(details.get("failure_reason") or getattr(msg, "message", "") or "")
        if event_code == "COLLISION_STOP_ACTIVE":
            reason_code = "COLLISION_STOP_ACTIVE"
            reason_text = self._blocking_reason_text(reason_code, reason_text)
        if not self._is_blocking_reason(reason_code):
            return
        with self._lock:
            self._last_blocking_nav_event_code = reason_code
            self._last_blocking_nav_event_text = self._blocking_reason_text(
                reason_code,
                reason_text,
            )

    def _activate_low_battery_response(
        self,
        *,
        battery_pct: float,
        detected_event_code: str,
        detected_message: str,
        detected_details: Dict[str, str],
    ) -> None:
        should_request_home = False
        should_stop_non_loop = False
        should_stop_missing_home = False
        already_active = False
        home_available = False
        exit_selection: Optional[ReturnHomeExitSelection] = None
        with self._lock:
            if self._low_battery_active:
                return
            self._low_battery_active = True
            if not self._mission_active or self._mission_paused:
                return
            already_active = bool(self._return_home_requested or self._return_home_active)
            if already_active:
                return
            home_available = self._effective_home_waypoint_locked() is not None
            if self._mission_loop:
                if home_available:
                    reference_waypoint = (
                        self._patrol_return_reference_waypoint_locked()
                        if self._patrol_mission_profile is not None
                        else self._effective_home_waypoint_locked()
                    )
                    exit_selection = _select_return_home_exit_waypoint(
                        self._route_input,
                        self._route_input_source_indices,
                        reference_waypoint,
                    )
                    if exit_selection is not None:
                        self._return_home_requested = True
                        self._return_home_exit_route_index = int(exit_selection.route_index)
                        self._return_home_exit_input_index = int(exit_selection.input_index)
                        if self._patrol_mission_profile is not None:
                            self._patrol_phase = PATROL_PHASE_RETURN_PENDING
                            self._patrol_return_exit_loop_index = int(exit_selection.input_index)
                        should_request_home = True
                        self._mission_status = self._status_with_note_locked(
                            "return home waiting for exit waypoint"
                        )
                    else:
                        should_stop_missing_home = True
                else:
                    should_stop_missing_home = True
            else:
                should_stop_non_loop = True

        self._publish_route_event(
            DiagnosticStatus.WARN,
            detected_event_code,
            detected_message,
            details=dict(detected_details),
        )
        if should_request_home:
            self._publish_route_event(
                DiagnosticStatus.WARN,
                "RETURN_HOME_REQUESTED",
                "Return home requested by low battery",
                details={
                    "battery_pct": f"{battery_pct:.1f}",
                    "home_exit_input_index": int(exit_selection.input_index)
                    if exit_selection is not None
                    else -1,
                    "home_exit_lat": f"{exit_selection.waypoint.lat:.8f}"
                    if exit_selection is not None
                    else "",
                    "home_exit_lon": f"{exit_selection.waypoint.lon:.8f}"
                    if exit_selection is not None
                    else "",
                },
            )
            return
        if should_stop_non_loop:
            self._stop_for_low_battery_non_loop(battery_pct)
            return
        if should_stop_missing_home:
            self._stop_for_missing_home(battery_pct)

    def _on_battery_state(self, msg: BatteryState) -> None:
        percentage = float(getattr(msg, "percentage", float("nan")))
        if not np.isfinite(percentage):
            return
        if percentage <= 1.0:
            percentage *= 100.0
        percentage = max(0.0, min(100.0, percentage))
        with self._lock:
            self._battery_pct = float(percentage)
            if self._battery_guard_seen:
                return
        if percentage > float(self.low_battery_threshold_pct):
            return
        self._activate_low_battery_response(
            battery_pct=float(percentage),
            detected_event_code="LOW_BATTERY_DETECTED",
            detected_message="Low battery detected during mission",
            detected_details={
                "battery_pct": f"{percentage:.1f}",
                "threshold_pct": f"{self.low_battery_threshold_pct:.1f}",
                "loop": int(self._mission_loop),
            },
        )

    def _on_battery_mission_guard(self, msg: BatteryMissionGuard) -> None:
        operator_soc_pct = float(getattr(msg, "operator_soc_pct", float("nan")))
        if np.isfinite(operator_soc_pct):
            with self._lock:
                self._battery_pct = max(0.0, min(100.0, operator_soc_pct))
        with self._lock:
            self._battery_guard_seen = True

        state = str(getattr(msg, "state", "") or "")
        if not bool(getattr(msg, "ready", False)):
            return
        if not bool(getattr(msg, "fresh", False)):
            return
        if state in {"STALE", "UNAVAILABLE", "SUSPECT"}:
            return
        if not bool(getattr(msg, "return_home_recommended", False)):
            self._clear_low_battery_recovery_state(
                battery_pct=(
                    max(0.0, min(100.0, operator_soc_pct))
                    if np.isfinite(operator_soc_pct)
                    else 0.0
                ),
                state=state,
            )
            return

        loaded_low_persist_s = float(getattr(msg, "loaded_low_persist_s", 0.0) or 0.0)
        loaded_low_threshold_v = float(
            getattr(msg, "loaded_low_threshold_v", 0.0) or 0.0
        )
        recovered_low_persist_s = float(
            getattr(msg, "recovered_low_persist_s", 0.0) or 0.0
        )
        recovered_low_threshold_v = float(
            getattr(msg, "recovered_low_threshold_v", 0.0) or 0.0
        )
        loaded_voltage_slow_v = float(
            getattr(msg, "loaded_voltage_slow_v", 0.0) or 0.0
        )
        recovered_voltage_v = float(
            getattr(msg, "recovered_voltage_v", 0.0) or 0.0
        )

        trigger_code = "BATTERY_GUARD_RECOVERED_LOW"
        trigger_message = "Battery guard requested return home after low recovered voltage"
        if bool(getattr(msg, "traction_active", False)) and (
            loaded_low_persist_s >= recovered_low_persist_s
        ):
            trigger_code = "BATTERY_GUARD_LOADED_LOW_SUSTAINED"
            trigger_message = (
                "Battery guard requested return home after sustained low loaded voltage"
            )

        battery_pct = (
            max(0.0, min(100.0, operator_soc_pct))
            if np.isfinite(operator_soc_pct)
            else 0.0
        )
        self._activate_low_battery_response(
            battery_pct=battery_pct,
            detected_event_code=trigger_code,
            detected_message=trigger_message,
            detected_details={
                "battery_pct": f"{battery_pct:.1f}",
                "state": state,
                "loaded_voltage_slow_v": f"{loaded_voltage_slow_v:.2f}",
                "recovered_voltage_v": f"{recovered_voltage_v:.2f}",
                "loaded_low_persist_s": f"{loaded_low_persist_s:.1f}",
                "recovered_low_persist_s": f"{recovered_low_persist_s:.1f}",
                "loaded_low_threshold_v": f"{loaded_low_threshold_v:.2f}",
                "recovered_low_threshold_v": f"{recovered_low_threshold_v:.2f}",
            },
        )

    def _clear_low_battery_recovery_state(
        self,
        *,
        battery_pct: float,
        state: str,
    ) -> None:
        should_brake = False
        should_cancel = False
        should_publish = False
        previous_phase = "idle"
        with self._lock:
            if not (
                self._low_battery_active
                or self._return_home_requested
                or self._return_home_active
            ):
                return
            previous_phase = self._return_home_phase_locked()
            should_brake = bool(
                self._mission_active
                or self._return_home_requested
                or self._return_home_active
            )
            should_cancel = bool(
                self._mission_active
                or self._awaiting_chunk_result
                or self._return_home_requested
                or self._return_home_active
            )
            self._mission_active = False
            self._mission_paused = False
            self._awaiting_chunk_result = False
            self._active_chunk = []
            self._action_active = False
            self._action_waypoint_index = 0
            self._action_type = ""
            self._action_until = None
            self._low_battery_active = False
            self._return_home_requested = False
            self._return_home_active = False
            self._return_home_exit_route_index = -1
            self._return_home_exit_input_index = -1
            self._mission_status = self._status_with_note_locked("idle")
            self._clear_blocked_state_locked()
            should_publish = True

        if should_cancel:
            self._cancel_nav_goal()
        if should_brake:
            self._apply_brake()
        if should_publish:
            self._publish_route_event(
                DiagnosticStatus.OK,
                "BATTERY_GUARD_RECOVERED_CLEARED",
                "Battery recovered; return-home state cleared and mission parked",
                details={
                    "battery_pct": f"{battery_pct:.1f}",
                    "state": str(state),
                    "previous_phase": str(previous_phase),
                },
            )
            self._publish_empty_route_paths()

    def _stop_for_low_battery_non_loop(self, battery_pct: float) -> None:
        self._apply_brake()
        self._cancel_nav_goal()
        with self._lock:
            self._mission_active = False
            self._mission_paused = False
            self._awaiting_chunk_result = False
            self._active_chunk = []
            self._return_home_requested = False
            self._return_home_active = False
            self._return_home_exit_route_index = -1
            self._return_home_exit_input_index = -1
            self._mission_status = self._status_with_note_locked(
                "route stopped: low battery on non-loop mission"
            )
        self._publish_route_event(
            DiagnosticStatus.WARN,
            "LOW_BATTERY_NON_LOOP_STOPPED",
            "Low battery stopped non-loop mission",
            details={"battery_pct": f"{battery_pct:.1f}"},
        )
        self._publish_empty_route_paths()

    def _stop_for_missing_home(self, battery_pct: float) -> None:
        self._apply_brake()
        self._cancel_nav_goal()
        with self._lock:
            self._mission_active = False
            self._mission_paused = False
            self._awaiting_chunk_result = False
            self._active_chunk = []
            self._return_home_requested = False
            self._return_home_active = False
            self._return_home_exit_route_index = -1
            self._return_home_exit_input_index = -1
            self._mission_status = self._status_with_note_locked(
                "return home unavailable: HOME waypoint missing"
            )
        self._publish_route_event(
            DiagnosticStatus.ERROR,
            "RETURN_HOME_UNAVAILABLE",
            "Low battery detected but no HOME waypoint is available",
            details={"battery_pct": f"{battery_pct:.1f}"},
        )
        self._publish_empty_route_paths()

    def _cancel_nav_goal(self) -> Tuple[bool, str]:
        res = self._call_service(
            self._nav_cancel_goal_client, CancelNavGoal.Request(), self.request_timeout_s
        )
        if res is None:
            return False, "cancel_goal timeout"
        return bool(res.ok), str(res.error)

    def _dispatch_return_home(self) -> Tuple[bool, str]:
        with self._lock:
            home_waypoint = self._effective_home_waypoint_locked()
            if home_waypoint is None:
                return False, "HOME waypoint unavailable"

        request = SetNavGoalLL.Request()
        request.lats = [float(home_waypoint.lat)]
        request.lons = [float(home_waypoint.lon)]
        request.yaws_deg = [float(home_waypoint.yaw_deg)]
        request.loop = False
        request.suppress_success_brake = False
        request.lat = float(home_waypoint.lat)
        request.lon = float(home_waypoint.lon)
        request.yaw_deg = float(home_waypoint.yaw_deg)
        response = self._call_service(self._nav_set_goal_client, request, self.request_timeout_s)
        if response is None:
            return False, "set_goal_ll timeout"
        if not bool(response.ok):
            return False, str(response.error)

        with self._lock:
            self._return_home_active = True
            self._awaiting_chunk_result = True
            self._active_chunk = [home_waypoint]
            self._mission_status = self._status_with_note_locked("return home active")
        self._publish_route_event(
            DiagnosticStatus.WARN,
            "RETURN_HOME_STARTED",
            "Return home started",
            details={
                "home_lat": f"{home_waypoint.lat:.8f}",
                "home_lon": f"{home_waypoint.lon:.8f}",
                "home_yaw_deg": f"{home_waypoint.yaw_deg:.2f}",
            },
        )
        self._publish_route_debug_path(
            self._active_chunk_path_pub,
            [home_waypoint],
            label="return home",
        )
        return True, ""

    def _send_chunk(self, *, start_index: int) -> Tuple[bool, str]:
        with self._lock:
            route = list(self._route_expanded)
            action_jsons = list(self._route_action_jsons)
            waypoint_roles = list(self._route_waypoint_roles)
            key_flags = list(self._route_key_waypoint_flags)
            input_indices = list(self._route_input_indices)
            loop_enabled = bool(self._mission_loop)
            follow_exact_path = bool(self._mission_follow_exact_path)
            resume_exact_at_start = int(self._coverage_exact_resume_index) == int(
                start_index
            )
            chunk_span_m = float(self._chunk_span_m)
            chunk_max_waypoints = int(self._chunk_max_waypoints)
            robot_pose = self._last_robot_pose

            if resume_exact_at_start:
                self._coverage_exact_resume_index = -1

        action_indices = {
            idx for idx, action_json in enumerate(action_jsons) if str(action_json or "")
        }
        key_indices = {
            idx for idx, is_key in enumerate(key_flags) if idx < len(route) and bool(is_key)
        }
        # Las keys flojas (tangentes de curva, reingresos de fila) se cruzan
        # dentro del bloque; cortar ahi es lo que rompia el empalme curva/fila.
        key_pass_indices: Set[int] = set()
        if follow_exact_path and len(waypoint_roles) == len(route):
            key_pass_indices = {
                idx
                for idx in coverage_flexible_chunk_pass_indices(
                    waypoint_roles, key_flags
                )
                if idx < len(route)
            }
        synthetic_indices = {
            idx for idx, is_key in enumerate(key_flags) if idx < len(route) and not bool(is_key)
        }
        if resume_exact_at_start:
            resolved_start_index, skipped_start_waypoints = int(start_index), 0
        else:
            reached_tolerance_m = (
                self.coverage_row_reached_tolerance_m
                if follow_exact_path
                else self.route_waypoint_reached_tolerance_m
            )
            resolved_start_index, skipped_start_waypoints = skip_reached_chunk_start(
                route,
                start_index=start_index,
                loop=loop_enabled,
                robot_lat=None if robot_pose is None else float(robot_pose[0]),
                robot_lon=None if robot_pose is None else float(robot_pose[1]),
                waypoint_reached_tolerance_m=reached_tolerance_m,
                protected_indices=action_indices | key_pass_indices,
                loose_indices={
                    idx
                    for idx, role in enumerate(waypoint_roles)
                    if str(role).strip().lower() == WAYPOINT_ROLE_COVERAGE_TRANSIT
                },
                loose_tolerance_m=self.coverage_transit_reached_tolerance_m,
                index_tolerances_m={
                    idx: self.coverage_curve_reached_tolerance_m
                    for idx, role in enumerate(waypoint_roles)
                    if str(role).strip().lower()
                    == WAYPOINT_ROLE_COVERAGE_CURVE
                },
            )
        if resolved_start_index is None:
            return False, "empty route chunk"
        synthetic_resolved_start_index, skipped_synthetic_waypoints = (
            (int(resolved_start_index), 0)
            if resume_exact_at_start
            else skip_passed_synthetic_chunk_start(
                route,
                start_index=resolved_start_index,
                loop=loop_enabled,
                robot_lat=None if robot_pose is None else float(robot_pose[0]),
                robot_lon=None if robot_pose is None else float(robot_pose[1]),
                segment_tolerance_m=self.route_segment_start_tolerance_m,
                skippable_indices=synthetic_indices,
                protected_indices=action_indices,
            )
        )
        if synthetic_resolved_start_index is None:
            return False, "empty route chunk"
        resolved_start_index = int(synthetic_resolved_start_index)
        if skipped_start_waypoints > 0:
            self.get_logger().info(
                "Skipped reached chunk waypoint"
                f"{'s' if skipped_start_waypoints != 1 else ''} "
                f"(requested_start={start_index}, "
                f"resolved_start={resolved_start_index}, "
                f"skipped={skipped_start_waypoints})"
            )
        if skipped_synthetic_waypoints > 0:
            self.get_logger().info(
                "Skipped passed synthetic chunk waypoint"
                f"{'s' if skipped_synthetic_waypoints != 1 else ''} "
                f"(requested_start={start_index}, "
                f"resolved_start={resolved_start_index}, "
                f"skipped={skipped_synthetic_waypoints})"
            )
            self._publish_route_event(
                DiagnosticStatus.WARN,
                "ROUTE_SYNTHETIC_SKIPPED",
                "Skipped stale synthetic route point",
                details={
                    "requested_start_index": int(start_index),
                    "resolved_start_index": int(resolved_start_index),
                    "skipped_count": int(skipped_synthetic_waypoints),
                },
            )

        # Si el bloque cierra en una meta que el vehiculo ya cruzo, despacharlo
        # es mandarle a Nav2 un camino entero por detras: el controlador lo poda
        # hasta dejarlo en cero poses y aborta la mision. En ese caso se avanza
        # al bloque siguiente, que es el que todavia tiene camino por delante.
        skipped_passed_blocks = 0
        while True:
            chunk, end_index = build_chunk_waypoints(
                route,
                start_index=resolved_start_index,
                loop=loop_enabled,
                chunk_span_m=chunk_span_m,
                chunk_max_waypoints=chunk_max_waypoints,
                action_stop_indices=action_indices,
                key_stop_indices=key_indices,
                key_pass_indices=key_pass_indices,
            )
            if not chunk:
                break
            next_start = int(end_index) + 1
            if (
                loop_enabled
                or int(end_index) in action_indices
                or next_start >= len(route)
                or skipped_passed_blocks >= max(1, len(route))
                or not block_target_already_passed(
                    route,
                    end_index=end_index,
                    robot_lat=None if robot_pose is None else float(robot_pose[0]),
                    robot_lon=None if robot_pose is None else float(robot_pose[1]),
                    corridor_tolerance_m=self.route_segment_start_tolerance_m,
                )
            ):
                break
            self.get_logger().info(
                "Skipping route block already driven past "
                f"(start={resolved_start_index}, target={end_index})"
            )
            self._publish_route_event(
                DiagnosticStatus.WARN,
                "ROUTE_BLOCK_PASSED_SKIPPED",
                "Route block target already driven past",
                details={
                    "start_index": int(resolved_start_index),
                    "target_index": int(end_index),
                },
            )
            resolved_start_index = next_start
            skipped_passed_blocks += 1
        if not chunk:
            return False, "empty route chunk"

        if len(waypoint_roles) == len(route):
            chunk_roles = [
                waypoint_roles[(resolved_start_index + offset) % len(route)]
                for offset in range(len(chunk))
            ]
        else:
            chunk_roles = [WAYPOINT_ROLE_NORMAL for _ in chunk]
        # Un bloque que empieza por una guia se conserva completo. La S interna,
        # las dos medias curvas exteriores y cada fila usan FollowPath con la
        # key anterior como ancla. Las metas de transito que no pertenecen a
        # una curva conservan NavigateThroughPoses.
        coverage_waypoint_chunk = bool(
            self._mission_follow_exact_path
            and any(
                str(role).strip().lower()
                in (
                    WAYPOINT_ROLE_COVERAGE,
                    WAYPOINT_ROLE_COVERAGE_CURVE,
                    WAYPOINT_ROLE_COVERAGE_TRANSIT,
                )
                for role in chunk_roles
            )
        )
        starts_with_guide = bool(
            0 <= resolved_start_index < len(key_flags)
            and (
                not bool(key_flags[resolved_start_index])
                or int(resolved_start_index) in key_pass_indices
            )
        )
        if self._mission_follow_exact_path:
            # Un bloque de guias termina en la key siguiente. La tolerancia se
            # decide por esa meta final, no por la primera guia de transito.
            target_role = str(
                chunk_roles[-1] if starts_with_guide else chunk_roles[0]
            ).strip().lower()
            if target_role == WAYPOINT_ROLE_COVERAGE_TRANSIT:
                desired_tolerance_m = self.coverage_transit_reached_tolerance_m
            elif target_role == WAYPOINT_ROLE_COVERAGE_CURVE:
                desired_tolerance_m = self.coverage_curve_reached_tolerance_m
            else:
                desired_tolerance_m = self.coverage_row_reached_tolerance_m
            tolerance_ok, tolerance_error = self._set_coverage_goal_tolerance(
                desired_tolerance_m
            )
            if not tolerance_ok:
                return False, tolerance_error
        elif getattr(self, "_coverage_goal_tolerance_applied_m", None) is not None:
            # Una Ruta o Patrol que reemplace/cancele CAMPO nunca puede heredar
            # los 3.0 m flexibles de una cabecera exterior.
            tolerance_ok, tolerance_error = self._set_coverage_goal_tolerance(
                self.route_waypoint_reached_tolerance_m
            )
            if not tolerance_ok:
                return False, tolerance_error
        follow_exact_path = False
        dispatch_chunk = list(chunk)
        if (
            coverage_waypoint_chunk
            and starts_with_guide
            and resolved_start_index > 0
        ):
            # El ancla es la key precisa anterior, no el indice previo a secas.
            # Si el arranque quedo adentro del bloque (por ejemplo tras dar por
            # alcanzada la tangente de salida de una curva), el camino se rearma
            # completo desde esa key: asi el tramo despachado es siempre el
            # mismo, con o sin waypoints dados por alcanzados.
            anchor_index = coverage_block_anchor_index(
                key_flags, key_pass_indices, resolved_start_index
            )
            prefix_indices = list(range(anchor_index, int(resolved_start_index)))
            prefix_roles = [
                waypoint_roles[index]
                if index < len(waypoint_roles)
                else WAYPOINT_ROLE_NORMAL
                for index in prefix_indices
            ]
            exact_roles = prefix_roles + list(chunk_roles)
            if (
                should_follow_exact_coverage_chunk(
                    self._mission_follow_exact_path,
                    exact_roles,
                )
                or should_follow_exact_precision_anchor_chunk(
                    self._mission_follow_exact_path,
                    exact_roles,
                )
            ):
                dispatch_chunk = [route[index] for index in prefix_indices] + list(
                    chunk
                )
                follow_exact_path = True
        if coverage_waypoint_chunk and not follow_exact_path:
            current_role = str(chunk_roles[0]).strip().lower()
            if (
                not starts_with_guide
                and resolved_start_index > 0
                and current_role == WAYPOINT_ROLE_COVERAGE
            ):
                # Una fila nunca se replantea como NavigateToPose: se sigue la
                # recta exacta desde su inicio nominal hasta su extremo. Esto
                # tambien permite arrancarla desde una reentrada exterior
                # aceptada con tolerancia sin habilitar diagonales en cultivo.
                # Si el reintento re-ancla en medio de un bloque (la key
                # anterior es floja o es una guia), la fila no puede arrancar
                # desde ahi: se rearma el camino completo desde la key precisa
                # que abre el bloque, curva incluida.
                anchor_index = coverage_block_anchor_index(
                    key_flags, key_pass_indices, resolved_start_index
                )
                dispatch_chunk = [
                    route[index]
                    for index in range(anchor_index, int(resolved_start_index))
                ] + [chunk[0]]
                follow_exact_path = True
                end_index = int(resolved_start_index)
            elif not starts_with_guide:
                dispatch_chunk = [chunk[0]]
                end_index = int(resolved_start_index)

        with self._lock:
            self._chunk_id += 1
            chunk_id = int(self._chunk_id)
            self._current_start_index = int(resolved_start_index)
            self._current_target_index = int(end_index)
        target_input_index = -1
        if len(input_indices) != len(key_flags):
            input_indices = expanded_input_indices(key_flags)
        if 0 <= end_index < len(input_indices):
            target_input_index = int(input_indices[end_index])
        self._publish_route_event(
            DiagnosticStatus.OK,
            "ROUTE_CHUNK_REQUESTED",
            "Route chunk requested",
            details={
                "chunk_id": chunk_id,
                "start_index": int(resolved_start_index),
                "target_index": int(end_index),
                "target_input_index": target_input_index,
                "waypoint_count": len(dispatch_chunk),
                "synthetic_count": sum(
                    1
                    for index in range(resolved_start_index, min(len(key_flags), end_index + 1))
                    if not bool(key_flags[index])
                ) if resolved_start_index <= end_index else sum(not bool(flag) for flag in key_flags),
            },
        )

        request = SetNavGoalLL.Request()
        request.lats = [float(entry.lat) for entry in dispatch_chunk]
        request.lons = [float(entry.lon) for entry in dispatch_chunk]
        request.yaws_deg = [float(entry.yaw_deg) for entry in dispatch_chunk]
        request.loop = False
        request.suppress_success_brake = should_suppress_chunk_success_brake(
            current_target_index=end_index,
            route_size=len(route),
            loop=loop_enabled,
        )
        request.follow_exact_path = bool(follow_exact_path)
        request.lat = float(request.lats[0])
        request.lon = float(request.lons[0])
        request.yaw_deg = float(request.yaws_deg[0])
        response = self._call_service(self._nav_set_goal_client, request, self.request_timeout_s)
        if response is None:
            self._publish_route_event(
                DiagnosticStatus.ERROR,
                "ROUTE_CHUNK_DISPATCH_FAILED",
                "Route chunk dispatch timed out",
                details={"chunk_id": chunk_id, "error": "set_goal_ll timeout"},
            )
            return False, "set_goal_ll timeout"
        if not response.ok:
            self._publish_route_event(
                DiagnosticStatus.ERROR,
                "ROUTE_CHUNK_DISPATCH_FAILED",
                "Route chunk dispatch failed",
                details={"chunk_id": chunk_id, "error": str(response.error)},
            )
            return False, str(response.error)

        with self._lock:
            # Cobertura se despacha siempre por waypoint; no queda una
            # aproximacion FollowPath pendiente que pueda reinyectar un tramo.
            self._coverage_approach_pending = False
            self._active_chunk = dispatch_chunk
            self._awaiting_chunk_result = True
            self._mission_status = self._status_with_note_locked(
                f"route active ({self._current_start_index + 1}->{self._current_target_index + 1})"
            )
        self.get_logger().info(
            "Route chunk dispatched "
            f"(start={resolved_start_index}, end={end_index}, size={len(chunk)})"
        )
        self._publish_route_event(
            DiagnosticStatus.OK,
            "ROUTE_CHUNK_DISPATCHED",
            "Route chunk accepted by navigation",
            details={
                "chunk_id": chunk_id,
                "start_index": int(resolved_start_index),
                "target_index": int(end_index),
                "target_input_index": target_input_index,
                "waypoint_count": len(chunk),
            },
        )
        self._publish_route_debug_path(
            self._active_chunk_path_pub,
            chunk,
            label="active chunk",
        )
        return True, ""

    def _complete_route_locked(self) -> None:
        self._mission_active = False
        self._mission_paused = False
        self._awaiting_chunk_result = False
        self._active_chunk = []
        self._mission_status = self._status_with_note_locked("route completed")

    def _run_coverage_backup(self, distance_m: float) -> Tuple[bool, str]:
        """Retroceder en la cabecera usando el behavior server de Nav2.

        Esta es la unica marcha atras del sistema y vive entera aca: el planner
        global sigue en DUBIN y el controlador con ``allow_reversing`` en false,
        asi que Ruta, Patrol y los goals sueltos no pueden retroceder ni por
        accidente. La reversa es una maniobra puntual, acotada y pedida a mano.

        Se espera con ``_wait_for_future``, que sondea sin spinear: el executor
        del nodo tiene varios hilos y spinear adentro de una callback esperando
        otra callback es exactamente lo que se va a timeout cuando hay trafico.

        Con ``coverage_f2c_allow_reverse`` en false el perfil es forward-only y
        la maniobra se rechaza aca, aunque la accion venga en la ruta. Es la
        barrera de EJECUCION: la de planificacion evita generarla, esta evita
        ejecutarla si la ruta fue armada por otro backend o quedo cacheada.

        Devuelve ``(ok, error)``. Nunca lanza: el llamador corre adentro de una
        callback de rclpy y una excepcion ahi mata el nodo entero.
        """
        try:
            if not bool(self.get_parameter("coverage_f2c_allow_reverse").value):
                return (
                    False,
                    "perfil forward-only: coverage_backup deshabilitado por "
                    "coverage_f2c_allow_reverse=false",
                )
            if self._backup_client is None:
                self._backup_client = ActionClient(
                    self, BackUp, self.coverage_backup_action
                )
            timeout_s = max(1.0, float(self.coverage_backup_timeout_s))
            if not self._backup_client.wait_for_server(
                timeout_sec=min(5.0, timeout_s)
            ):
                return False, "el behavior server de Nav2 no expone backup"

            goal = BackUp.Goal()
            goal.target = Point(x=float(abs(distance_m)), y=0.0, z=0.0)
            goal.speed = float(self.coverage_backup_speed_mps)
            goal.time_allowance = Duration(
                seconds=float(timeout_s)
            ).to_msg()

            handle = self._wait_for_future(
                self._backup_client.send_goal_async(goal), timeout_s
            )
            if handle is None:
                return False, "timeout esperando que Nav2 acepte la marcha atras"
            if not handle.accepted:
                return False, "Nav2 rechazo la marcha atras"
            envuelto = self._wait_for_future(handle.get_result_async(), timeout_s)
            if envuelto is None:
                return False, "timeout ejecutando la marcha atras"
            status = int(getattr(envuelto, "status", 0))
            if status != GoalStatus.STATUS_SUCCEEDED:
                return False, f"la marcha atras termino con status {status}"
            return True, ""
        except Exception as exc:  # pragma: no cover - red de seguridad
            return False, f"fallo inesperado en la marcha atras: {exc}"

    def _run_waypoint_actions_then_continue(
        self,
        *,
        waypoint_index: int,
        action_json: str,
        next_start_index: int,
        reached_end: bool,
    ) -> None:
        actions = _actions_from_json(action_json)
        for action in actions:
            if action.action_type == "set_navigation_profile":
                with self._lock:
                    if (
                        not self._mission_active
                        or self._mission_paused
                        or int(self._current_target_index) != int(waypoint_index)
                    ):
                        self._action_active = False
                        self._action_until = None
                        return
                    self._action_active = True
                    self._action_waypoint_index = int(waypoint_index)
                    self._action_type = str(action.action_type)
                    self._action_until = None
                    self._mission_status = self._status_with_note_locked(
                        f"route action set_navigation_profile ({action.profile})"
                    )
                self._publish_route_event(
                    DiagnosticStatus.OK,
                    "ROUTE_WAYPOINT_ACTION_STARTED",
                    "Route waypoint action started",
                    details={
                        "waypoint_index": int(waypoint_index),
                        "action_type": action.action_type,
                        "profile": action.profile,
                    },
                )
                ok, error = self._apply_navigation_profile(action.profile)
                if not ok:
                    self._apply_brake()
                    with self._lock:
                        self._action_active = False
                        self._action_until = None
                        self._mission_active = False
                        self._mission_paused = False
                        self._awaiting_chunk_result = False
                        self._active_chunk = []
                        self._mission_status = self._status_with_note_locked(
                            f"route failed: navigation profile {action.profile}: {error}"
                        )
                    self._publish_route_event(
                        DiagnosticStatus.ERROR,
                        "NAVIGATION_PROFILE_FAILED",
                        "Navigation profile action failed",
                        details={
                            "waypoint_index": int(waypoint_index),
                            "profile": action.profile,
                            "error": str(error),
                        },
                    )
                    self._restore_urban_navigation_profile()
                    self._publish_empty_route_paths()
                    return
                self._publish_route_event(
                    DiagnosticStatus.OK,
                    "NAVIGATION_PROFILE_APPLIED",
                    "Navigation profile applied",
                    details={
                        "waypoint_index": int(waypoint_index),
                        "profile": action.profile,
                    },
                )
                continue
            if action.action_type == "coverage_backup":
                distancia_m = max(0.0, float(action.distance_m))
                with self._lock:
                    if (
                        not self._mission_active
                        or self._mission_paused
                        or int(self._current_target_index) != int(waypoint_index)
                    ):
                        self._action_active = False
                        self._action_until = None
                        return
                    self._action_active = True
                    self._action_waypoint_index = int(waypoint_index)
                    self._action_type = str(action.action_type)
                    self._action_until = None
                    self._mission_status = self._status_with_note_locked(
                        f"route action coverage_backup ({distancia_m:.2f}m)"
                    )
                self._publish_route_event(
                    DiagnosticStatus.OK,
                    "ROUTE_WAYPOINT_ACTION_STARTED",
                    "Route waypoint action started",
                    details={
                        "waypoint_index": int(waypoint_index),
                        "action_type": action.action_type,
                        "distance_m": f"{distancia_m:.3f}",
                    },
                )
                ok, error = self._run_coverage_backup(distancia_m)
                if not ok:
                    # No se aborta la mision: la cabecera es transito afuera del
                    # lote. Si la marcha atras no salio, el vehiculo sigue al
                    # proximo waypoint y Nav2 resuelve como puede; abortar
                    # dejaria el lote a medio cubrir por una maniobra auxiliar.
                    self._publish_route_event(
                        DiagnosticStatus.WARN,
                        "COVERAGE_BACKUP_FAILED",
                        "Coverage headland backup failed",
                        details={
                            "waypoint_index": int(waypoint_index),
                            "distance_m": f"{distancia_m:.3f}",
                            "error": str(error),
                        },
                    )
                    self.get_logger().warning(
                        f"coverage backup failed at waypoint {waypoint_index}: {error}"
                    )
                else:
                    self._publish_route_event(
                        DiagnosticStatus.OK,
                        "COVERAGE_BACKUP_DONE",
                        "Coverage headland backup done",
                        details={
                            "waypoint_index": int(waypoint_index),
                            "distance_m": f"{distancia_m:.3f}",
                        },
                    )
                continue
            if action.action_type != "brake_hold":
                continue
            duration_s = max(0.0, float(action.duration_s))
            with self._lock:
                if (
                    not self._mission_active
                    or self._mission_paused
                    or int(self._current_target_index) != int(waypoint_index)
                ):
                    self._action_active = False
                    self._action_until = None
                    return
                self._action_active = True
                self._action_waypoint_index = int(waypoint_index)
                self._action_type = str(action.action_type)
                self._action_until = time.monotonic() + duration_s
                self._mission_status = self._status_with_note_locked(
                    f"route action brake_hold ({duration_s:.1f}s)"
                )

            self._publish_route_event(
                DiagnosticStatus.OK,
                "ROUTE_WAYPOINT_ACTION_STARTED",
                "Route waypoint action started",
                details={
                    "waypoint_index": int(waypoint_index),
                    "action_type": action.action_type,
                    "duration_s": f"{duration_s:.3f}",
                    "brake_pct": int(action.brake_pct),
                },
            )

            self._apply_brake(duration_s=duration_s, brake_pct=int(action.brake_pct))
            deadline = time.monotonic() + duration_s
            while rclpy.ok() and time.monotonic() < deadline:
                time.sleep(min(0.2, max(0.05, deadline - time.monotonic())))

            self._publish_route_event(
                DiagnosticStatus.OK,
                "ROUTE_WAYPOINT_ACTION_FINISHED",
                "Route waypoint action finished",
                details={
                    "waypoint_index": int(waypoint_index),
                    "action_type": action.action_type,
                },
            )

        should_clear_paths = False
        should_send_next = False
        should_send_home = False
        should_start_patrol_loop = False
        should_finish_patrol_home = False
        exit_waypoint_reached = False
        with self._lock:
            self._action_active = False
            self._action_waypoint_index = 0
            self._action_type = ""
            self._action_until = None
            if not self._mission_active or self._mission_paused:
                return
            exit_waypoint_reached = bool(
                self._return_home_requested
                and not self._return_home_active
                and self._return_home_exit_input_index >= 0
                and self._current_target_index < len(self._route_input_indices)
                and int(self._route_input_indices[self._current_target_index])
                == int(self._return_home_exit_input_index)
            )
            if exit_waypoint_reached:
                should_send_home = True
                self._mission_status = self._status_with_note_locked(
                    "return home exit waypoint reached"
                )
            elif reached_end and (not self._mission_loop):
                if self._patrol_active and self._patrol_phase == PATROL_PHASE_DEPART_HOME:
                    should_start_patrol_loop = True
                    self._mission_status = self._status_with_note_locked("patrol loop starting")
                elif self._patrol_active and self._patrol_phase == PATROL_PHASE_RETURN_CONNECTOR:
                    self._complete_route_locked()
                    self._patrol_phase = PATROL_PHASE_PARKED_HOME
                    self._patrol_active = False
                    self._return_home_active = False
                    should_finish_patrol_home = True
                    should_clear_paths = True
                else:
                    self._complete_route_locked()
                    should_clear_paths = True
            else:
                should_send_next = True

        if should_clear_paths:
            self._restore_urban_navigation_profile()
            self._publish_route_event(
                DiagnosticStatus.OK,
                "RETURN_HOME_COMPLETED" if should_finish_patrol_home else "ROUTE_MISSION_COMPLETED",
                "Return home completed" if should_finish_patrol_home else "Route mission completed after waypoint action",
                details={"reached_checkpoint_count": int(self._reached_checkpoint_count)},
            )
            self._publish_empty_route_paths()
            return
        if should_start_patrol_loop:
            ok, err, _, _ = self._start_patrol_phase(PATROL_PHASE_LOOP_MAIN)
            if ok:
                return
            self._publish_route_event(
                DiagnosticStatus.ERROR,
                "PATROL_MISSION_FAILED",
                "Patrol loop could not start after depart connector",
                details={"error": str(err)},
            )
            with self._lock:
                self._reset_mission_locked(f"patrol failed: {err}")
            self._restore_urban_navigation_profile()
            self._publish_empty_route_paths()
            return
        if should_send_home:
            self._publish_route_event(
                DiagnosticStatus.WARN,
                "RETURN_HOME_EXIT_REACHED",
                "Return home exit waypoint reached after waypoint action",
                details={"home_exit_input_index": int(self._return_home_exit_input_index)},
            )
            if self._patrol_active:
                ok, err, _, _ = self._start_patrol_phase(PATROL_PHASE_RETURN_CONNECTOR)
            else:
                ok, err = self._dispatch_return_home()
            if ok:
                return
            self._publish_route_event(
                DiagnosticStatus.ERROR,
                "RETURN_HOME_UNAVAILABLE",
                "Return home could not be dispatched after waypoint action",
                details={"error": str(err)},
            )
            with self._lock:
                self._mission_active = False
                self._mission_paused = False
                self._awaiting_chunk_result = False
                self._active_chunk = []
                self._return_home_requested = False
                self._return_home_active = False
                self._return_home_exit_route_index = -1
                self._return_home_exit_input_index = -1
                self._mission_status = self._status_with_note_locked(f"return home failed: {err}")
            self._restore_urban_navigation_profile()
            self._publish_empty_route_paths()
            return
        if should_send_next:
            ok, err = self._send_chunk(start_index=next_start_index)
            if ok:
                return
            self._publish_route_event(
                DiagnosticStatus.ERROR,
                "ROUTE_MISSION_FAILED",
                "Route mission failed after waypoint action",
                details={"error": str(err)},
            )
            with self._lock:
                self._mission_active = False
                self._mission_paused = False
                self._awaiting_chunk_result = False
                self._active_chunk = []
                self._mission_status = self._status_with_note_locked(f"route failed: {err}")
            self._restore_urban_navigation_profile()
            self._publish_empty_route_paths()

    def _start_next_chunk_after_success(self) -> None:
        should_clear_paths = False
        should_run_action = False
        action_json = ""
        action_waypoint_index = 0
        reached_end = False
        next_start_index = 0
        checkpoint_details: Optional[Dict[str, Any]] = None
        mission_completed = False
        should_send_home = False
        should_start_patrol_loop = False
        should_finish_patrol_home = False
        exit_waypoint_reached = False
        reached_input_index = -1
        with self._lock:
            if not self._mission_active or self._mission_paused:
                return
            expanded_count = len(self._route_expanded)
            loop_enabled = bool(self._mission_loop)
            if expanded_count == 0:
                self._reset_mission_locked("route failed: empty expanded route")
                should_clear_paths = True
            else:
                action_waypoint_index = int(self._current_target_index)
                start_index = int(self._current_start_index)
                input_indices = list(self._route_input_indices)
                if len(input_indices) != len(self._route_key_waypoint_flags):
                    input_indices = expanded_input_indices(self._route_key_waypoint_flags)
                input_index = (
                    int(input_indices[action_waypoint_index])
                    if 0 <= action_waypoint_index < len(input_indices)
                    else -1
                )
                reached_input_index = int(input_index)
                if loop_enabled and action_waypoint_index < start_index:
                    self._loop_iteration += 1
                if input_index >= 0:
                    self._reached_checkpoint_count += 1
                    checkpoint_details = {
                        "expanded_index": action_waypoint_index,
                        "input_index": input_index,
                        "loop_iteration": int(self._loop_iteration),
                        "reached_checkpoint_count": int(self._reached_checkpoint_count),
                        "has_action": int(bool(self._route_action_jsons[action_waypoint_index])),
                    }
                if 0 <= action_waypoint_index < len(self._route_action_jsons):
                    action_json = str(self._route_action_jsons[action_waypoint_index] or "")
                if bool(getattr(self, "_coverage_approach_pending", False)):
                    # La meta de aproximacion fue el primer punto de la
                    # pasada. Hay que conservarlo como inicio del FollowPath:
                    # si se avanzara al siguiente indice, el controlador veria
                    # toda la pasada fuera de su ventana local otra vez.
                    self._coverage_approach_pending = False
                    self._coverage_exact_resume_index = int(start_index)
                    next_start_index = int(start_index)
                else:
                    next_start_index = next_chunk_start_index(
                        current_target_index=self._current_target_index,
                        route_size=expanded_count,
                        loop=loop_enabled,
                    )
                reached_end = next_start_index >= expanded_count
                should_run_action = bool(action_json) and not self._action_active
                exit_waypoint_reached = bool(
                    self._return_home_requested
                    and not self._return_home_active
                    and input_index >= 0
                    and int(input_index) == int(self._return_home_exit_input_index)
                )
                should_send_home = bool(exit_waypoint_reached and not should_run_action)
                if should_run_action:
                    self._mission_status = self._status_with_note_locked("route action pending")
                elif should_send_home or exit_waypoint_reached:
                    self._mission_status = self._status_with_note_locked(
                        "return home exit waypoint reached"
                    )
                elif reached_end and (not loop_enabled):
                    if self._patrol_active and self._patrol_phase == PATROL_PHASE_DEPART_HOME:
                        should_start_patrol_loop = True
                        self._mission_status = self._status_with_note_locked("patrol loop starting")
                    elif self._patrol_active and self._patrol_phase == PATROL_PHASE_RETURN_CONNECTOR:
                        self._complete_route_locked()
                        self._patrol_phase = PATROL_PHASE_PARKED_HOME
                        self._patrol_active = False
                        self._return_home_active = False
                        should_finish_patrol_home = True
                        should_clear_paths = True
                    else:
                        self._complete_route_locked()
                        should_clear_paths = True
                        mission_completed = True
                elif self._return_home_requested and not self._return_home_active:
                    self._mission_status = self._status_with_note_locked(
                        "return home waiting for exit waypoint"
                    )

        if checkpoint_details is not None:
            self._publish_route_event(
                DiagnosticStatus.OK,
                "ROUTE_CHECKPOINT_REACHED",
                "Route checkpoint reached",
                details=checkpoint_details,
            )

        if should_clear_paths:
            self._restore_urban_navigation_profile()
            if should_finish_patrol_home:
                self._publish_route_event(
                    DiagnosticStatus.OK,
                    "RETURN_HOME_COMPLETED",
                    "Return home completed",
                    details={"reached_checkpoint_count": int(self._reached_checkpoint_count)},
                )
            elif mission_completed:
                self._publish_route_event(
                    DiagnosticStatus.OK,
                    "ROUTE_MISSION_COMPLETED",
                    "Route mission completed",
                    details={"reached_checkpoint_count": int(self._reached_checkpoint_count)},
                )
            self._publish_empty_route_paths()
            return
        if should_start_patrol_loop:
            ok, err, _, _ = self._start_patrol_phase(PATROL_PHASE_LOOP_MAIN)
            if ok:
                return
            self._publish_route_event(
                DiagnosticStatus.ERROR,
                "PATROL_MISSION_FAILED",
                "Patrol loop could not start after depart connector",
                details={"error": str(err)},
            )
            with self._lock:
                self._reset_mission_locked(f"patrol failed: {err}")
            self._restore_urban_navigation_profile()
            self._publish_empty_route_paths()
            return
        if should_run_action:
            thread = threading.Thread(
                target=self._run_waypoint_actions_then_continue,
                kwargs={
                    "waypoint_index": action_waypoint_index,
                    "action_json": action_json,
                    "next_start_index": next_start_index,
                    "reached_end": reached_end,
                },
                daemon=True,
                name="route_waypoint_action",
            )
            thread.start()
            return
        if should_send_home:
            self._publish_route_event(
                DiagnosticStatus.WARN,
                "RETURN_HOME_EXIT_REACHED",
                "Return home exit waypoint reached",
                details={"home_exit_input_index": int(reached_input_index)},
            )
            if self._patrol_active:
                ok, err, _, _ = self._start_patrol_phase(PATROL_PHASE_RETURN_CONNECTOR)
            else:
                ok, err = self._dispatch_return_home()
            if ok:
                return
            self._publish_route_event(
                DiagnosticStatus.ERROR,
                "RETURN_HOME_UNAVAILABLE",
                "Return home could not be dispatched",
                details={"error": str(err)},
            )
            with self._lock:
                self._mission_active = False
                self._mission_paused = False
                self._awaiting_chunk_result = False
                self._active_chunk = []
                self._return_home_requested = False
                self._return_home_active = False
                self._return_home_exit_route_index = -1
                self._return_home_exit_input_index = -1
                self._mission_status = self._status_with_note_locked(f"return home failed: {err}")
            self._publish_empty_route_paths()
            return

        ok, err = self._send_chunk(start_index=next_start_index)
        if ok:
            return
        self._publish_route_event(
            DiagnosticStatus.ERROR,
            "ROUTE_MISSION_FAILED",
            "Route mission failed while dispatching next chunk",
            details={"error": str(err)},
        )
        with self._lock:
            self._mission_active = False
            self._mission_paused = False
            self._awaiting_chunk_result = False
            self._active_chunk = []
            self._mission_status = self._status_with_note_locked(f"route failed: {err}")
        self._publish_empty_route_paths()

    def _on_nav_telemetry(self, msg: NavTelemetry) -> None:
        should_pause = False
        should_advance = False
        should_stop = False
        should_enter_blocked = False
        should_complete_return_home = False
        blocked_reason_code = ""
        blocked_reason_text = ""
        blocked_cancel_goal = False
        stop_reason = ""

        with self._lock:
            if np.isfinite(float(msg.robot_lat)) and np.isfinite(float(msg.robot_lon)):
                self._last_robot_pose = (float(msg.robot_lat), float(msg.robot_lon))
            self._last_nav_goal_active = bool(msg.goal_active)
            self._last_nav_result_status = int(msg.nav_result_status)
            self._last_nav_result_event_id = int(msg.nav_result_event_id)

            if not self._mission_active:
                return

            now = time.monotonic()
            if bool(msg.collision_stop_active) and bool(msg.goal_active):
                if self._last_collision_stop_started is None:
                    self._last_collision_stop_started = now
                    self._last_collision_stop_handled = False
                elif (
                    not self._last_collision_stop_handled
                    and not self._blocked_state
                    and (now - float(self._last_collision_stop_started))
                    >= self.collision_stop_persistent_s
                ):
                    self._last_collision_stop_handled = True
                    should_enter_blocked = True
                    blocked_reason_code = "COLLISION_STOP_ACTIVE"
                    blocked_reason_text = self._blocking_reason_text(blocked_reason_code)
                    blocked_cancel_goal = True
            else:
                self._last_collision_stop_started = None
                self._last_collision_stop_handled = False

            if bool(msg.manual_enabled) and (not self._mission_paused):
                self._mission_paused = True
                self._awaiting_chunk_result = False
                self._active_chunk = []
                self._action_active = False
                self._action_until = None
                self._clear_blocked_state_locked()
                self._mission_status = self._status_with_note_locked(
                    "route paused by manual takeover"
                )
                should_pause = True
            elif self._mission_paused:
                return

            terminal_result = (
                (not bool(msg.goal_active))
                and self._awaiting_chunk_result
                and int(msg.nav_result_event_id) > 0
                and int(msg.nav_result_event_id) != self._last_handled_nav_result_event_id
                and int(msg.nav_result_status)
                in (
                    int(GoalStatus.STATUS_SUCCEEDED),
                    int(GoalStatus.STATUS_ABORTED),
                    int(GoalStatus.STATUS_CANCELED),
                )
            )
            if not terminal_result:
                if not should_pause and not should_enter_blocked:
                    return
            else:
                self._last_handled_nav_result_event_id = int(msg.nav_result_event_id)
                self._awaiting_chunk_result = False
                status = int(msg.nav_result_status)
                if status == int(GoalStatus.STATUS_SUCCEEDED):
                    if self._return_home_active and self._patrol_phase != PATROL_PHASE_RETURN_CONNECTOR:
                        self._complete_route_locked()
                        self._return_home_active = False
                        self._return_home_requested = False
                        self._return_home_exit_route_index = -1
                        self._return_home_exit_input_index = -1
                        self._mission_status = self._status_with_note_locked(
                            "return home completed"
                        )
                        should_complete_return_home = True
                    else:
                        blocked_was_active = self._clear_blocked_state_locked()
                        should_advance = True
                        if blocked_was_active:
                            self._mission_status = self._status_with_note_locked("route active")
                elif status == int(GoalStatus.STATUS_CANCELED):
                    self._mission_active = False
                    self._active_chunk = []
                    self._action_active = False
                    self._action_until = None
                    self._clear_blocked_state_locked()
                    self._mission_status = self._status_with_note_locked("route cancelled")
                    should_stop = True
                    stop_reason = "cancelled"
                else:
                    reason_code, reason_text = self._blocking_reason_from_nav_telemetry_locked(msg)
                    if self._is_blocking_reason(reason_code):
                        should_enter_blocked = True
                        blocked_reason_code = reason_code
                        blocked_reason_text = reason_text
                    else:
                        self._mission_active = False
                        self._active_chunk = []
                        self._action_active = False
                        self._action_until = None
                        self._clear_blocked_state_locked()
                        self._mission_status = self._status_with_note_locked(
                            f"route failed: {str(msg.nav_result_text)}"
                        )
                        should_stop = True
                        stop_reason = str(msg.nav_result_text)

        if should_pause:
            self.get_logger().warning("Route mission paused by manual takeover")
            self._publish_empty_active_chunk_path()
            return
        if should_complete_return_home:
            self._restore_urban_navigation_profile()
            self._publish_route_event(
                DiagnosticStatus.OK,
                "RETURN_HOME_COMPLETED",
                "Return home completed",
                details={"reached_checkpoint_count": int(self._reached_checkpoint_count)},
            )
            self._publish_empty_route_paths()
            return
        if should_enter_blocked:
            self._enter_blocked_waiting(
                blocked_reason_code,
                blocked_reason_text,
                cancel_goal=blocked_cancel_goal,
                brake=True,
            )
            return
        if should_advance:
            self._start_next_chunk_after_success()
            return
        if should_stop:
            self._restore_urban_navigation_profile()
            event_code = (
                "ROUTE_MISSION_CANCELLED"
                if str(stop_reason).lower() == "cancelled"
                else "ROUTE_MISSION_FAILED"
            )
            self._publish_route_event(
                DiagnosticStatus.WARN if event_code.endswith("CANCELLED") else DiagnosticStatus.ERROR,
                event_code,
                "Route mission stopped by navigation result",
                details={"reason": str(stop_reason)},
            )
            self.get_logger().warning(f"Route mission stopped ({stop_reason})")
            self._publish_empty_route_paths()

    def _validate_generate_coverage_request(
        self,
        request: GenerateCoveragePlanLL.Request,
    ) -> Tuple[Optional[Dict[str, Any]], str]:
        # Importes deliberadamente locales: el proceso que atiende Ruta o
        # Patrol no debe cargar ningun modulo de Campo solo por arrancar.
        from navegacion_gps.coverage_field_polygon import validate_coverage_field
        from navegacion_gps.coverage_waypoint_core import headland_turn_length_m
        from navegacion_gps.coverage_waypoint_core import resolve_row_visit_order

        values = {
            "start_lat": float(request.start_lat),
            "start_lon": float(request.start_lon),
            "start_yaw_deg": float(request.start_yaw_deg),
            "start_is_field_corner": bool(request.start_is_field_corner),
            "field_length_m": float(request.field_length_m),
            "field_width_m": float(request.field_width_m),
            "cutter_width_m": float(request.cutter_width_m),
            "overlap_ratio": float(request.overlap_ratio),
            "min_turning_radius_m": float(request.min_turning_radius_m),
            "waypoint_spacing_m": float(request.waypoint_spacing_m),
            "side": str(request.side or "").strip().lower(),
        }

        # `values` se recorre entero convirtiendo a float mas abajo, asi que las
        # claves no numericas se declaran aca y se excluyen por nombre. La
        # geometria del lote se agrega despues de esa validacion.
        numeric_values = [
            float(value)
            for key, value in values.items()
            if key not in _NON_NUMERIC_COVERAGE_KEYS
        ]
        if not all(np.isfinite(value) for value in numeric_values):
            return None, "coverage parameters must be finite"
        if not -90.0 <= float(values["start_lat"]) <= 90.0:
            return None, "start_lat must be in [-90, 90]"
        if not -180.0 <= float(values["start_lon"]) <= 180.0:
            return None, "start_lon must be in [-180, 180]"
        if abs(float(values["start_yaw_deg"])) > 360.0:
            return None, "start_yaw_deg must be in [-360, 360]"

        max_dimension = float(self.coverage_max_field_dimension_m)
        for key in ("field_length_m", "field_width_m", "cutter_width_m"):
            value = float(values[key])
            if value <= 0.0 or value > max_dimension:
                return None, f"{key} must be > 0 and <= {max_dimension:g}"
        overlap = float(values["overlap_ratio"])
        if not 0.0 <= overlap < 1.0:
            return None, "overlap_ratio must be in [0, 1)"
        turning_radius = float(values["min_turning_radius_m"])
        if turning_radius <= 0.0 or turning_radius > float(
            self.coverage_max_turning_radius_m
        ):
            return (
                None,
                "min_turning_radius_m must be > 0 and <= "
                f"{float(self.coverage_max_turning_radius_m):g}",
            )
        planner_min_turning_radius = float(
            self.coverage_planner_min_turning_radius_m
        )
        if turning_radius + 1.0e-9 < planner_min_turning_radius:
            return (
                None,
                "min_turning_radius_m must be >= configured planner minimum "
                f"{planner_min_turning_radius:g}",
            )
        requested_waypoint_spacing = float(values["waypoint_spacing_m"])
        if requested_waypoint_spacing < 0.0:
            return None, "waypoint_spacing_m must be >= 0 (0 means automatic)"
        if values["side"] not in {"left", "right"}:
            return None, "side must be 'left' or 'right'"

        # El lote puede llegar como poligono o como el rectangulo de siempre.
        outer_ll, exclusions_ll, polygon_error = validate_coverage_field(
            [
                (float(vertex.lat), float(vertex.lon))
                for vertex in request.coverage_polygon.vertices
            ],
            [
                [(float(vertex.lat), float(vertex.lon)) for vertex in ring.vertices]
                for ring in request.coverage_exclusions
            ],
        )
        if polygon_error:
            return None, polygon_error
        values["coverage_polygon"] = outer_ll
        values["coverage_exclusions"] = exclusions_ll
        values["field_mode"] = "polygon" if outer_ll else "rectangle"

        # El cliente puede enviar 0 para pedir el valor adaptativo. La medida
        # sale del anillo cargado cuando existe; el rectangulo legacy conserva
        # field_length/field_width. Un override ROS positivo sigue disponible
        # para una instalacion que necesite fijar la densidad.
        geometry_length_m, geometry_width_m = _coverage_polygon_dimensions_m(
            outer_ll,
            float(values["field_length_m"]),
            float(values["field_width_m"]),
        )
        spacing_override_m = max(
            0.0,
            float(getattr(self, "coverage_waypoint_spacing_override_m", 0.0)),
        )
        if requested_waypoint_spacing <= 0.0:
            waypoint_spacing = (
                spacing_override_m
                if spacing_override_m > 0.0
                else coverage_auto_waypoint_spacing_m(
                    geometry_length_m,
                    geometry_width_m,
                    ratio=float(
                        getattr(
                            self,
                            "coverage_auto_waypoint_spacing_ratio",
                            0.10,
                        )
                    ),
                    minimum_m=float(
                        getattr(
                            self,
                            "coverage_auto_waypoint_spacing_min_m",
                            max(0.5, float(self.coverage_min_waypoint_spacing_m)),
                        )
                    ),
                    maximum_m=float(
                        getattr(
                            self,
                            "coverage_auto_waypoint_spacing_max_m",
                            4.0,
                        )
                    ),
                )
            )
            values["waypoint_spacing_auto"] = True
        else:
            waypoint_spacing = requested_waypoint_spacing
            values["waypoint_spacing_auto"] = False
        if waypoint_spacing < float(self.coverage_min_waypoint_spacing_m):
            return (
                None,
                "waypoint_spacing_m must be >= "
                f"{float(self.coverage_min_waypoint_spacing_m):g}",
            )
        if waypoint_spacing > max_dimension:
            return None, f"waypoint_spacing_m must be <= {max_dimension:g}"
        values["waypoint_spacing_m"] = float(waypoint_spacing)
        values["waypoint_spacing_field_length_m"] = float(geometry_length_m)
        values["waypoint_spacing_field_width_m"] = float(geometry_width_m)
        if outer_ll and self.coverage_planner != "fields2cover":
            # Planificar el rectangulo cuando el operador mando un poligono
            # seria dibujar una cosa y trabajar otra. Mejor negarse.
            return (
                None,
                "el planner legacy no soporta poligonos; usa "
                "coverage_planner=fields2cover o envia el lote rectangular",
            )

        values["start_yaw_deg"] = (
            (float(values["start_yaw_deg"]) + 180.0) % 360.0
        ) - 180.0
        values["route_start_lat"] = float(values["start_lat"])
        values["route_start_lon"] = float(values["start_lon"])
        values["centerline_length_m"] = float(values["field_length_m"])

        field_width = float(values["field_width_m"])
        cutter_width = float(values["cutter_width_m"])
        if bool(values["start_is_field_corner"]):
            field_length = float(values["field_length_m"])
            if cutter_width >= field_length:
                return (
                    None,
                    "cutter_width_m must be smaller than field_length_m for "
                    "field-corner coverage",
                )
            if cutter_width > field_width:
                return (
                    None,
                    "cutter_width_m must be <= field_width_m for field-corner "
                    "coverage",
                )
            side_sign = 1.0 if values["side"] == "left" else -1.0
            north_m, east_m = body_relative_offsets_to_north_east(
                float(values["start_yaw_deg"]),
                0.5 * cutter_width,
                side_sign * 0.5 * cutter_width,
            )
            route_start_lat, route_start_lon = offset_lat_lon(
                float(values["start_lat"]),
                float(values["start_lon"]),
                north_m=north_m,
                east_m=east_m,
            )
            if not (
                -90.0 <= route_start_lat <= 90.0
                and -180.0 <= route_start_lon <= 180.0
            ):
                return None, "field-corner inset leaves valid latitude/longitude range"
            values["route_start_lat"] = float(route_start_lat)
            values["route_start_lon"] = float(route_start_lon)
            values["centerline_length_m"] = field_length - cutter_width

        if values["field_mode"] == "polygon":
            # De aca en adelante todo el preflight deriva de la geometria
            # rectangular y del zigzag propio: cuantas pasadas entran en el
            # ancho, cuantos waypoints muestrea, en que orden se recorren. Con
            # un poligono esos numeros no describen el plan que se va a generar
            # —F2C decide las pasadas sobre la forma real— y aplicarlos rechaza
            # lotes perfectamente validos con cuentas que no corresponden.
            #
            # Los limites de verdad no se pierden: se verifican sobre el plan
            # que devuelve Fields2Cover, en _fill_fields2cover_response, contra
            # los mismos maximos de waypoints y de metas key.
            values["preflight_row_count"] = 0
            values["sampled_upper_bound"] = 0
            values["topology_audit_spacing_m"] = float(
                self.coverage_topology_audit_spacing_m
            )
            return values, ""

        centerline_span = max(0.0, field_width - cutter_width)
        if centerline_span <= 1.0e-9:
            row_count = 1
            lane_spacing = 0.0
        else:
            max_lane_spacing = cutter_width * (1.0 - overlap)
            row_count = int(math.ceil(centerline_span / max_lane_spacing)) + 1
            lane_spacing = centerline_span / float(row_count - 1)

        key_waypoint_count = 2 * int(row_count)
        if row_count > int(self.coverage_max_rows):
            return (
                None,
                f"coverage row_count {row_count} exceeds limit {self.coverage_max_rows}",
            )
        if key_waypoint_count > int(self.coverage_max_key_waypoints):
            return (
                None,
                "coverage key waypoint count "
                f"{key_waypoint_count} exceeds limit {self.coverage_max_key_waypoints}",
            )

        visit_order = resolve_row_visit_order(
            row_count=int(row_count),
            lane_spacing_m=float(lane_spacing),
            min_turning_radius_m=turning_radius,
            field_length_m=float(values["centerline_length_m"]),
            cutter_width_m=(
                cutter_width
                if self.coverage_allow_headland_conflicts
                else None
            ),
            allow_row_skipping=self.coverage_allow_row_skipping,
        )
        turn_separations = [
            abs(int(visit_order[index + 1]) - int(visit_order[index]))
            * float(lane_spacing)
            for index in range(len(visit_order) - 1)
        ]
        topology_audit_spacing_m = min(
            waypoint_spacing,
            float(self.coverage_topology_audit_spacing_m),
        )

        def turn_point_bound(spacing_m: float) -> int:
            return sum(
                int(
                    math.ceil(
                        headland_turn_length_m(
                            separation,
                            min_turning_radius_m=turning_radius,
                        )
                        / spacing_m
                    )
                )
                + 3
                for separation in turn_separations
            )

        # Dos poligonales con muestreos distintos: la de preview parte las pasadas
        # al detalle pedido y la de auditoria las toma enteras, porque una recta se
        # cruza o no con otra figura sin importar en cuantos puntos se la parta. El
        # tope mira la mas grande de las dos.
        preview_row_step_count = max(
            1,
            int(math.ceil(float(values["centerline_length_m"]) / waypoint_spacing)),
        )
        preview_upper_bound = (
            1 + int(row_count) * preview_row_step_count + turn_point_bound(waypoint_spacing)
        )
        audit_upper_bound = (
            1 + int(row_count) + turn_point_bound(topology_audit_spacing_m)
        )
        sampled_upper_bound = max(preview_upper_bound, audit_upper_bound)
        if sampled_upper_bound > int(self.coverage_max_sampled_waypoints):
            hint = (
                "increase waypoint_spacing_m"
                if preview_upper_bound >= audit_upper_bound
                else "reduce the field size"
            )
            return (
                None,
                "coverage sampled waypoint upper bound "
                f"{sampled_upper_bound} exceeds limit "
                f"{self.coverage_max_sampled_waypoints} "
                f"(preview={preview_upper_bound}, audit={audit_upper_bound}; "
                f"{hint})",
            )

        values["preflight_row_count"] = int(row_count)
        values["sampled_upper_bound"] = int(sampled_upper_bound)
        values["topology_audit_spacing_m"] = float(topology_audit_spacing_m)
        return values, ""

    def _refresh_no_go_zones(self) -> None:
        """Pedirle el estado de zonas a `zones_manager` sin bloquear el nodo."""
        client = self._zones_get_state_client
        if client is None or self._zones_request_pending:
            return
        if not client.service_is_ready():
            self._zones_note = "zones service unavailable"
            return
        self._zones_request_pending = True
        future = client.call_async(GetZonesState.Request())
        future.add_done_callback(self._on_zones_state_response)

    def _on_zones_state_response(self, future: Any) -> None:
        """Guardar el GeoJSON que devolvio `zones_manager`."""
        self._zones_request_pending = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - depende del middleware
            self._zones_note = f"zones call failed: {exc}"
            return
        if response is None:
            self._zones_note = "empty zones state response"
            return
        if not bool(getattr(response, "ok", False)):
            reason = str(getattr(response, "error", "") or "unknown error")
            self._zones_note = f"zones service error: {reason}"
            return
        self._zones_geojson = str(getattr(response, "geojson", ""))
        self._zones_updated_at = time.monotonic()
        self._zones_note = ""

    def _coverage_no_go_polygons(
        self,
        values: Dict[str, Any],
    ) -> Tuple[List[List[Tuple[float, float]]], float, str]:
        """Zonas no-go en marco del lote, con el margen y una nota de estado.

        Sale del cache que mantiene `_refresh_no_go_zones`; no hay ninguna
        llamada de red aca adentro. La nota queda vacia cuando el cache esta
        fresco. Si `zones_manager` no responde se planifica igual, sin recorte:
        dejar al operador sin cobertura porque un nodo auxiliar esta caido seria
        peor. Lo que no se hace es ocultarlo, por eso la nota viaja hasta la
        respuesta del preview y el cockpit puede bloquear el arranque.
        """
        from navegacion_gps.coverage_nogo_zones import polygons_from_geojson

        if not self.coverage_nogo_enabled:
            return [], 0.0, ""

        # Tres terminos: medio implemento (la ruta va por el centro), un colchon
        # fijo, y el radio de giro porque el vehiculo maniobra contra el contorno
        # y no puede doblar en escuadra.
        margin_m = (
            (0.5 * float(values["cutter_width_m"]))
            + float(self.coverage_nogo_extra_margin_m)
            + (
                float(self.coverage_nogo_turning_margin_ratio)
                * float(values["min_turning_radius_m"])
            )
        )
        age_s = time.monotonic() - self._zones_updated_at
        if self._zones_updated_at <= 0.0 or age_s > self.zones_max_age_s:
            note = self._zones_note or f"zones state is stale ({age_s:.1f}s)"
            self.get_logger().warning(
                f"coverage planned without no-go zones: {note}"
            )
            return [], margin_m, note

        try:
            polygons = polygons_from_geojson(
                self._zones_geojson,
                origin_lat=float(values["route_start_lat"]),
                origin_lon=float(values["route_start_lon"]),
                origin_yaw_deg=float(values["start_yaw_deg"]),
            )
        except (TypeError, ValueError) as exc:
            note = f"invalid zones geojson: {exc}"
            self.get_logger().warning(
                f"coverage planned without no-go zones: {note}"
            )
            return [], margin_m, note
        return polygons, margin_m, ""

    @staticmethod
    def _body_lengths_by_phase(waypoints: Sequence[Any]) -> Tuple[float, float]:
        """Largo de trabajo y de transiciones de la poligonal, en metros.

        Los enlaces entre una pasada y el primer/ultimo punto del rodeo siguen
        siendo trabajo: esos puntos caen justo en el borde de la zona. Solo el
        contorno ``nogo_detour`` es transicion. Sin esa excepcion, una pasada
        muestreada solo en sus extremos quedaba con cero metros de trabajo al
        cruzar una zona central.
        """
        trabajo = 0.0
        transicion = 0.0
        anterior = None
        for actual in waypoints:
            if anterior is not None:
                largo = math.hypot(
                    float(actual.forward_m) - float(anterior.forward_m),
                    float(actual.left_m) - float(anterior.left_m),
                )
                fases = {str(anterior.phase), str(actual.phase)}
                mismo_tramo = (
                    int(anterior.row_index) == int(actual.row_index)
                    and WORK_PHASE in fases
                    and fases <= {WORK_PHASE, "nogo_detour"}
                )
                if mismo_tramo:
                    trabajo += largo
                else:
                    transicion += largo
            anterior = actual
        return trabajo, transicion

    def _coverage_warmup(self) -> None:
        """Dejar listo el planificador agricola antes del primer pedido.

        Una sola vez. El Coverage Server viene del lanzamiento sin configurar y
        activarlo tarda mas que el timeout del cockpit: si se hace en el primer
        preview, ese preview falla y el siguiente anda, que es la peor forma de
        andar.
        """
        if self._coverage_warmup_timer is not None:
            self._coverage_warmup_timer.cancel()
            self._coverage_warmup_timer = None
        try:
            self._fields2cover_planner()
        except Exception as exc:  # pragma: no cover - depende del entorno
            # Que falte el overlay no puede impedir que ande el nodo: ruta
            # automatica y patrulla no dependen de cobertura.
            self.get_logger().warning(
                f"coverage_planner=fields2cover no disponible: {exc}"
            )

    def _fields2cover_planner(self) -> Any:
        """Cliente del Coverage Server, creado al primer uso."""
        if self._fields2cover is None:
            from navegacion_gps.coverage_fields2cover import Fields2CoverPlanner

            self._fields2cover = Fields2CoverPlanner(
                action_name=self.coverage_f2c_action,
                parameter_service=self.coverage_f2c_parameter_service,
                change_state_service=self.coverage_f2c_change_state_service,
                logger=self.get_logger(),
            )
        return self._fields2cover

    def _generate_coverage_plan_fields2cover(
        self,
        values: Dict[str, Any],
        response: GenerateCoveragePlanLL.Response,
    ) -> GenerateCoveragePlanLL.Response:
        """Planificar Campo con Fields2Cover.

        Exige poligono: caer al rectangulo seria planificar una cosa distinta de
        la que dibujo el operador. Cualquier falla del Coverage Server sale como
        respuesta con ok=False; nada escapa de aca, porque una excepcion en un
        callback de rclpy mata el nodo y se llevaria puestas la ruta y la
        patrulla, que no tienen nada que ver con cobertura.
        """
        from navegacion_gps.coverage_fields2cover import Fields2CoverError
        from navegacion_gps.coverage_fields2cover import (
            ForwardOnlyTurnError,
            reorder_internal_nogo_swaths,
            replace_turns_with_flexible_headlands,
        )
        from navegacion_gps.coverage_nogo import (
            clip_plan_to_nogo,
            inflate_polygon,
            internal_nogo_exclusions,
            mark_required_nogo_transition_guides,
            plan_nogo_conflicts,
            polygon_is_strictly_inside_field,
        )
        from navegacion_gps.coverage_nogo_zones import ll_to_body

        polygon_ll = values.get("coverage_polygon")
        if not polygon_ll:
            response.ok = False
            response.error = (
                "coverage_planner=fields2cover necesita un poligono; "
                "el lote rectangular solo lo atiende el planner legacy"
            )
            return response

        origin_lat = float(values["route_start_lat"])
        origin_lon = float(values["route_start_lon"])
        origin_yaw = float(values["start_yaw_deg"])

        def a_cuerpo(ring):
            return [
                ll_to_body(
                    float(lat),
                    float(lon),
                    origin_lat=origin_lat,
                    origin_lon=origin_lon,
                    origin_yaw_deg=origin_yaw,
                )
                for lat, lon in ring
            ]

        cuerpo = a_cuerpo(polygon_ll)
        bounds = (
            min(punto[0] for punto in cuerpo),
            max(punto[0] for punto in cuerpo),
            min(punto[1] for punto in cuerpo),
            max(punto[1] for punto in cuerpo),
        )
        (
            no_go_polygons,
            no_go_margin_m,
            no_go_note,
        ) = self._coverage_no_go_polygons(values)

        # Las zonas interiores se entregan ANTES de planificar como anillos del
        # lote. Fields2Cover parte los swaths y BOUSTROPHEDON cambia a la
        # pasada
        # contigua, en vez de construir una diagonal por el obstaculo y
        # parchearla despues. Se usa el margen operativo ya calculado, sin
        # volver a sumar el radio completo: esa inflacion fue medida antes y
        # hacia que el hueco tapara filas que nunca tocaban la zona, provocando
        # el rulo que justamente se busca evitar. La anticipacion sale de darle
        # el agujero al planner antes de generar las filas, no de agrandarlo.
        # Si la envolvente toca el borde deja de ser un agujero interior y queda
        # para el rodeo exterior del recorte posterior.
        internal_no_go_exclusions = internal_nogo_exclusions(
            no_go_polygons,
            safety_margin_m=no_go_margin_m,
            turn_anticipation_m=0.0,
            bounds=bounds,
            field_boundary=cuerpo,
        )
        internal_no_go_polygons = []
        boundary_no_go_polygons = []
        for polygon in no_go_polygons:
            inflated = inflate_polygon(polygon, no_go_margin_m)
            destination = (
                internal_no_go_polygons
                if polygon_is_strictly_inside_field(
                    inflated,
                    bounds=bounds,
                    field_boundary=cuerpo,
                )
                else boundary_no_go_polygons
            )
            destination.append(polygon)

        # Fresco en cada pedido, como el resto de los parametros de forma de
        # giro: es un valor que se tunea probando. Es un tramo lineal, no un
        # multiplo del radio; con 0.5 m y R=4 el pivote queda 4.5 m afuera.
        margen_cabecera_m = max(
            0.0, float(self.get_parameter("coverage_f2c_turn_lead_m").value)
        )
        permite_reversa = bool(
            self.get_parameter("coverage_f2c_allow_reverse").value
        )

        try:
            plan = self._fields2cover_planner().plan(
                polygon_body=cuerpo,
                exclusions_body=[
                    a_cuerpo(ring) for ring in values.get("coverage_exclusions", [])
                ]
                + internal_no_go_exclusions,
                cutter_width_m=float(values["cutter_width_m"]),
                robot_width_m=self.coverage_f2c_robot_width_m,
                overlap_ratio=float(values["overlap_ratio"]),
                min_turning_radius_m=float(values["min_turning_radius_m"]),
                waypoint_spacing_m=float(values["waypoint_spacing_m"]),
                swath_angle_deg=self.coverage_f2c_swath_angle_deg,
                # Se leen frescos en cada pedido: son los que se tocan para
                # tunear la forma de los giros, y tener que reiniciar el nodo
                # para probar un valor hace que no se prueben.
                route_type=str(self.get_parameter("coverage_f2c_route_type").value),
                path_type=str(self.get_parameter("coverage_f2c_path_type").value),
                path_continuity=str(
                    self.get_parameter("coverage_f2c_path_continuity").value
                ),
                # El paso de los giros escala con el detalle que pidio el
                # operador. Fijo en 0.5 m, un lote grande con detalle grueso
                # gasta casi todo el presupuesto de waypoints en cabeceras: 2525
                # de los cuales solo ~400 eran de trabajo. El parametro queda
                # como piso, asi que el caso tipico no cambia.
                turn_point_distance_m=max(
                    self.coverage_f2c_turn_point_distance_m,
                    float(values["waypoint_spacing_m"]) / 4.0,
                ),
                headland_width_m=self.coverage_f2c_headland_width_m,
                server_timeout_s=self.coverage_f2c_timeout_s,
            )
            # Antes del recorte de zonas: lo que se recorta tiene que ser el
            # plan que se va a ejecutar, no los arcos que este paso descarta.
            # Dentro del mismo try porque nada de Campo puede escapar de la
            # callback.
            if bool(self.get_parameter("coverage_f2c_flexible_turns").value):
                plan = reorder_internal_nogo_swaths(
                    plan,
                    internal_no_go_exclusions,
                    field_boundary=cuerpo,
                    min_turning_radius_m=float(values["min_turning_radius_m"]),
                    lane_change_radius_m=float(
                        self.get_parameter(
                            "coverage_nogo_lane_change_radius_m"
                        ).value
                    ),
                    lane_change_min_radius_m=float(
                        self.get_parameter(
                            "coverage_nogo_lane_change_min_radius_m"
                        ).value
                    ),
                    internal_strategy=str(
                        self.get_parameter(
                            "coverage_nogo_internal_strategy"
                        ).value
                    ),
                )
                plan = replace_turns_with_flexible_headlands(
                    plan,
                    margin_m=margen_cabecera_m,
                    # Con el radio, la cabecera arma el giro de tres puntos
                    # cuando las pasadas quedan mas juntas que el diametro de
                    # giro. Sin el, se queda en la transicion recta de siempre.
                    min_turning_radius_m=float(values["min_turning_radius_m"]),
                    # Con la reversa apagada la cabecera de tres puntos ni se
                    # evalua: la separacion corta se resuelve con la omega.
                    allow_reverse=permite_reversa,
                    # En un extremo creado por un agujero, la omega debe abrir
                    # sobre la parte ya libre del swath y no hacia la zona.
                    avoid_polygons=internal_no_go_exclusions,
                )
        except ForwardOnlyTurnError as exc:
            # No se degrada a marcha atras en silencio: el perfil pidio
            # forward-only y el enlace no existe hacia adelante.
            self.get_logger().warning(f"coverage forward-only turn failed: {exc}")
            response.ok = False
            response.error = f"cobertura forward-only: {exc}"
            return response
        except Fields2CoverError as exc:
            self.get_logger().warning(f"fields2cover coverage failed: {exc}")
            response.ok = False
            response.error = f"Fields2Cover: {exc}"
            return response
        except Exception as exc:  # pragma: no cover - depende del middleware
            self.get_logger().error(f"fields2cover unexpected failure: {exc}")
            response.ok = False
            response.error = f"Fields2Cover fallo inesperadamente: {exc}"
            return response

        nogo_dropped = 0
        nogo_detours = 0
        if no_go_polygons:
            # Segunda barrera sobre la trayectoria completa. Los agujeros
            # interiores ya dividieron las filas; aca se validan tambien los
            # giros. Las zonas de borde no se mandan como anillos invalidos a
            # Fields2Cover: este recorte las reconoce y elige el arco exterior.
            # El Cockpit repite la cuenta y detecta cualquier divergencia.
            try:
                # Una zona interna ya fue resuelta arriba como agujero y como
                # cambio de una sola fila. Nunca se la convierte despues en un
                # arco circular: eso haria que el vehiculo vuelva a circular
                # sobre cultivo. Se permite 5 cm de tolerancia exclusivamente
                # para la tangencia numerica entre una fila y la envolvente de
                # un circulo discretizado; una penetracion real sigue fallando.
                if internal_no_go_polygons:
                    internal_point_conflicts, internal_segment_conflicts = (
                        plan_nogo_conflicts(
                            plan.waypoints,
                            internal_no_go_polygons,
                            margin_m=max(0.0, no_go_margin_m - 0.05),
                        )
                    )
                    if internal_point_conflicts or internal_segment_conflicts:
                        conflict_segments = [
                            (
                                index,
                                str(plan.waypoints[index].phase),
                                round(float(plan.waypoints[index].forward_m), 2),
                                round(float(plan.waypoints[index].left_m), 2),
                                str(plan.waypoints[index + 1].phase),
                                round(float(plan.waypoints[index + 1].forward_m), 2),
                                round(float(plan.waypoints[index + 1].left_m), 2),
                            )
                            for index in internal_segment_conflicts[:8]
                        ]
                        raise ValueError(
                            "el cambio de fila interno invade la zona no-go "
                            f"(puntos={internal_point_conflicts}, "
                            f"tramos={conflict_segments})"
                        )
                recortados, clip_dropped, nogo_detours = clip_plan_to_nogo(
                    plan.waypoints,
                    boundary_no_go_polygons,
                    margin_m=no_go_margin_m,
                    bounds=bounds,
                    field_boundary=cuerpo,
                )
                nogo_dropped = int(clip_dropped) + int(
                    getattr(plan, "internal_nogo_dropped_waypoint_count", 0)
                )
            except ValueError as exc:
                response.ok = False
                response.error = f"zonas no-go: {exc}"
                return response
            except Exception as exc:  # pragma: no cover - red de seguridad
                # Nada de aca puede escapar: una excepcion en un callback de
                # servicio de rclpy mata el nodo, y con el se van la ruta
                # automatica y la patrulla, que no pidieron ninguna cobertura.
                self.get_logger().error(f"nogo clip failed: {exc}")
                response.ok = False
                response.error = f"zonas no-go: fallo inesperado ({exc})"
                return response
            recortados = mark_required_nogo_transition_guides(
                recortados,
                boundary_no_go_polygons,
                margin_m=no_go_margin_m,
            )
            # El largo lo calcula el plan sumando trabajo y transiciones, asi
            # que hay que rehacer los dos: sobran las pasadas que se borraron y
            # faltan los rodeos que se agregaron.
            trabajo_m, transicion_m = self._body_lengths_by_phase(recortados)
            plan = replace(
                plan,
                waypoints=recortados,
                work_length_m=trabajo_m,
                transition_length_m=transicion_m,
            )

        return self._fill_fields2cover_response(
            values,
            response,
            plan,
            nogo=(len(no_go_polygons), nogo_dropped, nogo_detours, no_go_note),
        )

    def _fill_fields2cover_response(
        self,
        values: Dict[str, Any],
        response: GenerateCoveragePlanLL.Response,
        plan: Any,
        *,
        nogo: Tuple[int, int, int, str] = (0, 0, 0, ""),
    ) -> GenerateCoveragePlanLL.Response:
        """Georreferenciar el plan y llenar la respuesta del servicio."""
        permite_reversa = bool(
            self.get_parameter("coverage_f2c_allow_reverse").value
        )
        geographic: List[Dict[str, object]] = []
        for point in plan.waypoints:
            north_m, east_m = body_relative_offsets_to_north_east(
                start_yaw_deg=float(values["start_yaw_deg"]),
                forward_m=float(point.forward_m),
                left_m=float(point.left_m),
            )
            lat, lon = offset_lat_lon(
                lat_deg=float(values["route_start_lat"]),
                lon_deg=float(values["route_start_lon"]),
                north_m=float(north_m),
                east_m=float(east_m),
            )
            geographic.append(
                {
                    "lat": float(lat),
                    "lon": float(lon),
                    "yaw_deg": float(
                        _normalize_yaw_deg(
                            float(values["start_yaw_deg"]) + float(point.yaw_delta_deg)
                        )
                    ),
                    "phase": str(point.phase),
                    "row_index": int(point.row_index),
                    "key": bool(point.is_key),
                    "guide": bool(getattr(point, "is_guide", False)),
                    "backup_m": float(getattr(point, "backup_m", 0.0)),
                }
            )

        if not permite_reversa:
            # Ultima barrera de la politica forward-only. La omega ya reemplazo
            # a la cabecera de tres puntos, asi que llegar aca con marcha atras
            # significa que algo la reintrodujo. Se falla con un error explicito
            # en vez de mandar una ruta que el perfil no puede ejecutar.
            con_reversa = [
                indice
                for indice, item in enumerate(geographic)
                if float(item.get("backup_m", 0.0)) > 0.0
            ]
            if con_reversa:
                response.ok = False
                response.error = (
                    "cobertura forward-only: el plan quedo con marcha atras en "
                    f"{len(con_reversa)} waypoints (primero el {con_reversa[0]}); "
                    "revisar coverage_f2c_allow_reverse y el radio de giro"
                )
                return response

        if len(geographic) > int(self.coverage_f2c_max_sampled_waypoints):
            # El detalle de preview es lo unico de esto que el operador tiene a
            # mano en el cockpit, asi que el aviso apunta ahi.
            response.ok = False
            response.error = (
                f"Fields2Cover devolvio {len(geographic)} waypoints, sobre el "
                f"limite de {self.coverage_f2c_max_sampled_waypoints}; subi el "
                "detalle de preview (waypoint_spacing_m) o achica el lote"
            )
            return response
        key_waypoints = [item for item in geographic if bool(item["key"])]
        if len(key_waypoints) > int(self.coverage_max_key_waypoints):
            # Las metas key son metas de parada reales, no dibujo: aca subir el
            # tope no es la salida, hay que pedir menos pasadas.
            response.ok = False
            response.error = (
                f"Fields2Cover devolvio {len(key_waypoints)} metas key, sobre "
                f"el limite de {self.coverage_max_key_waypoints}; achica el "
                "lote, subi el ancho de corte o baja el solape"
            )
            return response

        response.sampled_lats = [float(i["lat"]) for i in geographic]
        response.sampled_lons = [float(i["lon"]) for i in geographic]
        response.sampled_yaws_deg = [float(i["yaw_deg"]) for i in geographic]
        response.sampled_phases = [str(i["phase"]) for i in geographic]
        response.sampled_row_indices = [int(i["row_index"]) for i in geographic]
        response.sampled_key_flags = [bool(i["key"]) for i in geographic]
        response.key_lats = [float(i["lat"]) for i in key_waypoints]
        response.key_lons = [float(i["lon"]) for i in key_waypoints]
        response.key_yaws_deg = [float(i["yaw_deg"]) for i in key_waypoints]
        # La polilinea de preview incluye los arcos de Fields2Cover. Mandar
        # solo sus extremos hacia Nav2 los reemplazaba por diagonales, de modo
        # que el robot nunca recorria lo que el cockpit dibujaba. Los puntos
        # interiores de una pasada recta no hacen falta; las poses de giro si:
        # son guias no-key y conservan cada transicion planificada.
        route_waypoints = [
            item
            for item in geographic
            if bool(item["key"]) or bool(item["guide"])
        ]
        if len(route_waypoints) < 2:
            response.ok = False
            response.error = "Fields2Cover no devolvio una ruta ejecutable"
            return response

        # SetRouteMissionLL tiene un limite propio. Un preview de 43 m con
        # detalle de 0.5 m puede tener mas de 800 muestras solo en cabeceras;
        # enviarlas todas hace que el preview parezca correcto pero que START
        # falle despues con "waypoint count exceeds limit". Se conservan todas
        # las metas key y se eligen guias de giro uniformemente, en orden. El
        # cockpit dibuja esta misma lista, no el muestreo denso, asi que lo que
        # ve el operador es exactamente lo que recibe route_executor.
        route_limit = max(
            2,
            int(
                getattr(
                    self,
                    "max_route_input_waypoints",
                    self.coverage_max_key_waypoints,
                )
            ),
        )
        if len(route_waypoints) > route_limit:
            key_indices = [
                index
                for index, item in enumerate(route_waypoints)
                if bool(item["key"])
            ]
            if len(key_indices) > route_limit:
                response.ok = False
                response.error = (
                    f"Fields2Cover necesita {len(key_indices)} metas key, sobre el "
                    f"limite de ruta {route_limit}; achica el lote, subi el ancho "
                    "de corte o baja el solape"
                )
                return response
            # El vertice de la marcha atras es obligatorio: si se decima, la
            # cabecera de tres puntos queda a medias y el vehiculo intenta un
            # giro que su radio no permite. Cuenta como key para el presupuesto.
            key_indices = sorted(
                set(key_indices)
                | {
                    index
                    for index, item in enumerate(route_waypoints)
                    if float(item.get("backup_m", 0.0)) > 0.0
                }
            )
            guide_indices = [
                index
                for index, item in enumerate(route_waypoints)
                if index not in set(key_indices)
            ]
            guide_budget = route_limit - len(key_indices)
            selected_indices = set(key_indices)
            if guide_budget >= len(guide_indices):
                selected_indices.update(guide_indices)
            elif guide_budget == 1:
                selected_indices.add(guide_indices[len(guide_indices) // 2])
            elif guide_budget > 1:
                last = len(guide_indices) - 1
                selected_indices.update(
                    guide_indices[
                        round(position * last / (guide_budget - 1))
                    ]
                    for position in range(guide_budget)
                )
            route_waypoints = [
                item
                for index, item in enumerate(route_waypoints)
                if index in selected_indices
            ]
        response.route_lats = [float(i["lat"]) for i in route_waypoints]
        response.route_lons = [float(i["lon"]) for i in route_waypoints]
        response.route_yaws_deg = [float(i["yaw_deg"]) for i in route_waypoints]
        response.route_key_flags = [bool(i["key"]) for i in route_waypoints]
        response.route_phases = [str(i["phase"]) for i in route_waypoints]
        # Acciones por waypoint. Hoy la unica es la marcha atras de la cabecera
        # de tres puntos; el resto viaja vacio. Se manda siempre el arreglo
        # completo para que el indice coincida con el de los waypoints.
        response.route_action_jsons = [
            json.dumps(
                [
                    {
                        "type": "coverage_backup",
                        "distance_m": round(float(i["backup_m"]), 3),
                        "label": "cabecera 3 puntos",
                    }
                ],
                separators=(",", ":"),
            )
            if permite_reversa and float(i.get("backup_m", 0.0)) > 0.0
            else ""
            for i in route_waypoints
        ]

        response.row_count = int(plan.swath_count)
        response.lane_spacing_m = float(plan.lane_spacing_m)
        response.estimated_path_length_m = float(plan.total_length_m)
        response.field_mode = str(values["field_mode"])
        response.route_start_lat = float(values["route_start_lat"])
        response.route_start_lon = float(values["route_start_lon"])
        response.route_start_yaw_deg = float(values["start_yaw_deg"])
        response.centerline_length_m = float(values["centerline_length_m"])
        response.effective_waypoint_spacing_m = float(
            values.get("waypoint_spacing_m", 0.0)
        )
        # No se corre la auditoria topologica. La del planificador propio mide
        # autointersecciones de un zigzag con cabeceras Dubins y no aplica a una
        # trayectoria de Fields2Cover, que tiene otra estructura. Adaptarla a la
        # fuerza daria un numero sin significado.
        #
        # Con topology_scope="fields2cover", topology_safe=true significa
        # "Fields2Cover produjo la trayectoria sin errores" y nada mas. Los
        # contadores de conflictos quedan en cero porque no se midieron, no
        # porque se haya verificado que no hay.
        response.topology_safe = True
        response.topology_scope = "fields2cover"
        response.topology_audited = False
        response.planner_min_turning_radius_m = float(
            self.coverage_planner_min_turning_radius_m
        )
        # Los extremos de pasada ya son las referencias obligatorias. Partir
        # una fila de 40 m cada 5 m hacia que Nav2 finalizara y recalculara ocho
        # paths cortos, acumulando error de rumbo. Un espaciado mayor al largo
        # del lote deja cada fila como un unico tramo, sin tocar sus extremos.
        response.recommended_leg_spacing_m = coverage_full_row_leg_spacing_m(
            self.min_leg_spacing_m,
            float(values["field_length_m"]),
        )
        response.recommended_chunk_span_m = float(self.default_chunk_span_m)
        response.recommended_chunk_max_waypoints = int(
            self.default_chunk_max_waypoints
        )
        # Sin esto el cockpit ve nogo_polygon_count=0, repite el recorte por su
        # cuenta, encuentra diferencias y bloquea el arranque avisando que el
        # backend no aplico las zonas. Aca se aplicaron.
        response.nogo_polygon_count = int(nogo[0])
        response.nogo_dropped_count = int(nogo[1])
        response.nogo_detour_count = int(nogo[2])
        response.nogo_note = str(nogo[3])
        response.ok = True
        response.error = ""
        return response

    def _on_generate_coverage_plan_impl(
        self,
        request: GenerateCoveragePlanLL.Request,
        response: GenerateCoveragePlanLL.Response,
    ) -> GenerateCoveragePlanLL.Response:
        # El import del planificador propio tambien queda dentro del handler de
        # Campo: Ruta, Patrol y goals no cargan su geometria ni sus dependencias.
        from navegacion_gps.coverage_waypoint_core import build_lawnmower_waypoints

        values, error = self._validate_generate_coverage_request(request)
        if values is None:
            response.ok = False
            response.error = str(error)
            return response

        if self.coverage_planner == "fields2cover":
            # Ultimo cinturon. Cualquier excepcion que se escape de aca mata el
            # nodo entero —rclpy no atrapa nada dentro de un callback— y el
            # cockpit lo ve como "generate_coverage_plan_ll timeout", que no
            # dice nada de lo que paso. Un pedido de cobertura roto tiene que
            # volver como respuesta con el error, no llevarse puestas la ruta
            # automatica y la patrulla.
            try:
                return self._generate_coverage_plan_fields2cover(values, response)
            except Exception as exc:  # pragma: no cover - red de seguridad
                self.get_logger().error(f"coverage fields2cover crashed: {exc}")
                response.ok = False
                response.error = f"Fields2Cover fallo inesperadamente: {exc}"
                return response

        no_go_polygons, no_go_margin_m, no_go_note = self._coverage_no_go_polygons(
            values
        )

        try:
            plan, sampled_waypoints = build_lawnmower_waypoints(
                start_lat=float(values["route_start_lat"]),
                start_lon=float(values["route_start_lon"]),
                start_yaw_deg=float(values["start_yaw_deg"]),
                field_length_m=float(values["centerline_length_m"]),
                field_width_m=float(values["field_width_m"]),
                cutter_width_m=float(values["cutter_width_m"]),
                overlap_ratio=float(values["overlap_ratio"]),
                min_turning_radius_m=float(values["min_turning_radius_m"]),
                waypoint_spacing_m=float(values["waypoint_spacing_m"]),
                side=str(values["side"]),
                allow_headland_conflicts=self.coverage_allow_headland_conflicts,
                allow_row_skipping=self.coverage_allow_row_skipping,
                no_go_polygons_body=no_go_polygons,
                no_go_margin_m=no_go_margin_m,
            )
            audit_plan = plan
            audit_waypoints = sampled_waypoints
            if float(values["topology_audit_spacing_m"]) + 1.0e-9 < float(
                values["waypoint_spacing_m"]
            ):
                audit_plan, audit_waypoints = build_lawnmower_waypoints(
                    start_lat=float(values["route_start_lat"]),
                    start_lon=float(values["route_start_lon"]),
                    start_yaw_deg=float(values["start_yaw_deg"]),
                    field_length_m=float(values["centerline_length_m"]),
                    field_width_m=float(values["field_width_m"]),
                    cutter_width_m=float(values["cutter_width_m"]),
                    overlap_ratio=float(values["overlap_ratio"]),
                    min_turning_radius_m=float(values["min_turning_radius_m"]),
                    waypoint_spacing_m=float(values["topology_audit_spacing_m"]),
                    side=str(values["side"]),
                    allow_headland_conflicts=self.coverage_allow_headland_conflicts,
                    allow_row_skipping=self.coverage_allow_row_skipping,
                    # El detalle fino solo hace falta en las cabeceras: ahi vive
                    # la guia y ahi la poligonal recorta el arco. Las pasadas son
                    # rectas y entran enteras, que es lo que evita que el conteo
                    # crezca con el largo del lote.
                    row_waypoint_spacing_m=float(values["centerline_length_m"]),
                    no_go_polygons_body=no_go_polygons,
                    no_go_margin_m=no_go_margin_m,
                )
        except (ArithmeticError, RuntimeError, ValueError) as exc:
            response.ok = False
            response.error = f"coverage generation failed: {exc}"
            return response

        if int(plan.row_count) != int(values["preflight_row_count"]):
            response.ok = False
            response.error = "coverage row_count changed after preflight"
            return response
        if len(sampled_waypoints) > int(self.coverage_max_sampled_waypoints):
            response.ok = False
            response.error = (
                f"coverage sampled waypoint count {len(sampled_waypoints)} exceeds "
                f"limit {self.coverage_max_sampled_waypoints}"
            )
            return response
        if len(audit_waypoints) > int(self.coverage_max_sampled_waypoints):
            response.ok = False
            response.error = (
                f"coverage topology audit waypoint count {len(audit_waypoints)} "
                f"exceeds limit {self.coverage_max_sampled_waypoints}"
            )
            return response

        key_waypoints = [
            waypoint for waypoint in sampled_waypoints if bool(waypoint.get("key"))
        ]
        # Con zonas no-go la ruta ejecutable sale del mismo plan que se dibuja.
        # El plan de auditoria muestrea las cabeceras mas fino, y sobre un arco
        # esa poligonal corta la zona en puntos levemente distintos: el rodeo
        # ejecutado quedaria unos 30 cm corrido del dibujado. Es poco, pero es
        # exactamente la divergencia que este trabajo viene a eliminar. Los
        # contadores de topologia no se ven afectados: se calculan antes del
        # recorte, sobre el trazado nominal.
        route_source = audit_waypoints if not no_go_polygons else sampled_waypoints
        route_waypoints = [
            waypoint
            for waypoint in route_source
            if bool(waypoint.get("key")) or bool(waypoint.get("guide"))
        ]
        if len(key_waypoints) > int(self.coverage_max_key_waypoints):
            response.ok = False
            response.error = (
                f"coverage key waypoint count {len(key_waypoints)} exceeds "
                f"limit {self.coverage_max_key_waypoints}"
            )
            return response

        response.sampled_lats = [float(item["lat"]) for item in sampled_waypoints]
        response.sampled_lons = [float(item["lon"]) for item in sampled_waypoints]
        response.sampled_yaws_deg = [
            float(item["yaw_deg"]) for item in sampled_waypoints
        ]
        response.sampled_phases = [str(item["phase"]) for item in sampled_waypoints]
        response.sampled_row_indices = [
            int(item["row_index"]) for item in sampled_waypoints
        ]
        response.sampled_key_flags = [
            bool(item["key"]) for item in sampled_waypoints
        ]
        response.key_lats = [float(item["lat"]) for item in key_waypoints]
        response.key_lons = [float(item["lon"]) for item in key_waypoints]
        response.key_yaws_deg = [float(item["yaw_deg"]) for item in key_waypoints]
        response.route_lats = [float(item["lat"]) for item in route_waypoints]
        response.route_lons = [float(item["lon"]) for item in route_waypoints]
        response.route_yaws_deg = [
            float(item["yaw_deg"]) for item in route_waypoints
        ]
        response.route_key_flags = [
            bool(item["key"]) for item in route_waypoints
        ]
        response.route_phases = [str(item["phase"]) for item in route_waypoints]

        turn_count = len(audit_plan.turn_separations_m)
        omega_turn_count = max(0, turn_count - int(audit_plan.clean_uturn_count))
        strict_crossing_count = int(audit_plan.strict_crossing_count)
        nonadjacent_touch_count = int(audit_plan.nonadjacent_touch_count)
        collinear_overlap_count = int(audit_plan.collinear_overlap_count)
        topology_conflict_count = (
            strict_crossing_count
            + nonadjacent_touch_count
            + collinear_overlap_count
        )
        field_strict_crossing_count = int(
            audit_plan.field_strict_crossing_count
        )
        field_nonadjacent_touch_count = int(
            audit_plan.field_nonadjacent_touch_count
        )
        field_collinear_overlap_count = int(
            audit_plan.field_collinear_overlap_count
        )
        field_topology_conflict_count = (
            field_strict_crossing_count
            + field_nonadjacent_touch_count
            + field_collinear_overlap_count
        )
        response.row_count = int(plan.row_count)
        response.lane_spacing_m = float(plan.lane_spacing_m)
        response.row_visit_order = [int(index) for index in plan.row_visit_order]
        response.turn_separations_m = [
            float(value) for value in plan.turn_separations_m
        ]
        response.clean_uturn_count = int(audit_plan.clean_uturn_count)
        response.omega_turn_count = int(omega_turn_count)
        response.estimated_path_length_m = float(plan.estimated_path_length_m)
        response.headland_before_m = float(plan.headland_before_m)
        response.headland_after_m = float(plan.headland_after_m)
        response.lateral_overflow_m = float(plan.lateral_overflow_m)
        response.strict_crossing_count = strict_crossing_count
        response.nonadjacent_touch_count = nonadjacent_touch_count
        response.collinear_overlap_count = collinear_overlap_count
        response.topology_conflict_count = int(topology_conflict_count)
        response.field_strict_crossing_count = field_strict_crossing_count
        response.field_nonadjacent_touch_count = field_nonadjacent_touch_count
        response.field_collinear_overlap_count = field_collinear_overlap_count
        response.field_topology_conflict_count = int(
            field_topology_conflict_count
        )
        if self.coverage_allow_headland_conflicts:
            response.topology_safe = bool(field_topology_conflict_count == 0)
            response.topology_scope = "field_interior"
        else:
            response.topology_safe = bool(
                topology_conflict_count == 0 and omega_turn_count == 0
            )
            response.topology_scope = "global"
        response.topology_audited = True
        response.topology_audit_spacing_m = float(
            values["topology_audit_spacing_m"]
        )
        response.planner_min_turning_radius_m = float(
            self.coverage_planner_min_turning_radius_m
        )
        # Los contadores de topologia salen del plan sin recortar: el recorte
        # solo reemplaza la lista de waypoints y deja intactas las metricas que
        # calculo build_lawnmower_body_plan. Es lo que se quiere: un rodeo que
        # roza otra pasada es una desviacion deliberada, no un defecto del
        # trazado, y no tiene por que bloquear el arranque.
        response.nogo_polygon_count = int(plan.nogo_polygon_count)
        response.nogo_dropped_count = int(plan.nogo_dropped_count)
        response.nogo_detour_count = int(plan.nogo_detour_count)
        response.nogo_note = str(no_go_note)
        response.field_mode = str(values["field_mode"])
        response.route_start_lat = float(values["route_start_lat"])
        response.route_start_lon = float(values["route_start_lon"])
        response.route_start_yaw_deg = float(values["start_yaw_deg"])
        response.centerline_length_m = float(values["centerline_length_m"])
        response.effective_waypoint_spacing_m = float(
            values.get("waypoint_spacing_m", 0.0)
        )

        max_turn_separation = max(plan.turn_separations_m, default=0.0)
        response.recommended_leg_spacing_m = coverage_full_row_leg_spacing_m(
            self.min_leg_spacing_m,
            float(values["field_length_m"]),
            float(max_turn_separation),
        )
        response.recommended_chunk_span_m = float(
            max(self.min_chunk_span_m, 60.0)
        )
        response.recommended_chunk_max_waypoints = int(
            max(self.min_chunk_max_waypoints, 25)
        )
        response.ok = True
        response.error = ""
        return response

    def _on_generate_coverage_plan(
        self,
        request: GenerateCoveragePlanLL.Request,
        response: GenerateCoveragePlanLL.Response,
    ) -> GenerateCoveragePlanLL.Response:
        """Barrera final: una callback ROS nunca puede propagar una excepcion.

        La rama Fields2Cover ya traduce sus fallas para dar un diagnostico mas
        preciso. Esta envoltura cubre tambien validacion, planner propio y una
        regresion futura antes de que rclpy destruya el proceso que comparte
        Ruta, Patrol y Campo.
        """
        try:
            return self._on_generate_coverage_plan_impl(request, response)
        except Exception as exc:  # pragma: no cover - ultima barrera de rclpy
            self.get_logger().error(f"coverage callback crashed: {exc}")
            response.ok = False
            response.error = f"coverage generation failed unexpectedly: {exc}"
            return response

    def _validate_set_route_request(
        self, request: SetRouteMissionLL.Request
    ) -> Tuple[
        Optional[List[RouteWaypoint]],
        Optional[List[str]],
        Optional[List[str]],
        bool,
        float,
        float,
        int,
        str,
    ]:
        lats = [float(value) for value in request.lats]
        lons = [float(value) for value in request.lons]
        yaws = [float(value) for value in request.yaws_deg]
        if len(lats) == 0:
            return None, None, None, False, 0.0, 0.0, 0, "at least one waypoint is required"
        if len(lats) > int(self.max_route_input_waypoints):
            return (
                None,
                None,
                None,
                False,
                0.0,
                0.0,
                0,
                f"waypoint count exceeds limit {self.max_route_input_waypoints}",
            )
        if len(lats) != len(lons):
            return None, None, None, False, 0.0, 0.0, 0, "lats and lons must have the same length"
        if len(yaws) not in (0, len(lats)):
            return None, None, None, False, 0.0, 0.0, 0, "yaws_deg must be empty or match lats length"
        for idx, (lat, lon) in enumerate(zip(lats, lons)):
            if (not np.isfinite(lat)) or (not np.isfinite(lon)):
                return None, None, None, False, 0.0, 0.0, 0, f"invalid waypoint values at index {idx}"
        for idx, yaw_deg in enumerate(yaws):
            if not np.isfinite(yaw_deg):
                return None, None, None, False, 0.0, 0.0, 0, f"invalid yaw_deg at index {idx}"

        action_jsons, actions_error = _parse_route_action_jsons(
            list(getattr(request, "waypoint_action_jsons", [])),
            len(lats),
        )
        if action_jsons is None:
            return None, None, None, False, 0.0, 0.0, 0, actions_error
        if not bool(self.get_parameter("coverage_f2c_allow_reverse").value):
            # Barrera de ADMISION de la politica forward-only: una ruta con
            # marcha atras no entra, venga del cockpit, de un cache viejo o de
            # otro backend. Rechazarla aca es mejor que aceptarla y frenar el
            # recorrido a mitad de camino cuando el behavior server la niega.
            con_reversa = [
                indice
                for indice, entrada in enumerate(action_jsons)
                if "coverage_backup" in str(entrada or "")
            ]
            if con_reversa:
                return None, None, None, False, 0.0, 0.0, 0, (
                    "perfil forward-only: la ruta trae coverage_backup en los "
                    f"waypoints {con_reversa}; coverage_f2c_allow_reverse=false"
                )
        waypoint_roles, roles_error = _parse_waypoint_roles(
            list(getattr(request, "waypoint_roles", [])),
            len(lats),
        )
        if waypoint_roles is None:
            return None, None, None, False, 0.0, 0.0, 0, roles_error

        leg_spacing_m = (
            float(request.leg_spacing_m)
            if np.isfinite(float(request.leg_spacing_m)) and float(request.leg_spacing_m) > 0.0
            else float(self.default_leg_spacing_m)
        )
        chunk_span_m = (
            float(request.chunk_span_m)
            if np.isfinite(float(request.chunk_span_m)) and float(request.chunk_span_m) > 0.0
            else float(self.default_chunk_span_m)
        )
        chunk_max_waypoints = int(request.chunk_max_waypoints) or int(self.default_chunk_max_waypoints)
        leg_spacing_m = max(float(self.min_leg_spacing_m), leg_spacing_m)
        chunk_span_m = max(float(self.min_chunk_span_m), chunk_span_m)
        chunk_max_waypoints = max(int(self.min_chunk_max_waypoints), chunk_max_waypoints)

        resolved = _resolve_input_waypoints(lats, lons, yaws, bool(request.loop))
        return (
            resolved,
            action_jsons,
            waypoint_roles,
            bool(request.loop),
            leg_spacing_m,
            chunk_span_m,
            chunk_max_waypoints,
            "",
        )

    def _validate_patrol_waypoints(
        self,
        *,
        lats: Sequence[float],
        lons: Sequence[float],
        yaws_deg: Sequence[float],
        action_jsons: Sequence[str],
        label: str,
        allow_empty: bool,
        loop: bool,
    ) -> Tuple[Optional[List[RouteWaypoint]], Optional[List[str]], str]:
        lat_values = [float(value) for value in lats]
        lon_values = [float(value) for value in lons]
        yaw_values = [float(value) for value in yaws_deg]
        if len(lat_values) != len(lon_values):
            return None, None, f"{label} lats and lons must have the same length"
        if len(yaw_values) not in (0, len(lat_values)):
            return None, None, f"{label} yaws_deg must be empty or match lats length"
        if not lat_values and allow_empty:
            return [], [], ""
        if not lat_values:
            return None, None, f"{label} must contain at least one waypoint"
        normalized_actions, actions_error = _parse_route_action_jsons(
            list(action_jsons),
            len(lat_values),
        )
        if normalized_actions is None:
            return None, None, actions_error
        resolved = _resolve_input_waypoints(lat_values, lon_values, yaw_values, loop)
        return resolved, normalized_actions, ""

    def _validate_set_patrol_request(
        self, request: SetPatrolMissionLL.Request
    ) -> Tuple[Optional[PatrolMissionProfile], str]:
        loop_waypoints, loop_actions, loop_error = self._validate_patrol_waypoints(
            lats=list(request.loop_lats),
            lons=list(request.loop_lons),
            yaws_deg=list(request.loop_yaws_deg),
            action_jsons=list(getattr(request, "loop_waypoint_action_jsons", [])),
            label="loop",
            allow_empty=False,
            loop=True,
        )
        if loop_waypoints is None or loop_actions is None:
            return None, loop_error
        if len(loop_waypoints) < 2:
            return None, "loop must contain at least two waypoints"

        return_waypoints, return_actions, return_error = self._validate_patrol_waypoints(
            lats=list(request.return_lats),
            lons=list(request.return_lons),
            yaws_deg=list(request.return_yaws_deg),
            action_jsons=list(getattr(request, "return_waypoint_action_jsons", [])),
            label="return connector",
            allow_empty=True,
            loop=False,
        )
        if return_waypoints is None or return_actions is None:
            return None, return_error

        depart_waypoints, depart_actions, depart_error = self._validate_patrol_waypoints(
            lats=list(request.depart_lats),
            lons=list(request.depart_lons),
            yaws_deg=list(request.depart_yaws_deg),
            action_jsons=list(getattr(request, "depart_waypoint_action_jsons", [])),
            label="depart connector",
            allow_empty=True,
            loop=False,
        )
        if depart_waypoints is None or depart_actions is None:
            return None, depart_error

        home_lat = float(request.home_lat)
        home_lon = float(request.home_lon)
        home_yaw_deg = float(request.home_yaw_deg)
        if not (
            np.isfinite(home_lat) and np.isfinite(home_lon) and np.isfinite(home_yaw_deg)
        ):
            return None, "home waypoint must contain finite lat/lon/yaw"

        depart_entry_loop_index = int(request.depart_entry_loop_index)
        if not (0 <= depart_entry_loop_index < len(loop_waypoints)):
            return None, "depart_entry_loop_index must point to a loop waypoint"

        leg_spacing_m = (
            float(request.leg_spacing_m)
            if np.isfinite(float(request.leg_spacing_m)) and float(request.leg_spacing_m) > 0.0
            else float(self.default_leg_spacing_m)
        )
        chunk_span_m = (
            float(request.chunk_span_m)
            if np.isfinite(float(request.chunk_span_m)) and float(request.chunk_span_m) > 0.0
            else float(self.default_chunk_span_m)
        )
        chunk_max_waypoints = int(request.chunk_max_waypoints) or int(self.default_chunk_max_waypoints)

        return (
            PatrolMissionProfile(
                loop_waypoints=list(loop_waypoints),
                loop_action_jsons=list(loop_actions),
                home_waypoint=RouteWaypoint(
                    lat=home_lat,
                    lon=home_lon,
                    yaw_deg=_normalize_yaw_deg(home_yaw_deg),
                ),
                return_waypoints=list(return_waypoints),
                return_action_jsons=list(return_actions),
                depart_waypoints=list(depart_waypoints),
                depart_action_jsons=list(depart_actions),
                depart_entry_loop_index=depart_entry_loop_index,
                leg_spacing_m=max(float(self.min_leg_spacing_m), leg_spacing_m),
                chunk_span_m=max(float(self.min_chunk_span_m), chunk_span_m),
                chunk_max_waypoints=max(int(self.min_chunk_max_waypoints), chunk_max_waypoints),
            ),
            "",
        )

    def _on_set_navigation_profile(
        self,
        request: SetNavigationProfile.Request,
        response: SetNavigationProfile.Response,
    ) -> SetNavigationProfile.Response:
        target = str(request.profile or "").strip().lower()
        with self._lock:
            mission_active = bool(self._mission_active or self._mission_paused or self._patrol_active)
            active_profile = str(self._active_navigation_profile)
        if target not in NAVIGATION_PROFILES:
            response.ok = False
            response.error = "profile must be 'urban' or 'rural'"
            response.active_profile = active_profile
            return response
        if mission_active:
            response.ok = False
            response.error = "navigation profile cannot be changed while a mission is active"
            response.active_profile = active_profile
            return response

        ok, error = self._apply_navigation_profile(target)
        with self._lock:
            response.active_profile = str(self._active_navigation_profile)
        response.ok = bool(ok)
        response.error = "" if ok else str(error)
        if ok:
            self._publish_route_event(
                DiagnosticStatus.OK,
                "NAVIGATION_PROFILE_APPLIED",
                "Navigation profile applied by operator",
                details={"profile": target, "source": "operator"},
            )
        return response

    def _on_set_route(
        self, request: SetRouteMissionLL.Request, response: SetRouteMissionLL.Response
    ) -> SetRouteMissionLL.Response:
        route_input, action_jsons, waypoint_roles, loop_enabled, leg_spacing_m, chunk_span_m, chunk_max_waypoints, error = (
            self._validate_set_route_request(request)
        )
        if route_input is None or action_jsons is None or waypoint_roles is None:
            response.ok = False
            response.error = error
            return response

        requested_key_flags = [
            bool(value) for value in getattr(request, "waypoint_key_flags", [])
        ]
        if len(requested_key_flags) not in (0, len(route_input)):
            response.ok = False
            response.error = "waypoint_key_flags must be empty or match lats length"
            return response
        route_key_flags = (
            requested_key_flags
            if requested_key_flags
            else [True for _ in route_input]
        )
        if route_key_flags and (not route_key_flags[0] or not route_key_flags[-1]):
            response.ok = False
            response.error = "first and last route waypoints must be key"
            return response

        non_home_indices = [
            index
            for index, role in enumerate(waypoint_roles)
            if str(role) != WAYPOINT_ROLE_HOME
        ]
        route_input, action_jsons, waypoint_roles, home_waypoint, split_error = _split_home_waypoint(
            route_input,
            action_jsons,
            waypoint_roles,
        )
        if split_error:
            response.ok = False
            response.error = split_error
            response.input_waypoint_count = 0
            response.expanded_waypoint_count = 0
            return response
        route_key_flags = [route_key_flags[index] for index in non_home_indices]

        route_input, action_jsons, dropped_loop_closure = drop_duplicate_loop_closure_with_actions(
            route_input,
            action_jsons,
            loop=loop_enabled,
            closure_tolerance_m=self.route_waypoint_reached_tolerance_m,
        )
        route_key_flags = route_key_flags[: len(route_input)]

        with self._lock:
            robot_pose = self._last_robot_pose
        prepared, prepare_error = prepare_route_waypoints(
            route_input,
            loop=loop_enabled,
            robot_lat=None if robot_pose is None else float(robot_pose[0]),
            robot_lon=None if robot_pose is None else float(robot_pose[1]),
            waypoint_reached_tolerance_m=self.route_waypoint_reached_tolerance_m,
            segment_start_tolerance_m=self.route_segment_start_tolerance_m,
            action_jsons=action_jsons,
            waypoint_roles=waypoint_roles,
            allow_segment_join=not bool(
                getattr(request, "start_from_first_waypoint", False)
            ),
            force_first_waypoint=bool(
                getattr(request, "start_from_first_waypoint", False)
            ),
        )
        if prepared is None:
            response.ok = False
            response.error = prepare_error
            response.input_waypoint_count = 0
            response.expanded_waypoint_count = 0
            return response

        prepared_source_indices = list(
            prepared.input_indices
            if prepared.input_indices is not None
            else range(len(prepared.waypoints))
        )
        prepared_key_flags = [
            bool(route_key_flags[index])
            for index in prepared_source_indices
            if 0 <= int(index) < len(route_key_flags)
        ]
        if len(prepared_key_flags) != len(prepared.waypoints):
            response.ok = False
            response.error = "waypoint_key_flags could not be mapped to prepared route"
            response.input_waypoint_count = 0
            response.expanded_waypoint_count = 0
            return response

        mission_note = str(prepared.note)
        if dropped_loop_closure:
            mission_note = (
                f"{mission_note}; dropped duplicate loop closure"
                if mission_note
                else "dropped duplicate loop closure"
            )

        expanded, expanded_action_jsons, expanded_key_flags = expand_route_waypoints_with_actions(
            prepared.waypoints,
            prepared.action_jsons,
            leg_spacing_m=leg_spacing_m,
            loop=loop_enabled,
            base_key_flags=prepared_key_flags,
        )
        expanded_roles = expand_route_waypoint_roles(
            prepared.waypoints,
            prepared.waypoint_roles,
            leg_spacing_m=leg_spacing_m,
            loop=loop_enabled,
        )
        if len(expanded_roles) != len(expanded):
            response.ok = False
            response.error = "waypoint roles could not be mapped to expanded route"
            response.input_waypoint_count = 0
            response.expanded_waypoint_count = 0
            return response

        with self._lock:
            had_mission = self._mission_active or self._mission_paused
        if had_mission:
            self._publish_route_event(
                DiagnosticStatus.WARN,
                "ROUTE_MISSION_CANCELLED",
                "Route mission replaced by a new mission",
                details={"reason": "superseded"},
            )
            self._cancel_nav_goal()

        mission_id = uuid.uuid4().hex[:12]
        with self._lock:
            self._route_input = list(prepared.waypoints)
            self._route_input_source_indices = list(
                prepared.input_indices
                if prepared.input_indices is not None
                else range(len(prepared.waypoints))
            )
            self._route_expanded = list(expanded)
            self._route_action_jsons = list(expanded_action_jsons)
            self._route_waypoint_roles = list(expanded_roles)
            self._route_key_waypoint_flags = list(expanded_key_flags)
            key_source_indices = [
                int(source_index)
                for source_index, is_key in zip(
                    prepared_source_indices, prepared_key_flags
                )
                if bool(is_key)
            ]
            self._route_input_indices = expanded_input_indices(
                expanded_key_flags,
                key_source_indices,
            )
            self._home_waypoint = (
                home_waypoint.waypoint if home_waypoint is not None else self._configured_home_waypoint()
            )
            self._home_input_index = int(home_waypoint.input_index) if home_waypoint is not None else -1
            self._active_chunk = []
            self._mission_id = mission_id
            self._chunk_id = 0
            self._loop_iteration = 0
            self._reached_checkpoint_count = 0
            self._mission_active = True
            self._mission_paused = False
            self._mission_loop = bool(loop_enabled)
            self._mission_follow_exact_path = any(
                str(role).strip().lower().startswith("coverage")
                for role in waypoint_roles
            )
            self._mission_note = mission_note
            self._mission_status = self._status_with_note_locked("route starting")
            self._leg_spacing_m = float(leg_spacing_m)
            self._chunk_span_m = float(chunk_span_m)
            self._chunk_max_waypoints = int(chunk_max_waypoints)
            self._current_start_index = 0
            self._current_target_index = 0
            self._awaiting_chunk_result = False
            self._action_active = False
            self._action_waypoint_index = 0
            self._action_type = ""
            self._action_until = None
            self._low_battery_active = False
            self._return_home_requested = False
            self._return_home_active = False
            self._return_home_exit_route_index = -1
            self._return_home_exit_input_index = -1
            self._last_handled_nav_result_event_id = self._last_nav_result_event_id
            self._clear_blocked_state_locked()

        self._publish_route_event(
            DiagnosticStatus.OK,
            "ROUTE_MISSION_STARTED",
            "Route mission started",
            details={
                "mission_id": mission_id,
                "input_waypoint_count": len(prepared.waypoints),
                "expanded_waypoint_count": len(expanded),
                "synthetic_waypoint_count": sum(not bool(flag) for flag in expanded_key_flags),
                "loop": int(loop_enabled),
                "leg_spacing_m": float(leg_spacing_m),
                "home_available": int(self._home_waypoint is not None),
            },
        )
        ok, err = self._send_chunk(start_index=0)
        response.input_waypoint_count = int(len(prepared.waypoints))
        response.expanded_waypoint_count = int(len(expanded))
        response.ok = bool(ok)
        response.error = "" if ok else str(err)
        if ok:
            self._publish_route_debug_path(
                self._mission_path_pub,
                expanded,
                label="mission",
            )
        else:
            self._publish_route_event(
                DiagnosticStatus.ERROR,
                "ROUTE_MISSION_FAILED",
                "Route mission failed to dispatch",
                details={"error": str(err)},
            )
            with self._lock:
                self._mission_active = False
                self._mission_paused = False
                self._active_chunk = []
                self._route_action_jsons = []
                self._route_waypoint_roles = []
                self._route_key_waypoint_flags = []
                self._route_input_indices = []
                self._mission_status = self._status_with_note_locked(f"route failed: {err}")
            self._publish_empty_route_paths()
        return response

    def _on_set_patrol(
        self, request: SetPatrolMissionLL.Request, response: SetPatrolMissionLL.Response
    ) -> SetPatrolMissionLL.Response:
        profile, error = self._validate_set_patrol_request(request)
        if profile is None:
            response.ok = False
            response.error = error
            response.loop_input_waypoint_count = 0
            response.loop_expanded_waypoint_count = 0
            return response

        mission_id = uuid.uuid4().hex[:12]
        with self._lock:
            self._set_patrol_state_locked(
                profile=profile,
                mission_id=mission_id,
                phase=PATROL_PHASE_IDLE,
                active=True,
                return_exit_loop_index=-1,
            )
            start_phase = (
                PATROL_PHASE_DEPART_HOME
                if self._patrol_needs_depart_from_home_locked()
                else PATROL_PHASE_LOOP_MAIN
            )

        ok, err, input_count, expanded_count = self._start_patrol_phase(start_phase)
        response.ok = bool(ok)
        response.error = "" if ok else str(err)
        response.loop_input_waypoint_count = int(input_count)
        response.loop_expanded_waypoint_count = int(expanded_count)
        if ok:
            self._publish_route_event(
                DiagnosticStatus.OK,
                "PATROL_MISSION_STARTED",
                "Patrol mission started",
                details={
                    "mission_id": mission_id,
                    "phase": str(start_phase),
                    "loop_waypoint_count": int(len(profile.loop_waypoints)),
                    "return_connector_waypoint_count": int(len(profile.return_waypoints)),
                    "depart_connector_waypoint_count": int(len(profile.depart_waypoints)),
                    "depart_entry_loop_index": int(profile.depart_entry_loop_index),
                },
            )
            return response

        with self._lock:
            self._reset_mission_locked(f"patrol failed: {err}")
        self._publish_route_event(
            DiagnosticStatus.ERROR,
            "PATROL_MISSION_FAILED",
            "Patrol mission failed to start",
            details={"error": str(err), "phase": str(start_phase)},
        )
        self._publish_empty_route_paths()
        return response

    def _on_cancel_patrol(
        self, _request: CancelPatrolMission.Request, response: CancelPatrolMission.Response
    ) -> CancelPatrolMission.Response:
        cancel_ok, cancel_err = self._cancel_nav_goal()
        with self._lock:
            self._reset_mission_locked("patrol cancelled")
            self._patrol_phase = PATROL_PHASE_IDLE
        self._publish_route_event(
            DiagnosticStatus.WARN,
            "PATROL_MISSION_CANCELLED",
            "Patrol mission cancelled",
            details={"cancel_error": str(cancel_err)},
        )
        self._publish_empty_route_paths()
        response.ok = bool(cancel_ok or cancel_err == "cancel_goal timeout")
        response.error = "" if response.ok else str(cancel_err)
        return response

    def _on_request_return_home(
        self, _request: RequestReturnHome.Request, response: RequestReturnHome.Response
    ) -> RequestReturnHome.Response:
        with self._lock:
            if self._patrol_mission_profile is None or not self._patrol_active:
                response.ok = False
                response.error = "no active patrol mission"
                return response
            if self._patrol_phase == PATROL_PHASE_RETURN_CONNECTOR:
                response.ok = True
                response.error = ""
                return response
            if self._patrol_phase != PATROL_PHASE_LOOP_MAIN:
                response.ok = False
                response.error = f"return home unavailable during phase {self._patrol_phase}"
                return response
            reference_waypoint = self._patrol_return_reference_waypoint_locked()
            exit_selection = _select_return_home_exit_waypoint(
                self._route_input,
                self._route_input_source_indices,
                reference_waypoint,
            )
            if exit_selection is None:
                response.ok = False
                response.error = "could not resolve patrol return exit waypoint"
                return response
            self._return_home_requested = True
            self._return_home_active = False
            self._return_home_exit_route_index = int(exit_selection.route_index)
            self._return_home_exit_input_index = int(exit_selection.input_index)
            self._patrol_phase = PATROL_PHASE_RETURN_PENDING
            self._patrol_return_exit_loop_index = int(exit_selection.input_index)
            self._mission_status = self._status_with_note_locked(
                "return home waiting for exit waypoint"
            )
        self._publish_route_event(
            DiagnosticStatus.WARN,
            "RETURN_HOME_REQUESTED",
            "Structured return home requested",
            details={"home_exit_input_index": int(exit_selection.input_index)},
        )
        response.ok = True
        response.error = ""
        return response

    def _on_cancel_route(
        self, _request: CancelRouteMission.Request, response: CancelRouteMission.Response
    ) -> CancelRouteMission.Response:
        cancel_ok, cancel_err = self._cancel_nav_goal()
        if getattr(self, "_coverage_goal_tolerance_applied_m", None) is not None:
            tolerance_ok, tolerance_err = self._set_coverage_goal_tolerance(
                self.route_waypoint_reached_tolerance_m
            )
            if not tolerance_ok:
                self.get_logger().warning(
                    "Could not restore strict goal tolerance while cancelling route: "
                    f"{tolerance_err}"
                )
        self._publish_route_event(
            DiagnosticStatus.WARN,
            "ROUTE_MISSION_CANCELLED",
            "Route mission cancelled",
            details={"reason": "operator", "cancel_error": str(cancel_err)},
        )
        with self._lock:
            self._reset_mission_locked("route cancelled")
        self._restore_urban_navigation_profile()
        self._publish_empty_route_paths()
        response.ok = bool(cancel_ok or cancel_err == "cancel_goal timeout")
        response.error = "" if response.ok else str(cancel_err)
        return response

    def _fill_route_state_response(
        self, response: GetRouteMissionState.Response
    ) -> GetRouteMissionState.Response:
        with self._lock:
            input_indices = list(self._route_input_indices)
            if len(input_indices) != len(self._route_key_waypoint_flags):
                input_indices = expanded_input_indices(self._route_key_waypoint_flags)
            progress = route_progress(
                self._route_expanded,
                start_index=self._current_start_index,
                target_index=self._current_target_index,
                loop=self._mission_loop,
                robot_lat=None if self._last_robot_pose is None else self._last_robot_pose[0],
                robot_lon=None if self._last_robot_pose is None else self._last_robot_pose[1],
            )
            response.ok = True
            response.error = ""
            response.active = bool(self._mission_active)
            response.paused = bool(self._mission_paused)
            response.loop = bool(self._mission_loop)
            response.low_battery_active = bool(self._low_battery_active)
            response.return_home_requested = bool(self._return_home_requested)
            response.return_home_active = bool(self._return_home_active)
            response.return_home_exit_waypoint_index = int(self._return_home_exit_input_index)
            response.return_home_phase = str(self._return_home_phase_locked())
            effective_home = self._effective_home_waypoint_locked()
            response.home_available = bool(effective_home is not None)
            response.mission_id = str(self._mission_id)
            response.chunk_id = int(self._chunk_id)
            response.loop_iteration = int(self._loop_iteration)
            response.reached_checkpoint_count = int(self._reached_checkpoint_count)
            response.input_waypoint_count = int(len(self._route_input))
            response.expanded_waypoint_count = int(len(self._route_expanded))
            response.current_start_index = int(self._current_start_index)
            response.current_target_index = int(self._current_target_index)
            response.active_chunk_size = int(len(self._active_chunk))
            response.leg_spacing_m = float(self._leg_spacing_m)
            response.chunk_span_m = float(self._chunk_span_m)
            response.chunk_max_waypoints = int(self._chunk_max_waypoints)
            response.status = str(self._blocked_status_locked())
            response.blocked_state = str(self._blocked_state)
            response.blocked_reason_code = str(self._blocked_reason_code)
            response.blocked_reason_text = str(self._blocked_reason_text)
            response.blocked_retry_attempt = int(self._blocked_retry_attempt)
            response.blocked_retry_max_attempts = int(self.blocked_retry_max_attempts)
            response.blocked_wait_remaining_s = float(self._blocked_wait_remaining_s_locked())
            response.action_active = bool(self._action_active)
            response.action_waypoint_index = int(self._action_waypoint_index)
            response.action_type = str(self._action_type)
            response.action_remaining_s = float(self._action_remaining_s_locked())
            response.current_checkpoint_index = (
                int(input_indices[self._current_target_index])
                if 0 <= self._current_target_index < len(input_indices)
                else -1
            )
            response.current_progress_expanded_index = (
                int(progress.expanded_index) if progress is not None else -1
            )
            response.current_progress_ratio = float(progress.ratio) if progress is not None else 0.0
            response.cross_track_error_m = (
                float(progress.cross_track_error_m) if progress is not None else float("nan")
            )
            response.distance_to_target_m = (
                float(progress.distance_to_target_m) if progress is not None else float("nan")
            )
            response.home_lat = float(effective_home.lat) if effective_home is not None else float("nan")
            response.home_lon = float(effective_home.lon) if effective_home is not None else float("nan")
            response.home_yaw_deg = (
                float(effective_home.yaw_deg) if effective_home is not None else float("nan")
            )
            response.mission_lats = [float(entry.lat) for entry in self._route_expanded]
            response.mission_lons = [float(entry.lon) for entry in self._route_expanded]
            response.mission_yaws_deg = [float(entry.yaw_deg) for entry in self._route_expanded]
            response.mission_action_jsons = [str(entry or "") for entry in self._route_action_jsons]
            response.mission_waypoint_roles = [
                str(entry or WAYPOINT_ROLE_NORMAL) for entry in self._route_waypoint_roles
            ]
            response.mission_key_flags = [bool(entry) for entry in self._route_key_waypoint_flags]
            response.mission_input_indices = [int(entry) for entry in input_indices]
            response.active_lats = [float(entry.lat) for entry in self._active_chunk]
            response.active_lons = [float(entry.lon) for entry in self._active_chunk]
            response.active_yaws_deg = [float(entry.yaw_deg) for entry in self._active_chunk]
        return response

    def _on_get_state(
        self, _request: GetRouteMissionState.Request, response: GetRouteMissionState.Response
    ) -> GetRouteMissionState.Response:
        return self._fill_route_state_response(response)

    def _on_get_patrol_state(
        self, _request: GetPatrolMissionState.Request, response: GetPatrolMissionState.Response
    ) -> GetPatrolMissionState.Response:
        with self._lock:
            profile = self._patrol_mission_profile
            response.ok = True
            response.error = ""
            response.active = bool(self._patrol_active)
            response.phase = str(self._patrol_phase)
            response.low_battery_active = bool(self._low_battery_active)
            response.return_home_requested = bool(self._return_home_requested)
            response.return_home_active = bool(
                self._return_home_active or self._patrol_phase == PATROL_PHASE_RETURN_CONNECTOR
            )
            response.return_exit_loop_index = int(self._patrol_return_exit_loop_index)
            response.depart_entry_loop_index = (
                int(profile.depart_entry_loop_index) if profile is not None else -1
            )
            response.home_available = bool(profile is not None)
            response.mission_id = str(self._patrol_mission_id)
            response.status = str(self._mission_status)
            if profile is None:
                return response
            response.home_lat = float(profile.home_waypoint.lat)
            response.home_lon = float(profile.home_waypoint.lon)
            response.home_yaw_deg = float(profile.home_waypoint.yaw_deg)
            response.loop_lats = [float(entry.lat) for entry in profile.loop_waypoints]
            response.loop_lons = [float(entry.lon) for entry in profile.loop_waypoints]
            response.loop_yaws_deg = [float(entry.yaw_deg) for entry in profile.loop_waypoints]
            response.loop_action_jsons = [str(entry or "") for entry in profile.loop_action_jsons]
            response.return_lats = [float(entry.lat) for entry in profile.return_waypoints]
            response.return_lons = [float(entry.lon) for entry in profile.return_waypoints]
            response.return_yaws_deg = [float(entry.yaw_deg) for entry in profile.return_waypoints]
            response.return_action_jsons = [
                str(entry or "") for entry in profile.return_action_jsons
            ]
            response.depart_lats = [float(entry.lat) for entry in profile.depart_waypoints]
            response.depart_lons = [float(entry.lon) for entry in profile.depart_waypoints]
            response.depart_yaws_deg = [float(entry.yaw_deg) for entry in profile.depart_waypoints]
            response.depart_action_jsons = [
                str(entry or "") for entry in profile.depart_action_jsons
            ]
            response.active_lats = [float(entry.lat) for entry in self._active_chunk]
            response.active_lons = [float(entry.lon) for entry in self._active_chunk]
            response.active_yaws_deg = [float(entry.yaw_deg) for entry in self._active_chunk]
        return response


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = RouteExecutorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
