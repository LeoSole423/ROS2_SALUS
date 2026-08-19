"""Generacion geometrica de recorridos de cobertura para vehiculos Ackermann."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Callable, Optional, Sequence, Tuple

from navegacion_gps.heading_math import normalize_yaw_deg
from navegacion_gps.nav_benchmarking import body_relative_offsets_to_north_east
from navegacion_gps.nav_benchmarking import offset_lat_lon
from navegacion_gps.coverage_nogo import clip_plan_to_nogo


_DUBINS_PATH_TYPES = ("LSL", "RSR", "LSR", "RSL", "RLR", "LRL")
_TOPOLOGY_EPSILON_M = 1.0e-9

# Un tramo Dubins mas corto que esto no es una maniobra: aparece cuando la
# solucion optima degenera (por ejemplo una RSL con recta nula, que en realidad
# es un RL puro). No se usa como cambio de curvatura para ubicar la guia.
_DEGENERATE_DUBINS_SEGMENT_M = 0.25

# Resolucion de la auditoria topologica. El muestreo de preview puede ser mucho
# mas grueso; auditar con esa poligonal recortaria las curvas y podria ocultar
# un cruce real, asi que la auditoria siempre remuestrea a este paso.
COVERAGE_TOPOLOGY_AUDIT_SPACING_M = 0.4


@dataclass(frozen=True)
class CoverageBodyWaypoint:
    """Waypoint expresado en el marco inicial del vehiculo."""

    forward_m: float
    left_m: float
    yaw_delta_deg: float
    phase: str
    row_index: int
    is_key: bool = False
    is_guide: bool = False


@dataclass(frozen=True)
class CoveragePlan:
    """Resultado y metricas de una planificacion de cobertura."""

    waypoints: Tuple[CoverageBodyWaypoint, ...]
    row_count: int
    lane_spacing_m: float
    cutter_width_m: float
    overlap_ratio: float
    min_turning_radius_m: float
    estimated_path_length_m: float
    headland_before_m: float
    headland_after_m: float
    lateral_overflow_m: float
    row_visit_order: Tuple[int, ...] = ()
    turn_separations_m: Tuple[float, ...] = ()
    clean_uturn_count: int = 0
    strict_crossing_count: int = 0
    nonadjacent_touch_count: int = 0
    collinear_overlap_count: int = 0
    field_strict_crossing_count: int = 0
    field_nonadjacent_touch_count: int = 0
    field_collinear_overlap_count: int = 0
    topology_audit_spacing_m: float = 0.0
    minimum_safe_lane_spacing_m: float = 0.0
    # Efecto de las zonas no-go. Quedan en cero cuando no se pasa ninguna, que es
    # el caso de toda la operacion previa a esta funcionalidad.
    nogo_polygon_count: int = 0
    nogo_dropped_count: int = 0
    nogo_detour_count: int = 0

    @property
    def key_waypoints(self) -> Tuple[CoverageBodyWaypoint, ...]:
        """Extremos de pasada: lo unico que conviene enviar al route_executor."""

        return tuple(point for point in self.waypoints if point.is_key)

    @property
    def route_waypoints(self) -> Tuple[CoverageBodyWaypoint, ...]:
        """Extremos key y una guia exterior no-key por cada cabecera."""

        return tuple(
            point for point in self.waypoints if point.is_key or point.is_guide
        )

    @property
    def topology_conflict_count(self) -> int:
        """Pares de segmentos no adyacentes que se cruzan, tocan o solapan."""

        return int(
            self.strict_crossing_count
            + self.nonadjacent_touch_count
            + self.collinear_overlap_count
        )

    @property
    def is_topologically_safe(self) -> bool:
        """Indicar si la poligonal nominal no se pisa en ningun lugar."""

        return self.topology_conflict_count == 0

    @property
    def field_topology_conflict_count(self) -> int:
        """Conflictos cuyo punto o tramo comun cae dentro del lote fisico."""

        return int(
            self.field_strict_crossing_count
            + self.field_nonadjacent_touch_count
            + self.field_collinear_overlap_count
        )

    @property
    def is_field_topologically_safe(self) -> bool:
        """Indicar si las lineas no se pisan dentro del lote fisico."""

        return self.field_topology_conflict_count == 0


@dataclass(frozen=True)
class PolylineTopologyMetrics:
    """Conflictos entre pares de segmentos no adyacentes de una poligonal abierta."""

    strict_crossing_count: int = 0
    nonadjacent_touch_count: int = 0
    collinear_overlap_count: int = 0

    @property
    def conflict_count(self) -> int:
        return int(
            self.strict_crossing_count
            + self.nonadjacent_touch_count
            + self.collinear_overlap_count
        )

    @property
    def is_safe(self) -> bool:
        return self.conflict_count == 0


@dataclass(frozen=True)
class RowOrderMetrics:
    """Lo que decide entre dos ordenes de pasada del mismo lote."""

    conflict_count: int
    headland_depth_m: float
    path_length_m: float
    minimum_separation_m: float

    @property
    def is_safe(self) -> bool:
        return int(self.conflict_count) == 0


@dataclass(frozen=True)
class _PolylineSegment:
    index: int
    start: Tuple[float, float]
    end: Tuple[float, float]
    length_m: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float


def _cross_product(
    first: Tuple[float, float],
    second: Tuple[float, float],
    third: Tuple[float, float],
) -> float:
    return float(
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def _orientation_sign(value: float, *, segment_length_m: float, epsilon_m: float) -> int:
    tolerance = float(epsilon_m) * float(segment_length_m)
    if value > tolerance:
        return 1
    if value < -tolerance:
        return -1
    return 0


def _point_on_segment(
    point: Tuple[float, float],
    segment: _PolylineSegment,
    *,
    epsilon_m: float,
) -> bool:
    if abs(_cross_product(segment.start, segment.end, point)) > (
        float(epsilon_m) * segment.length_m
    ):
        return False
    return bool(
        segment.min_x - epsilon_m <= point[0] <= segment.max_x + epsilon_m
        and segment.min_y - epsilon_m <= point[1] <= segment.max_y + epsilon_m
    )


def _segment_conflict_kind(
    first: _PolylineSegment,
    second: _PolylineSegment,
    *,
    epsilon_m: float,
) -> Optional[str]:
    if (
        first.max_x < second.min_x - epsilon_m
        or second.max_x < first.min_x - epsilon_m
        or first.max_y < second.min_y - epsilon_m
        or second.max_y < first.min_y - epsilon_m
    ):
        return None

    first_second_start = _cross_product(first.start, first.end, second.start)
    first_second_end = _cross_product(first.start, first.end, second.end)
    second_first_start = _cross_product(second.start, second.end, first.start)
    second_first_end = _cross_product(second.start, second.end, first.end)
    signs = (
        _orientation_sign(
            first_second_start,
            segment_length_m=first.length_m,
            epsilon_m=epsilon_m,
        ),
        _orientation_sign(
            first_second_end,
            segment_length_m=first.length_m,
            epsilon_m=epsilon_m,
        ),
        _orientation_sign(
            second_first_start,
            segment_length_m=second.length_m,
            epsilon_m=epsilon_m,
        ),
        _orientation_sign(
            second_first_end,
            segment_length_m=second.length_m,
            epsilon_m=epsilon_m,
        ),
    )

    if signs == (0, 0, 0, 0):
        direction_x = (first.end[0] - first.start[0]) / first.length_m
        direction_y = (first.end[1] - first.start[1]) / first.length_m

        def projected_distance(point: Tuple[float, float]) -> float:
            return float(
                (point[0] - first.start[0]) * direction_x
                + (point[1] - first.start[1]) * direction_y
            )

        second_start = projected_distance(second.start)
        second_end = projected_distance(second.end)
        overlap_start = max(0.0, min(second_start, second_end))
        overlap_end = min(first.length_m, max(second_start, second_end))
        if overlap_end - overlap_start > epsilon_m:
            return "collinear_overlap"
        if overlap_end >= overlap_start - epsilon_m:
            return "nonadjacent_touch"
        return None

    if signs[0] * signs[1] < 0 and signs[2] * signs[3] < 0:
        return "strict_crossing"

    if (
        (signs[0] == 0 and _point_on_segment(second.start, first, epsilon_m=epsilon_m))
        or (signs[1] == 0 and _point_on_segment(second.end, first, epsilon_m=epsilon_m))
        or (signs[2] == 0 and _point_on_segment(first.start, second, epsilon_m=epsilon_m))
        or (signs[3] == 0 and _point_on_segment(first.end, second, epsilon_m=epsilon_m))
    ):
        return "nonadjacent_touch"
    return None


def analyze_polyline_topology(
    points: Sequence[Tuple[float, float]],
    *,
    epsilon_m: float = _TOPOLOGY_EPSILON_M,
) -> PolylineTopologyMetrics:
    """Clasificar todos los conflictos de una poligonal abierta.

    Los conteos representan pares de segmentos, no puntos geometricos unicos.
    Las uniones entre segmentos consecutivos son validas y se excluyen. Los
    contactos entre segmentos no adyacentes y los solapes collineales se cuentan
    por separado de los cruces estrictos.
    """

    tolerance = float(epsilon_m)
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("epsilon_m debe ser finito y mayor que cero")

    clean_points: list[Tuple[float, float]] = []
    for raw_point in points:
        point = (float(raw_point[0]), float(raw_point[1]))
        if not math.isfinite(point[0]) or not math.isfinite(point[1]):
            raise ValueError("la poligonal contiene coordenadas no finitas")
        if clean_points and math.hypot(
            point[0] - clean_points[-1][0], point[1] - clean_points[-1][1]
        ) <= tolerance:
            continue
        clean_points.append(point)

    segments: list[_PolylineSegment] = []
    for index, (start, end) in enumerate(zip(clean_points, clean_points[1:])):
        length_m = math.hypot(end[0] - start[0], end[1] - start[1])
        if length_m <= tolerance:
            continue
        segments.append(
            _PolylineSegment(
                index=int(index),
                start=start,
                end=end,
                length_m=float(length_m),
                min_x=min(start[0], end[0]),
                max_x=max(start[0], end[0]),
                min_y=min(start[1], end[1]),
                max_y=max(start[1], end[1]),
            )
        )

    strict_crossings = 0
    nonadjacent_touches = 0
    collinear_overlaps = 0
    active: list[_PolylineSegment] = []
    for current in sorted(segments, key=lambda item: (item.min_x, item.min_y, item.index)):
        active = [
            item for item in active if item.max_x >= current.min_x - tolerance
        ]
        for previous in active:
            if abs(current.index - previous.index) <= 1:
                continue
            kind = _segment_conflict_kind(previous, current, epsilon_m=tolerance)
            if kind == "strict_crossing":
                strict_crossings += 1
            elif kind == "nonadjacent_touch":
                nonadjacent_touches += 1
            elif kind == "collinear_overlap":
                collinear_overlaps += 1
        active.append(current)

    return PolylineTopologyMetrics(
        strict_crossing_count=int(strict_crossings),
        nonadjacent_touch_count=int(nonadjacent_touches),
        collinear_overlap_count=int(collinear_overlaps),
    )


def _clip_segment_to_bounds(
    segment: _PolylineSegment,
    *,
    bounds_m: Tuple[float, float, float, float],
    boundary_epsilon_m: float,
) -> Optional[_PolylineSegment]:
    """Recortar un segmento al interior abierto de un rectangulo."""

    min_x, max_x, min_y, max_y = (float(value) for value in bounds_m)
    inset = max(0.0, float(boundary_epsilon_m))
    min_x += inset
    max_x -= inset
    min_y += inset
    max_y -= inset
    if min_x >= max_x or min_y >= max_y:
        raise ValueError("bounds_m no deja un interior valido")

    delta_x = segment.end[0] - segment.start[0]
    delta_y = segment.end[1] - segment.start[1]
    lower = 0.0
    upper = 1.0
    for direction, distance in (
        (-delta_x, segment.start[0] - min_x),
        (delta_x, max_x - segment.start[0]),
        (-delta_y, segment.start[1] - min_y),
        (delta_y, max_y - segment.start[1]),
    ):
        if abs(direction) <= _TOPOLOGY_EPSILON_M:
            if distance < 0.0:
                return None
            continue
        ratio = distance / direction
        if direction < 0.0:
            lower = max(lower, ratio)
        else:
            upper = min(upper, ratio)
        if lower > upper:
            return None

    start = (
        segment.start[0] + lower * delta_x,
        segment.start[1] + lower * delta_y,
    )
    end = (
        segment.start[0] + upper * delta_x,
        segment.start[1] + upper * delta_y,
    )
    length_m = math.hypot(end[0] - start[0], end[1] - start[1])
    if length_m <= _TOPOLOGY_EPSILON_M:
        return None
    return _PolylineSegment(
        index=int(segment.index),
        start=start,
        end=end,
        length_m=float(length_m),
        min_x=min(start[0], end[0]),
        max_x=max(start[0], end[0]),
        min_y=min(start[1], end[1]),
        max_y=max(start[1], end[1]),
    )


def analyze_polyline_topology_in_bounds(
    points: Sequence[Tuple[float, float]],
    *,
    bounds_m: Tuple[float, float, float, float],
    epsilon_m: float = _TOPOLOGY_EPSILON_M,
    boundary_epsilon_m: float = 1.0e-6,
) -> PolylineTopologyMetrics:
    """Contar solo conflictos presentes dentro de un rectangulo fisico.

    Los segmentos se recortan antes de auditarlos. De ese modo una cabecera
    puede cruzarse fuera del lote sin bloquear la cobertura, mientras cualquier
    cruce, contacto o solape que invade su interior sigue fallando cerrado.
    El borde exacto se considera cabecera y no interior.
    """

    min_x, max_x, min_y, max_y = (float(value) for value in bounds_m)
    if not all(math.isfinite(value) for value in (min_x, max_x, min_y, max_y)):
        raise ValueError("bounds_m debe contener valores finitos")
    if min_x >= max_x or min_y >= max_y:
        raise ValueError("bounds_m debe definir un rectangulo valido")

    tolerance = float(epsilon_m)
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("epsilon_m debe ser finito y mayor que cero")

    clean_points: list[Tuple[float, float]] = []
    for raw_point in points:
        point = (float(raw_point[0]), float(raw_point[1]))
        if not math.isfinite(point[0]) or not math.isfinite(point[1]):
            raise ValueError("la poligonal contiene coordenadas no finitas")
        if clean_points and math.hypot(
            point[0] - clean_points[-1][0], point[1] - clean_points[-1][1]
        ) <= tolerance:
            continue
        clean_points.append(point)

    clipped: list[_PolylineSegment] = []
    for index, (start, end) in enumerate(zip(clean_points, clean_points[1:])):
        length_m = math.hypot(end[0] - start[0], end[1] - start[1])
        if length_m <= tolerance:
            continue
        segment = _PolylineSegment(
            index=int(index),
            start=start,
            end=end,
            length_m=float(length_m),
            min_x=min(start[0], end[0]),
            max_x=max(start[0], end[0]),
            min_y=min(start[1], end[1]),
            max_y=max(start[1], end[1]),
        )
        clipped_segment = _clip_segment_to_bounds(
            segment,
            bounds_m=(min_x, max_x, min_y, max_y),
            boundary_epsilon_m=boundary_epsilon_m,
        )
        if clipped_segment is not None:
            clipped.append(clipped_segment)

    strict_crossings = 0
    nonadjacent_touches = 0
    collinear_overlaps = 0
    active: list[_PolylineSegment] = []
    for current in sorted(clipped, key=lambda item: (item.min_x, item.min_y, item.index)):
        active = [item for item in active if item.max_x >= current.min_x - tolerance]
        for previous in active:
            if abs(current.index - previous.index) <= 1:
                continue
            kind = _segment_conflict_kind(previous, current, epsilon_m=tolerance)
            if kind == "strict_crossing":
                strict_crossings += 1
            elif kind == "nonadjacent_touch":
                nonadjacent_touches += 1
            elif kind == "collinear_overlap":
                collinear_overlaps += 1
        active.append(current)

    return PolylineTopologyMetrics(
        strict_crossing_count=int(strict_crossings),
        nonadjacent_touch_count=int(nonadjacent_touches),
        collinear_overlap_count=int(collinear_overlaps),
    )


def _mod2pi(angle_rad: float) -> float:
    return float(angle_rad) % (2.0 * math.pi)


def _lsl(alpha: float, beta: float, distance: float) -> Optional[Tuple[float, ...]]:
    sin_alpha = math.sin(alpha)
    sin_beta = math.sin(beta)
    cos_alpha = math.cos(alpha)
    cos_beta = math.cos(beta)
    cos_alpha_beta = math.cos(alpha - beta)
    p_squared = (
        2.0
        + distance * distance
        - 2.0 * cos_alpha_beta
        + 2.0 * distance * (sin_alpha - sin_beta)
    )
    if p_squared < -1.0e-12:
        return None
    p = math.sqrt(max(0.0, p_squared))
    tmp = math.atan2(
        cos_beta - cos_alpha,
        distance + sin_alpha - sin_beta,
    )
    return _mod2pi(-alpha + tmp), p, _mod2pi(beta - tmp)


def _rsr(alpha: float, beta: float, distance: float) -> Optional[Tuple[float, ...]]:
    sin_alpha = math.sin(alpha)
    sin_beta = math.sin(beta)
    cos_alpha = math.cos(alpha)
    cos_beta = math.cos(beta)
    cos_alpha_beta = math.cos(alpha - beta)
    p_squared = (
        2.0
        + distance * distance
        - 2.0 * cos_alpha_beta
        + 2.0 * distance * (-sin_alpha + sin_beta)
    )
    if p_squared < -1.0e-12:
        return None
    p = math.sqrt(max(0.0, p_squared))
    tmp = math.atan2(
        cos_alpha - cos_beta,
        distance - sin_alpha + sin_beta,
    )
    return _mod2pi(alpha - tmp), p, _mod2pi(-beta + tmp)


def _lsr(alpha: float, beta: float, distance: float) -> Optional[Tuple[float, ...]]:
    sin_alpha = math.sin(alpha)
    sin_beta = math.sin(beta)
    cos_alpha = math.cos(alpha)
    cos_beta = math.cos(beta)
    cos_alpha_beta = math.cos(alpha - beta)
    p_squared = (
        -2.0
        + distance * distance
        + 2.0 * cos_alpha_beta
        + 2.0 * distance * (sin_alpha + sin_beta)
    )
    if p_squared < -1.0e-12:
        return None
    p = math.sqrt(max(0.0, p_squared))
    tmp = math.atan2(
        -cos_alpha - cos_beta,
        distance + sin_alpha + sin_beta,
    ) - math.atan2(-2.0, p)
    return _mod2pi(-alpha + tmp), p, _mod2pi(-beta + tmp)


def _rsl(alpha: float, beta: float, distance: float) -> Optional[Tuple[float, ...]]:
    sin_alpha = math.sin(alpha)
    sin_beta = math.sin(beta)
    cos_alpha = math.cos(alpha)
    cos_beta = math.cos(beta)
    cos_alpha_beta = math.cos(alpha - beta)
    p_squared = (
        distance * distance
        - 2.0
        + 2.0 * cos_alpha_beta
        - 2.0 * distance * (sin_alpha + sin_beta)
    )
    if p_squared < -1.0e-12:
        return None
    p = math.sqrt(max(0.0, p_squared))
    tmp = math.atan2(
        cos_alpha + cos_beta,
        distance - sin_alpha - sin_beta,
    ) - math.atan2(2.0, p)
    return _mod2pi(alpha - tmp), p, _mod2pi(beta - tmp)


def _rlr(alpha: float, beta: float, distance: float) -> Optional[Tuple[float, ...]]:
    sin_alpha = math.sin(alpha)
    sin_beta = math.sin(beta)
    cos_alpha = math.cos(alpha)
    cos_beta = math.cos(beta)
    cos_alpha_beta = math.cos(alpha - beta)
    tmp = (
        6.0
        - distance * distance
        + 2.0 * cos_alpha_beta
        + 2.0 * distance * (sin_alpha - sin_beta)
    ) / 8.0
    if abs(tmp) > 1.0 + 1.0e-12:
        return None
    p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp))))
    t = _mod2pi(
        alpha
        - math.atan2(
            cos_alpha - cos_beta,
            distance - sin_alpha + sin_beta,
        )
        + p / 2.0
    )
    return t, p, _mod2pi(alpha - beta - t + p)


def _lrl(alpha: float, beta: float, distance: float) -> Optional[Tuple[float, ...]]:
    sin_alpha = math.sin(alpha)
    sin_beta = math.sin(beta)
    cos_alpha = math.cos(alpha)
    cos_beta = math.cos(beta)
    cos_alpha_beta = math.cos(alpha - beta)
    tmp = (
        6.0
        - distance * distance
        + 2.0 * cos_alpha_beta
        + 2.0 * distance * (-sin_alpha + sin_beta)
    ) / 8.0
    if abs(tmp) > 1.0 + 1.0e-12:
        return None
    p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, tmp))))
    t = _mod2pi(
        -alpha
        - math.atan2(
            cos_alpha - cos_beta,
            distance + sin_alpha - sin_beta,
        )
        + p / 2.0
    )
    return t, p, _mod2pi(beta - alpha - t + p)


_DUBINS_SOLVERS: Tuple[
    Callable[[float, float, float], Optional[Tuple[float, ...]]], ...
] = (_lsl, _rsr, _lsr, _rsl, _rlr, _lrl)


def _shortest_dubins_parameters(
    *,
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    turning_radius_m: float,
) -> Tuple[str, Tuple[float, float, float]]:
    radius = float(turning_radius_m)
    dx = float(goal[0]) - float(start[0])
    dy = float(goal[1]) - float(start[1])
    normalized_distance = math.hypot(dx, dy) / radius
    bearing = _mod2pi(math.atan2(dy, dx))
    alpha = _mod2pi(float(start[2]) - bearing)
    beta = _mod2pi(float(goal[2]) - bearing)

    candidates = []
    for path_type, solver in zip(_DUBINS_PATH_TYPES, _DUBINS_SOLVERS):
        params = solver(alpha, beta, normalized_distance)
        if params is not None:
            candidates.append((sum(params), path_type, params))
    if not candidates:
        raise ValueError("no existe una curva Dubins valida para la transicion")
    _, path_type, params = min(candidates, key=lambda item: item[0])
    return path_type, (float(params[0]), float(params[1]), float(params[2]))


def _advance_dubins_pose(
    pose: Tuple[float, float, float],
    *,
    segment_type: str,
    normalized_step: float,
    turning_radius_m: float,
) -> Tuple[float, float, float]:
    x, y, yaw = pose
    step = float(normalized_step)
    radius = float(turning_radius_m)
    if segment_type == "S":
        return (
            x + radius * step * math.cos(yaw),
            y + radius * step * math.sin(yaw),
            yaw,
        )
    if segment_type == "L":
        next_yaw = yaw + step
        return (
            x + radius * (math.sin(next_yaw) - math.sin(yaw)),
            y + radius * (-math.cos(next_yaw) + math.cos(yaw)),
            next_yaw,
        )
    next_yaw = yaw - step
    return (
        x + radius * (math.sin(yaw) - math.sin(next_yaw)),
        y + radius * (math.cos(next_yaw) - math.cos(yaw)),
        next_yaw,
    )


def _sample_dubins_transition(
    *,
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    turning_radius_m: float,
    waypoint_spacing_m: float,
    row_index: int,
) -> list[CoverageBodyWaypoint]:
    path_type, parameters = _shortest_dubins_parameters(
        start=start,
        goal=goal,
        turning_radius_m=turning_radius_m,
    )
    pose = (float(start[0]), float(start[1]), float(start[2]))
    points = [
        CoverageBodyWaypoint(
            forward_m=pose[0],
            left_m=pose[1],
            yaw_delta_deg=math.degrees(pose[2]),
            phase="turn",
            row_index=int(row_index),
        )
    ]
    # Indice del punto donde arranca cada tramo Dubins. Como el muestreo se
    # reinicia en cada tramo, esos indices caen exactamente sobre el cambio de
    # curvatura, sin depender de waypoint_spacing_m.
    segment_start_indices: list[int] = []
    segment_lengths_m: list[float] = []
    for segment_type, parameter in zip(path_type, parameters):
        segment_length_m = float(parameter) * float(turning_radius_m)
        segment_start_indices.append(len(points) - 1)
        segment_lengths_m.append(segment_length_m)
        step_count = max(1, int(math.ceil(segment_length_m / waypoint_spacing_m)))
        normalized_step = float(parameter) / float(step_count)
        for _ in range(step_count):
            pose = _advance_dubins_pose(
                pose,
                segment_type=segment_type,
                normalized_step=normalized_step,
                turning_radius_m=turning_radius_m,
            )
            points.append(
                CoverageBodyWaypoint(
                    forward_m=float(pose[0]),
                    left_m=float(pose[1]),
                    yaw_delta_deg=float(math.degrees(pose[2])),
                    phase="turn",
                    row_index=int(row_index),
                )
            )

    position_error = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    yaw_error = abs(math.atan2(math.sin(pose[2] - goal[2]), math.cos(pose[2] - goal[2])))
    if position_error > 1.0e-6 or yaw_error > 1.0e-6:
        raise RuntimeError("la integracion de la curva Dubins no alcanzo el waypoint final")
    points[-1] = CoverageBodyWaypoint(
        forward_m=float(goal[0]),
        left_m=float(goal[1]),
        yaw_delta_deg=float(math.degrees(goal[2])),
        phase="turn",
        row_index=int(row_index),
    )

    guide_index = _last_curvature_switch_index(
        segment_start_indices,
        segment_lengths_m,
        point_count=len(points),
    )
    if guide_index is not None:
        points[guide_index] = replace(points[guide_index], is_guide=True)
    return points


def _last_curvature_switch_index(
    segment_start_indices: Sequence[int],
    segment_lengths_m: Sequence[float],
    *,
    point_count: int,
) -> Optional[int]:
    """Ultimo cambio de curvatura util de la curva Dubins.

    Es el punto donde arranca el tramo final: para un omega es el paso del arco
    largo al arco de salida, y para una U limpia es el paso de la recta al arco
    de salida. Los tramos de longitud despreciable no cuentan como cambio: no
    representan una maniobra y su indice cae pegado a un extremo.
    """

    boundaries = [
        int(start_index)
        for index, start_index in enumerate(segment_start_indices)
        if index > 0
        and segment_lengths_m[index] > _DEGENERATE_DUBINS_SEGMENT_M
        and segment_lengths_m[index - 1] > _DEGENERATE_DUBINS_SEGMENT_M
        and 0 < int(start_index) < int(point_count) - 1
    ]
    if not boundaries:
        return None
    return boundaries[-1]


def headland_turn_length_m(
    lane_separation_m: float,
    *,
    min_turning_radius_m: float,
) -> float:
    """Largo del giro de cabecera entre dos pasadas separadas lateralmente."""

    separation = abs(float(lane_separation_m))
    radius = float(min_turning_radius_m)
    if radius <= 0.0:
        raise ValueError("min_turning_radius_m debe ser mayor que cero")
    _, parameters = _shortest_dubins_parameters(
        start=(0.0, 0.0, 0.0),
        goal=(0.0, separation, math.pi),
        turning_radius_m=radius,
    )
    return float(sum(parameters) * radius)


def _arc_reaches_yaw(
    *,
    start_yaw_rad: float,
    swept_rad: float,
    target_yaw_rad: float,
) -> bool:
    """Indicar si un arco pasa por un rumbo dado antes de terminar."""

    if swept_rad >= 0.0:
        delta = _mod2pi(float(target_yaw_rad) - float(start_yaw_rad))
    else:
        delta = _mod2pi(float(start_yaw_rad) - float(target_yaw_rad))
    return bool(delta <= abs(float(swept_rad)) + 1.0e-12)


def _dubins_segment_forward_extent(
    pose: Tuple[float, float, float],
    *,
    segment_type: str,
    parameter: float,
    turning_radius_m: float,
) -> Tuple[float, Tuple[float, float, float]]:
    """Maximo avance de un tramo Dubins y la pose con la que termina.

    Sobre un arco el avance maximo no siempre esta en un extremo: si el tramo
    pasa por el rumbo perpendicular, el maximo es la tangente vertical del
    circulo. Resolverlo asi evita muestrear la curva para medir la cabecera.
    """

    radius = float(turning_radius_m)
    end = _advance_dubins_pose(
        pose,
        segment_type=segment_type,
        normalized_step=float(parameter),
        turning_radius_m=radius,
    )
    extent = max(float(pose[0]), float(end[0]))
    if segment_type == "S":
        return extent, end

    start_yaw = float(pose[2])
    if segment_type == "L":
        center_x = float(pose[0]) - radius * math.sin(start_yaw)
        swept = float(parameter)
        target_yaw = 0.5 * math.pi
    else:
        center_x = float(pose[0]) + radius * math.sin(start_yaw)
        swept = -float(parameter)
        target_yaw = -0.5 * math.pi
    if _arc_reaches_yaw(
        start_yaw_rad=start_yaw,
        swept_rad=swept,
        target_yaw_rad=target_yaw,
    ):
        extent = max(extent, center_x + radius)
    return extent, end


def headland_turn_depth_m(
    lane_separation_m: float,
    *,
    min_turning_radius_m: float,
) -> float:
    """Cuanto sobresale de la cabecera el enlace entre dos pasadas.

    Es la medida que decide si un lote se puede trabajar: la U simple sobresale
    exactamente un radio, mientras que el omega que aparece por debajo del
    diametro de giro sobresale mucho mas. Con ``R = 4 m`` va de ``4.0 m`` a
    ``8 m`` de separacion hasta ``10.8 m`` cuando las pasadas casi se tocan, y
    decrece de forma monotona con la separacion.
    """

    separation = abs(float(lane_separation_m))
    radius = float(min_turning_radius_m)
    if radius <= 0.0:
        raise ValueError("min_turning_radius_m debe ser mayor que cero")
    path_type, parameters = _shortest_dubins_parameters(
        start=(0.0, 0.0, 0.0),
        goal=(0.0, separation, math.pi),
        turning_radius_m=radius,
    )
    pose = (0.0, 0.0, 0.0)
    depth = 0.0
    for segment_type, parameter in zip(path_type, parameters):
        extent, pose = _dubins_segment_forward_extent(
            pose,
            segment_type=segment_type,
            parameter=float(parameter),
            turning_radius_m=radius,
        )
        depth = max(depth, extent)
    return float(depth)


def is_clean_uturn(
    lane_separation_m: float,
    *,
    min_turning_radius_m: float,
) -> bool:
    """Una U simple solo existe si las pasadas distan al menos el diametro de giro."""

    return abs(float(lane_separation_m)) >= (2.0 * float(min_turning_radius_m)) - 1.0e-9


def max_separation_row_order(row_count: int) -> Tuple[int, ...]:
    """Orden que maximiza la separacion minima entre pasadas consecutivas.

    Parte la lista de pasadas en dos bloques contiguos y los intercala:
    ``0, b, 1, b+1, ...`` con ``b = ceil(row_count / 2)``. Las separaciones
    alternan ``b`` y ``b - 1`` pasadas, de modo que la minima es ``b - 1``, el
    maximo posible para cualquier recorrido que arranque en la pasada ``0``.

    Arrancar en la pasada ``0`` no es negociable: es la que pisa el vehiculo, y
    el preflight de ``start_coverage`` exige que la primera meta este a pocos
    metros de la pose actual.

    Importa porque el largo del giro y su desborde longitudinal decrecen de
    forma monotona con la separacion: para ``R = 4 m`` el enlace entre pasadas a
    ``1.64 m`` sobresale ``10.4 m`` de la cabecera, mientras que uno a ``8 m`` o
    mas es una U simple que sobresale exactamente ``R``.
    """

    total_rows = int(row_count)
    if total_rows <= 2:
        return tuple(range(max(0, total_rows)))
    block = (total_rows + 1) // 2
    order: list[int] = []
    for index in range(block):
        order.append(index)
        if block + index < total_rows:
            order.append(block + index)
    return tuple(order)


def _grouped_serpentine_order(row_count: int, skip: int) -> list[int]:
    order: list[int] = []
    for group_index, first in enumerate(range(skip)):
        group = list(range(first, row_count, skip))
        if group_index % 2 == 1:
            group.reverse()
        order.extend(group)
    return order


def _alternating_serpentine_order(row_count: int, skip: int) -> list[int]:
    order: list[int] = []
    seen: set[int] = set()
    for first in range(skip):
        group = [index for index in range(first, row_count, skip)]
        if len(order) % 2 == 1:
            group.reverse()
        for index in group:
            if index not in seen:
                order.append(index)
                seen.add(index)
    return order


def minimum_safe_lane_spacing_m(min_turning_radius_m: float) -> float:
    """Separacion minima entre pasadas consecutivas para que el giro no se pise.

    Por debajo del diametro de giro el enlace de cabecera deja de ser una U y
    pasa a ser un omega, que sobresale lateralmente. Mientras ese desborde no
    alcanza la pasada vecina el trazado sigue siendo limpio. El limite medido
    sobre la geometria Dubins real es exactamente un radio, y es invariante de
    escala: verificado para R = 2, 3, 4 y 6 m.

    Es una condicion necesaria, no suficiente: la autoridad final siempre es
    ``analyze_polyline_topology`` sobre la poligonal completa.
    """

    radius = float(min_turning_radius_m)
    if radius <= 0.0:
        raise ValueError("min_turning_radius_m debe ser mayor que cero")
    return radius


def _order_turn_cost(
    order: Sequence[int],
    *,
    lane_spacing_m: float,
    min_turning_radius_m: float,
) -> float:
    return float(
        sum(
            headland_turn_length_m(
                (int(order[index + 1]) - int(order[index])) * float(lane_spacing_m),
                min_turning_radius_m=min_turning_radius_m,
            )
            for index in range(len(order) - 1)
        )
    )


def _order_headland_depth_m(
    order: Sequence[int],
    *,
    lane_spacing_m: float,
    min_turning_radius_m: float,
) -> float:
    """Cabecera que necesita un orden: la del peor de sus giros."""

    return float(
        max(
            (
                headland_turn_depth_m(
                    (int(order[index + 1]) - int(order[index])) * float(lane_spacing_m),
                    min_turning_radius_m=min_turning_radius_m,
                )
                for index in range(len(order) - 1)
            ),
            default=0.0,
        )
    )


def resolve_row_visit_order(
    *,
    row_count: int,
    lane_spacing_m: float,
    min_turning_radius_m: float,
    field_length_m: Optional[float] = None,
    cutter_width_m: Optional[float] = None,
    audit_spacing_m: float = COVERAGE_TOPOLOGY_AUDIT_SPACING_M,
    allow_row_skipping: bool = False,
) -> Tuple[int, ...]:
    """Elegir en que orden se recorren las pasadas.

    Por defecto el orden es la serpentina adyacente ``0, 1, 2, ...``: se avanza
    pasada por pasada desde la del vehiculo hacia el otro borde del lote, sin
    saltear ninguna. Es el recorrido que se espera de una cosechadora y el que
    deja el trabajo sin huecos aunque la mision se corte a mitad de camino, y
    tambien el que mantiene juntas en el tiempo dos pasadas vecinas, que es lo
    que acota el error de solape frente a la deriva de la localizacion.

    El precio es la cabecera. Cuando las pasadas distan menos que el diametro de
    giro el enlace deja de ser una U simple y pasa a ser un omega: con ``R = 4 m``
    y pasadas a ``1.64 m`` el giro mide ``27.4 m`` y se va ``10.4 m`` fuera del
    lote, contra ``12.6 m`` y ``4 m`` de la U. El omega es un giro valido y el
    vehiculo lo ejecuta; solo pide cabecera libre.

    ``allow_row_skipping=True`` habilita la busqueda del orden que menos
    sobresale, que saltea pasadas para separar los giros (ver
    ``max_separation_row_order``). Cambia el aspecto del recorrido y el orden en
    que queda cubierto el lote, asi que es una decision del perfil, no un ajuste
    automatico.

    Con la busqueda habilitada y ``field_length_m`` presente, cada candidato se
    construye completo y se audita. Si tambien se pasa ``cutter_width_m``, solo
    se consideran los conflictos dentro del rectangulo fisico cubierto; las
    cabeceras exteriores pueden cruzarse. Un candidato sin conflictos en el
    alcance auditado siempre le gana a uno con conflictos, por poco que
    sobresalga.

    Saltar pasadas no elimina los cruces de cabecera: los enlaces de un mismo
    extremo se entrelazan igual. Lo que si elimina es el omega.
    """

    total_rows = int(row_count)
    spacing = float(lane_spacing_m)
    radius = float(min_turning_radius_m)
    adjacent = tuple(range(total_rows))
    if not bool(allow_row_skipping):
        return adjacent
    if total_rows <= 2 or spacing <= 0.0 or radius <= 0.0:
        return adjacent

    required_separation = 2.0 * radius
    minimum_skip = int(math.ceil(required_separation / spacing))
    if minimum_skip <= 1:
        # Las pasadas vecinas ya distan un diametro de giro: la serpentina
        # adyacente es a la vez la mas corta y la unica sin cruces.
        return adjacent

    candidates: list[Sequence[int]] = [adjacent, max_separation_row_order(total_rows)]
    for skip in range(minimum_skip, min(minimum_skip + 3, total_rows)):
        candidates.append(_grouped_serpentine_order(total_rows, skip))
        candidates.append(_alternating_serpentine_order(total_rows, skip))

    valid: list[Tuple[int, ...]] = []
    for candidate in candidates:
        order = tuple(int(index) for index in candidate)
        if sorted(order) == list(adjacent) and order not in valid:
            valid.append(order)

    def preference_key(candidate: Tuple[int, ...]) -> Tuple[float, float]:
        # La cabecera y el largo salen de la geometria Dubins de cada giro por
        # separado, sin construir la poligonal: ordenar candidatos es analitico y
        # la auditoria, que es lo caro, solo corre sobre los que hagan falta.
        return (
            round(
                _order_headland_depth_m(
                    candidate,
                    lane_spacing_m=spacing,
                    min_turning_radius_m=radius,
                ),
                2,
            ),
            round(
                _order_turn_cost(
                    candidate,
                    lane_spacing_m=spacing,
                    min_turning_radius_m=radius,
                ),
                3,
            ),
        )

    # Empates exactos conservan el orden de `valid`, que empieza por la
    # serpentina adyacente: si nada mejora el recorrido, se recorre linea a linea.
    ranked = sorted(valid, key=preference_key)

    if field_length_m is None:
        # Sin el largo de pasada no se puede construir la poligonal ni auditarla.
        # Los llamadores del planificador siempre pasan field_length_m; esta rama
        # existe para consultas del orden solo.
        return ranked[0]

    side_sign = 1.0 if spacing >= 0.0 else -1.0
    row_offsets = [side_sign * spacing * index for index in range(total_rows)]
    field_bounds: Optional[Tuple[float, float, float, float]] = None
    if cutter_width_m is not None:
        half_cutter = 0.5 * float(cutter_width_m)
        if not math.isfinite(half_cutter) or half_cutter <= 0.0:
            raise ValueError("cutter_width_m debe ser mayor que cero")
        field_bounds = (
            -half_cutter,
            float(field_length_m) + half_cutter,
            min(row_offsets) - half_cutter,
            max(row_offsets) + half_cutter,
        )

    # La auditoria solo filtra: se recorre la preferencia de mejor a peor y se
    # devuelve el primero que no se pise en el alcance auditado. Asi el elegido es
    # el mas superficial entre los seguros, y en el caso habitual se audita uno
    # solo.
    for candidate in ranked:
        if audit_order_topology(
            field_length_m=float(field_length_m),
            row_offsets=row_offsets,
            visit_order=candidate,
            turning_radius_m=radius,
            audit_spacing_m=audit_spacing_m,
            bounds_m=field_bounds,
        ).is_safe:
            return candidate

    # Ninguno esta libre de conflictos: se devuelve el de menor cabecera, que es
    # el que deja el lote mas practicable. El plan reporta los conflictos y el
    # arranque queda bloqueado aguas arriba si la politica del perfil lo exige.
    return ranked[0]


def _normalized_lawnmower_inputs(
    *,
    field_length_m: float,
    field_width_m: float,
    cutter_width_m: float,
    overlap_ratio: float,
    min_turning_radius_m: float,
    waypoint_spacing_m: float,
    side: str,
) -> Tuple[float, float, float, float, float, float, str]:
    field_length = float(field_length_m)
    field_width = float(field_width_m)
    cutter_width = float(cutter_width_m)
    overlap = float(overlap_ratio)
    turning_radius = float(min_turning_radius_m)
    waypoint_spacing = float(waypoint_spacing_m)
    normalized_side = str(side).strip().lower()

    numeric_values = (
        field_length,
        field_width,
        cutter_width,
        overlap,
        turning_radius,
        waypoint_spacing,
    )
    if not all(math.isfinite(value) for value in numeric_values):
        raise ValueError("todos los parametros numericos deben ser finitos")
    if field_length <= 0.0 or field_width <= 0.0:
        raise ValueError("field_length_m y field_width_m deben ser mayores que cero")
    if cutter_width <= 0.0:
        raise ValueError("cutter_width_m debe ser mayor que cero")
    if not 0.0 <= overlap < 1.0:
        raise ValueError("overlap_ratio debe estar en el rango [0, 1)")
    if turning_radius <= 0.0:
        raise ValueError("min_turning_radius_m debe ser mayor que cero")
    if waypoint_spacing <= 0.0:
        raise ValueError("waypoint_spacing_m debe ser mayor que cero")
    if normalized_side not in {"left", "right"}:
        raise ValueError("side debe ser 'left' o 'right'")
    return (
        field_length,
        field_width,
        cutter_width,
        overlap,
        turning_radius,
        waypoint_spacing,
        normalized_side,
    )


def _build_body_waypoints(
    *,
    field_length_m: float,
    row_offsets: Sequence[float],
    visit_order: Sequence[int],
    turning_radius_m: float,
    waypoint_spacing_m: float,
    row_waypoint_spacing_m: Optional[float] = None,
) -> list[CoverageBodyWaypoint]:
    """Recorrer las pasadas en el orden dado y unirlas con curvas Dubins.

    ``row_waypoint_spacing_m`` permite muestrear las pasadas mas grueso que las
    cabeceras. La auditoria de topologia lo usa con las pasadas enteras: una recta
    se cruza o no con otra figura sin importar en cuantos puntos se la parta, asi
    que muestrearla fino solo agrega puntos. Las curvas si necesitan el muestreo
    fino, porque ahi la poligonal recorta el arco real.
    """

    row_spacing = float(
        waypoint_spacing_m if row_waypoint_spacing_m is None else row_waypoint_spacing_m
    )
    waypoints: list[CoverageBodyWaypoint] = []
    for visit_index, row_index in enumerate(visit_order):
        left_m = float(row_offsets[int(row_index)])
        heading_rad = 0.0 if visit_index % 2 == 0 else math.pi
        start_x_m = 0.0 if visit_index % 2 == 0 else float(field_length_m)
        end_x_m = float(field_length_m) if visit_index % 2 == 0 else 0.0
        row_points = _sample_straight_row(
            start_x_m=start_x_m,
            end_x_m=end_x_m,
            left_m=left_m,
            yaw_rad=heading_rad,
            waypoint_spacing_m=row_spacing,
            row_index=int(row_index),
        )
        if waypoints:
            previous = waypoints[-1]
            transition = _sample_dubins_transition(
                start=(
                    float(previous.forward_m),
                    float(previous.left_m),
                    math.radians(float(previous.yaw_delta_deg)),
                ),
                goal=(float(start_x_m), float(left_m), float(heading_rad)),
                turning_radius_m=float(turning_radius_m),
                waypoint_spacing_m=waypoint_spacing_m,
                row_index=int(row_index),
            )
            waypoints.extend(transition[1:])
            waypoints[-1] = row_points[0]
            waypoints.extend(row_points[1:])
        else:
            waypoints.extend(row_points)
    return _mark_headland_turn_guides(waypoints)


def _mark_headland_turn_guides(
    waypoints: Sequence[CoverageBodyWaypoint],
) -> list[CoverageBodyWaypoint]:
    """Dejar exactamente una guia no-key por cabecera.

    La guia es informativa: marca el ultimo cambio de curvatura del giro, que
    ``_sample_dubins_transition`` ya ubica sobre el punto exacto, y solo cae al
    punto medio del arco cuando el giro no tiene ningun cambio utilizable.

    **No debe usarse como meta.** Medido contra el Smac de este repositorio
    (``minimum_turning_radius`` 4.0 m) barriendo el rumbo de la pasada, partir
    la cabecera en dos metas deja un tramo que pide el radio minimo exacto; la
    busqueda no lo puede cerrar y lo resuelve con una vuelta completa de 25 m.
    Con guia el giro cuesta 39-64 m contra 14.4 m nominal en U simple, y 46-55 m
    contra 27.4 m en omega. Enviando el giro entero como una sola meta el plan
    queda en 14.0-18.0 m y 27.9-29.1 m respectivamente, para todo rumbo. Por eso
    ``coverage_use_headland_guides`` esta apagado en los dos perfiles.
    """

    marked = list(waypoints)
    index = 0
    while index < len(marked):
        if marked[index].phase != "turn":
            index += 1
            continue
        run_start = index
        while index + 1 < len(marked) and marked[index + 1].phase == "turn":
            index += 1
        run_end = index
        if any(marked[point].is_guide for point in range(run_start, run_end + 1)):
            index += 1
            continue
        path_start = max(0, run_start - 1)
        path_end = min(len(marked) - 1, run_end + 1)
        cumulative = [0.0]
        for point_index in range(path_start + 1, path_end + 1):
            previous = marked[point_index - 1]
            point = marked[point_index]
            cumulative.append(
                cumulative[-1]
                + math.hypot(
                    float(point.forward_m) - float(previous.forward_m),
                    float(point.left_m) - float(previous.left_m),
                )
            )
        target = 0.5 * cumulative[-1]
        guide_index = min(
            range(run_start, run_end + 1),
            key=lambda candidate: abs(
                cumulative[candidate - path_start] - target
            ),
        )
        marked[guide_index] = replace(marked[guide_index], is_guide=True)
        index += 1
    return marked


def audit_order_topology(
    *,
    field_length_m: float,
    row_offsets: Sequence[float],
    visit_order: Sequence[int],
    turning_radius_m: float,
    audit_spacing_m: float = COVERAGE_TOPOLOGY_AUDIT_SPACING_M,
    bounds_m: Optional[Tuple[float, float, float, float]] = None,
) -> PolylineTopologyMetrics:
    """Auditar la poligonal nominal remuestreada a resolucion fina."""

    waypoints = _build_body_waypoints(
        field_length_m=field_length_m,
        row_offsets=row_offsets,
        visit_order=visit_order,
        turning_radius_m=turning_radius_m,
        waypoint_spacing_m=max(1.0e-3, float(audit_spacing_m)),
    )
    points = [(point.forward_m, point.left_m) for point in waypoints]
    if bounds_m is None:
        return analyze_polyline_topology(points, epsilon_m=1.0e-6)
    return analyze_polyline_topology_in_bounds(
        points,
        bounds_m=bounds_m,
        epsilon_m=1.0e-6,
    )


def evaluate_row_visit_order(
    *,
    field_length_m: float,
    row_offsets: Sequence[float],
    visit_order: Sequence[int],
    turning_radius_m: float,
    audit_spacing_m: float = COVERAGE_TOPOLOGY_AUDIT_SPACING_M,
    bounds_m: Optional[Tuple[float, float, float, float]] = None,
) -> RowOrderMetrics:
    """Medir un orden de pasada sobre la poligonal Dubins que realmente genera.

    Devuelve, en una sola construccion, lo que hace falta para compararlo con
    otro orden: conflictos en el alcance auditado, cuanto sobresale de las
    cabeceras y cuanto camino cuesta.
    """

    waypoints = _build_body_waypoints(
        field_length_m=field_length_m,
        row_offsets=row_offsets,
        visit_order=visit_order,
        turning_radius_m=turning_radius_m,
        waypoint_spacing_m=max(1.0e-3, float(audit_spacing_m)),
    )
    points = [(point.forward_m, point.left_m) for point in waypoints]
    if bounds_m is None:
        topology = analyze_polyline_topology(points, epsilon_m=1.0e-6)
    else:
        topology = analyze_polyline_topology_in_bounds(
            points,
            bounds_m=bounds_m,
            epsilon_m=1.0e-6,
        )
    forwards = [point[0] for point in points]
    separations = [
        abs(
            float(row_offsets[int(visit_order[index + 1])])
            - float(row_offsets[int(visit_order[index])])
        )
        for index in range(len(visit_order) - 1)
    ]
    return RowOrderMetrics(
        conflict_count=int(topology.conflict_count),
        headland_depth_m=float(
            max(
                0.0,
                -min(forwards),
                max(forwards) - float(field_length_m),
            )
        ),
        path_length_m=float(_path_length(waypoints)),
        minimum_separation_m=float(min(separations)) if separations else 0.0,
    )


def _resolve_coverage_row_layout(
    *,
    field_width_m: float,
    cutter_width_m: float,
    overlap_ratio: float,
    min_turning_radius_m: float,
    side: str,
    field_length_m: Optional[float] = None,
    allow_headland_conflicts: bool = False,
    allow_row_skipping: bool = False,
) -> Tuple[int, float, list[float], Tuple[int, ...]]:
    max_lane_spacing = float(cutter_width_m) * (1.0 - float(overlap_ratio))
    centerline_span = max(0.0, float(field_width_m) - float(cutter_width_m))
    if centerline_span <= 1.0e-9:
        row_count = 1
        lane_spacing = 0.0
    else:
        row_count = int(math.ceil(centerline_span / max_lane_spacing)) + 1
        lane_spacing = centerline_span / float(row_count - 1)

    side_sign = 1.0 if str(side) == "left" else -1.0
    row_offsets = [side_sign * lane_spacing * index for index in range(row_count)]
    visit_order = resolve_row_visit_order(
        row_count=row_count,
        lane_spacing_m=lane_spacing,
        min_turning_radius_m=float(min_turning_radius_m),
        field_length_m=field_length_m,
        cutter_width_m=(
            float(cutter_width_m)
            if field_length_m is not None and bool(allow_headland_conflicts)
            else None
        ),
        allow_row_skipping=bool(allow_row_skipping),
    )
    return int(row_count), float(lane_spacing), row_offsets, visit_order


def estimate_lawnmower_waypoint_count(
    *,
    field_length_m: float,
    field_width_m: float,
    cutter_width_m: float,
    overlap_ratio: float,
    min_turning_radius_m: float,
    waypoint_spacing_m: float = 2.0,
    side: str = "left",
    allow_headland_conflicts: bool = False,
    allow_row_skipping: bool = False,
) -> Tuple[int, int]:
    """Estimar exactamente filas y puntos antes de materializar el plan."""

    (
        field_length,
        field_width,
        cutter_width,
        overlap,
        turning_radius,
        waypoint_spacing,
        normalized_side,
    ) = _normalized_lawnmower_inputs(
        field_length_m=field_length_m,
        field_width_m=field_width_m,
        cutter_width_m=cutter_width_m,
        overlap_ratio=overlap_ratio,
        min_turning_radius_m=min_turning_radius_m,
        waypoint_spacing_m=waypoint_spacing_m,
        side=side,
    )
    row_count, _, row_offsets, visit_order = _resolve_coverage_row_layout(
        field_width_m=field_width,
        cutter_width_m=cutter_width,
        overlap_ratio=overlap,
        min_turning_radius_m=turning_radius,
        side=normalized_side,
        field_length_m=field_length,
        allow_headland_conflicts=allow_headland_conflicts,
        allow_row_skipping=allow_row_skipping,
    )
    row_step_count = max(1, int(math.ceil(field_length / waypoint_spacing)))
    waypoint_count = row_step_count + 1
    for visit_index in range(1, len(visit_order)):
        previous_row_index = int(visit_order[visit_index - 1])
        row_index = int(visit_order[visit_index])
        previous_heading = 0.0 if (visit_index - 1) % 2 == 0 else math.pi
        heading = 0.0 if visit_index % 2 == 0 else math.pi
        previous_end_x = field_length if (visit_index - 1) % 2 == 0 else 0.0
        start_x = 0.0 if visit_index % 2 == 0 else field_length
        path_type, parameters = _shortest_dubins_parameters(
            start=(
                previous_end_x,
                row_offsets[previous_row_index],
                previous_heading,
            ),
            goal=(start_x, row_offsets[row_index], heading),
            turning_radius_m=turning_radius,
        )
        _ = path_type
        waypoint_count += sum(
            max(
                1,
                int(
                    math.ceil(
                        float(parameter) * turning_radius / waypoint_spacing
                    )
                ),
            )
            for parameter in parameters
        )
        waypoint_count += row_step_count
    return int(row_count), int(waypoint_count)


def _sample_straight_row(
    *,
    start_x_m: float,
    end_x_m: float,
    left_m: float,
    yaw_rad: float,
    waypoint_spacing_m: float,
    row_index: int,
) -> list[CoverageBodyWaypoint]:
    row_length_m = abs(float(end_x_m) - float(start_x_m))
    step_count = max(1, int(math.ceil(row_length_m / waypoint_spacing_m)))
    return [
        CoverageBodyWaypoint(
            forward_m=float(start_x_m)
            + (float(end_x_m) - float(start_x_m)) * float(index) / float(step_count),
            left_m=float(left_m),
            yaw_delta_deg=float(math.degrees(yaw_rad)),
            phase="row",
            row_index=int(row_index),
            is_key=index in (0, step_count),
        )
        for index in range(step_count + 1)
    ]


def _path_length(points: Sequence[CoverageBodyWaypoint]) -> float:
    return float(
        sum(
            math.hypot(
                float(current.forward_m) - float(previous.forward_m),
                float(current.left_m) - float(previous.left_m),
            )
            for previous, current in zip(points, points[1:])
        )
    )


def recommended_coverage_leg_spacing_m(
    plan: CoveragePlan,
    *,
    field_length_m: float,
) -> float:
    """Evitar que route_executor interpole rectas dentro de las cabeceras."""

    max_turn_separation = max(plan.turn_separations_m, default=0.0)
    return float(max(5.0, float(field_length_m), max_turn_separation) + 1.0)


def build_lawnmower_body_plan(
    *,
    field_length_m: float,
    field_width_m: float,
    cutter_width_m: float,
    overlap_ratio: float,
    min_turning_radius_m: float,
    waypoint_spacing_m: float = 2.0,
    side: str = "left",
    allow_headland_conflicts: bool = False,
    allow_row_skipping: bool = False,
    row_waypoint_spacing_m: Optional[float] = None,
) -> CoveragePlan:
    """Generar pasadas paralelas unidas por giros Dubins sin marcha atras.

    ``row_waypoint_spacing_m`` separa el muestreo de las pasadas del de las
    cabeceras. Sirve para pedir cabeceras finas sin pagar puntos en las rectas,
    que es lo unico que necesita el detalle: la guia de cabecera y la auditoria de
    cruces viven en las curvas.
    """

    (
        field_length,
        field_width,
        cutter_width,
        overlap,
        turning_radius,
        waypoint_spacing,
        normalized_side,
    ) = _normalized_lawnmower_inputs(
        field_length_m=field_length_m,
        field_width_m=field_width_m,
        cutter_width_m=cutter_width_m,
        overlap_ratio=overlap_ratio,
        min_turning_radius_m=min_turning_radius_m,
        waypoint_spacing_m=waypoint_spacing_m,
        side=side,
    )
    row_count, lane_spacing, row_offsets, visit_order = _resolve_coverage_row_layout(
        field_width_m=field_width,
        cutter_width_m=cutter_width,
        overlap_ratio=overlap,
        min_turning_radius_m=turning_radius,
        side=normalized_side,
        field_length_m=field_length,
        allow_headland_conflicts=allow_headland_conflicts,
        allow_row_skipping=allow_row_skipping,
    )
    turn_separations = [
        abs(row_offsets[visit_order[index + 1]] - row_offsets[visit_order[index]])
        for index in range(len(visit_order) - 1)
    ]
    waypoints = _build_body_waypoints(
        field_length_m=field_length,
        row_offsets=row_offsets,
        visit_order=visit_order,
        turning_radius_m=turning_radius,
        waypoint_spacing_m=waypoint_spacing,
        row_waypoint_spacing_m=(
            None
            if row_waypoint_spacing_m is None
            else max(1.0e-3, float(row_waypoint_spacing_m))
        ),
    )

    min_forward = min(point.forward_m for point in waypoints)
    max_forward = max(point.forward_m for point in waypoints)
    min_row_left = min(row_offsets)
    max_row_left = max(row_offsets)
    lateral_overflow = max(
        0.0,
        min_row_left - min(point.left_m for point in waypoints),
        max(point.left_m for point in waypoints) - max_row_left,
    )
    # La auditoria no usa la poligonal de preview: con un muestreo grueso las
    # curvas se recortan y un cruce real puede desaparecer. Siempre se remuestrea,
    # pero solo las curvas: las pasadas son rectas y entran enteras, con sus dos
    # extremos. Partirlas no cambia ningun cruce y hace crecer la auditoria con el
    # largo del lote, que es lo que antes chocaba con el tope de puntos.
    audit_spacing = min(
        float(waypoint_spacing), float(COVERAGE_TOPOLOGY_AUDIT_SPACING_M)
    )
    audit_points = [
        (point.forward_m, point.left_m)
        for point in _build_body_waypoints(
            field_length_m=field_length,
            row_offsets=row_offsets,
            visit_order=visit_order,
            turning_radius_m=turning_radius,
            waypoint_spacing_m=max(1.0e-3, audit_spacing),
            row_waypoint_spacing_m=max(field_length, 1.0e-3),
        )
    ]
    topology = analyze_polyline_topology(audit_points, epsilon_m=1.0e-6)
    half_cutter = 0.5 * cutter_width
    field_topology = analyze_polyline_topology_in_bounds(
        audit_points,
        bounds_m=(
            -half_cutter,
            field_length + half_cutter,
            min(row_offsets) - half_cutter,
            max(row_offsets) + half_cutter,
        ),
        epsilon_m=1.0e-6,
    )
    return CoveragePlan(
        waypoints=tuple(waypoints),
        row_count=int(row_count),
        lane_spacing_m=float(lane_spacing),
        cutter_width_m=cutter_width,
        overlap_ratio=overlap,
        min_turning_radius_m=turning_radius,
        estimated_path_length_m=_path_length(waypoints),
        headland_before_m=max(0.0, -float(min_forward)),
        headland_after_m=max(0.0, float(max_forward) - field_length),
        lateral_overflow_m=float(lateral_overflow),
        row_visit_order=tuple(int(index) for index in visit_order),
        turn_separations_m=tuple(float(value) for value in turn_separations),
        clean_uturn_count=sum(
            1
            for separation in turn_separations
            if is_clean_uturn(separation, min_turning_radius_m=turning_radius)
        ),
        strict_crossing_count=int(topology.strict_crossing_count),
        nonadjacent_touch_count=int(topology.nonadjacent_touch_count),
        collinear_overlap_count=int(topology.collinear_overlap_count),
        field_strict_crossing_count=int(field_topology.strict_crossing_count),
        field_nonadjacent_touch_count=int(
            field_topology.nonadjacent_touch_count
        ),
        field_collinear_overlap_count=int(
            field_topology.collinear_overlap_count
        ),
        topology_audit_spacing_m=float(audit_spacing),
        minimum_safe_lane_spacing_m=float(
            minimum_safe_lane_spacing_m(turning_radius)
        ),
    )


def build_lawnmower_waypoints(
    *,
    start_lat: float,
    start_lon: float,
    start_yaw_deg: float,
    field_length_m: float,
    field_width_m: float,
    cutter_width_m: float,
    overlap_ratio: float,
    min_turning_radius_m: float,
    waypoint_spacing_m: float = 2.0,
    side: str = "left",
    allow_headland_conflicts: bool = False,
    allow_row_skipping: bool = False,
    row_waypoint_spacing_m: Optional[float] = None,
    no_go_polygons_body: Optional[Sequence[Sequence[Tuple[float, float]]]] = None,
    no_go_margin_m: float = 0.0,
) -> Tuple[CoveragePlan, list[dict[str, object]]]:
    """Generar waypoints GPS de cobertura relativos a la pose inicial."""

    plan = build_lawnmower_body_plan(
        field_length_m=field_length_m,
        field_width_m=field_width_m,
        cutter_width_m=cutter_width_m,
        overlap_ratio=overlap_ratio,
        min_turning_radius_m=min_turning_radius_m,
        waypoint_spacing_m=waypoint_spacing_m,
        side=side,
        allow_headland_conflicts=allow_headland_conflicts,
        allow_row_skipping=allow_row_skipping,
        row_waypoint_spacing_m=row_waypoint_spacing_m,
    )
    # El recorte va aca y no despues de georreferenciar: en el marco del cuerpo
    # las cuentas son euclideas, y lo que salga de aca pasa por el mismo bucle de
    # georreferenciacion que el resto, asi que se comporta igual con el lote
    # derecho que en diagonal. Ademas esta funcion es comun al preview y al
    # arranque, con lo cual dibujo y ejecucion no pueden divergir.
    if no_go_polygons_body:
        # El rectangulo del lote en marco del cuerpo. Sin esto, una zona pegada
        # al borde hace que el rodeo salga del lote para bordearla, que es peor
        # que dar la vuelta larga por adentro.
        side_sign = 1.0 if str(side) == "left" else -1.0
        field_bounds = (
            0.0,
            float(field_length_m),
            min(0.0, side_sign * float(field_width_m)),
            max(0.0, side_sign * float(field_width_m)),
        )
        clipped, dropped, detours = clip_plan_to_nogo(
            plan.waypoints,
            no_go_polygons_body,
            margin_m=float(no_go_margin_m),
            bounds=field_bounds,
        )
        plan = replace(
            plan,
            waypoints=tuple(clipped),
            estimated_path_length_m=_path_length(clipped),
            nogo_polygon_count=len(no_go_polygons_body),
            nogo_dropped_count=int(dropped),
            nogo_detour_count=int(detours),
        )
    geographic_waypoints: list[dict[str, object]] = []
    for point in plan.waypoints:
        north_m, east_m = body_relative_offsets_to_north_east(
            start_yaw_deg=float(start_yaw_deg),
            forward_m=float(point.forward_m),
            left_m=float(point.left_m),
        )
        lat, lon = offset_lat_lon(
            lat_deg=float(start_lat),
            lon_deg=float(start_lon),
            north_m=float(north_m),
            east_m=float(east_m),
        )
        geographic_waypoints.append(
            {
                "lat": float(lat),
                "lon": float(lon),
                "yaw_deg": float(
                    normalize_yaw_deg(
                        float(start_yaw_deg) + float(point.yaw_delta_deg)
                    )
                ),
                "phase": str(point.phase),
                "row_index": int(point.row_index),
                "key": bool(point.is_key),
                "guide": bool(point.is_guide),
            }
        )
    return plan, geographic_waypoints
