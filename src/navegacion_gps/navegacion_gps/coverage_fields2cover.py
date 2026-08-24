"""Puente entre el modo Campo de SALUS y el Coverage Server de OpenNav.

Fields2Cover planifica sobre el poligono real del lote y trata las exclusiones
como anillos interiores, asi que las pasadas que cruzan una exclusion salen
partidas de fabrica en vez de parchearse despues.

Este modulo hace tres cosas y nada mas: arma la meta de la accion, la manda, y
traduce el resultado al mismo ``CoverageBodyWaypoint`` que ya produce el
planificador propio. Todo lo que viene despues de ``route_executor`` —el
troceado de la ruta, ``nav_command_server``, Nav2, el control Ackermann— no se
entera de que cambio el planificador.

Trabaja en el marco del cuerpo del lote, en metros: se le manda al server
coordenadas cartesianas y vuelven en el mismo marco, asi que la
georreferenciacion sigue siendo la de siempre.

**Executor propio.** El cliente vive en su propio nodo con su propio executor en
un hilo aparte. No es paranoia: el executor del ``route_executor`` tiene dos
hilos, y ya se midio que bloquear dentro de un callback de servicio esperando
otro servicio se va a timeout justo cuando hay trafico —que es siempre que el
cockpit esta conectado—. Planificar con Fields2Cover tarda segundos, o sea
bastante mas que aquella llamada que fallaba.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

try:
    from opennav_coverage_msgs.action import ComputeCoveragePath
    from opennav_coverage_msgs.msg import Coordinate, Coordinates
    _IMPORT_ERROR = ""
except ImportError as _exc:  # pragma: no cover - depende del overlay instalado
    # Importar esto arriba y sin red mataba al nodo ENTERO cuando el overlay de
    # Fields2Cover no estaba: route_executor no llegaba ni a arrancar y se caian
    # con el la ruta automatica, la patrulla y los goals, que no tienen nada que
    # ver con cobertura. El modulo tiene que poder importarse siempre; lo unico
    # que puede fallar es planificar Campo con fields2cover, y falla con un
    # mensaje que dice que instalar.
    ComputeCoveragePath = None  # type: ignore[assignment]
    Coordinate = None  # type: ignore[assignment]
    Coordinates = None  # type: ignore[assignment]
    _IMPORT_ERROR = str(_exc)

from navegacion_gps.coverage_waypoint_core import CoverageBodyWaypoint

Point = Tuple[float, float]

# Fases, iguales a las del planificador propio para que nada aguas abajo tenga
# que aprender vocabulario nuevo.
WORK_PHASE = "row"
TRANSITION_PHASE = "turn"
NOGO_LANE_CHANGE_PHASE = "nogo_lane_change"

# Que hacer con una fila que una zona no-go interna deja partida.
#   headland    : no se recorre; se enlaza el resto con el giro de cabecera.
#   lane_change : se cambia a la fila vecina con una S adentro del cultivo.
NOGO_INTERNAL_STRATEGY_HEADLAND = "headland"
NOGO_INTERNAL_STRATEGY_LANE_CHANGE = "lane_change"
# Una omega construida para cumplir la politica forward-only no es una guia
# decorativa de Fields2Cover: sus tres vertices fijan los arcos que un
# Ackermann necesita para cambiar de fila sin retroceder. El servidor web usa
# esta fase para conservarlos aunque las guias de cabecera comunes esten
# apagadas.
FORWARD_TURN_PHASE = "forward_turn"


class Fields2CoverError(RuntimeError):
    """Falla al planificar con el Coverage Server.

    Siempre se atrapa en el llamador: una excepcion que escape de un callback de
    servicio de rclpy mata el nodo entero, y eso dejaria sin ruta y sin patrulla
    a alguien que solo queria un preview de cobertura.
    """


class ForwardOnlyTurnError(Fields2CoverError):
    """No hay enlace hacia adelante entre dos pasadas con la reversa apagada.

    Se levanta a proposito en vez de caer de vuelta en la cabecera de tres
    puntos: el perfil de simulacion no puede retroceder, y reemplazar la
    maniobra en silencio por una marcha atras seria justamente el error que
    esta politica busca impedir.
    """


@dataclass
class Fields2CoverPlan:
    """Resultado ya traducido al vocabulario de SALUS."""

    waypoints: List[CoverageBodyWaypoint] = field(default_factory=list)
    swath_count: int = 0
    lane_spacing_m: float = 0.0
    work_length_m: float = 0.0
    transition_length_m: float = 0.0
    route_type: str = ""
    path_type: str = ""
    internal_nogo_dropped_waypoint_count: int = 0

    @property
    def total_length_m(self) -> float:
        """Largo de trabajo mas largo de transiciones."""
        return float(self.work_length_m + self.transition_length_m)


def _yaw_from_quaternion(orientation: Any) -> float:
    """Rumbo en grados a partir del cuaternion de una pose."""
    z = float(orientation.z)
    w = float(orientation.w)
    x = float(orientation.x)
    y = float(orientation.y)
    siny = 2.0 * ((w * z) + (x * y))
    cosy = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.degrees(math.atan2(siny, cosy))


def _sample_segment(start: Point, end: Point, spacing_m: float) -> List[Point]:
    """Puntos sobre el segmento, incluidos los dos extremos."""
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(length / max(0.05, float(spacing_m)))))
    return [
        (
            start[0] + ((end[0] - start[0]) * (index / steps)),
            start[1] + ((end[1] - start[1]) * (index / steps)),
        )
        for index in range(steps + 1)
    ]


def _lane_spacing_from_swaths(swaths: Sequence[Any]) -> float:
    """Separacion nominal entre las lineas paralelas de trabajo.

    Se ordenan los offsets geometricos en vez de usar el orden de visita. Asi
    el reporte sigue dando el ancho entre surcos cuando ``SNAKE`` saltea filas
    o una exclusion parte una misma linea en dos swaths.
    """
    swath_list = list(swaths)
    if len(swath_list) < 2:
        return 0.0

    reference = None
    for swath in swath_list:
        dx = float(swath.end.x) - float(swath.start.x)
        dy = float(swath.end.y) - float(swath.start.y)
        length = math.hypot(dx, dy)
        if length > 1.0e-9:
            reference = (-dy / length, dx / length)
            break
    if reference is None:
        return 0.0

    offsets = sorted(
        (
            (0.5 * (float(swath.start.x) + float(swath.end.x)) * reference[0])
            + (0.5 * (float(swath.start.y) + float(swath.end.y)) * reference[1])
        )
        for swath in swath_list
    )
    unique_offsets: List[float] = []
    for offset in offsets:
        if not unique_offsets or abs(offset - unique_offsets[-1]) > 1.0e-6:
            unique_offsets.append(float(offset))
    gaps = sorted(
        unique_offsets[index + 1] - unique_offsets[index]
        for index in range(len(unique_offsets) - 1)
        if unique_offsets[index + 1] - unique_offsets[index] > 1.0e-6
    )
    if not gaps:
        return 0.0
    middle = len(gaps) // 2
    if len(gaps) % 2:
        return float(gaps[middle])
    return float(0.5 * (gaps[middle - 1] + gaps[middle]))


def fields2cover_disponible() -> bool:
    """Si el overlay de Fields2Cover esta instalado en este entorno."""
    return not _IMPORT_ERROR


def _ring_to_coordinates(ring: Sequence[Point]) -> Any:
    """Anillo en metros locales al tipo que espera la accion.

    Se cierra repitiendo el primer vertice al final. SALUS maneja los anillos
    abiertos, pero Fields2Cover los pasa a OGR y ahi un anillo sin cerrar es
    invalido: la accion responde INVALID_COORDS (803) sin mas explicacion.
    """
    puntos = [(float(x), float(y)) for x, y in ring]
    if puntos and puntos[0] != puntos[-1]:
        puntos.append(puntos[0])
    out = Coordinates()
    out.coordinates = [Coordinate(axis1=x, axis2=y) for x, y in puntos]
    return out


def plan_to_body_waypoints(
    result: Any,
    *,
    waypoint_spacing_m: float,
) -> Fields2CoverPlan:
    """Traducir el resultado de la accion a waypoints de SALUS.

    Se usa ``coverage_path`` y no ``nav_path`` porque el primero viene separado
    en ``swaths`` (trabajo) y ``turns`` (transiciones). Esa separacion es la que
    despues necesita el implemento para saber cuando cortar, y la que permite
    medir trabajo y giros por separado. El ``nav_path`` es la misma trayectoria
    pero ya aplanada, sin decir que es cada tramo.
    """
    components = result.coverage_path
    swaths = list(components.swaths)
    turns = list(components.turns)
    if not swaths:
        raise Fields2CoverError("Fields2Cover no devolvio ninguna pasada")

    plan = Fields2CoverPlan(
        swath_count=len(swaths),
        lane_spacing_m=_lane_spacing_from_swaths(swaths),
    )
    waypoints: List[CoverageBodyWaypoint] = []

    for index, swath in enumerate(swaths):
        start = (float(swath.start.x), float(swath.start.y))
        end = (float(swath.end.x), float(swath.end.y))
        plan.work_length_m += math.hypot(end[0] - start[0], end[1] - start[1])
        heading = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        muestras = _sample_segment(start, end, waypoint_spacing_m)
        for sample_index, (x, y) in enumerate(muestras):
            # Solo los extremos de pasada son metas de parada, igual que en el
            # planificador propio: los intermedios son para dibujar y auditar.
            es_extremo = sample_index in (0, len(muestras) - 1)
            waypoints.append(
                CoverageBodyWaypoint(
                    forward_m=float(x),
                    left_m=float(y),
                    yaw_delta_deg=float(heading),
                    phase=WORK_PHASE,
                    row_index=int(index),
                    is_key=bool(es_extremo),
                )
            )

        if index < len(turns):
            poses = list(turns[index].poses)
            previa: Optional[Point] = end
            for pose in poses:
                punto = (float(pose.pose.position.x), float(pose.pose.position.y))
                if previa is not None:
                    plan.transition_length_m += math.hypot(
                        punto[0] - previa[0], punto[1] - previa[1]
                    )
                previa = punto
                waypoints.append(
                    CoverageBodyWaypoint(
                        forward_m=punto[0],
                        left_m=punto[1],
                        yaw_delta_deg=_yaw_from_quaternion(pose.pose.orientation),
                        phase=TRANSITION_PHASE,
                        row_index=int(index),
                        is_key=False,
                        # La curva no es una meta de trabajo, pero si una guia
                        # obligatoria: sin ella el ejecutor une dos extremos de
                        # pasada con una recta y no sigue el preview de F2C.
                        is_guide=True,
                    )
                )

    plan.waypoints = waypoints
    return plan


# Tramo recto que despeja el implemento antes y despues de los dos arcos.
DEFAULT_HEADLAND_LEAD_M = 0.5

# Distancia por debajo de la cual dos puntos son el mismo y no se agrega guia.
_EPSILON_M = 1.0e-6


def _direction_from_yaw(yaw_deg: float) -> Point:
    """Versor a partir de un rumbo en grados."""
    radianes = math.radians(float(yaw_deg))
    return (math.cos(radianes), math.sin(radianes))


def _unit(desde: Point, hasta: Point) -> Optional[Point]:
    """Versor de ``desde`` a ``hasta``, o None si son el mismo punto."""
    dx = float(hasta[0]) - float(desde[0])
    dy = float(hasta[1]) - float(desde[1])
    largo = math.hypot(dx, dy)
    if largo <= _EPSILON_M:
        return None
    return (dx / largo, dy / largo)


def _work_runs(
    waypoints: Sequence[CoverageBodyWaypoint],
) -> List[List[CoverageBodyWaypoint]]:
    """Tramos contiguos de trabajo, en orden de visita.

    Se agrupa por contiguidad y no por ``row_index`` a proposito: el orden de la
    lista ES el orden de recorrido, y agrupar por indice mezclaria dos visitas a
    la misma pasada si alguna vez las hubiera.
    """
    runs: List[List[CoverageBodyWaypoint]] = []
    actual: List[CoverageBodyWaypoint] = []
    for waypoint in waypoints:
        if str(waypoint.phase) in {WORK_PHASE, NOGO_LANE_CHANGE_PHASE}:
            # El reordenamiento de swaths partidos por un no-go deja solamente
            # puntos de trabajo: las transiciones originales ya no describen
            # el nuevo orden y se descartan. El indice sigue separando dos
            # pasadas contiguas aun cuando no haya un punto ``turn`` entre
            # ellas.
            if actual and int(waypoint.row_index) != int(actual[-1].row_index):
                runs.append(actual)
                actual = []
            actual.append(waypoint)
            continue
        if actual:
            runs.append(actual)
            actual = []
    if actual:
        runs.append(actual)
    return runs


def _point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    """Ray casting chico para reconocer el hueco entre dos medios swaths."""
    if len(polygon) < 3:
        return False
    x, y = float(point[0]), float(point[1])
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = float(previous[0]), float(previous[1])
        x2, y2 = float(current[0]), float(current[1])
        if (y1 > y) != (y2 > y):
            crossing_x = x1 + ((y - y1) * (x2 - x1) / (y2 - y1))
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def _reverse_work_run(
    run: Sequence[CoverageBodyWaypoint],
) -> List[CoverageBodyWaypoint]:
    """Invertir una pasada sin cambiar la superficie que cubre."""
    return [
        replace(
            waypoint,
            yaw_delta_deg=(float(waypoint.yaw_delta_deg) + 180.0) % 360.0,
        )
        for waypoint in reversed(run)
    ]


def _orient_run_from(
    run: Sequence[CoverageBodyWaypoint],
    current: Point,
) -> List[CoverageBodyWaypoint]:
    """Orientar el swath desde el extremo mas cercano a la posicion actual."""
    normal = list(run)
    reverse_run = _reverse_work_run(run)
    start_normal = (float(normal[0].forward_m), float(normal[0].left_m))
    start_reverse = (
        float(reverse_run[0].forward_m),
        float(reverse_run[0].left_m),
    )
    if math.dist(current, start_reverse) + _EPSILON_M < math.dist(
        current, start_normal
    ):
        return reverse_run
    return normal


# Cuanto tiene que meterse una fila dentro de la exclusion inflada para que
# valga la pena cambiarla de fila. Con el implemento centrado en la ruta, la
# exclusion ya trae medio ancho de trabajo mas el colchon fijo, asi que una fila
# TANGENTE cumple el despeje exacto: no hay nada que esquivar. Y como la
# separacion entre pasadas suele ser igual al ancho de trabajo, las vecinas de
# la fila bloqueada caen justo sobre el borde: sin esta tolerancia, un empate
# numerico las mandaba a cambiar de fila y a repisar una pasada ya hecha.
_TANGENT_EXCLUSION_TOLERANCE_M = 0.10


def _distance_to_polygon_edges(point: Point, polygon: Sequence[Point]) -> float:
    """Distancia del punto al borde del poligono, sin signo."""
    vertices = list(polygon)
    if len(vertices) < 2:
        return math.inf
    best = math.inf
    for index, start in enumerate(vertices):
        end = vertices[(index + 1) % len(vertices)]
        seg_x = float(end[0]) - float(start[0])
        seg_y = float(end[1]) - float(start[1])
        length_sq = (seg_x * seg_x) + (seg_y * seg_y)
        if length_sq <= 1.0e-12:
            best = min(best, math.dist(point, (float(start[0]), float(start[1]))))
            continue
        ratio = (
            ((point[0] - float(start[0])) * seg_x)
            + ((point[1] - float(start[1])) * seg_y)
        ) / length_sq
        ratio = max(0.0, min(1.0, ratio))
        projection = (
            float(start[0]) + (seg_x * ratio),
            float(start[1]) + (seg_y * ratio),
        )
        best = min(best, math.dist(point, projection))
    return best


def _point_inside_beyond_tolerance(
    point: Point,
    polygon: Sequence[Point],
    tolerance_m: float,
) -> bool:
    """Adentro de verdad, no apenas apoyado sobre el borde."""
    if not _point_in_polygon(point, polygon):
        return False
    return _distance_to_polygon_edges(point, polygon) > float(tolerance_m)


def _merge_collinear_runs(
    runs: Sequence[Sequence[CoverageBodyWaypoint]],
) -> List[CoverageBodyWaypoint]:
    """Volver a unir las mitades de una fila que no hacia falta partir."""
    points = [waypoint for run in runs for waypoint in run]
    if len(points) < 2:
        return list(points)
    origin = (float(points[0].forward_m), float(points[0].left_m))
    far = max(
        points,
        key=lambda waypoint: math.dist(
            origin, (float(waypoint.forward_m), float(waypoint.left_m))
        ),
    )
    axis = (
        float(far.forward_m) - origin[0],
        float(far.left_m) - origin[1],
    )
    norm = math.hypot(axis[0], axis[1])
    if norm <= _EPSILON_M:
        return list(points)
    unit = (axis[0] / norm, axis[1] / norm)
    return sorted(
        points,
        key=lambda waypoint: (
            ((float(waypoint.forward_m) - origin[0]) * unit[0])
            + ((float(waypoint.left_m) - origin[1]) * unit[1])
        ),
    )


def _split_lane_crosses_exclusion(
    runs: Sequence[Sequence[CoverageBodyWaypoint]],
    exclusions: Sequence[Sequence[Point]],
) -> bool:
    """Si dos medios swaths estan separados por uno de los no-go internos."""
    if len(runs) != 2 or not exclusions:
        return False
    first_endpoints = (runs[0][0], runs[0][-1])
    second_endpoints = (runs[1][0], runs[1][-1])
    closest = min(
        (
            (
                math.hypot(
                    float(a.forward_m) - float(b.forward_m),
                    float(a.left_m) - float(b.left_m),
                ),
                a,
                b,
            )
            for a in first_endpoints
            for b in second_endpoints
        ),
        key=lambda item: item[0],
    )
    midpoint = (
        0.5 * (float(closest[1].forward_m) + float(closest[2].forward_m)),
        0.5 * (float(closest[1].left_m) + float(closest[2].left_m)),
    )
    return any(
        _point_inside_beyond_tolerance(
            midpoint, polygon, _TANGENT_EXCLUSION_TOLERANCE_M
        )
        for polygon in exclusions
    )


def _ray_to_boundary(
    origin: Point,
    direction: Point,
    boundary: Sequence[Point],
) -> Optional[Point]:
    """Primera interseccion positiva de un rayo con el contorno del lote."""
    best_t = math.inf
    best: Optional[Point] = None
    for index, start in enumerate(boundary):
        end = boundary[(index + 1) % len(boundary)]
        edge = (float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))
        denominator = (direction[0] * edge[1]) - (direction[1] * edge[0])
        if abs(denominator) <= _EPSILON_M:
            continue
        relative = (float(start[0]) - origin[0], float(start[1]) - origin[1])
        ray_t = ((relative[0] * edge[1]) - (relative[1] * edge[0])) / denominator
        edge_t = (
            (relative[0] * direction[1]) - (relative[1] * direction[0])
        ) / denominator
        if ray_t < -1.0e-6 or edge_t < -1.0e-6 or edge_t > 1.0 + 1.0e-6:
            continue
        if ray_t < best_t:
            best_t = max(0.0, ray_t)
            best = (
                origin[0] + (best_t * direction[0]),
                origin[1] + (best_t * direction[1]),
            )
    return best


def _extend_run_to_boundary(
    run: Sequence[CoverageBodyWaypoint],
    boundary: Sequence[Point],
) -> List[CoverageBodyWaypoint]:
    """Extender recto el eje de una fila hasta las dos cabeceras reales."""
    result = list(run)
    if len(result) < 2 or len(boundary) < 3:
        return result
    direction = _unit(
        (float(result[0].forward_m), float(result[0].left_m)),
        (float(result[-1].forward_m), float(result[-1].left_m)),
    )
    if direction is None:
        return result
    start = (float(result[0].forward_m), float(result[0].left_m))
    end = (float(result[-1].forward_m), float(result[-1].left_m))
    boundary_start = _ray_to_boundary(
        start, (-direction[0], -direction[1]), boundary
    )
    boundary_end = _ray_to_boundary(end, direction, boundary)
    if boundary_start is not None:
        result[0] = replace(
            result[0],
            forward_m=float(boundary_start[0]),
            left_m=float(boundary_start[1]),
        )
    if boundary_end is not None:
        result[-1] = replace(
            result[-1],
            forward_m=float(boundary_end[0]),
            left_m=float(boundary_end[1]),
        )
    return result


def _extend_run_outer_start_to_boundary(
    run: Sequence[CoverageBodyWaypoint],
    boundary: Sequence[Point],
) -> List[CoverageBodyWaypoint]:
    """Extender solo la cabecera exterior de una media fila no-go.

    ``run`` ya esta orientada desde el contorno hacia la zona. Extender tambien
    el ultimo punto reconstruiria la mitad eliminada por Fields2Cover y trazaria
    una recta a traves del obstaculo antes de volver al inicio de la S.
    """
    result = list(run)
    if len(result) < 2 or len(boundary) < 3:
        return result
    direction = _unit(
        (float(result[0].forward_m), float(result[0].left_m)),
        (float(result[-1].forward_m), float(result[-1].left_m)),
    )
    if direction is None:
        return result
    start = (float(result[0].forward_m), float(result[0].left_m))
    boundary_start = _ray_to_boundary(
        start,
        (-direction[0], -direction[1]),
        boundary,
    )
    if boundary_start is not None:
        result[0] = replace(
            result[0],
            forward_m=float(boundary_start[0]),
            left_m=float(boundary_start[1]),
        )
    return result


def _lane_change_anticipation_m(radius_m: float, lane_spacing_m: float) -> float:
    """Largo de fila que consume la S de un cambio de una pasada.

    Sale de la geometria de los dos arcos tangentes: ``sqrt(s * (4R - s))``.
    Con separacion 4 m da 7.38 m para R=4.4 y 5.66 m para R=3.0, que es lo que
    se mide en el plan.
    """
    s = abs(float(lane_spacing_m))
    r = float(radius_m)
    if s <= _EPSILON_M or r < (0.5 * s):
        return math.inf
    return math.sqrt(s * ((4.0 * r) - s))


def _lane_change_radius_for_run_m(available_m: float, lane_spacing_m: float) -> float:
    """Radio mas grande cuya S entra en ``available_m`` de fila."""
    s = abs(float(lane_spacing_m))
    a = max(0.0, float(available_m))
    if s <= _EPSILON_M:
        return 0.0
    return ((a * a / s) + s) / 4.0


def _lane_change_escape_run(
    run: Sequence[CoverageBodyWaypoint],
    *,
    exclusions: Sequence[Sequence[Point]],
    field_boundary: Sequence[Point],
    lane_spacing_m: float,
    min_turning_radius_m: float,
    lane_change_radius_m: float = 0.0,
    lane_change_min_radius_m: float = 0.0,
) -> Tuple[List[CoverageBodyWaypoint], int]:
    """Anticipar el no-go, cambiar una fila y escapar recto a la cabecera."""
    if len(run) < 2:
        raise Fields2CoverError("una media fila no-go no tiene largo util")

    endpoints = [
        (float(run[0].forward_m), float(run[0].left_m)),
        (float(run[-1].forward_m), float(run[-1].left_m)),
    ]

    def exclusion_distance(point: Point) -> float:
        return min(
            math.dist(point, (float(vertex[0]), float(vertex[1])))
            for polygon in exclusions
            for vertex in polygon
        )

    # El extremo mas cercano al no-go es el callejon sin salida. La pasada se
    # orienta desde la cabecera hacia ese extremo para anticipar el cambio.
    oriented = (
        list(run)
        if exclusion_distance(endpoints[-1]) <= exclusion_distance(endpoints[0])
        else _reverse_work_run(run)
    )
    outer = (float(oriented[0].forward_m), float(oriented[0].left_m))
    inner = (float(oriented[-1].forward_m), float(oriented[-1].left_m))
    direction = _unit(outer, inner)
    if direction is None:
        raise Fields2CoverError("una media fila no-go tiene rumbo indefinido")

    nearest_polygon = min(
        exclusions,
        key=lambda polygon: min(
            math.dist(inner, (float(vertex[0]), float(vertex[1])))
            for vertex in polygon
        ),
    )
    center = (
        sum(float(vertex[0]) for vertex in nearest_polygon) / len(nearest_polygon),
        sum(float(vertex[1]) for vertex in nearest_polygon) / len(nearest_polygon),
    )
    normal = (-direction[1], direction[0])
    center_side = (
        ((center[0] - inner[0]) * normal[0])
        + ((center[1] - inner[1]) * normal[1])
    )
    away = (
        (-normal[0], -normal[1])
        if center_side > 0.0
        else normal
    )

    # Cuantas pasadas hay que correrse para salir de verdad de la exclusion.
    #
    # Correrse UNA fila alcanza solo si la separacion entre pasadas supera el
    # alcance de la exclusion. Con 4 m de corte (separacion 4.00 m) contra una
    # exclusion de 4.00 m, alcanza justo. Con 2 m de corte y 15% de solape la
    # separacion baja a 1.70 m y la exclusion de una zona de 1.5 m llega a
    # 3.00 m: correrse una fila deja al vehiculo todavia adentro, y por eso el
    # plan se rechazaba con "el cambio de fila interno invade la zona no-go".
    #
    # Se mide cuanto se mete la exclusion en la direccion de escape y se corre
    # el multiplo de separacion que la deja atras. Sigue siendo UN solo cambio
    # de fila, no un paseo por el lote.
    separacion = abs(float(lane_spacing_m))
    # Solo cuenta la zona que de verdad cruza ESTA fila, y solo si la cruza.
    # Proyectando todas las exclusiones desde el punto de la fila, una zona
    # lejana sumaba su propia distancia al alcance y mandaba a correrse cinco
    # filas para esquivar algo que esa pasada ni tocaba.
    proyecciones = [
        ((float(vertex[0]) - inner[0]) * away[0])
        + ((float(vertex[1]) - inner[1]) * away[1])
        for vertex in nearest_polygon
    ]
    pasadas = 1
    if proyecciones and separacion > _EPSILON_M:
        cerca_m = min(proyecciones)
        lejos_m = max(proyecciones)
        # La fila esta bloqueada solo si la exclusion la cruza de lado a lado.
        # Si queda entera de un lado, correrse una pasada ya la deja atras.
        if cerca_m < 0.0 < lejos_m:
            # Se usa la MISMA tolerancia con la que se decide si una fila esta
            # bloqueada. Una fila tangente ya cuenta como libre, asi que exigir
            # margen extra aca mandaba a correrse dos pasadas donde una
            # alcanzaba: con exclusion de 4.01 m y separacion de 4.00 m,
            # ceil(4.06/4.00)=2 en vez de 1, y el corrimiento de 8 m rompia la S.
            faltante_m = lejos_m - _TANGENT_EXCLUSION_TOLERANCE_M
            pasadas = max(1, int(math.ceil(faltante_m / separacion)))
    shift = separacion * float(pasadas)
    # El esquive del no-go no es una cabecera: es una S suave, sin inversion de
    # marcha y adentro del cultivo. Usar el radio de cabecera lo vuelve enorme
    # -con 4.0 m de radio y 4.0 m de separacion la S arranca 7.4 m antes de la
    # zona- y ese rodeo se come filas que no hacia falta tocar. Por eso lleva su
    # propio radio, mas chico que el de cabecera pero con margen sobre el limite
    # fisico de direccion. Con 0.0 se mantiene el comportamiento anterior.
    requested_radius = float(lane_change_radius_m)
    radius = (
        requested_radius
        if requested_radius > 0.0
        else float(min_turning_radius_m) * 1.10
    )
    # Piso geometrico: con radio menor que la mitad de la separacion la S no
    # cierra el cambio de una fila.
    radius = max(radius, (0.5 * shift) + 0.05)
    if shift <= _EPSILON_M or shift > (2.0 * radius):
        raise Fields2CoverError(
            "no se puede cambiar exactamente una fila con la separacion y "
            "el radio de giro configurados"
        )
    run_length = math.dist(outer, inner)
    disponible = run_length - 0.05

    # La S de un cambio de fila cumple, exactamente:
    #     anticipacion = sqrt(s * (4R - s))        con s = separacion de pasada
    # y su inversa da el radio mas grande que entra en el largo disponible:
    #     R = (A^2 / s + s) / 4
    # Cuando el radio preferido no entra, en vez de rechazar el plan se achica
    # hasta el mayor que si entre, nunca por debajo del piso configurado. El
    # piso protege el limite fisico de direccion: el geometrico es s/2 -con
    # separacion de 4 m, 2.0 m-, donde la S degenera en dos cuartos de circulo.
    piso_m = max((0.5 * shift) + 0.05, float(lane_change_min_radius_m))
    if _lane_change_anticipation_m(radius, shift) > disponible:
        radius = max(piso_m, min(radius, _lane_change_radius_for_run_m(disponible, shift)))

    if shift > (2.0 * radius):
        raise Fields2CoverError(
            "no se puede cambiar exactamente una fila con la separacion y "
            "el radio de giro configurados"
        )
    theta = math.acos(max(-1.0, min(1.0, 1.0 - (shift / (2.0 * radius)))))
    anticipation = 2.0 * radius * math.sin(theta)
    if anticipation > disponible:
        raise Fields2CoverError(
            "la zona no-go queda demasiado cerca de la cabecera para anticipar "
            f"un cambio de una fila: necesita {anticipation:.2f} m con el radio "
            f"minimo de {radius:.2f} m y hay {run_length:.2f} m"
        )

    change_start = (
        inner[0] - (anticipation * direction[0]),
        inner[1] - (anticipation * direction[1]),
    )
    midpoint = (
        change_start[0]
        + (radius * math.sin(theta) * direction[0])
        + (radius * (1.0 - math.cos(theta)) * away[0]),
        change_start[1]
        + (radius * math.sin(theta) * direction[1])
        + (radius * (1.0 - math.cos(theta)) * away[1]),
    )
    change_end = (
        change_start[0]
        + (anticipation * direction[0])
        + (shift * away[0]),
        change_start[1]
        + (anticipation * direction[1])
        + (shift * away[1]),
    )
    escape = _ray_to_boundary(change_end, direction, field_boundary)
    if escape is None:
        raise Fields2CoverError(
            "el cambio de fila no encuentra una cabecera exterior por delante"
        )

    # Tres puntos definen la S: inicio tangente, cambio de curvatura y fin
    # tangente. Los tres son guias del mismo FollowPath que trae al vehiculo por
    # la fila recta y lo deja en la cabecera: el bloque cierra recien en el
    # escape, que se alcanza yendo derecho.
    #
    # El cambio de curvatura NO es key. Siendolo, el bloque terminaba justo en
    # el apex y le exigia al Ackermann parar ahi con la tolerancia de trabajo
    # (0.35 m). Barriendo una curva de 3 m de radio no se llega: el vehiculo
    # pasaba a 1.3 m, la meta no se daba nunca por cumplida, se iba de largo y
    # Nav2 abortaba con ``Resulting plan has 0 poses in it``. Como guia, el apex
    # es un punto interno del camino y la precision se pide donde el vehiculo va
    # derecho. No cambia la cantidad de waypoints, solo cual corta el bloque.
    #
    # El radio sale de ``lane_change_radius_m``; si no viene, se cae al minimo
    # de Smac con un 10% de margen para no quedar justo en el limite numerico
    # del lattice.
    heading_mid = (
        (math.cos(theta) * direction[0]) + (math.sin(theta) * away[0]),
        (math.cos(theta) * direction[1]) + (math.sin(theta) * away[1]),
    )
    along_limit = run_length - anticipation
    kept = []
    for waypoint in oriented[:-1]:
        position = (float(waypoint.forward_m), float(waypoint.left_m))
        along = (
            ((position[0] - outer[0]) * direction[0])
            + ((position[1] - outer[1]) * direction[1])
        )
        if along < along_limit - 1.0e-6:
            kept.append(waypoint)
    if not kept:
        kept.append(oriented[0])
    kept = _extend_run_outer_start_to_boundary(kept, field_boundary)
    kept_original_count = len(kept)
    base = oriented[-1]
    lane_points = [
        (change_start, direction, False, True),
        (midpoint, heading_mid, False, True),
        (change_end, direction, False, True),
        (escape, direction, True, False),
    ]
    for position, heading, is_key, is_guide in lane_points:
        kept.append(
            CoverageBodyWaypoint(
                forward_m=float(position[0]),
                left_m=float(position[1]),
                yaw_delta_deg=float(
                    math.degrees(math.atan2(heading[1], heading[0]))
                ),
                phase=NOGO_LANE_CHANGE_PHASE,
                row_index=int(base.row_index),
                is_key=bool(is_key),
                is_guide=bool(is_guide),
                backup_m=0.0,
            )
        )
    dropped = max(0, len(oriented) - kept_original_count)
    return kept, dropped


def reorder_internal_nogo_swaths(
    plan: Fields2CoverPlan,
    exclusions: Sequence[Sequence[Point]],
    *,
    field_boundary: Sequence[Point] = (),
    min_turning_radius_m: float = 0.0,
    lane_change_radius_m: float = 0.0,
    lane_change_min_radius_m: float = 0.0,
    internal_strategy: str = NOGO_INTERNAL_STRATEGY_LANE_CHANGE,
) -> Fields2CoverPlan:
    """Resolver un extremo interno con un unico cambio de fila anticipado.

    Cada media pasada se recorre desde la cabecera hacia el obstaculo. Antes de
    llegar se hace una S a la fila vecina, alejandose del no-go, y desde ahi se
    sigue recto hasta la cabecera opuesta. No hay omega, vuelta al circulo,
    diagonal libre ni reversa dentro del lote. La fila vecina puede repetirse:
    es la via de escape que mantiene al vehiculo siempre sobre filas.
    """
    runs = _work_runs(plan.waypoints)
    if len(runs) < 4 or not exclusions:
        return replace(plan, waypoints=list(plan.waypoints))
    if len(field_boundary) < 3:
        raise Fields2CoverError(
            "el cambio de fila no-go necesita el contorno original del lote"
        )

    reference = _unit(
        (float(runs[0][0].forward_m), float(runs[0][0].left_m)),
        (float(runs[0][-1].forward_m), float(runs[0][-1].left_m)),
    )
    if reference is None:
        return replace(plan, waypoints=list(plan.waypoints))
    normal = (-reference[1], reference[0])
    lane_tolerance = max(0.05, abs(float(plan.lane_spacing_m)) * 0.15)

    lane_groups: List[Dict[str, Any]] = []
    for run in runs:
        midpoint = (
            0.5 * (float(run[0].forward_m) + float(run[-1].forward_m)),
            0.5 * (float(run[0].left_m) + float(run[-1].left_m)),
        )
        offset = (midpoint[0] * normal[0]) + (midpoint[1] * normal[1])
        matching_group = next(
            (
                group
                for group in lane_groups
                if abs(float(group["offset"]) - offset) <= lane_tolerance
            ),
            None,
        )
        if matching_group is None:
            lane_groups.append({"offset": float(offset), "runs": [list(run)]})
        else:
            # Fields2Cover no garantiza que las dos mitades de un swath
            # partido aparezcan contiguas. En lotes grandes puede intercalar
            # mitades de otras filas; agrupar solo contra el elemento anterior
            # las trataba como pasadas completas y `_extend_run_to_boundary`
            # reconstruia una recta a traves de la zona no-go.
            matching_group["runs"].append(list(run))

    affected = [
        _split_lane_crosses_exclusion(group["runs"], exclusions)
        for group in lane_groups
    ]
    if not any(affected):
        return replace(plan, waypoints=list(plan.waypoints))

    estrategia = str(internal_strategy or "").strip().lower()
    if estrategia not in (
        NOGO_INTERNAL_STRATEGY_HEADLAND,
        NOGO_INTERNAL_STRATEGY_LANE_CHANGE,
    ):
        estrategia = NOGO_INTERNAL_STRATEGY_LANE_CHANGE

    ordered: List[List[CoverageBodyWaypoint]] = []
    dropped_waypoints = 0
    current = (
        float(runs[0][0].forward_m),
        float(runs[0][0].left_m),
    )
    for group, is_affected in zip(lane_groups, affected):
        candidates: List[List[CoverageBodyWaypoint]] = []
        if is_affected and estrategia == NOGO_INTERNAL_STRATEGY_HEADLAND:
            # La fila bloqueada no se recorre. Cada mitad tiene un extremo
            # contra la zona, asi que entrar por la cabecera deja al vehiculo
            # en un callejon: para volver a salir habria que retroceder o
            # maniobrar adentro del cultivo. Se saltea entera y las filas que
            # quedan se enlazan con el giro de cabecera de siempre, que pide
            # 13 grados de direccion contra los 17-20 de la S.
            dropped_waypoints += sum(len(run) for run in group["runs"])
        elif is_affected:
            for run in group["runs"]:
                try:
                    changed, dropped = _lane_change_escape_run(
                        run,
                        exclusions=exclusions,
                        field_boundary=field_boundary,
                        lane_spacing_m=float(plan.lane_spacing_m),
                        min_turning_radius_m=float(min_turning_radius_m),
                        lane_change_radius_m=float(lane_change_radius_m),
                        lane_change_min_radius_m=float(lane_change_min_radius_m),
                    )
                except Fields2CoverError:
                    # La S no entra: la zona quedo demasiado cerca de la
                    # cabecera y no hay fila libre para anticipar el cambio.
                    #
                    # Antes esto rechazaba el PLAN ENTERO. El cockpit se
                    # quedaba sin ruta con la zona aplicada, su recorte local
                    # intentaba arreglarla caminando el perimetro de la
                    # exclusion, y con una zona circular de 32 vertices eso
                    # dibujaba una espiral de treinta y pico de puntos.
                    #
                    # Se cae a la estrategia de cabecera SOLO para esta media
                    # fila: no se recorre y el resto del lote se enlaza normal.
                    # Perder una pasada es mejor que no entregar plan.
                    dropped_waypoints += len(run)
                    continue
                candidates.append(changed)
                dropped_waypoints += dropped
        else:
            # Fields2Cover parte el swath apenas roza la exclusion inflada. Si
            # el hueco no era un conflicto real, las mitades se vuelven a unir
            # en UNA fila entera: extenderlas por separado hasta el contorno
            # dejaba dos pasadas superpuestas sobre la misma linea.
            group_runs = list(group["runs"])
            if len(group_runs) > 1:
                candidates.append(_merge_collinear_runs(group_runs))
            else:
                candidates.extend(list(run) for run in group_runs)

        while candidates:
            candidate_index = min(
                range(len(candidates)),
                key=lambda index: math.dist(
                    current,
                    (
                        float(candidates[index][0].forward_m),
                        float(candidates[index][0].left_m),
                    ),
                ),
            )
            selected = candidates.pop(candidate_index)
            if not is_affected:
                selected = _orient_run_from(selected, current)
                selected = _extend_run_to_boundary(selected, field_boundary)
            ordered.append(selected)
            current = (
                float(selected[-1].forward_m),
                float(selected[-1].left_m),
            )

    # Los indices de Fields2Cover eran indices de swath en orden de visita. Al
    # cambiar ese orden se renumeran para que el contrato siga siendo cierto y
    # la auditoria N -> N+1 no confunda el id viejo con un salto real.
    flattened: List[CoverageBodyWaypoint] = []
    for row_index, run in enumerate(ordered):
        flattened.extend(replace(point, row_index=row_index) for point in run)
    return replace(
        plan,
        waypoints=flattened,
        swath_count=len(ordered),
        internal_nogo_dropped_waypoint_count=(
            int(plan.internal_nogo_dropped_waypoint_count) + dropped_waypoints
        ),
    )


def _exit_direction(run: Sequence[CoverageBodyWaypoint]) -> Point:
    """Direccion con la que se abandona la pasada."""
    ultimo = run[-1]
    fin = (float(ultimo.forward_m), float(ultimo.left_m))
    for previo in reversed(run[:-1]):
        versor = _unit((float(previo.forward_m), float(previo.left_m)), fin)
        if versor is not None:
            return versor
    return _direction_from_yaw(ultimo.yaw_delta_deg)


def _entry_direction(run: Sequence[CoverageBodyWaypoint]) -> Point:
    """Direccion con la que se entra a la pasada."""
    primero = run[0]
    inicio = (float(primero.forward_m), float(primero.left_m))
    for siguiente in run[1:]:
        versor = _unit(inicio, (float(siguiente.forward_m), float(siguiente.left_m)))
        if versor is not None:
            return versor
    return _direction_from_yaw(primero.yaw_delta_deg)


# Cabecera de tres puntos. Cuando las pasadas quedan mas juntas que el diametro
# de giro (2R), NO existe ninguna curva hacia adelante que lleve del final de un
# surco al inicio del siguiente: Fields2Cover resuelve eso con una omega, que se
# come 7.3 m de cabecera y dibuja los petalos. Retrocediendo, la misma
# transicion entra en R metros de cabecera.
#
# La maniobra, con R el radio y d la separacion entre pasadas:
#
#   1. salir derecho del surco `lead` metros          (despeja el implemento)
#   2. arco de 90 grados hacia el surco siguiente
#   3. marcha atras recta de L = 2R - d metros
#   4. otro arco de 90 grados, que deja el rumbo invertido
#   5. entrar derecho al surco siguiente `lead` metros
#
# L sale de cerrar la geometria: los dos arcos desplazan 2R de costado, y la
# reversa descuenta lo que sobra hasta la separacion real. Si d >= 2R la cuenta
# da L <= 0, que es justo el caso en que el giro hacia adelante SI existe y no
# hace falta retroceder.
REVERSE_ARC_DEG = 90.0


def _rotate(vector: Point, degrees: float) -> Point:
    """Rotar un versor en el plano."""
    radianes = math.radians(float(degrees))
    coseno = math.cos(radianes)
    seno = math.sin(radianes)
    return (
        (vector[0] * coseno) - (vector[1] * seno),
        (vector[0] * seno) + (vector[1] * coseno),
    )


def reverse_leg_length_m(min_turning_radius_m: float, lane_spacing_m: float) -> float:
    """Metros de marcha atras que pide la cabecera de tres puntos.

    Cero cuando las pasadas estan mas separadas que el diametro de giro: ahi el
    giro hacia adelante existe y no hay nada que retroceder.
    """
    radio = max(0.0, float(min_turning_radius_m))
    separacion = abs(float(lane_spacing_m))
    return max(0.0, (2.0 * radio) - separacion)


def _headland_poses(
    fin: Point,
    salida: Point,
    inicio: Point,
    entrada: Point,
    lead_m: float,
) -> Tuple[Point, Point]:
    """Pose de salida del surco y pose de reentrada al siguiente."""
    return (
        (fin[0] + (salida[0] * lead_m), fin[1] + (salida[1] * lead_m)),
        (inicio[0] - (entrada[0] * lead_m), inicio[1] - (entrada[1] * lead_m)),
    )


def _needs_headland_maneuver(
    fin: Point,
    salida: Point,
    inicio: Point,
    entrada: Point,
    *,
    radio_m: float,
    lead_m: float,
) -> bool:
    """Si la transicion NO se cierra con un enlace Dubins de tipo CSC.

    Entre dos poses siempre existe un camino Dubins hacia adelante, pero solo
    cuando es CSC —curva, recta, curva— alcanza con dejar las dos guias y que
    Nav2 lo encuentre. El unico caso que no admite CSC es la cabecera clasica:
    rumbos opuestos y las dos poses mas cerca que el diametro de giro. Ahi hay
    que armar la maniobra a mano, sea la cabecera de tres puntos (con reversa)
    o la omega (sin ella).

    Los dos filtros importan, y el segundo es el que faltaba:

    - **Rumbos opuestos.** Dos tramos de la MISMA pasada partida por una zona
      no-go se encadenan con el mismo rumbo. Ahi el enlace recto siempre
      existe; construir un giro seria inventar una maniobra que nadie pidio.
    - **Distancia menor que 2R.** Con rumbos opuestos, el CSC de media vuelta
      existe exactamente cuando las poses estan a 2R o mas. Dos tramos de
      pasadas vecinas separados por el hueco de una zona quedan lejos en el eje
      del surco aunque esten pegados de costado: ahi tambien hay CSC.
    """
    radio = float(radio_m)
    if radio <= _EPSILON_M:
        return False
    alineacion = (salida[0] * entrada[0]) + (salida[1] * entrada[1])
    # cos(120 grados). Mas abierto que eso ya no es una cabecera.
    if alineacion > -0.5:
        return False
    arranque, reentrada = _headland_poses(fin, salida, inicio, entrada, lead_m)
    separacion = math.hypot(
        reentrada[0] - arranque[0],
        reentrada[1] - arranque[1],
    )
    return separacion < ((2.0 * radio) - _EPSILON_M)


def _three_point_turn(
    fin: Point,
    salida: Point,
    inicio: Point,
    *,
    radio_m: float,
    lead_m: float,
) -> Optional[Tuple[List[Tuple[Point, Point, float]], float]]:
    """Vertices de la cabecera de tres puntos, o None si no hace falta.

    Devuelve ``[(punto, versor_de_rumbo, marcha_atras_m), ...]`` y el largo
    recorrido. La marcha atras viaja en el vertice donde hay que hacerla.

    Quien decide si la cabecera hace falta es ``_needs_headland_maneuver``, en
    el llamador. Aca solo queda el caso trivial: con las pasadas mas separadas
    que el diametro de giro no hay nada que retroceder.
    """
    normal = (-salida[1], salida[0])
    hacia = (inicio[0] - fin[0], inicio[1] - fin[1])
    lateral = (hacia[0] * normal[0]) + (hacia[1] * normal[1])
    separacion = abs(lateral)
    reversa = reverse_leg_length_m(radio_m, separacion)
    if reversa <= _EPSILON_M:
        return None

    # Se dobla hacia donde esta el surco siguiente.
    sentido = 1.0 if lateral >= 0.0 else -1.0
    giro = (sentido * normal[0], sentido * normal[1])

    salida_arranque = (
        fin[0] + (salida[0] * lead_m),
        fin[1] + (salida[1] * lead_m),
    )
    # Fin del primer arco: R adelante y R al costado, con el rumbo ya girado 90.
    pivote = (
        salida_arranque[0] + (radio_m * salida[0]) + (radio_m * giro[0]),
        salida_arranque[1] + (radio_m * salida[1]) + (radio_m * giro[1]),
    )
    rumbo_pivote = giro
    tras_reversa = (
        pivote[0] - (reversa * rumbo_pivote[0]),
        pivote[1] - (reversa * rumbo_pivote[1]),
    )
    # Fin del segundo arco: R hacia atras del eje del surco y R mas al costado.
    reentrada = (
        tras_reversa[0] - (radio_m * salida[0]) + (radio_m * giro[0]),
        tras_reversa[1] - (radio_m * salida[1]) + (radio_m * giro[1]),
    )
    rumbo_reentrada = (-salida[0], -salida[1])

    # Dos guias y nada mas. Las otras dos que salian de la construccion no
    # aportan y una hacia dano:
    #
    #   salida_arranque  el vehiculo ya viene con ese rumbo al terminar el
    #                    surco, asi que pedirle que pase por ahi no agrega nada.
    #   tras_reversa     queda DETRAS del pivote. Si la marcha atras no llega a
    #                    correr, hacia adelante solo se llega dando un lazo: son
    #                    los circulos que aparecian en el preview.
    #
    # Afuera del lote la precision no importa, asi que cuantos menos puntos haya
    # que clavar, menos maniobra. El pivote se conserva porque ahi va la marcha
    # atras; la reentrada, porque alinea el vehiculo con el surco siguiente.
    vertices = [
        (pivote, rumbo_pivote, float(reversa)),
        (reentrada, rumbo_reentrada, 0.0),
    ]
    # Largo real: los arcos son cuartos de circunferencia, no cuerdas.
    arcos = 2.0 * (math.pi * radio_m / 2.0)
    recorrido = (2.0 * lead_m) + arcos + reversa
    recorrido += math.hypot(
        inicio[0] - reentrada[0] - (salida[0] * -lead_m),
        inicio[1] - reentrada[1] - (salida[1] * -lead_m),
    )
    return vertices, recorrido


def _forward_omega_turn(
    fin: Point,
    salida: Point,
    inicio: Point,
    entrada: Point,
    *,
    radio_m: float,
    lead_m: float,
    open_forward: bool = True,
) -> Optional[List[Tuple[Point, Point, float]]]:
    """Vertices de una omega hacia adelante, o None si alcanza una U.

    Cuando las pasadas quedan mas juntas que el diametro de giro no existe
    ninguna U hacia adelante, y la cabecera de tres puntos resuelve eso
    retrocediendo. Con la reversa apagada la unica maniobra que queda es la
    omega: el enlace Dubins CCC —girar para el lado contrario, dar el bucle y
    volver a alinearse— que sale del lote por la cabecera y vuelve a entrar.

    La construccion es la clasica de tres circunferencias de radio ``R``:

        C1  circunferencia de salida, apoyada del lado opuesto al surco
            siguiente, para que el bucle abra hacia afuera;
        C3  circunferencia de entrada al surco siguiente;
        C2  circunferencia del bucle, tangente a las dos, con el centro
            corrido hacia adelante del lote.

    C2 existe mientras ``|C3 - C1| <= 4R``, que con las pasadas mas juntas que
    ``2R`` se cumple siempre. Si no se cumple —geometria degenerada, pasadas
    con desfasaje longitudinal grande— se levanta ``ForwardOnlyTurnError`` en
    vez de volver en silencio a la marcha atras.

    Devuelve ``[(punto, versor_de_rumbo, 0.0), ...]``: los dos puntos de
    tangencia, el apice del bucle y la pose de reentrada. El apice esta para que ningun tramo entre
    guias supere media vuelta; sin el, el enlace mas corto entre las dos
    tangencias deja de ser el arco largo del bucle y Nav2 cortaria camino por
    adentro del lote. La reentrada fija el rumbo antes de la fila: sin ella el
    PositionGoalChecker puede aceptar la posicion exterior con mas de 30 grados
    de error y la recta siguiente empieza con una correccion circular.
    """
    radio = float(radio_m)
    if radio <= _EPSILON_M:
        return None

    normal = (-salida[1], salida[0])
    hacia = (inicio[0] - fin[0], inicio[1] - fin[1])
    lateral = (hacia[0] * normal[0]) + (hacia[1] * normal[1])
    sentido = 1.0 if lateral >= 0.0 else -1.0
    giro = (sentido * normal[0], sentido * normal[1])

    arranque, reentrada = _headland_poses(
        fin, salida, inicio, entrada, lead_m
    )

    centro_salida = (
        arranque[0] - (radio * giro[0]),
        arranque[1] - (radio * giro[1]),
    )
    centro_entrada = (
        reentrada[0] + (radio * giro[0]),
        reentrada[1] + (radio * giro[1]),
    )
    eje = (
        centro_entrada[0] - centro_salida[0],
        centro_entrada[1] - centro_salida[1],
    )
    distancia = math.hypot(eje[0], eje[1])
    if distancia <= _EPSILON_M or distancia > (4.0 * radio) + 1.0e-6:
        raise ForwardOnlyTurnError(
            "no hay enlace hacia adelante entre dos pasadas con la reversa "
            f"apagada: centros a {distancia:.2f} m con radio {radio:.2f} m "
            f"(maximo {4.0 * radio:.2f} m). Subi la separacion entre pasadas, "
            "baja coverage_planner_min_turning_radius_m, o habilita "
            "coverage_f2c_allow_reverse si el perfil puede retroceder"
        )

    altura = math.sqrt(
        max(0.0, (4.0 * radio * radio) - (0.25 * distancia * distancia))
    )
    perpendicular = (-eje[1] / distancia, eje[0] / distancia)
    abre_hacia_salida = (
        (perpendicular[0] * salida[0]) + (perpendicular[1] * salida[1])
    ) >= 0.0
    if abre_hacia_salida != bool(open_forward):
        # En una cabecera normal el bucle abre hacia adelante del surco, afuera
        # del lote. Si el extremo del swath lo produjo una exclusion interna,
        # adelante esta justamente el obstaculo y se elige la otra solucion
        # CCC: el mismo giro hacia adelante, abierto sobre el lado ya libre.
        perpendicular = (-perpendicular[0], -perpendicular[1])
    medio = (
        0.5 * (centro_salida[0] + centro_entrada[0]),
        0.5 * (centro_salida[1] + centro_entrada[1]),
    )
    centro_bucle = (
        medio[0] + (altura * perpendicular[0]),
        medio[1] + (altura * perpendicular[1]),
    )

    def _tangencia(centro: Point) -> Point:
        dx = centro_bucle[0] - centro[0]
        dy = centro_bucle[1] - centro[1]
        largo = math.hypot(dx, dy)
        return (
            centro[0] + (radio * dx / largo),
            centro[1] + (radio * dy / largo),
        )

    def _rumbo(punto: Point, centro: Point, sentido_giro: float) -> Point:
        dx = punto[0] - centro[0]
        dy = punto[1] - centro[1]
        largo = math.hypot(dx, dy)
        return (
            sentido_giro * (-dy / largo),
            sentido_giro * (dx / largo),
        )

    tangencia_salida = _tangencia(centro_salida)
    tangencia_entrada = _tangencia(centro_entrada)
    sentido_bucle = sentido

    # Apice: punto medio angular del bucle en su propio sentido de giro. El
    # bucle barre mas de media vuelta, asi que partirlo al medio deja dos arcos
    # que si son el enlace Dubins mas corto entre sus extremos.
    angulo_inicial = math.atan2(
        tangencia_salida[1] - centro_bucle[1],
        tangencia_salida[0] - centro_bucle[0],
    )
    angulo_final = math.atan2(
        tangencia_entrada[1] - centro_bucle[1],
        tangencia_entrada[0] - centro_bucle[0],
    )
    barrido = (angulo_final - angulo_inicial) * sentido_bucle
    barrido = barrido % (2.0 * math.pi)
    angulo_apice = angulo_inicial + (sentido_bucle * 0.5 * barrido)
    apice = (
        centro_bucle[0] + (radio * math.cos(angulo_apice)),
        centro_bucle[1] + (radio * math.sin(angulo_apice)),
    )

    return [
        (tangencia_salida, _rumbo(tangencia_salida, centro_salida, -sentido), 0.0),
        (apice, _rumbo(apice, centro_bucle, sentido_bucle), 0.0),
        (tangencia_entrada, _rumbo(tangencia_entrada, centro_entrada, -sentido), 0.0),
        (reentrada, entrada, 0.0),
    ]


def replace_turns_with_flexible_headlands(
    plan: Fields2CoverPlan,
    margin_m: float,
    *,
    min_turning_radius_m: float = 0.0,
    allow_reverse: bool = True,
    avoid_polygons: Sequence[Sequence[Point]] = (),
) -> Fields2CoverPlan:
    """Cambiar los giros de Fields2Cover por transiciones exteriores simples.

    Fields2Cover resuelve la cabecera con curvas Dubins: cuando la separacion
    entre pasadas es menor que dos veces el radio minimo —2.0 m de ancho con 15%
    de solape dan 1.7 m, contra 4 m de radio— la unica solucion posible es una
    omega, y el preview se llena de petalos gigantes fuera del lote. Al operador
    no le importa por donde pasa el vehiculo afuera del lote: le importa que los
    surcos se recorran exactos. Asi que el surco se deja tal cual y solo se
    reemplaza la cabecera por tres tramos rectos:

        salir derecho por el eje de la pasada -> desplazarse afuera ->
        reentrar derecho por el eje de la siguiente

    Los dos vertices de ese recorrido quedan como guias no-key: el ejecutor las
    respeta como puntos de paso, pero Nav2 planifica libremente entre ellas, que
    es exactamente la flexibilidad que se busca afuera del lote. Los waypoints
    de trabajo no se tocan.

    ``margin_m`` es el tramo recto que despeja el implemento antes y despues
    de los arcos (ver ``DEFAULT_HEADLAND_LEAD_M``).

    ``allow_reverse`` decide con que se resuelve una separacion menor que el
    diametro de giro. En ``True`` (perfil real) se usa la cabecera de tres
    puntos, que retrocede en un vertice. En ``False`` (simulacion) esa maniobra
    esta prohibida y se arma una omega hacia adelante; ningun punto sale con
    ``backup_m`` distinto de cero, asi que aguas abajo no puede aparecer una
    accion ``coverage_backup``. Si tampoco hay omega posible, la funcion levanta
    ``ForwardOnlyTurnError`` en lugar de volver a la marcha atras.

    La funcion es pura e idempotente: no toca el plan que recibe y aplicarla dos
    veces da lo mismo, porque vuelve a derivar las guias de los surcos, que son
    los que nunca cambian.

    Nota de alcance: el tramo exterior se traza recto entre las dos guias. En un
    lote muy concavo ese tramo podria rozar el poligono; se acepta a proposito,
    porque el recorte de zonas no-go corre despues y es el que decide por donde
    no se puede pasar.
    """
    margen = max(0.0, float(margin_m))
    radio = max(0.0, float(min_turning_radius_m))
    runs = _work_runs(plan.waypoints)
    if not runs:
        return replace(plan, waypoints=list(plan.waypoints))

    waypoints: List[CoverageBodyWaypoint] = []
    transicion_m = 0.0
    for indice, run in enumerate(runs):
        waypoints.extend(run)
        if indice + 1 >= len(runs):
            continue

        siguiente = runs[indice + 1]
        salida = _exit_direction(run)
        entrada = _entry_direction(siguiente)
        fin = (float(run[-1].forward_m), float(run[-1].left_m))
        inicio = (float(siguiente[0].forward_m), float(siguiente[0].left_m))
        obstacle_ahead = False

        if not _needs_headland_maneuver(
            fin, salida, inicio, entrada, radio_m=radio, lead_m=margen
        ):
            # Hay enlace Dubins CSC hacia adelante: dos guias alcanzan y
            # cualquier maniobra construida a mano solo agregaria metros.
            maniobra = None
        elif not allow_reverse:
            # Politica forward-only: la cabecera de tres puntos ni se evalua,
            # asi no hay forma de que se cuele un backup_m.
            for polygon in avoid_polygons:
                if not polygon:
                    continue
                center = (
                    sum(float(point[0]) for point in polygon) / len(polygon),
                    sum(float(point[1]) for point in polygon) / len(polygon),
                )
                forward_distance = (
                    ((center[0] - fin[0]) * salida[0])
                    + ((center[1] - fin[1]) * salida[1])
                )
                nearest = min(
                    math.hypot(fin[0] - point[0], fin[1] - point[1])
                    for point in polygon
                )
                if forward_distance > 0.0 and nearest <= (2.0 * radio):
                    obstacle_ahead = True
                    break
            maniobra = _forward_omega_turn(
                fin,
                salida,
                inicio,
                entrada,
                # Smac usa un lattice discreto: un arco construido con el
                # minimo exacto puede ser geometricamente valido y aun asi no
                # tener ninguna primitiva admisible. La omega ocurre fuera del
                # lote, asi que darle 25% de radio no pisa cultivo, no agrega
                # waypoints y evita NO_VALID_PATH en la ultima tangencia.
                radio_m=max(radio * 1.25, radio + 0.5),
                lead_m=margen,
                open_forward=not obstacle_ahead,
            )
        else:
            tres_puntos = _three_point_turn(
                fin, salida, inicio, radio_m=radio, lead_m=margen
            )
            maniobra = tres_puntos[0] if tres_puntos is not None else None
        if maniobra is not None:
            vertices = maniobra
        else:
            # Separacion mayor que el diametro de giro: el giro hacia adelante
            # existe y alcanza con salir, correrse y entrar.
            vertices = [
                (
                    (fin[0] + (salida[0] * margen), fin[1] + (salida[1] * margen)),
                    salida,
                    0.0,
                ),
                (
                    (
                        inicio[0] - (entrada[0] * margen),
                        inicio[1] - (entrada[1] * margen),
                    ),
                    entrada,
                    0.0,
                ),
            ]

        row_index = int(run[-1].row_index)
        if not allow_reverse and obstacle_ahead:
            transition_phase = "nogo_transition"
        elif not allow_reverse and maniobra is not None:
            transition_phase = FORWARD_TURN_PHASE
        else:
            transition_phase = TRANSITION_PHASE
        previo = fin
        for vertex_index, (punto, versor, reversa_m) in enumerate(vertices):
            largo = math.hypot(punto[0] - previo[0], punto[1] - previo[1])
            hasta_inicio = math.hypot(punto[0] - inicio[0], punto[1] - inicio[1])
            if largo <= _EPSILON_M or hasta_inicio <= _EPSILON_M:
                # Con margen nulo la guia cae encima de un extremo de surco, y un
                # waypoint repetido es una meta de largo cero aguas abajo.
                continue
            transicion_m += largo
            previo = punto
            precise_midpoint = bool(
                not allow_reverse
                and len(vertices) >= 3
                and vertex_index == 1
            )
            waypoints.append(
                CoverageBodyWaypoint(
                    forward_m=float(punto[0]),
                    left_m=float(punto[1]),
                    yaw_delta_deg=float(math.degrees(math.atan2(versor[1], versor[0]))),
                    phase=transition_phase,
                    row_index=row_index,
                    # Las tangencias y la alineacion de salida son flexibles.
                    # El apice central es la unica ancla moderada: evita
                    # recortar la omega sin obligar al Ackermann a cerrar otro
                    # giro para acertar una pose exterior al centimetro.
                    is_key=precise_midpoint,
                    is_guide=not precise_midpoint,
                    backup_m=float(reversa_m),
                )
            )
        transicion_m += math.hypot(inicio[0] - previo[0], inicio[1] - previo[1])

    return replace(plan, waypoints=waypoints, transition_length_m=float(transicion_m))


class Fields2CoverPlanner:
    """Cliente del Coverage Server con executor propio."""

    def __init__(
        self,
        *,
        action_name: str = "compute_coverage_path",
        parameter_service: str = "/coverage_server/set_parameters",
        change_state_service: str = "/coverage_server/change_state",
        node_name: str = "route_executor_fields2cover_client",
        logger: Any = None,
    ) -> None:
        """Levantar el nodo cliente y su executor en un hilo aparte."""
        if _IMPORT_ERROR:
            raise Fields2CoverError(
                "el overlay de Fields2Cover no esta instalado en este entorno "
                f"({_IMPORT_ERROR}); source del workspace de opennav_coverage "
                "antes de lanzar, o coverage_planner:=legacy"
            )
        self._logger = logger
        # Este cliente vive dentro del proceso route_executor, cuyo launch
        # agrega ``__node:=route_executor``. Si hereda los argumentos globales,
        # el helper tambien se renombra /route_executor y sus servicios de
        # parametros compiten con los del nodo real (respuestas vacias al azar).
        self._node = rclpy.create_node(node_name, use_global_arguments=False)
        self._action = ActionClient(self._node, ComputeCoveragePath, action_name)
        self._parameters = self._node.create_client(SetParameters, parameter_service)
        self._change_state = self._node.create_client(ChangeState, change_state_service)
        # Ultimos parametros fisicos aplicados de verdad, para no reciclar el
        # servidor en cada pedido.
        self._applied: Dict[str, float] = {}
        # Warmup, previews de dos clientes y cambios de parametros comparten un
        # unico Coverage Server lifecycle. Las transiciones no son idempotentes:
        # dos CONFIGURE concurrentes dejan a uno rechazado aunque el otro haya
        # funcionado. Serializar tambien la accion evita reciclar el robot
        # mientras otro pedido todavia esta calculando con esos parametros.
        self._server_lock = threading.RLock()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin, name=node_name, daemon=True
        )
        self._thread.start()
        # Levantar el Coverage Server ahora y no en el primer pedido: viene del
        # lanzamiento en `unconfigured`, y configurarlo tarda mas que el timeout
        # del cockpit. Sin esto el primer preview de la sesion falla siempre y
        # el segundo anda, que es la peor forma de andar. En un hilo aparte para
        # no demorar el arranque del route_executor.
        threading.Thread(
            target=self._warmup, name=f"{node_name}_warmup", daemon=True
        ).start()

    def _warmup(self) -> None:
        """Dejar el Coverage Server activo, si es que hay uno."""
        try:
            with self._server_lock:
                if self.available(timeout_s=10.0):
                    return
                self._cycle_server(10.0)
                # Los parametros del vehiculo todavia no se saben —dependen del
                # pedido—, asi que el primero que llegue va a reciclar igual.
                self._applied = {}
            if self._logger is not None:
                self._logger.info("Coverage Server activado desde el arranque")
        except Exception as exc:  # pragma: no cover - depende del entorno
            if self._logger is not None:
                self._logger.warning(f"no se pudo activar el Coverage Server: {exc}")

    def shutdown(self) -> None:
        """Bajar el executor y el nodo cliente."""
        try:
            self._executor.shutdown()
            self._node.destroy_node()
        except Exception:  # pragma: no cover - solo en el apagado
            pass

    def available(self, timeout_s: float = 2.0) -> bool:
        """Decir si el Coverage Server esta escuchando."""
        return bool(self._action.wait_for_server(timeout_sec=float(timeout_s)))

    def _cycle_server(self, timeout_s: float) -> None:
        """Reciclar el Coverage Server para que relea los parametros.

        Sin esto los parametros del vehiculo no tienen efecto: el servidor arma
        su objeto de robot al configurarse y no vuelve a mirarlos. Medido: con
        radio minimo 2.9 m seteado por parametro pero sin reciclar, los giros
        salian con radio de 0.32 m —fisicamente inejecutables— porque adentro
        seguia el valor de la configuracion anterior. Despues del ciclo, 2.90 m
        exactos y ningun punto por debajo.
        """
        if not self._change_state.wait_for_service(timeout_sec=float(timeout_s)):
            raise Fields2CoverError(
                "el Coverage Server no expone change_state; no se pueden "
                "aplicar los parametros del vehiculo"
            )
        # Bajar solo tiene sentido si el server ya estaba arriba. Recien
        # lanzado esta en `unconfigured` y ahi DEACTIVATE y CLEANUP se rechazan
        # —correctamente—, asi que exigirlas dejaria a Campo sin funcionar
        # cuando el server viene del lanzamiento y nadie lo activo a mano.
        # Configurar y activar, en cambio, tienen que salir si o si.
        transiciones = (
            (Transition.TRANSITION_DEACTIVATE, False),
            (Transition.TRANSITION_CLEANUP, False),
            (Transition.TRANSITION_CONFIGURE, True),
            (Transition.TRANSITION_ACTIVATE, True),
        )
        for transicion, obligatoria in transiciones:
            request = ChangeState.Request()
            request.transition.id = int(transicion)
            future = self._change_state.call_async(request)
            if not self._wait(future, timeout_s):
                raise Fields2CoverError(
                    "timeout reciclando el Coverage Server para aplicar los "
                    "parametros del vehiculo"
                )
            respuesta = future.result()
            if respuesta is None or not bool(respuesta.success):
                if obligatoria:
                    raise Fields2CoverError(
                        f"el Coverage Server rechazo la transicion {int(transicion)}"
                    )

    def _push_parameters(self, values: Dict[str, float], timeout_s: float) -> None:
        """Mandarle al server los parametros fisicos del vehiculo."""
        if not self._parameters.wait_for_service(timeout_sec=float(timeout_s)):
            raise Fields2CoverError(
                "el Coverage Server no expone set_parameters; "
                "revisa que este activo"
            )
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=str(name),
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
                ),
            )
            for name, value in values.items()
        ]
        future = self._parameters.call_async(request)
        if not self._wait(future, timeout_s):
            raise Fields2CoverError("timeout mandando parametros al Coverage Server")
        response = future.result()
        fallidos = [
            request.parameters[index].name
            for index, result in enumerate(response.results)
            if not result.successful
        ]
        if fallidos:
            raise Fields2CoverError(
                f"el Coverage Server rechazo los parametros {fallidos}"
            )
        # Solo se recicla cuando algo cambio de verdad: el ciclo tarda y no
        # tiene sentido pagarlo en cada preview con los mismos numeros.
        if any(
            abs(float(values[name]) - float(self._applied.get(name, float("nan"))))
            > 1.0e-9
            or name not in self._applied
            for name in values
        ):
            self._cycle_server(timeout_s)
            self._applied = {name: float(value) for name, value in values.items()}

    @staticmethod
    def _wait(future: Any, timeout_s: float) -> bool:
        """Esperar un future que resuelve el executor propio."""
        evento = threading.Event()
        future.add_done_callback(lambda _: evento.set())
        return bool(evento.wait(timeout=float(timeout_s)))

    def _plan_locked(
        self,
        *,
        polygon_body: Sequence[Point],
        exclusions_body: Sequence[Sequence[Point]] = (),
        cutter_width_m: float,
        robot_width_m: float,
        overlap_ratio: float,
        min_turning_radius_m: float,
        waypoint_spacing_m: float,
        swath_angle_deg: Optional[float] = None,
        route_type: str = "BOUSTROPHEDON",
        path_type: str = "DUBIN",
        path_continuity: str = "CONTINUOUS",
        turn_point_distance_m: float = 0.5,
        headland_width_m: float = 0.0,
        server_timeout_s: float = 30.0,
    ) -> Fields2CoverPlan:
        """Planificar el lote y devolver los waypoints en marco del cuerpo."""
        if len(polygon_body) < 3:
            raise Fields2CoverError("el poligono del lote necesita al menos 3 vertices")
        if not self.available(timeout_s=min(5.0, server_timeout_s)):
            # Recien lanzado, el Coverage Server esta en `unconfigured` y su
            # action server todavia no existe: nadie lo activo. Antes de darlo
            # por caido se lo intenta levantar, que es lo que el lanzamiento
            # espera que pase en el primer pedido de CAMPO. Si tampoco asi
            # aparece, entonces si no esta.
            try:
                self._cycle_server(min(5.0, server_timeout_s))
                self._applied = {}
            except Fields2CoverError:
                pass
            if not self.available(timeout_s=min(5.0, server_timeout_s)):
                raise Fields2CoverError(
                    "el Coverage Server no responde; revisa que opennav_coverage "
                    "este corriendo y activo"
                )

        # El solape se traduce a un ancho de operacion menor, que es como lo
        # expresa Fields2Cover. Da la misma separacion entre pasadas que la
        # cuenta del planificador propio: ancho * (1 - solape).
        operation_width = float(cutter_width_m) * (1.0 - float(overlap_ratio))
        if operation_width <= 0.0:
            raise Fields2CoverError("el solape deja un ancho de trabajo nulo")
        self._push_parameters(
            {
                "operation_width": operation_width,
                "robot_width": float(robot_width_m),
                "min_turning_radius": float(min_turning_radius_m),
            },
            timeout_s=min(5.0, server_timeout_s),
        )

        goal = ComputeCoveragePath.Goal()
        goal.use_gml_file = False
        goal.frame_id = "map"
        goal.polygons = [_ring_to_coordinates(polygon_body)] + [
            _ring_to_coordinates(ring) for ring in exclusions_body
        ]
        # Sin cabecera interior: el lote entero es superficie de trabajo y los
        # giros pueden salirse. Reservar una banda interior achicaria el area
        # cubierta, que es justo lo que no se quiere.
        goal.generate_headland = float(headland_width_m) > 0.0
        goal.headland_mode.width = float(headland_width_m)
        goal.generate_route = True
        goal.generate_path = True
        goal.swath_mode.mode = "BRUTE_FORCE" if swath_angle_deg is None else "SET_ANGLE"
        goal.swath_mode.objective = "LENGTH"
        if swath_angle_deg is not None:
            goal.swath_mode.best_angle = float(math.radians(swath_angle_deg))
        goal.route_mode.mode = str(route_type)
        goal.path_mode.mode = str(path_type)
        goal.path_mode.continuity_mode = str(path_continuity)
        # Fields2Cover muestrea los giros cada 10 cm por defecto. Sobre un lote
        # de 40 m eso son ~1400 poses solo de cabeceras, que se comen el tope de
        # waypoints sin aportar nada: el arco queda igual de fiel con un paso
        # varias veces mayor.
        goal.path_mode.turn_point_distance = float(turn_point_distance_m)

        enviado = self._action.send_goal_async(goal)
        if not self._wait(enviado, server_timeout_s):
            raise Fields2CoverError("timeout esperando que el Coverage Server acepte")
        handle = enviado.result()
        if handle is None or not handle.accepted:
            raise Fields2CoverError("el Coverage Server rechazo la meta")

        resultado = handle.get_result_async()
        if not self._wait(resultado, server_timeout_s):
            raise Fields2CoverError("timeout esperando el plan de Fields2Cover")
        envuelto = resultado.result()
        if envuelto is None:
            raise Fields2CoverError("el Coverage Server no devolvio resultado")
        respuesta = envuelto.result
        if int(getattr(respuesta, "error_code", 0)) != 0:
            raise Fields2CoverError(
                f"Fields2Cover fallo con error_code={int(respuesta.error_code)}"
            )

        plan = plan_to_body_waypoints(
            respuesta, waypoint_spacing_m=float(waypoint_spacing_m)
        )
        plan.route_type = str(route_type)
        plan.path_type = str(path_type)
        return plan

    def plan(
        self,
        *,
        polygon_body: Sequence[Point],
        exclusions_body: Sequence[Sequence[Point]] = (),
        cutter_width_m: float,
        robot_width_m: float,
        overlap_ratio: float,
        min_turning_radius_m: float,
        waypoint_spacing_m: float,
        swath_angle_deg: Optional[float] = None,
        route_type: str = "BOUSTROPHEDON",
        path_type: str = "DUBIN",
        path_continuity: str = "CONTINUOUS",
        turn_point_distance_m: float = 0.5,
        headland_width_m: float = 0.0,
        server_timeout_s: float = 30.0,
    ) -> Fields2CoverPlan:
        """Planificar sin competir con warmup ni con otro pedido de Campo."""
        with self._server_lock:
            return self._plan_locked(
                polygon_body=polygon_body,
                exclusions_body=exclusions_body,
                cutter_width_m=cutter_width_m,
                robot_width_m=robot_width_m,
                overlap_ratio=overlap_ratio,
                min_turning_radius_m=min_turning_radius_m,
                waypoint_spacing_m=waypoint_spacing_m,
                swath_angle_deg=swath_angle_deg,
                route_type=route_type,
                path_type=path_type,
                path_continuity=path_continuity,
                turn_point_distance_m=turn_point_distance_m,
                headland_width_m=headland_width_m,
                server_timeout_s=server_timeout_s,
            )
