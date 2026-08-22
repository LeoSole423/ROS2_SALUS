"""Recorte del plan de cobertura contra zonas no-go.

Trabaja en el marco del cuerpo del lote —``forward_m`` / ``left_m``—, que es
donde vive el plan antes de georreferenciarse. Recortar ahi y no sobre lat/lon
tiene dos ventajas: las cuentas son euclideas sin correcciones de latitud, y el
resultado hereda gratis la georreferenciacion que ya hace
``build_lawnmower_waypoints``, asi que el recorte se comporta igual con el lote
derecho que en diagonal.

El recorte es **idempotente**: aplicarlo sobre una ruta ya recortada no la
cambia, porque no quedan waypoints adentro de una zona ni segmentos que la
crucen. Eso es lo que permite que el backend y el cockpit hagan la misma cuenta
sin pisarse.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import TYPE_CHECKING, List, Optional, Sequence, Tuple

if TYPE_CHECKING:  # pragma: no cover - solo para los tipos
    from .coverage_waypoint_core import CoverageBodyWaypoint

Point = Tuple[float, float]
Polygon = Sequence[Point]
# Rectangulo del lote en marco del cuerpo: (forward_min, forward_max, left_min,
# left_max). Sirve para distinguir zonas internas de zonas que tocan el borde.
Bounds = Tuple[float, float, float, float]

# Fase con la que se marcan los puntos que agrega el rodeo. El route_executor no
# la interpreta, pero deja el plan legible en los logs y en el preview.
NOGO_DETOUR_PHASE = "nogo_detour"

# Guia original de Fields2Cover que no se puede simplificar porque la cuerda
# entre sus metas key vecinas volveria a cruzar una zona no-go.
NOGO_TRANSITION_PHASE = "nogo_transition"

# Cambio lateral anticipado para una zona interna. Ya es una guia obligatoria
# por definicion y no debe renombrarse como ``nogo_transition``: esa otra fase
# representa el rodeo/omega que justamente esta prohibido dentro del lote.
NOGO_LANE_CHANGE_PHASE = "nogo_lane_change"

# Tope de pasadas de rodeo. Un rodeo puede meter la ruta dentro de otra zona, asi
# que hay que reprocesar; el tope evita quedarse en un ciclo cuando dos zonas se
# encierran mutuamente.
_MAX_DETOUR_PASSES = 8

# Una esquina muy filosa manda el vertice inflado infinitamente lejos. Se corta a
# este multiplo del margen: deforma la esquina pero no dispara una punta.
_MAX_MITER_RATIO = 4.0

_EPSILON_M = 1.0e-9

# Holgura al chequear si un punto del rodeo cae dentro del lote. Las cabeceras
# ya sobresalen por diseno, asi que un rodeo que roza el borde no es el problema
# que se busca evitar: lo que no puede pasar es que salga metros afuera.
_BOUNDS_TOLERANCE_M = 0.5

# Una zona que toca exactamente el borde no produce por si sola ningun punto
# exterior: su contorno inflado queda apoyado sobre la linea del lote. Se corre
# el arco exterior un poco mas que la tolerancia para que la maniobra salga de
# verdad y no quede numericamente ambigua.
_EDGE_EXIT_CLEARANCE_M = 0.75

# Una zona circular llega discretizada en 32 lados. Su poligono inflado puede
# quedar apenas 1--2 cm del punto matematicamente tangente de una pasada (por
# ejemplo, la vertical x=14 frente a una exclusion de radio 4). Ese error de
# rasterizacion no es una penetracion: si el punto medio queda a menos de este
# margen del contorno, se conserva la pasada y no se fabrica un rodeo de casi
# 360 grados para una tangencia.
_TANGENT_EXCLUSION_TOLERANCE_M = 0.10

# Las zonas circulares llegan del Cockpit como poligonos de 32 lados. Seguir
# los 32 vertices obliga a Nav2 a clavar metas cada pocos centimetros y termina
# pareciendo una vuelta completa aunque solo se pidio un arco. Las guias se
# separan radialmente este margen y se conservan como maximo cada ~45 grados:
# las cuerdas quedan afuera del poligono y el Ackermann recibe pocos hitos.
_ROUND_DETOUR_CLEARANCE_M = 0.75
_ROUND_DETOUR_MAX_ANGLE_RAD = math.radians(50.0)

# Tolerancia para decidir si un punto cae sobre el contorno. Es mas floja que
# `_EPSILON_M` a proposito: los puntos del rodeo se calculan como interseccion
# de segmentos y caen sobre el borde con el error de redondeo de esa cuenta, no
# exactamente encima. Un micrometro sobra para el campo y no confunde nada.
_ON_EDGE_TOLERANCE_M = 1.0e-6

# Dos posiciones mas juntas que esto son el mismo waypoint aguas abajo: la meta
# tendria largo cero y rumbo indefinido. Es milimetrica a proposito, para no
# tragarse ningun vertice real del contorno.
_DUPLICATE_POINT_TOLERANCE_M = 1.0e-3


def polygon_signed_area(polygon: Polygon) -> float:
    """Area con signo del poligono; positiva si los vertices van antihorarios."""
    total = 0.0
    count = len(polygon)
    for index in range(count):
        current = polygon[index]
        following = polygon[(index + 1) % count]
        total += (float(current[0]) * float(following[1])) - (
            float(following[0]) * float(current[1])
        )
    return float(total) / 2.0


def point_in_polygon(point: Point, polygon: Polygon) -> bool:
    """Devolver True si el punto cae dentro del poligono (ray casting).

    Un punto justo sobre el borde cuenta como adentro: el poligono que se pasa
    ya viene inflado por el margen de seguridad, asi que dudar hacia adentro es
    lo conservador.
    """
    if len(polygon) < 3:
        return False

    x = float(point[0])
    y = float(point[1])
    inside = False
    count = len(polygon)
    for index in range(count):
        ax, ay = float(polygon[index][0]), float(polygon[index][1])
        bx, by = (
            float(polygon[(index + 1) % count][0]),
            float(polygon[(index + 1) % count][1]),
        )
        if _point_on_edge((x, y), (ax, ay), (bx, by)):
            return True
        if (ay > y) != (by > y):
            crossing_x = ax + ((y - ay) / (by - ay)) * (bx - ax)
            if crossing_x > x:
                inside = not inside
    return inside


def _strictly_inside(point: Point, polygon: Polygon) -> bool:
    """Adentro del poligono y sin tocar el contorno.

    Hace falta para distinguir un tramo que atraviesa la zona de uno que corre
    pegado a un lado. El segundo no tiene nada que rodear: si se lo tratara como
    cruce, el rodeo se volveria a insertar en cada pasada y la ruta crece sola.
    """
    if not point_in_polygon(point, polygon):
        return False
    count = len(polygon)
    return not any(
        _point_on_edge(point, polygon[index], polygon[(index + 1) % count])
        for index in range(count)
    )


def _point_on_edge(point: Point, start: Point, end: Point) -> bool:
    """Decir si el punto cae sobre el segmento, dentro de la tolerancia de borde."""
    edge_x = float(end[0]) - float(start[0])
    edge_y = float(end[1]) - float(start[1])
    length = math.hypot(edge_x, edge_y)
    if length <= _EPSILON_M:
        return math.hypot(
            float(point[0]) - float(start[0]), float(point[1]) - float(start[1])
        ) <= _ON_EDGE_TOLERANCE_M
    cross = (edge_x * (float(point[1]) - float(start[1]))) - (
        edge_y * (float(point[0]) - float(start[0]))
    )
    if abs(cross) > (_ON_EDGE_TOLERANCE_M * length):
        return False
    dot = ((float(point[0]) - float(start[0])) * edge_x) + (
        (float(point[1]) - float(start[1])) * edge_y
    )
    tolerance = _ON_EDGE_TOLERANCE_M * length
    return bool(-tolerance <= dot <= (length * length) + tolerance)


def _field_polygon(
    bounds: Optional[Bounds],
    field_boundary: Optional[Polygon],
) -> Optional[List[Point]]:
    """Contorno efectivo del lote, priorizando el poligono real."""
    if field_boundary is not None and len(field_boundary) >= 3:
        return [(float(x), float(y)) for x, y in field_boundary]
    if bounds is None:
        return None
    forward_min, forward_max, left_min, left_max = bounds
    return [
        (float(forward_min), float(left_min)),
        (float(forward_max), float(left_min)),
        (float(forward_max), float(left_max)),
        (float(forward_min), float(left_max)),
    ]


def polygon_is_strictly_inside_field(
    polygon: Polygon,
    *,
    bounds: Optional[Bounds] = None,
    field_boundary: Optional[Polygon] = None,
) -> bool:
    """Distinguir una exclusion interna de una zona pegada al borde.

    Una zona solo cuenta como interna cuando todos sus vertices y sus lados
    quedan estrictamente dentro del lote. Tocar el contorno es suficiente para
    clasificarla como zona de borde: en ese caso la maniobra permitida es salir
    temporalmente del lote y rodearla por afuera.
    """
    field = _field_polygon(bounds, field_boundary)
    if field is None or len(polygon) < 3:
        return False

    vertices = [(float(x), float(y)) for x, y in polygon]
    if not all(_strictly_inside(point, field) for point in vertices):
        return False

    # En un lote concavo dos extremos pueden estar adentro y el segmento que
    # los une atravesar un hueco. Cualquier contacto entre bordes invalida la
    # clasificacion interna. Los puntos medios agregan una defensa barata para
    # geometria casi colineal, donde la interseccion cae dentro de la
    # tolerancia.
    for index, start in enumerate(vertices):
        end = vertices[(index + 1) % len(vertices)]
        midpoint = (
            0.5 * (float(start[0]) + float(end[0])),
            0.5 * (float(start[1]) + float(end[1])),
        )
        if not _strictly_inside(midpoint, field):
            return False
        for field_index, field_start in enumerate(field):
            field_end = field[(field_index + 1) % len(field)]
            intersection = _segment_intersection(
                start, end, field_start, field_end
            )
            if intersection is not None:
                return False
            if (
                _point_on_edge(start, field_start, field_end)
                or _point_on_edge(end, field_start, field_end)
                or _point_on_edge(field_start, start, end)
                or _point_on_edge(field_end, start, end)
            ):
                return False
    return True


def internal_nogo_exclusions(
    polygons: Sequence[Polygon],
    *,
    safety_margin_m: float,
    turn_anticipation_m: float,
    bounds: Optional[Bounds] = None,
    field_boundary: Optional[Polygon] = None,
) -> List[List[Point]]:
    """Envolventes que pueden entrar al planner como agujeros interiores.

    El margen de seguridad protege el implemento. La anticipacion agrega el
    espacio longitudinal/lateral que necesita el Ackermann para empezar el giro
    antes de alcanzar la zona. Si esa envolvente toca el borde, se excluye de
    esta lista y la resuelve el rodeo exterior posterior.
    """
    total_margin = max(0.0, float(safety_margin_m)) + max(
        0.0, float(turn_anticipation_m)
    )
    exclusions: List[List[Point]] = []
    for polygon in polygons:
        anticipated = inflate_polygon(polygon, total_margin)
        if polygon_is_strictly_inside_field(
            anticipated,
            bounds=bounds,
            field_boundary=field_boundary,
        ):
            exclusions.append(anticipated)
    return exclusions


def mark_required_nogo_transition_guides(
    waypoints: Sequence["CoverageBodyWaypoint"],
    polygons: Sequence[Polygon],
    *,
    margin_m: float,
) -> List["CoverageBodyWaypoint"]:
    """Marcar transiciones que no se pueden reemplazar por una cuerda.

    El preview de Fields2Cover puede rodear correctamente una exclusion con
    guias ``turn`` entre dos extremos key. Cuando el perfil de ejecucion apaga
    guias de cabecera, esos puntos desaparecen y Nav2 recibe la diagonal entre
    las keys. Si esa diagonal cruza una zona inflada, todo el bloque de guias es
    obligatorio y se marca como ``nogo_transition`` para que sobreviva.
    """
    marked = list(waypoints)
    if len(marked) < 3 or not polygons:
        return marked
    inflated = [inflate_polygon(polygon, margin_m) for polygon in polygons]
    key_indices = [
        index
        for index, waypoint in enumerate(marked)
        if bool(getattr(waypoint, "is_key", False))
    ]
    for start_index, end_index in zip(key_indices, key_indices[1:]):
        if end_index <= start_index + 1:
            continue
        guide_indices = [
            index
            for index in range(start_index + 1, end_index)
            if bool(getattr(marked[index], "is_guide", False))
        ]
        if not guide_indices:
            continue
        start = (
            float(marked[start_index].forward_m),
            float(marked[start_index].left_m),
        )
        end = (
            float(marked[end_index].forward_m),
            float(marked[end_index].left_m),
        )
        midpoint = (
            0.5 * (start[0] + end[0]),
            0.5 * (start[1] + end[1]),
        )
        crosses = any(
            len(segment_polygon_intersections(start, end, polygon)) >= 2
            or _strictly_inside(midpoint, polygon)
            for polygon in inflated
        )
        if not crosses:
            continue
        for index in guide_indices:
            if str(getattr(marked[index], "phase", "")) in {
                NOGO_DETOUR_PHASE,
                NOGO_LANE_CHANGE_PHASE,
            }:
                continue
            marked[index] = replace(
                marked[index],
                phase=NOGO_TRANSITION_PHASE,
                is_guide=True,
            )
    return marked


def inflate_polygon(polygon: Polygon, margin_m: float) -> List[Point]:
    """Correr cada vertice hacia afuera para dejar un margen de seguridad.

    El margen sale del ancho de corte: si la ruta pasara pegada al borde de la
    zona, el implemento igual la pisaria. Con rectangulos —que es lo que dibuja
    el operador— el desplazamiento por la bisectriz es exacto.
    """
    margin = float(margin_m)
    if len(polygon) < 3 or margin <= 0.0:
        return [(float(vertex[0]), float(vertex[1])) for vertex in polygon]

    # Con vertices horarios las normales salen al reves; el signo del area lo
    # corrige sin tener que reordenar el poligono.
    orientation = 1.0 if polygon_signed_area(polygon) >= 0.0 else -1.0
    count = len(polygon)
    inflated: List[Point] = []
    for index in range(count):
        previous = polygon[(index - 1) % count]
        current = polygon[index]
        following = polygon[(index + 1) % count]

        incoming = _outward_normal(previous, current, orientation)
        outgoing = _outward_normal(current, following, orientation)
        bisector_x = incoming[0] + outgoing[0]
        bisector_y = incoming[1] + outgoing[1]
        bisector_length = math.hypot(bisector_x, bisector_y)
        if bisector_length <= _EPSILON_M:
            # Vertice que se dobla sobre si mismo: no hay hacia donde inflar.
            inflated.append((float(current[0]), float(current[1])))
            continue
        bisector_x /= bisector_length
        bisector_y /= bisector_length

        cosine = (bisector_x * incoming[0]) + (bisector_y * incoming[1])
        scale = margin / cosine if cosine > _EPSILON_M else margin * _MAX_MITER_RATIO
        scale = min(scale, margin * _MAX_MITER_RATIO)
        inflated.append(
            (
                float(current[0]) + (bisector_x * scale),
                float(current[1]) + (bisector_y * scale),
            )
        )
    return inflated


def _outward_normal(start: Point, end: Point, orientation: float) -> Point:
    """Normal unitaria que apunta hacia afuera del poligono para ese lado."""
    edge_x = float(end[0]) - float(start[0])
    edge_y = float(end[1]) - float(start[1])
    length = math.hypot(edge_x, edge_y)
    if length <= _EPSILON_M:
        return (0.0, 0.0)
    return (
        (edge_y / length) * orientation,
        (-edge_x / length) * orientation,
    )


def _segment_intersection(
    first_start: Point,
    first_end: Point,
    second_start: Point,
    second_end: Point,
) -> Optional[Tuple[float, Point]]:
    """Cruce de dos segmentos; devuelve el parametro sobre el primero y el punto.

    Los casos paralelos y colineales devuelven None a proposito: si el segmento
    corre pegado a un lado de la zona no hay nada que rodear, y la contencion de
    los extremos ya la resolvio ``point_in_polygon``.
    """
    first_dx = float(first_end[0]) - float(first_start[0])
    first_dy = float(first_end[1]) - float(first_start[1])
    second_dx = float(second_end[0]) - float(second_start[0])
    second_dy = float(second_end[1]) - float(second_start[1])

    denominator = (first_dx * second_dy) - (first_dy * second_dx)
    if abs(denominator) <= _EPSILON_M:
        return None

    offset_x = float(second_start[0]) - float(first_start[0])
    offset_y = float(second_start[1]) - float(first_start[1])
    first_t = ((offset_x * second_dy) - (offset_y * second_dx)) / denominator
    second_t = ((offset_x * first_dy) - (offset_y * first_dx)) / denominator
    if not (0.0 <= first_t <= 1.0 and 0.0 <= second_t <= 1.0):
        return None
    return (
        float(first_t),
        (
            float(first_start[0]) + (first_dx * first_t),
            float(first_start[1]) + (first_dy * first_t),
        ),
    )


def segment_polygon_intersections(
    start: Point,
    end: Point,
    polygon: Polygon,
) -> List[Tuple[float, Point, int]]:
    """Cortes del segmento con el contorno, ordenados desde ``start``.

    Cada elemento es ``(t, punto, indice_de_lado)``, donde el lado va del
    vertice ``indice`` al ``indice + 1``.
    """
    crossings: List[Tuple[float, Point, int]] = []
    count = len(polygon)
    for index in range(count):
        edge_start = polygon[index]
        edge_end = polygon[(index + 1) % count]
        hit = _segment_intersection(start, end, edge_start, edge_end)
        if hit is None:
            continue
        crossings.append((hit[0], hit[1], index))
    crossings.sort(key=lambda item: item[0])
    return crossings


def _contour_walk(
    entry_edge: int,
    exit_edge: int,
    polygon: Polygon,
    *,
    forward: bool,
) -> List[Point]:
    """Vertices que se recorren yendo de un lado al otro por el contorno."""
    count = len(polygon)
    vertices: List[Point] = []
    if forward:
        index = (entry_edge + 1) % count
        while True:
            vertices.append((float(polygon[index][0]), float(polygon[index][1])))
            if index == (exit_edge % count):
                break
            index = (index + 1) % count
            if len(vertices) > count:
                break
    else:
        index = entry_edge % count
        while True:
            vertices.append((float(polygon[index][0]), float(polygon[index][1])))
            if index == ((exit_edge + 1) % count):
                break
            index = (index - 1) % count
            if len(vertices) > count:
                break
    return vertices


def _path_length(points: Sequence[Point]) -> float:
    """Largo de la poligonal que une los puntos en orden."""
    return float(
        sum(
            math.hypot(
                float(following[0]) - float(current[0]),
                float(following[1]) - float(current[1]),
            )
            for current, following in zip(points, points[1:])
        )
    )


def _within_bounds(
    points: Sequence[Point],
    bounds: Optional[Bounds],
    field_boundary: Optional[Polygon] = None,
) -> bool:
    """Decir si todos los puntos caben en el lote.

    ``bounds`` conserva el caso rectangular legacy. Para Campo poligonal se
    pasa tambien ``field_boundary``: la caja envolvente de una L o una U deja
    huecos que no son lote y no puede servir de permiso para un rodeo.
    """
    if bounds is not None:
        forward_min, forward_max, left_min, left_max = bounds
        if not all(
            (forward_min - _BOUNDS_TOLERANCE_M) <= float(point[0])
            <= (forward_max + _BOUNDS_TOLERANCE_M)
            and (left_min - _BOUNDS_TOLERANCE_M) <= float(point[1])
            <= (left_max + _BOUNDS_TOLERANCE_M)
            for point in points
        ):
            return False
    if field_boundary is not None:
        return all(point_in_polygon(point, field_boundary) for point in points)
    return True


def _point_segment_distance(point: Point, start: Point, end: Point) -> float:
    """Distancia euclidea de un punto a un segmento."""
    edge_x = float(end[0]) - float(start[0])
    edge_y = float(end[1]) - float(start[1])
    length_sq = (edge_x * edge_x) + (edge_y * edge_y)
    if length_sq <= _EPSILON_M:
        return math.hypot(
            float(point[0]) - float(start[0]),
            float(point[1]) - float(start[1]),
        )
    projection = (
        ((float(point[0]) - float(start[0])) * edge_x)
        + ((float(point[1]) - float(start[1])) * edge_y)
    ) / length_sq
    t = min(1.0, max(0.0, projection))
    closest = (
        float(start[0]) + (t * edge_x),
        float(start[1]) + (t * edge_y),
    )
    return math.hypot(
        float(point[0]) - closest[0],
        float(point[1]) - closest[1],
    )


def _distance_to_polygon_edges(point: Point, polygon: Polygon) -> float:
    """Distancia minima de un punto al contorno de un poligono."""
    if len(polygon) < 2:
        return math.inf
    return min(
        _point_segment_distance(
            point,
            polygon[index],
            polygon[(index + 1) % len(polygon)],
        )
        for index in range(len(polygon))
    )


def _path_strictly_inside_field(
    points: Sequence[Point],
    bounds: Optional[Bounds],
    field_boundary: Optional[Polygon],
) -> bool:
    field = _field_polygon(bounds, field_boundary)
    return bool(field) and all(
        _strictly_inside(point, field) for point in points
    )


def _path_has_exterior_point(
    points: Sequence[Point],
    bounds: Optional[Bounds],
    field_boundary: Optional[Polygon],
) -> bool:
    """Si el camino ya tiene un punto realmente afuera, no solo en el borde."""
    field = _field_polygon(bounds, field_boundary)
    if not field:
        return False
    return any(not point_in_polygon(point, field) for point in points)


def _simplify_round_boundary_detour(
    path: Sequence[Point],
    polygon: Polygon,
) -> List[Point]:
    """Reducir un arco circular sin convertirlo en una cuerda interior.

    El primer y ultimo punto originales quedan sobre el contorno como entrada y
    salida. Entre ellos se usa una copia radialmente exterior y raleada. Asi los
    segmentos largos siguen fuera de la exclusion aun cuando se reduzcan los
    32 vertices del circulo a unos pocos hitos.
    """
    original = [(float(x), float(y)) for x, y in path]
    vertices = [(float(x), float(y)) for x, y in polygon]
    if len(original) < 6 or len(vertices) < 12:
        return original

    center = (
        sum(point[0] for point in vertices) / len(vertices),
        sum(point[1] for point in vertices) / len(vertices),
    )
    radii = [math.hypot(point[0] - center[0], point[1] - center[1]) for point in vertices]
    average_radius = sum(radii) / len(radii)
    if average_radius <= _EPSILON_M:
        return original
    if (max(radii) - min(radii)) > max(0.10, 0.08 * average_radius):
        return original

    def expanded(point: Point) -> Point:
        dx = float(point[0]) - center[0]
        dy = float(point[1]) - center[1]
        radius = math.hypot(dx, dy)
        if radius <= _EPSILON_M:
            return (float(point[0]), float(point[1]))
        scale = (radius + _ROUND_DETOUR_CLEARANCE_M) / radius
        return (center[0] + (dx * scale), center[1] + (dy * scale))

    angles = [math.atan2(point[1] - center[1], point[0] - center[0]) for point in original]
    expanded_points = [expanded(point) for point in original]
    selected_indices = [0]
    accumulated = 0.0
    last_selected = 0
    for index in range(1, len(original)):
        delta = abs(
            (angles[index] - angles[index - 1] + math.pi)
            % (2.0 * math.pi)
            - math.pi
        )
        if (
            accumulated + delta > _ROUND_DETOUR_MAX_ANGLE_RAD
            and index - 1 > last_selected
        ):
            selected_indices.append(index - 1)
            last_selected = index - 1
            accumulated = delta
        else:
            accumulated += delta
    if selected_indices[-1] != len(original) - 1:
        selected_indices.append(len(original) - 1)

    simplified = [original[0]]
    for index in selected_indices:
        point = expanded_points[index]
        if not _same_point(point, simplified[-1]):
            simplified.append(point)
    if not _same_point(original[-1], simplified[-1]):
        simplified.append(original[-1])
    return simplified


def _shift_detour_outside_field(
    path: Sequence[Point],
    bounds: Optional[Bounds],
    field_boundary: Optional[Polygon],
) -> List[Point]:
    """Trasladar el arco de borde hacia el exterior mas cercano del lote.

    Se mantienen entrada y salida sobre la exclusion; solo se corre el tramo
    intermedio. En una zona circular esto conserva la forma general del arco y
    evita dejarlo apoyado numericamente sobre el limite.
    """
    field = _field_polygon(bounds, field_boundary)
    result = [(float(x), float(y)) for x, y in path]
    if field is None or len(result) <= 2:
        return result

    middle = result[1:-1]
    orientation = 1 if polygon_signed_area(field) >= 0.0 else -1
    nearest: Optional[Tuple[float, Point]] = None
    for index, edge_start in enumerate(field):
        edge_end = field[(index + 1) % len(field)]
        distance = min(
            _point_segment_distance(point, edge_start, edge_end)
            for point in middle
        )
        normal = _outward_normal(edge_start, edge_end, orientation)
        if nearest is None or distance < nearest[0]:
            nearest = (distance, normal)
    if nearest is None:
        return result

    normal = nearest[1]
    shifted = [result[0]]
    shifted.extend(
        (
            float(point[0]) + (_EDGE_EXIT_CLEARANCE_M * float(normal[0])),
            float(point[1]) + (_EDGE_EXIT_CLEARANCE_M * float(normal[1])),
        )
        for point in middle
    )
    shifted.append(result[-1])
    return shifted


def _closest_edge_index(point: Point, polygon: Polygon) -> int:
    """Lado del contorno mas cercano al punto."""
    return min(
        range(len(polygon)),
        key=lambda index: _point_segment_distance(
            point, polygon[index], polygon[(index + 1) % len(polygon)]
        ),
    )


def _detour_endpoints(
    start: Point,
    end: Point,
    polygon: Polygon,
) -> Optional[Tuple[Point, int, Point, int]]:
    """Por donde entra y por donde sale el tramo de la zona.

    Devuelve ``(entrada, lado_entrada, salida, lado_salida)`` o ``None`` si el
    tramo no se mete en la zona.

    El caso clasico es cortar el contorno dos veces. Desde que las zonas se le
    entregan a Fields2Cover como agujeros del lote hay otros dos, y los dos
    dejaban la ruta cruzando la zona porque el codigo exigia dos cortes:

    - la pasada TERMINA sobre el contorno, asi que el tramo que sale de ese
      extremo corta una sola vez;
    - el tramo va de un extremo de pasada al otro, los dos apoyados sobre el
      contorno, y no corta ninguna vez: es la cuerda que pasa por el medio.

    Un extremo apoyado en el contorno no se descarta —es una meta de trabajo
    valida, justo sobre la linea segura—, asi que el descarte previo de puntos
    no puede resolverlo: hay que rodear igual.
    """
    crossings = segment_polygon_intersections(start, end, polygon)
    start_dentro = point_in_polygon(start, polygon)
    end_dentro = point_in_polygon(end, polygon)

    if start_dentro:
        entry_t, entry_point, entry_edge = (
            0.0, start, _closest_edge_index(start, polygon)
        )
    elif crossings:
        entry_t, entry_point, entry_edge = crossings[0]
    else:
        return None

    if end_dentro:
        exit_t, exit_point, exit_edge = (
            1.0, end, _closest_edge_index(end, polygon)
        )
    elif crossings:
        exit_t, exit_point, exit_edge = crossings[-1]
    else:
        return None

    if exit_t <= entry_t:
        return None
    # El tramo puede estar corriendo pegado a un lado en vez de atravesar la
    # zona; ahi no hay nada que rodear.
    midpoint = (
        (entry_point[0] + exit_point[0]) / 2.0,
        (entry_point[1] + exit_point[1]) / 2.0,
    )
    if not _strictly_inside(midpoint, polygon):
        return None
    return entry_point, entry_edge, exit_point, exit_edge


def detour_along_contour(
    start: Point,
    end: Point,
    polygon: Polygon,
    *,
    bounds: Optional[Bounds] = None,
    field_boundary: Optional[Polygon] = None,
) -> List[Point]:
    """Camino que bordea la zona en vez de atravesarla.

    Devuelve los puntos intermedios —entrada, vertices del contorno y
    salida— que hay que meter entre ``start`` y ``end``. Una zona
    completamente interna se rodea por el camino mas corto que siga dentro
    del lote. Si la zona toca el borde, se elige en cambio el camino exterior:
    el vehiculo sale temporalmente del contorno original, sigue el arco de la
    exclusion inflada y vuelve a entrar. Si el segmento no corta la zona
    devuelve una lista vacia.
    """
    extremos = _detour_endpoints(start, end, polygon)
    if extremos is None:
        return []
    entry_point, entry_edge, exit_point, exit_edge = extremos

    # No confundir una tangencia numerica con una penetracion. La mascara de
    # una zona circular es poligonal y puede hacer que una recta que deberia
    # tocar el borde aparezca apenas por dentro; en ese caso rodearla completa
    # es peor que mantener la pasada recta y deja un rulo innecesario. Esta
    # tolerancia solo se aplica cuando conocemos el contorno del lote: los
    # consumidores legacy sin ese dato conservan la semantica nominal.
    if field_boundary is not None:
        midpoint = (
            (entry_point[0] + exit_point[0]) / 2.0,
            (entry_point[1] + exit_point[1]) / 2.0,
        )
        if (
            _distance_to_polygon_edges(midpoint, polygon)
            <= _TANGENT_EXCLUSION_TOLERANCE_M
        ):
            return []

    forward_vertices = _contour_walk(entry_edge, exit_edge, polygon, forward=True)
    backward_vertices = _contour_walk(entry_edge, exit_edge, polygon, forward=False)

    candidates = [
        [entry_point] + forward_vertices + [exit_point],
        [entry_point] + backward_vertices + [exit_point],
    ]
    inside = [
        path
        for path in candidates
        if _within_bounds(path, bounds, field_boundary)
    ]
    boundary_zone = not polygon_is_strictly_inside_field(
        polygon,
        bounds=bounds,
        field_boundary=field_boundary,
    )
    if boundary_zone and (bounds is not None or field_boundary is not None):
        # En una zona de borde el rodeo interior obliga a volver sobre filas ya
        # trabajadas. La regla operativa es la contraria: salir por el hueco
        # ya ocupa la exclusion, completar el arco exterior y reingresar.
        outside = [
            path
            for path in candidates
            if not _path_strictly_inside_field(path, bounds, field_boundary)
        ]
        selected = min(outside or candidates, key=_path_length)
        selected = _simplify_round_boundary_detour(selected, polygon)
        # Si el propio contorno inflado ya cruza el limite, desplazar solamente
        # sus puntos intermedios rompe la continuidad: el ultimo segmento vuelve
        # oblicuo desde el arco desplazado hasta la salida original y corta la
        # misma zona. En la pasada siguiente se vuelve a rodear y la cantidad
        # de puntos crece sin limite. El contorno ya es exterior y seguro; el
        # corrimiento solo hace falta en el caso tangente, donde ningun punto
        # salio realmente del lote.
        if _path_has_exterior_point(selected, bounds, field_boundary):
            return selected
        return _shift_detour_outside_field(selected, bounds, field_boundary)
    else:
        elegibles = inside or candidates
    return min(elegibles, key=_path_length)


def _heading_deg(start: Point, end: Point, fallback_deg: float) -> float:
    """Rumbo del tramo en el marco del cuerpo, en grados."""
    delta_x = float(end[0]) - float(start[0])
    delta_y = float(end[1]) - float(start[1])
    if math.hypot(delta_x, delta_y) <= _EPSILON_M:
        return float(fallback_deg)
    return float(math.degrees(math.atan2(delta_y, delta_x)))


def clip_plan_to_nogo(
    waypoints: Sequence[CoverageBodyWaypoint],
    polygons: Sequence[Polygon],
    *,
    margin_m: float = 0.0,
    bounds: Optional[Bounds] = None,
    field_boundary: Optional[Polygon] = None,
) -> Tuple[List[CoverageBodyWaypoint], int, int]:
    """Sacar del plan lo que cae en una zona y bordear lo que la cruza.

    Devuelve ``(waypoints, descartados, rodeos)``. Los waypoints que caen dentro
    de una zona se descartan; los tramos que la cruzan se reemplazan por un
    camino que sigue el contorno. Lanza ``ValueError`` si no queda plan.
    """
    if not polygons:
        return list(waypoints), 0, 0

    inflated = [inflate_polygon(polygon, margin_m) for polygon in polygons]
    inflated = [polygon for polygon in inflated if len(polygon) >= 3]
    if not inflated:
        return list(waypoints), 0, 0

    kept: List[CoverageBodyWaypoint] = []
    dropped = 0
    for waypoint in waypoints:
        position = (float(waypoint.forward_m), float(waypoint.left_m))
        # `_strictly_inside` y no `point_in_polygon`: el contorno inflado es
        # justamente la linea segura por donde se puede circular, asi que los
        # puntos que un rodeo dejo apoyados ahi tienen que sobrevivir. Si no, una
        # segunda pasada del recorte —la del cockpit sobre lo que ya recorto el
        # backend— borraria el rodeo que acaba de construir.
        if any(_strictly_inside(position, polygon) for polygon in inflated):
            dropped += 1
            continue
        kept.append(waypoint)

    if len(kept) < 2:
        raise ValueError("las zonas no-go no dejan superficie para cubrir")

    detoured, dropped_during_detour, detours = _insert_detours(
        kept, inflated, bounds, field_boundary
    )
    return detoured, dropped + dropped_during_detour, detours


def plan_nogo_conflicts(
    waypoints: Sequence[CoverageBodyWaypoint],
    polygons: Sequence[Polygon],
    *,
    margin_m: float = 0.0,
) -> Tuple[List[int], List[int]]:
    """Indices de puntos y tramos que penetran una zona inflada.

    A diferencia de :func:`clip_plan_to_nogo`, esta funcion nunca construye un
    rodeo. Sirve para auditar los cambios de fila interiores, donde una
    penetracion debe rechazar el plan y nunca convertirse en una circunferencia
    sobre el cultivo.
    """
    inflated = [inflate_polygon(polygon, margin_m) for polygon in polygons]
    inflated = [polygon for polygon in inflated if len(polygon) >= 3]
    point_conflicts: List[int] = []
    segment_conflicts: List[int] = []
    for index, waypoint in enumerate(waypoints):
        position = (float(waypoint.forward_m), float(waypoint.left_m))
        if any(_strictly_inside(position, polygon) for polygon in inflated):
            point_conflicts.append(index)
    for index, (origin, target) in enumerate(zip(waypoints, waypoints[1:])):
        start = (float(origin.forward_m), float(origin.left_m))
        end = (float(target.forward_m), float(target.left_m))
        if any(_detour_endpoints(start, end, polygon) is not None for polygon in inflated):
            segment_conflicts.append(index)
    return point_conflicts, segment_conflicts


def _last_position(
    waypoints: Sequence["CoverageBodyWaypoint"],
) -> Optional[Point]:
    """Posicion del ultimo waypoint acumulado, o None si todavia no hay."""
    if not waypoints:
        return None
    ultimo = waypoints[-1]
    return (float(ultimo.forward_m), float(ultimo.left_m))


def _same_point(point: Point, other: Optional[Point]) -> bool:
    """Dos posiciones que aguas abajo serian el mismo waypoint."""
    if other is None:
        return False
    return math.hypot(
        float(point[0]) - float(other[0]),
        float(point[1]) - float(other[1]),
    ) <= _DUPLICATE_POINT_TOLERANCE_M


def _insert_detours(
    waypoints: Sequence[CoverageBodyWaypoint],
    polygons: Sequence[Polygon],
    bounds: Optional[Bounds] = None,
    field_boundary: Optional[Polygon] = None,
) -> Tuple[List[CoverageBodyWaypoint], int, int]:
    """Reemplazar cada tramo que cruza una zona por un rodeo del contorno.

    Se repite hasta que ningun tramo cruce: un rodeo puede meter la ruta dentro
    de otra zona. El tope de pasadas evita el ciclo cuando dos zonas se encierran
    entre si; si se agota, se devuelve lo que haya y el llamador vera que el
    conteo de cruces no dio cero.
    """
    current = list(waypoints)
    dropped = 0
    total_detours = 0
    for _ in range(_MAX_DETOUR_PASSES):
        # Un rodeo de una zona solapada puede dejar uno de sus vertices dentro
        # de otra. El filtrado inicial no lo ve porque esos puntos todavia no
        # existian. Repetirlo antes de inspeccionar segmentos es lo que hace al
        # recorte realmente idempotente: si una segunda pasada tendria que
        # borrar un punto, esta primera todavia no termino.
        filtered = []
        for waypoint in current:
            position = (float(waypoint.forward_m), float(waypoint.left_m))
            if any(_strictly_inside(position, polygon) for polygon in polygons):
                dropped += 1
                continue
            filtered.append(waypoint)
        if len(filtered) < 2:
            raise ValueError("las zonas no-go no dejan superficie para cubrir")
        current = filtered

        rebuilt: List[CoverageBodyWaypoint] = []
        pass_detours = 0
        for index in range(len(current) - 1):
            origin = current[index]
            target = current[index + 1]
            rebuilt.append(origin)

            start = (float(origin.forward_m), float(origin.left_m))
            end = (float(target.forward_m), float(target.left_m))
            detour = _shortest_detour(
                start, end, polygons, bounds, field_boundary
            )
            if not detour:
                continue

            pass_detours += 1
            for step, point in enumerate(detour):
                following = detour[step + 1] if (step + 1) < len(detour) else end
                # La entrada y la salida del rodeo se calculan como
                # interseccion del tramo con el contorno, y en una zona
                # circular esa interseccion cae justo sobre un vertice del
                # contorno bastante seguido. Repetirlo dejaria una meta de
                # largo cero aguas abajo, que ademas no tiene rumbo definido.
                if _same_point(point, _last_position(rebuilt)):
                    continue
                precise_midpoint = bool(
                    len(detour) >= 3 and step == (len(detour) // 2)
                )
                # Se clona el waypoint destino en vez de construir uno nuevo:
                # asi este modulo no necesita importar la clase, que vive en
                # coverage_waypoint_core y llama aca (seria un ciclo).
                rebuilt.append(
                    replace(
                        target,
                        forward_m=float(point[0]),
                        left_m=float(point[1]),
                        yaw_delta_deg=_heading_deg(
                            point, following, origin.yaw_delta_deg
                        ),
                        phase=NOGO_DETOUR_PHASE,
                        # Entrada y salida son guias flexibles. Solo el punto
                        # medio del rodeo es una meta precisa, para que Nav2 no
                        # corte el circulo sin detenerse en cada vertice.
                        is_key=precise_midpoint,
                        is_guide=not precise_midpoint,
                    )
                )
        rebuilt.append(current[-1])
        current = rebuilt
        total_detours += pass_detours
        if pass_detours == 0:
            return current, dropped, total_detours

    # No devolver una ruta que aun requeriria otro rodeo: el llamador la toma
    # como ejecutable y una segunda aplicacion del recorte cambiaria el plan.
    raise ValueError("las zonas no-go solapadas no estabilizaron el rodeo")


def _shortest_detour(
    start: Point,
    end: Point,
    polygons: Sequence[Polygon],
    bounds: Optional[Bounds] = None,
    field_boundary: Optional[Polygon] = None,
) -> List[Point]:
    """Rodeo de la primera zona que corta el tramo, yendo desde ``start``."""
    best: Optional[Tuple[float, List[Point]]] = None
    largo = math.hypot(
        float(end[0]) - float(start[0]),
        float(end[1]) - float(start[1]),
    )
    for polygon in polygons:
        extremos = _detour_endpoints(start, end, polygon)
        if extremos is None:
            continue
        detour = detour_along_contour(
            start,
            end,
            polygon,
            bounds=bounds,
            field_boundary=field_boundary,
        )
        if not detour:
            continue
        # Cual entra primero, para rodear las zonas en el orden en que
        # aparecen. Con un extremo apoyado en el contorno no hay corte del que
        # sacar la fraccion, asi que se mide sobre el propio punto de entrada.
        entrada = extremos[0]
        entry_t = (
            0.0
            if largo <= _EPSILON_M
            else math.hypot(
                entrada[0] - float(start[0]),
                entrada[1] - float(start[1]),
            ) / largo
        )
        if best is None or entry_t < best[0]:
            best = (entry_t, detour)
    return list(best[1]) if best is not None else []
