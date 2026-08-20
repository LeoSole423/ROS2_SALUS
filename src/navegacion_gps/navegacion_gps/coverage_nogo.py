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
# left_max). El rodeo no puede salirse de ahi.
Bounds = Tuple[float, float, float, float]

# Fase con la que se marcan los puntos que agrega el rodeo. El route_executor no
# la interpreta, pero deja el plan legible en los logs y en el preview.
NOGO_DETOUR_PHASE = "nogo_detour"

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

# Tolerancia para decidir si un punto cae sobre el contorno. Es mas floja que
# `_EPSILON_M` a proposito: los puntos del rodeo se calculan como interseccion
# de segmentos y caen sobre el borde con el error de redondeo de esa cuenta, no
# exactamente encima. Un micrometro sobra para el campo y no confunde nada.
_ON_EDGE_TOLERANCE_M = 1.0e-6


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


def detour_along_contour(
    start: Point,
    end: Point,
    polygon: Polygon,
    *,
    bounds: Optional[Bounds] = None,
    field_boundary: Optional[Polygon] = None,
) -> List[Point]:
    """Camino que bordea la zona en vez de atravesarla.

    Devuelve los puntos intermedios —entrada, vertices del contorno y salida—
    que hay que meter entre ``start`` y ``end``. Se prueban los dos sentidos del
    perimetro y gana el mas corto **de los que se quedan dentro del lote**: una
    zona pegada al borde tiene un lado cuyo contorno cae afuera, y salir del
    lote para rodearla es peor que dar la vuelta larga por adentro. Si el
    segmento no corta la zona devuelve una lista vacia.
    """
    crossings = segment_polygon_intersections(start, end, polygon)
    if len(crossings) < 2:
        # Un solo corte significa que una punta esta adentro; eso lo resuelve el
        # descarte previo de waypoints, no el rodeo.
        return []

    entry_t, entry_point, entry_edge = crossings[0]
    exit_t, exit_point, exit_edge = crossings[-1]
    if exit_t <= entry_t:
        return []

    # El tramo puede estar corriendo pegado a un lado en vez de atravesar la
    # zona; ahi no hay nada que rodear.
    midpoint = (
        (entry_point[0] + exit_point[0]) / 2.0,
        (entry_point[1] + exit_point[1]) / 2.0,
    )
    if not _strictly_inside(midpoint, polygon):
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
    # Si los dos lados se salen del lote, la zona lo cruza de lado a lado: no hay
    # por donde bordear. Se devuelve el mas corto igual —el llamador ya no tiene
    # mejor opcion— pero el caso queda documentado aca y no como una sorpresa.
    if not inside and field_boundary is not None:
        raise ValueError("no hay rodeo no-go que se mantenga dentro del lote")
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
                        is_key=True,
                        is_guide=False,
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
    for polygon in polygons:
        crossings = segment_polygon_intersections(start, end, polygon)
        if len(crossings) < 2:
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
        entry_t = crossings[0][0]
        if best is None or entry_t < best[0]:
            best = (entry_t, detour)
    return list(best[1]) if best is not None else []
