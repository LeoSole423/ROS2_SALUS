"""Validacion del lote de cobertura cuando llega como poligono.

El lote historico es un rectangulo definido por pose + largo + ancho. Este
modulo agrega la forma general: un anillo exterior y cero o mas anillos
interiores. La validacion vive aparte del planificador a proposito, para poder
probarla sin ROS y para que el planificador reciba geometria ya sana.

Dos conceptos que no hay que mezclar:

``coverage_exclusion``
    Los anillos interiores de este modulo. Zona donde NO se debe cortar. El
    planificador no genera pasadas de trabajo adentro, pero un giro de cabecera
    puede atravesarla: los enlaces Dubins entre pasadas no son conscientes de
    obstaculos, medido tanto con el planificador propio como con Fields2Cover.

``keepout`` / ``no_go``
    Zona donde el vehiculo no puede entrar nunca. Eso lo hace el filtro keepout
    del costmap de Nav2, no este modulo. Una exclusion de cobertura NO es una
    garantia de seguridad de navegacion.

Los chequeos geometricos corren en metros locales y no en grados: a 31 grados de
latitud un grado de longitud mide 0.85 de uno de latitud, y sobre coordenadas
crudas un anillo sano puede parecer que se cruza.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

Vertex = Tuple[float, float]
Ring = Sequence[Vertex]

_METERS_PER_DEG_LAT = 111_320.0

# Dos vertices mas juntos que esto se toman como repetidos. Un GPS RTK resuelve
# centimetros, asi que por debajo del milimetro es ruido de serializacion.
_DUPLICATE_TOLERANCE_M = 1.0e-3

_EPSILON = 1.0e-12


def ring_to_local_m(ring: Ring, origin: Vertex) -> List[Vertex]:
    """Pasar un anillo lat/lon a metros locales planos contra un origen."""
    origin_lat = float(origin[0])
    meters_per_deg_lon = _METERS_PER_DEG_LAT * max(
        1.0e-6, abs(math.cos(math.radians(origin_lat)))
    )
    return [
        (
            (float(lon) - float(origin[1])) * meters_per_deg_lon,
            (float(lat) - origin_lat) * _METERS_PER_DEG_LAT,
        )
        for lat, lon in ring
    ]


def ring_signed_area_m2(points: Ring) -> float:
    """Area con signo del anillo en metros locales; positiva si es antihorario."""
    total = 0.0
    count = len(points)
    for index in range(count):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % count]
        total += (float(x1) * float(y2)) - (float(x2) * float(y1))
    return total / 2.0


def _segments_cross(
    a1: Vertex, a2: Vertex, b1: Vertex, b2: Vertex
) -> bool:
    """Decir si dos segmentos se cruzan, sin contar el toque en un extremo."""
    def orientation(p: Vertex, q: Vertex, r: Vertex) -> float:
        return ((q[0] - p[0]) * (r[1] - p[1])) - ((q[1] - p[1]) * (r[0] - p[0]))

    d1 = orientation(b1, b2, a1)
    d2 = orientation(b1, b2, a2)
    d3 = orientation(a1, a2, b1)
    d4 = orientation(a1, a2, b2)
    if ((d1 > _EPSILON and d2 < -_EPSILON) or (d1 < -_EPSILON and d2 > _EPSILON)) and (
        (d3 > _EPSILON and d4 < -_EPSILON) or (d3 < -_EPSILON and d4 > _EPSILON)
    ):
        return True

    def on_segment(p: Vertex, q: Vertex, r: Vertex) -> bool:
        # r colineal con pq y adentro del rectangulo que los contiene.
        return (
            min(p[0], q[0]) - _EPSILON <= r[0] <= max(p[0], q[0]) + _EPSILON
            and min(p[1], q[1]) - _EPSILON <= r[1] <= max(p[1], q[1]) + _EPSILON
        )

    for d, (p, q, r) in (
        (d1, (b1, b2, a1)),
        (d2, (b1, b2, a2)),
        (d3, (a1, a2, b1)),
        (d4, (a1, a2, b2)),
    ):
        if abs(d) <= _EPSILON and on_segment(p, q, r):
            return True
    return False


def ring_is_simple(points: Ring) -> bool:
    """Decir si el anillo no se cruza a si mismo.

    Se comparan todos los pares de lados no adyacentes. Es O(n^2), y esta bien:
    un lote dibujado a mano tiene decenas de vertices, no miles.
    """
    count = len(points)
    if count < 3:
        return False
    for i in range(count):
        a1, a2 = points[i], points[(i + 1) % count]
        for j in range(i + 1, count):
            if j == i or (j + 1) % count == i or (i + 1) % count == j:
                continue
            if _segments_cross(a1, a2, points[j], points[(j + 1) % count]):
                return False
    return True


def point_in_ring(point: Vertex, points: Ring) -> bool:
    """Ray casting; un punto sobre el borde cuenta como adentro."""
    x, y = float(point[0]), float(point[1])
    inside = False
    count = len(points)
    for index in range(count):
        ax, ay = points[index]
        bx, by = points[(index + 1) % count]
        if (ay > y) != (by > y):
            crossing = ax + ((y - ay) / (by - ay)) * (bx - ax)
            if crossing > x:
                inside = not inside
    return inside


def _clean_ring(ring: Ring) -> List[Vertex]:
    """Sacar el cierre repetido y los vertices duplicados consecutivos."""
    points = [(float(lat), float(lon)) for lat, lon in ring]
    if len(points) >= 2 and points[0] == points[-1]:
        points = points[:-1]
    return points


def _is_collinear(points: Ring) -> bool:
    """Decir si todos los vertices caen sobre una misma recta."""
    if len(points) < 3:
        return True
    x0, y0 = points[0]
    for index in range(1, len(points) - 1):
        x1, y1 = points[index]
        x2, y2 = points[index + 1]
        cross = ((x1 - x0) * (y2 - y0)) - ((y1 - y0) * (x2 - x0))
        if abs(cross) > _DUPLICATE_TOLERANCE_M:
            return False
    return True


def _ring_problem(points: List[Vertex], nombre: str) -> Optional[str]:
    """Motivo por el que el anillo no sirve, o None si esta bien."""
    if len(points) < 3:
        return f"{nombre} necesita al menos 3 vertices (llegaron {len(points)})"
    for lat, lon in points:
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return f"{nombre} tiene un vertice no finito"
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return f"{nombre} tiene un vertice fuera del rango lat/lon"

    local = ring_to_local_m(points, points[0])
    count = len(local)
    for index in range(count):
        x1, y1 = local[index]
        x2, y2 = local[(index + 1) % count]
        if math.hypot(x2 - x1, y2 - y1) <= _DUPLICATE_TOLERANCE_M:
            return f"{nombre} tiene vertices repetidos"
    # El orden importa para que el mensaje sea util. Un anillo colineal tambien
    # da "se cruza a si mismo" —sus lados se superponen— y un moño simetrico
    # tambien da area cero, asi que cada caso se detecta con su propia prueba en
    # vez de deducirlo del area.
    if _is_collinear(local):
        return f"{nombre} es degenerado: los vertices son colineales"
    if not ring_is_simple(local):
        return f"{nombre} se cruza a si mismo"
    if abs(ring_signed_area_m2(local)) <= _DUPLICATE_TOLERANCE_M:
        return f"{nombre} es degenerado: encierra area cero"
    return None


def validate_coverage_field(
    outer_ll: Ring,
    exclusions_ll: Sequence[Ring] = (),
) -> Tuple[Optional[List[Vertex]], List[List[Vertex]], str]:
    """Validar el lote y devolver ``(exterior, exclusiones, error)``.

    El exterior vuelve como anillo abierto en lat/lon. Con ``error`` no vacio los
    dos primeros valores no sirven. Un exterior vacio no es un error: significa
    que el pedido viene en modo rectangulo legacy.
    """
    outer = _clean_ring(outer_ll)
    if not outer:
        return None, [], ""

    problem = _ring_problem(outer, "el poligono del lote")
    if problem:
        return None, [], problem

    origin = outer[0]
    outer_local = ring_to_local_m(outer, origin)

    exclusions: List[List[Vertex]] = []
    for index, raw in enumerate(exclusions_ll, start=1):
        hole = _clean_ring(raw)
        if not hole:
            continue
        nombre = f"la exclusion de cobertura #{index}"
        problem = _ring_problem(hole, nombre)
        if problem:
            return None, [], problem

        hole_local = ring_to_local_m(hole, origin)
        # Adentro de verdad: todos los vertices adentro y ningun lado que corte
        # el contorno. Con solo los vertices, una exclusion en forma de C podria
        # abrazar el borde del lote con todas sus puntas adentro.
        if not all(point_in_ring(vertex, outer_local) for vertex in hole_local):
            return None, [], f"{nombre} cae fuera del poligono del lote"
        count_h, count_o = len(hole_local), len(outer_local)
        for i in range(count_h):
            a1, a2 = hole_local[i], hole_local[(i + 1) % count_h]
            for j in range(count_o):
                if _segments_cross(
                    a1, a2, outer_local[j], outer_local[(j + 1) % count_o]
                ):
                    return None, [], f"{nombre} cruza el borde del lote"
        exclusions.append(hole)

    return outer, exclusions, ""
