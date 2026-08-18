"""Puente entre las zonas no-go del cockpit y el planificador de cobertura.

`zones_manager` guarda las zonas como GeoJSON en lat/lon; el planificador de
cobertura trabaja en el marco del cuerpo del lote. Este modulo hace la
traduccion y nada mas: pide el estado al servicio, filtra lo que corresponde y
proyecta los vertices.

La degradacion es a proposito silenciosa en cuanto a fallas: si `zones_manager`
no esta corriendo —una simulacion sin el editor de zonas, por ejemplo— se
planifica sin zonas y se devuelve el motivo, para que el llamador lo loguee y lo
reporte. Nunca se aborta la planificacion por no poder leer las zonas: dejar al
operador sin cobertura porque un nodo auxiliar esta caido es peor que
planificar sin recorte, siempre que se avise.
"""

from __future__ import annotations

import json
import math
from typing import Any, List, Optional, Sequence, Tuple

from navegacion_gps.zones_geojson_utils import iter_polygons
from navegacion_gps.zones_geojson_utils import normalize_geojson_object

Point = Tuple[float, float]

# Solo este tipo de zona recorta la cobertura. `zones_manager` normaliza el
# campo y por defecto pone "no_go", asi que en la practica entra todo lo que
# dibuja el cockpit.
NO_GO_ZONE_TYPE = "no_go"

_METERS_PER_DEG_LAT = 111_320.0


class NoGoZonesResult:
    """Zonas listas para el planificador, con el motivo si no se pudieron leer."""

    def __init__(
        self,
        polygons: Sequence[Sequence[Point]],
        *,
        available: bool,
        note: str = "",
    ) -> None:
        """Guardar los poligonos proyectados y el estado de la consulta."""
        self.polygons: List[List[Point]] = [
            [(float(x), float(y)) for x, y in polygon] for polygon in polygons
        ]
        self.available = bool(available)
        self.note = str(note)

    def __bool__(self) -> bool:
        """Ser verdadero cuando hay al menos un poligono utilizable."""
        return bool(self.polygons)


def ll_to_body(
    lat: float,
    lon: float,
    *,
    origin_lat: float,
    origin_lon: float,
    origin_yaw_deg: float,
) -> Point:
    """Proyectar un lat/lon al marco del cuerpo del lote.

    Es la inversa exacta de la georreferenciacion que aplica
    ``build_lawnmower_waypoints`` (``body_relative_offsets_to_north_east`` mas
    ``offset_lat_lon``). Usar el mismo ancla y la misma aproximacion de tierra
    plana es lo que evita que el recorte se corra respecto del trazado cuando el
    lote esta en diagonal.
    """
    meters_per_deg_lon = _METERS_PER_DEG_LAT * max(
        1.0e-6, abs(math.cos(math.radians(float(origin_lat))))
    )
    north_m = (float(lat) - float(origin_lat)) * _METERS_PER_DEG_LAT
    east_m = (float(lon) - float(origin_lon)) * meters_per_deg_lon

    yaw_rad = math.radians(float(origin_yaw_deg))
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    forward_m = (cos_yaw * east_m) + (sin_yaw * north_m)
    left_m = (-sin_yaw * east_m) + (cos_yaw * north_m)
    return (float(forward_m), float(left_m))


def polygons_from_geojson(
    geojson_text: str,
    *,
    origin_lat: float,
    origin_lon: float,
    origin_yaw_deg: float,
) -> List[List[Point]]:
    """Sacar los contornos no-go habilitados del GeoJSON, en marco del cuerpo.

    Los agujeros (``holes_ll``) se ignoran: una zona con agujero se trata como
    maciza, que es el lado conservador. El cockpit hoy solo dibuja rectangulos y
    poligonos simples, asi que no aparece en la practica.
    """
    if not str(geojson_text).strip():
        return []
    document = normalize_geojson_object(json.loads(str(geojson_text)))

    polygons: List[List[Point]] = []
    for entry in iter_polygons(document):
        if not bool(entry.get("enabled", True)):
            continue
        if str(entry.get("type", NO_GO_ZONE_TYPE)) != NO_GO_ZONE_TYPE:
            continue
        ring = entry.get("outer_ll", [])
        if not isinstance(ring, list) or len(ring) < 4:
            continue
        # El anillo GeoJSON viene cerrado (primer vertice repetido al final) y
        # en orden [lon, lat]; el planificador quiere el contorno abierto.
        body_ring = [
            ll_to_body(
                float(vertex[1]),
                float(vertex[0]),
                origin_lat=origin_lat,
                origin_lon=origin_lon,
                origin_yaw_deg=origin_yaw_deg,
            )
            for vertex in ring[:-1]
        ]
        if len(body_ring) >= 3:
            polygons.append(body_ring)
    return polygons


def fetch_no_go_polygons_body(
    client: Optional[Any],
    request_type: Any,
    *,
    origin_lat: float,
    origin_lon: float,
    origin_yaw_deg: float,
    wait_timeout_s: float = 1.0,
    call_timeout_s: float = 2.0,
    wait_for_future: Optional[Any] = None,
) -> NoGoZonesResult:
    """Pedirle las zonas a ``zones_manager`` y devolverlas en marco del cuerpo.

    ``wait_for_future`` es la funcion que bloquea esperando la respuesta; se
    inyecta porque cada nodo ya tiene la suya y no conviene armar un executor
    nuevo desde adentro de un callback de servicio.
    """
    if client is None:
        return NoGoZonesResult([], available=False, note="zones service not configured")
    if not client.wait_for_service(timeout_sec=float(wait_timeout_s)):
        return NoGoZonesResult([], available=False, note="zones service unavailable")

    future = client.call_async(request_type())
    try:
        if wait_for_future is None:
            return NoGoZonesResult(
                [], available=False, note="no wait_for_future provided"
            )
        response = wait_for_future(future, float(call_timeout_s))
    except Exception as exc:  # pragma: no cover - depende del middleware
        return NoGoZonesResult([], available=False, note=f"zones call failed: {exc}")
    if response is None:
        return NoGoZonesResult([], available=False, note="timeout waiting zones state")
    if not bool(getattr(response, "ok", False)):
        reason = str(getattr(response, "error", "") or "unknown error")
        return NoGoZonesResult([], available=False, note=f"zones service error: {reason}")

    try:
        polygons = polygons_from_geojson(
            str(getattr(response, "geojson", "")),
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            origin_yaw_deg=origin_yaw_deg,
        )
    except (TypeError, ValueError) as exc:
        return NoGoZonesResult([], available=False, note=f"invalid zones geojson: {exc}")
    return NoGoZonesResult(polygons, available=True)
