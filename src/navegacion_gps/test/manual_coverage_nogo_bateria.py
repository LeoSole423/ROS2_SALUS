"""Campo + zona no-go dibujada en el mapa, contra el backend real.

Comprueba lo que rompia el arranque: que el backend informe las zonas que
aplico. Si informa cero, el cockpit repite el recorte, ve la diferencia y
bloquea el inicio con "el backend no aplico las zonas no-go".
"""
import json
import math
import sys

import rclpy
from rclpy.node import Node

from interfaces.srv import GenerateCoveragePlanLL, GetZonesState, SetZonesGeoJson
from interfaces.msg import GeoRing, NoGoPoint

LAT0, LON0 = -31.485802, -64.241050
MLAT = 111320.0
MLON = 111320.0 * math.cos(math.radians(LAT0))
RMIN = 2.9

ll = lambda x, y: (LAT0 + y / MLAT, LON0 + x / MLON)
xy = lambda lat, lon: ((lon - LON0) * MLON, (lat - LAT0) * MLAT)


def ring(pts):
    r = GeoRing()
    r.vertices = [NoGoPoint(lat=ll(x, y)[0], lon=ll(x, y)[1]) for x, y in pts]
    return r


def octogono(R=25.0, N=8):
    return [(R * math.cos(2 * math.pi * i / N), R * math.sin(2 * math.pi * i / N))
            for i in range(N)]


def cuadro(cx, cy, w, h):
    return [(cx - w / 2, cy - h / 2), (cx + w / 2, cy - h / 2),
            (cx + w / 2, cy + h / 2), (cx - w / 2, cy + h / 2)]


def dentro(p, poly):
    x, y = p
    adentro = False
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        if (ay > y) != (by > y):
            if ax + ((y - ay) / (by - ay)) * (bx - ax) > x:
                adentro = not adentro
    return adentro


def geojson_de(poly, nombre):
    anillo = [[ll(x, y)[1], ll(x, y)[0]] for x, y in poly]
    anillo.append(anillo[0])
    return json.dumps({
        "type": "FeatureCollection",
        "features": [{
            "type": "Feature",
            "properties": {"id": nombre, "type": "no_go", "enabled": True},
            "geometry": {"type": "Polygon", "coordinates": [anillo]},
        }],
    })


def llamar(cliente, req, timeout=60.0):
    fut = cliente.call_async(req)
    rclpy.spin_until_future_complete(cliente._node_ref if False else NODO, fut, timeout_sec=timeout)
    if not fut.done():
        raise TimeoutError("el servicio no respondio")
    return fut.result()


def pedido(lote):
    req = GenerateCoveragePlanLL.Request()
    req.start_lat, req.start_lon, req.start_yaw_deg = LAT0, LON0, 0.0
    req.field_length_m = req.field_width_m = 60.0
    req.cutter_width_m, req.overlap_ratio = 2.0, 0.15
    req.min_turning_radius_m, req.waypoint_spacing_m = RMIN, 2.0
    req.side = "left"
    req.coverage_polygon = ring(lote)
    req.coverage_exclusions = []
    return req


CASOS = [
    ("zona EN EL MEDIO", cuadro(0.0, 0.0, 10.0, 8.0)),
    ("zona EN EL BORDE este", cuadro(22.0, 0.0, 10.0, 12.0)),
    ("zona en una ESQUINA", cuadro(16.0, 16.0, 12.0, 12.0)),
    ("zona chica al borde norte", cuadro(0.0, 22.0, 6.0, 8.0)),
]


def main():
    global NODO
    rclpy.init()
    NODO = Node("prueba_zona_nogo")
    zonas = NODO.create_client(SetZonesGeoJson, "/zones_manager/set_geojson")
    zonas_estado = NODO.create_client(GetZonesState, "/zones_manager/get_state")
    plan = NODO.create_client(GenerateCoveragePlanLL, "/route_executor/generate_coverage_plan_ll")
    for cliente, nombre in (
        (zonas, "set_geojson"),
        (zonas_estado, "get_state"),
        (plan, "generate_coverage_plan_ll"),
    ):
        if not cliente.wait_for_service(timeout_sec=15.0):
            print(f"no esta {nombre}")
            return 1

    lote = octogono(25.0)
    fallos = 0
    for nombre, zona in CASOS:
        res = llamar(zonas, SetZonesGeoJson.Request(geojson=geojson_de(zona, nombre)))
        # ok=False puede ser solo la mascara keepout: el costmap no se recarga
        # en esta sim. Lo que le importa a la cobertura es el estado, que es de
        # donde el route_executor lee las zonas. Se comprueba ahi.
        estado = llamar(zonas_estado, GetZonesState.Request())
        if nombre.split()[0] not in estado.geojson and '"no_go"' not in estado.geojson:
            print(f"  {nombre:32s} FALLO al cargar la zona: {res.error}")
            fallos += 1
            continue
        # El route_executor refresca las zonas por timer; se le da tiempo.
        fin = NODO.get_clock().now().nanoseconds + int(4e9)
        while NODO.get_clock().now().nanoseconds < fin:
            rclpy.spin_once(NODO, timeout_sec=0.1)

        r = llamar(plan, pedido(lote))
        if not r.ok:
            print(f"  {nombre:32s} FALLO: {r.error}")
            fallos += 1
            continue

        puntos = [xy(la, lo) for la, lo in zip(r.sampled_lats, r.sampled_lons)]
        trabajo = [p for p, f in zip(puntos, r.sampled_phases) if f == "row"]
        en_zona = sum(1 for p in trabajo if dentro(p, zona))
        margen = 0.5 * 2.0 + 0.5  # medio implemento + colchon, igual que el backend
        problemas = []
        if int(r.nogo_polygon_count) == 0:
            problemas.append("el backend informa 0 zonas aplicadas -> el cockpit bloquea el inicio")
        if en_zona:
            problemas.append(f"{en_zona} puntos de trabajo DENTRO de la zona")
        estado = "OK " if not problemas else "MAL"
        print(
            f"  {nombre:32s} {estado} zonas={r.nogo_polygon_count} "
            f"borrados={r.nogo_dropped_count} rodeos={r.nogo_detour_count} "
            f"trabajo_en_zona={en_zona} nota='{r.nogo_note}'"
        )
        for problema in problemas:
            print(f"       -> {problema}")
            fallos += 1

    # Se deja el mapa limpio para no dejarle zonas puestas al operador.
    llamar(zonas, SetZonesGeoJson.Request(
        geojson='{"type":"FeatureCollection","features":[]}'))
    print(f"\nresultado: {len(CASOS) - fallos}/{len(CASOS)} casos OK")
    NODO.destroy_node()
    rclpy.shutdown()
    return 1 if fallos else 0


if __name__ == "__main__":
    sys.exit(main())
