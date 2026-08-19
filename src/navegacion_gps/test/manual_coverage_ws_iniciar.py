"""CAMPO de punta a punta por el WebSocket, igual que el cockpit.

Prueba lo que el operador ve: preview y despues INICIAR COBERTURA. No pasa por
los servicios ROS a mano — habla el mismo protocolo que el cockpit, porque el
sintoma ("no me deja iniciar") vivia justamente en ese camino y no en el
servicio.

    python3 src/navegacion_gps/test/manual_coverage_ws_iniciar.py [ws://host:puerto]
"""
import asyncio
import json
import math
import sys

import websockets

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
CON_ZONA = "--con-zona" in sys.argv
URL = ARGS[0] if ARGS else "ws://localhost:8766"
RMIN = 2.9
LADO_M = 40.0


def octogono(lat, lon, lado_m, yaw_deg=0.0, n=8):
    """Octogono regular centrado en (lat, lon), igual al que siembra el cockpit."""
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat))
    radio = (lado_m / 2.0) / math.cos(math.pi / n)
    rumbo = math.radians(90.0 - yaw_deg)
    salida = []
    for i in range(n):
        a = rumbo + math.pi / n + 2 * math.pi * i / n
        salida.append({
            "lat": lat + radio * math.sin(a) / mlat,
            "lon": lon + radio * math.cos(a) / mlon,
        })
    return {"vertices": salida}


def zona_geojson(lat, lon, ancho_m=10.0, alto_m=8.0, corrimiento_m=8.0):
    """Cuadro no-go pisando el lote, como el que se dibuja a mano en el mapa."""
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat))
    cx, cy = corrimiento_m, 0.0
    esquinas = [
        (cx - ancho_m / 2, cy - alto_m / 2),
        (cx + ancho_m / 2, cy - alto_m / 2),
        (cx + ancho_m / 2, cy + alto_m / 2),
        (cx - ancho_m / 2, cy + alto_m / 2),
    ]
    anillo = [[lon + x / mlon, lat + y / mlat] for x, y in esquinas]
    anillo.append(anillo[0])
    return {
        "type": "FeatureCollection",
        "features": [{
            "type": "Feature",
            "properties": {"id": "prueba-nogo", "type": "no_go", "enabled": True},
            "geometry": {"type": "Polygon", "coordinates": [anillo]},
        }],
    }


async def pedir(ws, op, extra, timeout=90.0):
    req_id = f"prueba-{op}"
    await ws.send(json.dumps({"op": op, "client_req_id": req_id, **extra}))
    fin = asyncio.get_event_loop().time() + timeout
    while asyncio.get_event_loop().time() < fin:
        crudo = await asyncio.wait_for(ws.recv(), timeout=timeout)
        msg = json.loads(crudo)
        if msg.get("op") == "ack" and (
            msg.get("client_req_id") == req_id or msg.get("request") == op
        ):
            return msg
    raise TimeoutError(f"{op} no respondio")


async def main():
    async with websockets.connect(URL, max_size=None) as ws:
        # Pose del vehiculo, que es donde el cockpit siembra el lote.
        pose = None
        fin = asyncio.get_event_loop().time() + 20.0
        while pose is None and asyncio.get_event_loop().time() < fin:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=20.0))
            robot = msg.get("robot_pose") or (msg.get("state") or {}).get("robot_pose")
            if isinstance(robot, dict) and robot.get("lat") is not None:
                pose = robot
        if pose is None:
            print("no llego la pose del vehiculo por el websocket")
            return 1
        lat, lon = float(pose["lat"]), float(pose["lon"])
        yaw = float(pose.get("heading_deg") or pose.get("yaw_deg") or 0.0)
        print(f"vehiculo en {lat:.6f},{lon:.6f} rumbo {yaw:.1f}°")

        if CON_ZONA:
            res = await pedir(ws, "set_zones_geojson", {"geojson": zona_geojson(lat, lon)})
            print(f"zona no-go cargada: ok={res.get('ok')} error={res.get('error')}")
            # El route_executor refresca las zonas por timer, no en el pedido.
            # En esta sim el zones_manager tarda ~8 s en set_geojson (escribe la
            # mascara keepout y el load_map del map_server falla), y mientras
            # tanto no contesta get_state: hay que esperar a que el cache del
            # route_executor se ponga al dia o se planifica sin zonas.
            await asyncio.sleep(20.0)

        comun = {
            "coverage_polygon": octogono(lat, lon, LADO_M, yaw),
            "coverage_exclusions": [],
            "field_length_m": LADO_M,
            "field_width_m": LADO_M,
            "cutter_width_m": 2.0,
            "overlap_ratio": 0.15,
            "min_turning_radius_m": RMIN,
            "waypoint_spacing_m": 2.0,
            "side": "left",
        }

        prev = await pedir(ws, "preview_coverage", comun)
        if not prev.get("ok"):
            print(f"PREVIEW FALLO: {prev.get('error')}")
            return 1
        plan = prev.get("coverage_plan") or {}
        metricas = plan.get("metrics") or {}
        print(
            f"preview OK  pasadas={metricas.get('row_count')} "
            f"metas={len(plan.get('key_waypoints') or [])} "
            f"muestreo={len(plan.get('sampled_waypoints') or [])} "
            f"modo={plan.get('field_mode')} auditado={plan.get('topology_audited')} "
            f"zonas={plan.get('nogo_polygon_count')} nota='{plan.get('nogo_note', '')}'"
        )

        ini = await pedir(ws, "start_coverage", comun)
        if not ini.get("ok") or not ini.get("route_started"):
            print(f"INICIAR FALLO: {ini.get('error')} estado={ini.get('route_submission_state')}")
            return 1
        print(
            f"INICIO OK  metas={ini.get('input_count') or ini.get('waypoint_count')} "
            f"estado={ini.get('route_submission_state')}"
        )

        if CON_ZONA and int(plan.get("nogo_polygon_count") or 0) == 0:
            print(
                "MAL: el backend informa 0 zonas aplicadas. El cockpit va a "
                "recortar por su cuenta, ver la diferencia y bloquear el inicio."
            )
            return 1

        # Se cancela: esto es una prueba, no una salida a cortar.
        fin_ruta = await pedir(ws, "cancel_route", {})
        print(f"cancelado: ok={fin_ruta.get('ok')}")
        if CON_ZONA:
            await pedir(ws, "set_zones_geojson", {
                "geojson": {"type": "FeatureCollection", "features": []}
            })
            print("zona de prueba borrada")
        return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
