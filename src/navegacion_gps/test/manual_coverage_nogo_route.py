"""Ejecuta una cobertura no-go completa sobre la simulacion global.

Este probe usa el mismo WebSocket que el Cockpit para PREVIEW/START y los
servicios ROS solamente para preparar/restaurar las zonas y observar la mision.
Los entrypoints habituales son:

    ./tools/test_coverage_nogo_inside.sh
    ./tools/test_coverage_nogo_boundary.sh
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
from pathlib import Path
import signal
import sys
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from interfaces.msg import CmdVelFinal, DriveTelemetry
from interfaces.srv import (
    CancelRouteMission,
    GetRouteMissionState,
    GetZonesState,
    SetZonesGeoJson,
)

import websockets


Point = Tuple[float, float]
LatLon = Tuple[float, float]
METERS_PER_DEG_LAT = 111_320.0

# Umbral de marcha atras, en m/s. Por debajo de esto un comando o una medicion
# ya no es ruido de redondeo: es marcha atras.
REVERSE_THRESHOLD_MPS = -0.02

# Retroceso acumulado que se tolera en la ESTIMACION de velocidad global.
#
# Medido en esta simulacion: con el vehiculo sin un solo comando negativo
# (/cmd_vel_final minimo 0.000 m/s) y sin un solo reporte de reversa del drive
# (drive_telemetry.reverse_requested siempre false), el twist de
# /odometry/global igual baja hasta -0.074 m/s en rachas de decimas de segundo.
# Es ruido del EKF con GPS: alrededor del reposo y en los giros lentos, la
# componente longitudinal estimada cambia de signo aunque el vehiculo no se
# mueva para atras.
#
# Aplicar el umbral instantaneo a esa senal marcaria reversa en cada frenada.
# Lo que si distingue ruido de maniobra es la DISTANCIA: una racha de ruido de
# 0.3 s a 0.05 m/s son 15 mm, mientras que la marcha atras mas corta que podria
# generar la cobertura —la cabecera de tres puntos— son 2R menos la separacion
# entre pasadas, o sea metros. Con 0.25 m no entra ninguna maniobra real y no
# se cuela ninguna racha de ruido.
#
# Las otras dos fuentes NO llevan esta tolerancia: un comando negativo o un
# reverse_requested del drive son inequivocos y fallan al primero.
REVERSE_DISTANCE_TOLERANCE_M = 0.25


def _yaw_deg(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.degrees(
        math.atan2(
            2.0 * ((q.w * q.z) + (q.x * q.y)),
            1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))),
        )
    )


def _point_segment_distance(point: Point, start: Point, end: Point) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx = bx - ax
    dy = by - ay
    squared = (dx * dx) + (dy * dy)
    if squared <= 1.0e-12:
        return math.hypot(px - ax, py - ay)
    fraction = max(
        0.0,
        min(1.0, (((px - ax) * dx) + ((py - ay) * dy)) / squared),
    )
    nearest = (ax + (fraction * dx), ay + (fraction * dy))
    return math.hypot(px - nearest[0], py - nearest[1])


def _polyline_distance(point: Point, polyline: Sequence[Point]) -> float:
    if not polyline:
        return math.inf
    if len(polyline) == 1:
        return math.hypot(point[0] - polyline[0][0], point[1] - polyline[0][1])
    return min(
        _point_segment_distance(point, polyline[index], polyline[index + 1])
        for index in range(len(polyline) - 1)
    )


class ScenarioNode(Node):
    """Cliente de preparacion, seguimiento y restauracion del escenario."""

    def __init__(self) -> None:
        super().__init__("coverage_nogo_full_route_probe")
        self.last_fix: Optional[LatLon] = None
        self.last_yaw_deg: Optional[float] = None
        self.record_track = False
        self.track: List[LatLon] = []
        self._last_track_at = 0.0
        # Vigilancia de marcha atras. Se miran las tres fuentes a la vez porque
        # cada una tapa un agujero de la otra: /cmd_vel_final es lo que se le
        # ORDENA al vehiculo, drive_telemetry es lo que el controlador dice que
        # esta HACIENDO, y la odometria es lo que efectivamente se MOVIO. Una
        # reversa que no aparezca en ninguna de las tres no existe.
        self.min_cmd_vel_mps: Optional[float] = None
        self.min_telemetry_mps: Optional[float] = None
        self.min_odom_mps: Optional[float] = None
        # Violaciones duras: comando negativo o reversa reportada por el drive.
        self.reverse_samples: List[Dict[str, Any]] = []
        self.telemetry_reverse_requested = 0
        # Rachas de estimacion global por debajo del umbral, con la distancia
        # que representan. Ver REVERSE_DISTANCE_TOLERANCE_M.
        self.odom_backward_runs: List[Dict[str, float]] = []
        self._odom_run: Optional[Dict[str, float]] = None
        self._odom_sample_at: Optional[float] = None
        self.create_subscription(NavSatFix, "/gps/fix", self._on_fix, 20)
        self.create_subscription(Odometry, "/odometry/global", self._on_odom, 20)
        self.create_subscription(
            CmdVelFinal, "/cmd_vel_final", self._on_cmd_vel_final, 20
        )
        # Nav2 publica /cmd_vel crudo antes del arbitraje. Se mira tambien: una
        # reversa ahi es un plan con reversa aunque el arbitro despues la corte.
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 20)
        self.create_subscription(
            DriveTelemetry,
            "/controller/drive_telemetry",
            self._on_drive_telemetry,
            20,
        )
        self.zones_get = self.create_client(GetZonesState, "/zones_manager/get_state")
        self.zones_set = self.create_client(SetZonesGeoJson, "/zones_manager/set_geojson")
        self.route_get = self.create_client(
            GetRouteMissionState,
            "/route_executor/get_state",
        )
        self.route_cancel = self.create_client(
            CancelRouteMission,
            "/route_executor/cancel_route",
        )

    def _on_fix(self, msg: NavSatFix) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return
        self.last_fix = (lat, lon)
        if not self.record_track:
            return
        now = time.monotonic()
        if now - self._last_track_at < 0.1:
            return
        self._last_track_at = now
        self.track.append((lat, lon))

    def _on_odom(self, msg: Odometry) -> None:
        yaw = _yaw_deg(msg)
        if math.isfinite(yaw):
            self.last_yaw_deg = yaw
        if not self.record_track:
            return
        # Velocidad longitudinal en el marco del cuerpo. La twist de
        # /odometry/global viene en child_frame_id = base_footprint, asi que
        # linear.x ya es "hacia adelante" y su signo es el que importa.
        speed = float(msg.twist.twist.linear.x)
        if math.isfinite(speed):
            self._register_speed("odometry/global", speed)
            self._accumulate_backward(speed)

    def _accumulate_backward(self, speed_mps: float) -> None:
        """Integrar el retroceso de la estimacion global, racha por racha."""
        ahora = time.monotonic()
        previo = self._odom_sample_at
        self._odom_sample_at = ahora
        if speed_mps >= REVERSE_THRESHOLD_MPS:
            if self._odom_run is not None:
                self.odom_backward_runs.append(self._odom_run)
                self._odom_run = None
            return
        # Paso acotado para que una pausa del proceso —esperar un servicio,
        # por ejemplo— no se convierta en metros imaginarios.
        paso_s = 0.0 if previo is None else min(0.5, max(0.0, ahora - previo))
        if self._odom_run is None:
            self._odom_run = {
                "distance_m": 0.0,
                "min_mps": float(speed_mps),
                "sample_count": 0.0,
            }
        self._odom_run["distance_m"] += abs(speed_mps) * paso_s
        self._odom_run["min_mps"] = min(
            self._odom_run["min_mps"], float(speed_mps)
        )
        self._odom_run["sample_count"] += 1.0

    def backward_runs(self) -> List[Dict[str, float]]:
        """Rachas cerradas mas la que este abierta en este momento."""
        rachas = list(self.odom_backward_runs)
        if self._odom_run is not None:
            rachas.append(dict(self._odom_run))
        return rachas

    def worst_backward_run(self) -> Optional[Dict[str, float]]:
        rachas = self.backward_runs()
        if not rachas:
            return None
        return max(rachas, key=lambda racha: racha["distance_m"])

    def _on_cmd_vel_final(self, msg: CmdVelFinal) -> None:
        if not self.record_track:
            return
        speed = float(msg.twist.linear.x)
        if math.isfinite(speed):
            self._register_speed("cmd_vel_final", speed)

    def _on_cmd_vel(self, msg: Twist) -> None:
        if not self.record_track:
            return
        speed = float(msg.linear.x)
        if math.isfinite(speed):
            self._register_speed("cmd_vel", speed)

    def _on_drive_telemetry(self, msg: DriveTelemetry) -> None:
        if not self.record_track:
            return
        if bool(msg.reverse_requested):
            self.telemetry_reverse_requested += 1
            self.reverse_samples.append(
                {
                    "source": "drive_telemetry.reverse_requested",
                    "value_mps": float(msg.speed_mps_measured),
                }
            )
        if not bool(msg.speed_valid):
            return
        speed = float(msg.speed_mps_measured)
        if math.isfinite(speed):
            self._register_speed("drive_telemetry", speed)

    def _register_speed(self, source: str, speed_mps: float) -> None:
        if source in ("cmd_vel_final", "cmd_vel"):
            previo = self.min_cmd_vel_mps
            self.min_cmd_vel_mps = (
                speed_mps if previo is None else min(previo, speed_mps)
            )
        elif source == "drive_telemetry":
            previo = self.min_telemetry_mps
            self.min_telemetry_mps = (
                speed_mps if previo is None else min(previo, speed_mps)
            )
        else:
            previo = self.min_odom_mps
            self.min_odom_mps = (
                speed_mps if previo is None else min(previo, speed_mps)
            )
            # La estimacion global no falla al primer sample: se integra en
            # _accumulate_backward y se juzga por distancia recorrida.
            return
        if speed_mps < REVERSE_THRESHOLD_MPS:
            self.reverse_samples.append(
                {"source": source, "value_mps": float(speed_mps)}
            )

    def speed_report(self) -> Dict[str, Any]:
        """Resumen de la vigilancia de reversa para el reporte JSON."""
        candidatos = [
            valor
            for valor in (
                self.min_cmd_vel_mps,
                self.min_telemetry_mps,
                self.min_odom_mps,
            )
            if valor is not None
        ]
        peor = self.worst_backward_run()
        return {
            "min_cmd_vel_mps": self.min_cmd_vel_mps,
            "min_drive_telemetry_mps": self.min_telemetry_mps,
            "min_odometry_mps": self.min_odom_mps,
            "min_linear_velocity_mps": min(candidatos) if candidatos else None,
            "reverse_threshold_mps": REVERSE_THRESHOLD_MPS,
            "reverse_sample_count": len(self.reverse_samples),
            "reverse_samples": self.reverse_samples[:20],
            "telemetry_reverse_requested_count": self.telemetry_reverse_requested,
            # Retroceso integrado de la ESTIMACION global. Ver
            # REVERSE_DISTANCE_TOLERANCE_M: el ruido del EKF baja del umbral
            # instantaneo sin que el vehiculo se mueva para atras.
            "odometry_backward_distance_tolerance_m": (
                REVERSE_DISTANCE_TOLERANCE_M
            ),
            "odometry_backward_run_count": len(self.backward_runs()),
            "odometry_backward_worst_distance_m": (
                None if peor is None else round(peor["distance_m"], 4)
            ),
            "odometry_backward_worst_min_mps": (
                None if peor is None else round(peor["min_mps"], 4)
            ),
        }

    def wait_services(self, timeout_s: float) -> None:
        for client, name in (
            (self.zones_get, "/zones_manager/get_state"),
            (self.zones_set, "/zones_manager/set_geojson"),
            (self.route_get, "/route_executor/get_state"),
            (self.route_cancel, "/route_executor/cancel_route"),
        ):
            if not client.wait_for_service(timeout_sec=timeout_s):
                raise RuntimeError(f"servicio no disponible: {name}")

    def call(self, client: Any, request: Any, timeout_s: float = 30.0) -> Any:
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError("servicio ROS sin respuesta")
        result = future.result()
        if result is None:
            raise RuntimeError("servicio ROS devolvio una respuesta vacia")
        return result

    def reference(self, timeout_s: float) -> Tuple[float, float, float]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_fix is not None and self.last_yaw_deg is not None:
                return self.last_fix[0], self.last_fix[1], self.last_yaw_deg
        raise TimeoutError("no llegaron /gps/fix y /odometry/global")


class Geometry:
    """Conversion local cuerpo <-> lat/lon para un escenario."""

    def __init__(self, lat: float, lon: float, yaw_deg: float) -> None:
        self.lat = float(lat)
        self.lon = float(lon)
        self.yaw_rad = math.radians(float(yaw_deg))
        self.meters_per_deg_lon = METERS_PER_DEG_LAT * max(
            1.0e-6,
            abs(math.cos(math.radians(lat))),
        )

    def to_ll(self, forward_m: float, left_m: float) -> Dict[str, float]:
        east_m = (
            (forward_m * math.cos(self.yaw_rad))
            - (left_m * math.sin(self.yaw_rad))
        )
        north_m = (
            (forward_m * math.sin(self.yaw_rad))
            + (left_m * math.cos(self.yaw_rad))
        )
        return {
            "lat": self.lat + (north_m / METERS_PER_DEG_LAT),
            "lon": self.lon + (east_m / self.meters_per_deg_lon),
        }

    def to_body(self, lat: float, lon: float) -> Point:
        east_m = (float(lon) - self.lon) * self.meters_per_deg_lon
        north_m = (float(lat) - self.lat) * METERS_PER_DEG_LAT
        return (
            (east_m * math.cos(self.yaw_rad))
            + (north_m * math.sin(self.yaw_rad)),
            (-east_m * math.sin(self.yaw_rad))
            + (north_m * math.cos(self.yaw_rad)),
        )


async def _ws_request(
    websocket: Any,
    operation: str,
    payload: Dict[str, Any],
    request_id: str,
    timeout_s: float = 180.0,
) -> Dict[str, Any]:
    await websocket.send(
        json.dumps(
            {
                "op": operation,
                "client_req_id": request_id,
                **payload,
            }
        )
    )
    deadline = asyncio.get_running_loop().time() + timeout_s
    while asyncio.get_running_loop().time() < deadline:
        remaining = max(0.1, deadline - asyncio.get_running_loop().time())
        raw = await asyncio.wait_for(websocket.recv(), timeout=remaining)
        message = json.loads(raw)
        if message.get("op") != "ack":
            continue
        if message.get("client_req_id") == request_id:
            return message
    raise TimeoutError(f"{operation} no respondio por WebSocket")


def _circle_geojson(
    geometry: Geometry,
    center: Point,
    radius_m: float,
    scenario: str,
) -> str:
    vertices = []
    for index in range(32):
        angle = (2.0 * math.pi * index) / 32.0
        point = geometry.to_ll(
            center[0] + (radius_m * math.cos(angle)),
            center[1] + (radius_m * math.sin(angle)),
        )
        vertices.append([point["lon"], point["lat"]])
    vertices.append(vertices[0])
    return json.dumps(
        {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {
                        "id": f"coverage-nogo-{scenario}",
                        "name": f"Prueba no-go {scenario}",
                        "type": "no_go",
                        "enabled": True,
                    },
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [vertices],
                    },
                }
            ],
        },
        separators=(",", ":"),
    )


def _coverage_payload(
    geometry: Geometry,
    args: argparse.Namespace,
) -> Dict[str, Any]:
    polygon = [
        geometry.to_ll(0.0, 0.0),
        geometry.to_ll(args.field_length_m, 0.0),
        geometry.to_ll(args.field_length_m, args.field_width_m),
        geometry.to_ll(0.0, args.field_width_m),
    ]
    return {
        "reference": {
            "lat": geometry.lat,
            "lon": geometry.lon,
            "yaw_deg": math.degrees(geometry.yaw_rad),
        },
        "coverage_polygon": {"vertices": polygon},
        "coverage_exclusions": [],
        "field_length_m": args.field_length_m,
        "field_width_m": args.field_width_m,
        "cutter_width_m": args.cutter_width_m,
        "overlap_ratio": args.overlap_ratio,
        "min_turning_radius_m": args.turning_radius_m,
        "waypoint_spacing_m": args.waypoint_spacing_m,
        "side": "left",
    }


def _body_points(
    geometry: Geometry,
    waypoints: Sequence[Dict[str, Any]],
) -> List[Point]:
    return [
        geometry.to_body(float(waypoint["lat"]), float(waypoint["lon"]))
        for waypoint in waypoints
    ]


def _validate_plan(
    plan: Dict[str, Any],
    geometry: Geometry,
    center: Point,
    args: argparse.Namespace,
) -> Dict[str, Any]:
    if plan.get("field_mode") != "polygon":
        raise RuntimeError(f"el backend no uso el poligono: {plan.get('field_mode')!r}")
    if int(plan.get("nogo_polygon_count") or 0) != 1:
        raise RuntimeError(
            "el backend no aplico exactamente una zona no-go "
            f"(count={plan.get('nogo_polygon_count')}, note={plan.get('nogo_note')!r})"
        )
    if args.scenario == "boundary" and int(plan.get("nogo_detour_count") or 0) <= 0:
        raise RuntimeError("el plan no genero ningun rodeo no-go")

    sampled = list(plan.get("sampled_waypoints") or [])
    detours = [wp for wp in sampled if wp.get("phase") == "nogo_detour"]
    no_go_transitions = [
        wp
        for wp in sampled
        if wp.get("phase") in {
            "nogo_detour",
            "nogo_lane_change",
            "nogo_transition",
        }
    ]
    if args.scenario == "boundary" and not no_go_transitions:
        raise RuntimeError("el preview no contiene ninguna transicion no-go")
    if args.scenario == "inside":
        lane_changes = [
            wp
            for wp in no_go_transitions
            if wp.get("phase") == "nogo_lane_change"
        ]
        if detours or any(
            wp.get("phase") == "nogo_transition" for wp in no_go_transitions
        ):
            transition_count = sum(
                1
                for waypoint in no_go_transitions
                if waypoint.get("phase") == "nogo_transition"
            )
            raise RuntimeError(
                "la zona interna genero una omega o un rodeo dentro del lote "
                f"(rodeos={len(detours)}, transiciones={transition_count}, "
                f"cambios_fila={len(lane_changes)})"
            )
        if not lane_changes and not args.allow_skipped_rows:
            raise RuntimeError("la zona interna no genero ningun cambio de fila")
        if not lane_changes:
            # Estrategia "headland": la fila bloqueada no se recorre y el resto
            # se enlaza por cabecera. No hay S que auditar, pero si tiene que
            # haberse salteado algo: un plan identico al de un lote sin zona
            # significaria que la exclusion no se aplico.
            if int(plan.get("nogo_dropped_count") or 0) <= 0:
                raise RuntimeError(
                    "la zona interna no genero cambio de fila NI salteo ninguna "
                    "pasada: la exclusion no se estaria aplicando"
                )
        if int(plan.get("nogo_dropped_count") or 0) <= 0:
            raise RuntimeError(
                "la zona interna no anticipo ningun punto de la fila cortada"
            )
    sampled_body = _body_points(geometry, sampled)
    detour_body = _body_points(geometry, detours)

    route = list(((plan.get("route_request") or {}).get("waypoints")) or [])
    no_go_guides = [
        wp
        for wp in route
        if (
            not bool(wp.get("key", True))
            and wp.get("phase")
            in {"nogo_detour", "nogo_lane_change", "nogo_transition"}
        )
    ]
    if args.scenario == "boundary" and not no_go_guides:
        raise RuntimeError("la ruta ejecutable perdio las guias del rodeo no-go")
    # Tres guias por cambio de fila (inicio, apex y fin de la S) por seis
    # cambios. El apex cuenta como guia desde que dejo de cortar el bloque: son
    # los mismos waypoints de siempre, no hay ninguno nuevo. Lo que de verdad
    # vigila que no vuelva a circular de mas son `rodeos` y `max_row_jump`.
    if args.scenario == "inside" and len(no_go_guides) > 18:
        raise RuntimeError(
            "la zona interna genero demasiadas guias y volveria a circular de "
            f"mas: {len(no_go_guides)} (maximo 18)"
        )

    # route_action_jsons: la politica forward-only prohibe cualquier accion de
    # marcha atras. Se revisa el plan ANTES de arrancar, para no descubrirlo
    # con el vehiculo ya en movimiento.
    backup_actions = [
        indice
        for indice, waypoint in enumerate(route)
        if "coverage_backup" in str(waypoint.get("action_json") or "")
    ]
    if backup_actions:
        raise RuntimeError(
            "la ruta trae acciones coverage_backup en los waypoints "
            f"{backup_actions}: el perfil de simulacion no puede retroceder"
        )
    route_body = _body_points(geometry, route)

    if args.scenario == "inside":
        # Regla agricola estricta del escenario: dentro del rectangulo solo se
        # admiten tramos sobre una fila (uno de los ejes queda constante) o la
        # S marcada explicitamente como cambio de fila. Los giros forward-only
        # tienen que quedar del lado exterior de la cabecera.
        illegal_inside_segments = []
        for index, (start, end) in enumerate(zip(route_body, route_body[1:])):
            phases = {
                str(route[index].get("phase") or ""),
                str(route[index + 1].get("phase") or ""),
            }
            if "nogo_lane_change" in phases:
                continue
            crosses_both_axes = (
                abs(end[0] - start[0]) > 0.05
                and abs(end[1] - start[1]) > 0.05
            )
            if not crosses_both_axes:
                continue
            interior_sample = any(
                0.05 < start[0] + (fraction * (end[0] - start[0])) < args.field_length_m - 0.05
                and 0.05 < start[1] + (fraction * (end[1] - start[1])) < args.field_width_m - 0.05
                for fraction in (0.25, 0.5, 0.75)
            )
            if interior_sample:
                illegal_inside_segments.append(index)
        if illegal_inside_segments:
            raise RuntimeError(
                "la ruta gira o cruza cultivo fuera de una fila: segmentos "
                f"{illegal_inside_segments}"
            )

    route_clearance_m = _polyline_distance(center, route_body)
    if route_clearance_m + 0.05 < args.zone_radius_m:
        nearest_index = min(
            range(max(1, len(route_body) - 1)),
            key=lambda index: _point_segment_distance(
                center,
                route_body[index],
                route_body[min(index + 1, len(route_body) - 1)],
            ),
        )
        claves = [
            route[index].get("key")
            for index in range(
                nearest_index, min(nearest_index + 2, len(route))
            )
        ]
        raise RuntimeError(
            "la ruta ejecutable corta la zona no-go: "
            f"distancia={route_clearance_m:.2f}m radio={args.zone_radius_m:.2f}m; "
            f"segmento={nearest_index}->{nearest_index + 1} "
            f"puntos={route_body[nearest_index:min(nearest_index + 2, len(route_body))]} "
            f"keys={claves}"
        )

    row_order: List[int] = []
    for waypoint in sampled:
        row = int(waypoint.get("row_index", -1))
        if row < 0:
            continue
        if not row_order or row_order[-1] != row:
            row_order.append(row)
    max_row_jump = max(
        (abs(row_order[index] - row_order[index - 1]) for index in range(1, len(row_order))),
        default=0,
    )
    if max_row_jump > 1:
        raise RuntimeError(
            f"el plan salta filas: orden={row_order}, salto_maximo={max_row_jump}"
        )

    outside_detours = [
        point
        for point in detour_body
        if (
            point[0] < -0.05
            or point[0] > args.field_length_m + 0.05
            or point[1] < -0.05
            or point[1] > args.field_width_m + 0.05
        )
    ]
    if args.scenario == "inside" and outside_detours:
        raise RuntimeError("la zona interna genero un rodeo fuera del lote")
    if args.scenario == "boundary" and not outside_detours:
        raise RuntimeError("la zona de borde no hizo salir el rodeo del lote")

    outside_route = [
        point
        for point in route_body
        if (
            point[0] < -0.05
            or point[0] > args.field_length_m + 0.05
            or point[1] < -0.05
            or point[1] > args.field_width_m + 0.05
        )
    ]

    return {
        "row_order": row_order,
        "row_count": len(set(row_order)),
        "max_row_jump": max_row_jump,
        "sampled_count": len(sampled_body),
        "detour_count": len(detour_body),
        "nogo_transition_count": len(no_go_transitions),
        "outside_detour_count": len(outside_detours),
        "outside_route_count": len(outside_route),
        "route_count": len(route_body),
        "route_guide_count": len(no_go_guides),
        "route_clearance_m": route_clearance_m,
        "route_backup_action_count": len(backup_actions),
        "backend_detour_count": int(plan.get("nogo_detour_count") or 0),
        "backend_dropped_count": int(plan.get("nogo_dropped_count") or 0),
    }


async def _preview_and_start(
    args: argparse.Namespace,
    payload: Dict[str, Any],
    geometry: Geometry,
    center: Point,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    async with websockets.connect(args.websocket_url, max_size=None) as websocket:
        preview: Optional[Dict[str, Any]] = None
        preview_error = ""
        deadline = time.monotonic() + args.zone_cache_timeout_s
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            response = await _ws_request(
                websocket,
                "preview_coverage",
                payload,
                f"nogo-{args.scenario}-preview-{attempt}",
            )
            if response.get("ok"):
                candidate = response.get("coverage_plan") or {}
                if int(candidate.get("nogo_polygon_count") or 0) == 1:
                    preview = candidate
                    break
                preview_error = str(candidate.get("nogo_note") or "cache sin zona")
            else:
                preview_error = str(response.get("error") or "preview rechazado")
                if "cambio de fila interno invade la zona no-go" in preview_error:
                    break
            await asyncio.sleep(1.0)
        if preview is None:
            raise RuntimeError(
                "route_executor no refresco la zona dentro del plazo: "
                f"{preview_error}"
            )

        if args.report:
            Path(f"{args.report}.preview.json").write_text(
                json.dumps(preview, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        summary = _validate_plan(preview, geometry, center, args)
        if args.preview_only:
            return preview, {"preview_only": True, "plan_summary": summary}

        start = await _ws_request(
            websocket,
            "start_coverage",
            payload,
            f"nogo-{args.scenario}-start",
        )
        if not start.get("ok") or not start.get("route_started"):
            raise RuntimeError(
                "start_coverage rechazado: "
                f"{start.get('error') or start.get('route_submission_state')}"
            )
        return preview, {**start, "plan_summary": summary}


def _raise_on_reverse(node: ScenarioNode, args: argparse.Namespace) -> None:
    """Cortar si el vehiculo retrocedio. Tres fuentes, dos criterios.

    Un comando negativo en ``/cmd_vel``/``/cmd_vel_final`` o un
    ``reverse_requested`` del drive son inequivocos: falla el primero que
    aparezca. La velocidad estimada de ``/odometry/global`` se juzga por la
    distancia acumulada de cada racha, porque su ruido baja del umbral
    instantaneo con el vehiculo quieto (ver ``REVERSE_DISTANCE_TOLERANCE_M``).
    """
    if node.reverse_samples:
        muestra = node.reverse_samples[0]
        raise RuntimeError(
            "se observo marcha atras: "
            f"{muestra['source']}={muestra['value_mps']:.3f} m/s "
            f"(umbral {REVERSE_THRESHOLD_MPS:.3f} m/s)"
        )
    peor = node.worst_backward_run()
    tolerancia_m = float(args.reverse_distance_tolerance_m)
    if peor is not None and peor["distance_m"] > tolerancia_m:
        raise RuntimeError(
            "el vehiculo retrocedio segun /odometry/global: "
            f"{peor['distance_m']:.3f} m seguidos a {peor['min_mps']:.3f} m/s "
            f"como minimo, sobre la tolerancia de {tolerancia_m:.3f} m"
        )


def _monitor_route(
    node: ScenarioNode,
    geometry: Geometry,
    center: Point,
    args: argparse.Namespace,
) -> Dict[str, Any]:
    deadline = time.monotonic() + args.mission_timeout_s
    next_print = 0.0
    seen_active = False
    max_cross_track_m = 0.0
    mission_backup_actions: List[int] = []
    node.record_track = True
    while time.monotonic() < deadline:
        state = node.call(
            node.route_get,
            GetRouteMissionState.Request(),
            timeout_s=10.0,
        )
        if not bool(state.ok):
            raise RuntimeError(f"get_state fallo: {state.error}")
        active = bool(state.active)
        seen_active = seen_active or active
        status = str(state.status or "")
        max_cross_track_m = max(max_cross_track_m, float(state.cross_track_error_m))

        # La ruta expandida que el ejecutor tiene cargada AHORA. El preview ya
        # se valido, pero el ejecutor puede rearmar chunks: si apareciera una
        # marcha atras aca, hay que cortar antes de que se ejecute.
        mission_backup_actions = [
            indice
            for indice, entrada in enumerate(
                list(getattr(state, "mission_action_jsons", []) or [])
            )
            if "coverage_backup" in str(entrada or "")
        ]
        if mission_backup_actions:
            raise RuntimeError(
                "la mision cargada trae coverage_backup en los waypoints "
                f"{mission_backup_actions}"
            )
        if str(getattr(state, "action_type", "") or "") == "coverage_backup":
            raise RuntimeError(
                "el ejecutor arranco una accion coverage_backup en el waypoint "
                f"{int(state.action_waypoint_index)}"
            )
        _raise_on_reverse(node, args)

        now = time.monotonic()
        if now >= next_print:
            print(
                "    estado=%-28s waypoint=%3d/%3d progreso=%5.1f%% "
                "distancia=%6.2fm xte=%5.2fm"
                % (
                    status,
                    int(state.current_target_index),
                    int(state.expanded_waypoint_count),
                    100.0 * float(state.current_progress_ratio),
                    float(state.distance_to_target_m),
                    float(state.cross_track_error_m),
                ),
                flush=True,
            )
            next_print = now + args.print_period_s

        lowered = status.lower()
        if seen_active and not active:
            if "completed" in lowered:
                break
            raise RuntimeError(f"la mision termino sin completar: {status}")
        if "failed" in lowered or "cancelled" in lowered:
            raise RuntimeError(f"la mision fallo: {status}")

        end_wait = min(deadline, time.monotonic() + args.poll_period_s)
        while time.monotonic() < end_wait:
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        raise TimeoutError(
            f"la mision no termino en {args.mission_timeout_s:.0f}s"
        )
    node.record_track = False
    _raise_on_reverse(node, args)
    velocidades = node.speed_report()
    if velocidades["min_linear_velocity_mps"] is None:
        raise RuntimeError(
            "no llego ninguna velocidad por /cmd_vel_final, "
            "/controller/drive_telemetry ni /odometry/global: la vigilancia de "
            "reversa no midio nada y el resultado no seria concluyente"
        )

    track_body = [geometry.to_body(lat, lon) for lat, lon in node.track]
    actual_clearance_m = _polyline_distance(center, track_body)
    if actual_clearance_m + args.actual_clearance_tolerance_m < args.zone_radius_m:
        raise RuntimeError(
            "el vehiculo entro a la zona no-go: "
            f"distancia={actual_clearance_m:.2f}m radio={args.zone_radius_m:.2f}m"
        )

    near_zone = [
        point
        for point in track_body
        if abs(point[0] - center[0]) <= (
            args.zone_radius_m + args.turning_radius_m + 2.0
        )
    ]
    outside_near_zone = [point for point in near_zone if point[1] < -0.10]
    if args.scenario == "boundary" and not outside_near_zone:
        raise RuntimeError(
            "el plan salia del lote, pero el vehiculo no recorrio el tramo exterior"
        )

    outside_field = [
        point
        for point in track_body
        if (
            point[0] < -0.05
            or point[0] > args.field_length_m + 0.05
            or point[1] < -0.05
            or point[1] > args.field_width_m + 0.05
        )
    ]

    return {
        "track_sample_count": len(track_body),
        "actual_clearance_m": actual_clearance_m,
        "outside_near_zone_count": len(outside_near_zone),
        "outside_field_count": len(outside_field),
        "max_cross_track_m": max_cross_track_m,
        "mission_backup_action_count": len(mission_backup_actions),
        "speeds": velocidades,
        "status": "route completed",
    }


def _set_zone_state(node: ScenarioNode, geojson_text: str) -> Any:
    response = node.call(
        node.zones_set,
        SetZonesGeoJson.Request(geojson=geojson_text),
        timeout_s=30.0,
    )
    state = node.call(node.zones_get, GetZonesState.Request(), timeout_s=15.0)
    if not bool(response.ok):
        raise RuntimeError(
            "zones_manager rechazo la zona: "
            f"{response.error or 'sin detalle'} "
            f"(map_reloaded={bool(response.map_reloaded)})"
        )
    if not bool(response.map_reloaded):
        raise RuntimeError(
            "zones_manager escribio la zona pero Nav2 no recargo la mascara "
            "keepout (map_reloaded=False)"
        )
    if not bool(state.ok):
        raise RuntimeError(f"no se pudo verificar la zona: {state.error}")
    if not bool(state.mask_ready):
        raise RuntimeError(
            "zones_manager no dejo la mascara lista: "
            f"mask_source={state.mask_source!r}"
        )
    return response, state


def _cancel_if_active(node: ScenarioNode) -> None:
    try:
        state = node.call(node.route_get, GetRouteMissionState.Request(), timeout_s=5.0)
        if bool(state.ok) and bool(state.active):
            node.call(
                node.route_cancel,
                CancelRouteMission.Request(),
                timeout_s=10.0,
            )
    except Exception as exc:
        print(f"AVISO: no se pudo cancelar la ruta activa: {exc}", file=sys.stderr)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=("inside", "boundary"), required=True)
    parser.add_argument("--websocket-url", default="ws://127.0.0.1:8766")
    parser.add_argument("--field-length-m", type=float, default=24.0)
    parser.add_argument("--field-width-m", type=float, default=22.0)
    parser.add_argument("--cutter-width-m", type=float, default=4.0)
    parser.add_argument("--overlap-ratio", type=float, default=0.0)
    parser.add_argument("--turning-radius-m", type=float, default=4.0)
    parser.add_argument(
        "--waypoint-spacing-m",
        type=float,
        default=0.0,
        help="Espaciado del preview; 0 calcula uno segun el tamano del lote.",
    )
    parser.add_argument(
        "--allow-skipped-rows",
        action="store_true",
        help="acepta la estrategia headland, donde la fila bloqueada no se "
             "recorre en vez de resolverse con una S adentro del cultivo",
    )
    parser.add_argument("--zone-radius-m", type=float, default=1.5)
    parser.add_argument("--zone-forward-m", type=float, default=float("nan"))
    parser.add_argument("--zone-left-m", type=float, default=float("nan"))
    parser.add_argument("--mission-timeout-s", type=float, default=900.0)
    parser.add_argument("--zone-cache-timeout-s", type=float, default=45.0)
    parser.add_argument("--service-timeout-s", type=float, default=30.0)
    parser.add_argument("--poll-period-s", type=float, default=1.0)
    parser.add_argument("--print-period-s", type=float, default=5.0)
    parser.add_argument("--actual-clearance-tolerance-m", type=float, default=0.25)
    parser.add_argument(
        "--reverse-distance-tolerance-m",
        type=float,
        default=REVERSE_DISTANCE_TOLERANCE_M,
    )
    parser.add_argument("--preview-only", action="store_true")
    parser.add_argument("--keep-test-zone", action="store_true")
    parser.add_argument("--report", default="")
    return parser.parse_args()


def main() -> int:
    args = _arguments()
    rclpy.init()
    node = ScenarioNode()
    original_geojson: Optional[str] = None
    route_started = False
    report: Dict[str, Any] = {"scenario": args.scenario, "ok": False}

    def interrupt(_signum: int, _frame: Any) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        print(f"==> Escenario no-go: {args.scenario}")
        node.wait_services(args.service_timeout_s)
        original = node.call(
            node.zones_get,
            GetZonesState.Request(),
            timeout_s=args.service_timeout_s,
        )
        if not bool(original.ok):
            raise RuntimeError(f"get_state de zonas fallo: {original.error}")
        original_geojson = str(original.geojson or "")

        lat, lon, yaw_deg = node.reference(args.service_timeout_s)
        geometry = Geometry(lat, lon, yaw_deg)
        zone_forward_m = (
            float(args.zone_forward_m)
            if math.isfinite(float(args.zone_forward_m))
            else 0.5 * args.field_length_m
        )
        center = (
            zone_forward_m,
            float(args.zone_left_m)
            if math.isfinite(float(args.zone_left_m))
            else (
                0.5 * args.field_width_m
                if args.scenario == "inside"
                else args.zone_radius_m
            ),
        )
        print(
            "    referencia lat=%.8f lon=%.8f yaw=%.1f°; centro zona=(%.1f, %.1f)m"
            % (lat, lon, yaw_deg, center[0], center[1])
        )

        test_geojson = _circle_geojson(
            geometry,
            center,
            args.zone_radius_m,
            args.scenario,
        )
        set_response, state = _set_zone_state(node, test_geojson)
        if f"coverage-nogo-{args.scenario}" not in str(state.geojson):
            raise RuntimeError("zones_manager no guardo la zona del escenario")
        print(
            "    zona cargada: polygons=%d map_reloaded=%s aviso=%r"
            % (
                int(set_response.polygon_count),
                bool(set_response.map_reloaded),
                str(set_response.error or ""),
            )
        )

        payload = _coverage_payload(geometry, args)
        preview, start = asyncio.run(
            _preview_and_start(args, payload, geometry, center)
        )
        plan_summary = dict(start["plan_summary"])
        print(
            "    plan OK: filas=%s salto_max=%d rodeos=%d guias_nogo=%d "
            "puntos_exteriores=%d clearance_ruta=%.2fm spacing=%.2fm "
            "coverage_backup=%d"
            % (
                (preview.get("metrics") or {}).get("row_count"),
                int(plan_summary["max_row_jump"]),
                int(plan_summary["backend_detour_count"]),
                int(plan_summary["route_guide_count"]),
                int(plan_summary["outside_route_count"]),
                float(plan_summary["route_clearance_m"]),
                float(
                    (preview.get("metrics") or {}).get(
                        "waypoint_spacing_m", args.waypoint_spacing_m
                    )
                ),
                int(plan_summary["route_backup_action_count"]),
            )
        )

        report["plan"] = plan_summary
        report["row_count"] = int((preview.get("metrics") or {}).get("row_count") or 0)
        report["max_row_jump"] = int(plan_summary["max_row_jump"])
        report["coverage_backup_action_count"] = int(
            plan_summary["route_backup_action_count"]
        )
        report["outside_field_waypoint_count"] = int(
            plan_summary["outside_route_count"]
        )
        report["min_distance_to_nogo_m"] = float(
            plan_summary["route_clearance_m"]
        )
        report["nogo_zone_radius_m"] = float(args.zone_radius_m)
        report["effective_waypoint_spacing_m"] = float(
            (preview.get("metrics") or {}).get(
                "waypoint_spacing_m", args.waypoint_spacing_m
            )
        )
        if args.preview_only:
            report["ok"] = True
            report["result"] = "preview validated"
            report["final_status"] = "preview validated"
            print("==> PREVIEW VALIDADO; no se movio el vehiculo")
            return 0

        route_started = True
        print(
            "==> Ruta iniciada: input=%s expanded=%s. Esperando recorrido completo..."
            % (
                start.get("input_waypoint_count"),
                start.get("expanded_waypoint_count"),
            )
        )
        execution = _monitor_route(node, geometry, center, args)
        route_started = False
        report["execution"] = execution
        report["ok"] = True
        report["result"] = "full route completed"
        report["final_status"] = str(execution["status"])
        report["min_distance_to_nogo_m"] = min(
            float(report["min_distance_to_nogo_m"]),
            float(execution["actual_clearance_m"]),
        )
        report["min_linear_velocity_mps"] = execution["speeds"][
            "min_linear_velocity_mps"
        ]
        report["outside_field_track_count"] = int(execution["outside_field_count"])
        report["coverage_backup_action_count"] = int(
            report["coverage_backup_action_count"]
        ) + int(execution["mission_backup_action_count"])
        print(
            "==> RECORRIDO COMPLETO OK: muestras=%d clearance_real=%.2fm "
            "salidas_cerca_zona=%d xte_max=%.2fm v_min=%.3fm/s "
            "coverage_backup=%d"
            % (
                int(execution["track_sample_count"]),
                float(execution["actual_clearance_m"]),
                int(execution["outside_near_zone_count"]),
                float(execution["max_cross_track_m"]),
                float(execution["speeds"]["min_linear_velocity_mps"]),
                int(report["coverage_backup_action_count"]),
            )
        )
        return 0
    except KeyboardInterrupt:
        report["error"] = "interrumpido por el operador"
        print("\nPrueba interrumpida.", file=sys.stderr)
        return 130
    except Exception as exc:
        report["error"] = str(exc)
        print(f"\nFALLO: {exc}", file=sys.stderr)
        return 1
    finally:
        if route_started:
            _cancel_if_active(node)
        if original_geojson is not None and not args.keep_test_zone:
            try:
                _set_zone_state(node, original_geojson)
                print("    zonas originales restauradas")
            except Exception as exc:
                print(
                    f"AVISO: no se pudieron restaurar las zonas: {exc}",
                    file=sys.stderr,
                )
        if not report.get("ok"):
            report["final_status"] = str(report.get("error") or "failed")
        # La vigilancia de reversa se vuelca siempre, tambien cuando la prueba
        # falla: la velocidad minima observada es justamente lo que se quiere
        # leer cuando algo salio mal.
        report.setdefault("speeds", node.speed_report())
        report.setdefault(
            "min_linear_velocity_mps",
            report["speeds"]["min_linear_velocity_mps"],
        )
        if args.report:
            report["report_path"] = str(args.report)
            try:
                report_path = Path(args.report)
                report_path.parent.mkdir(parents=True, exist_ok=True)
                report_path.write_text(
                    json.dumps(report, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                print(f"    reporte: {report_path}")
            except Exception as exc:
                print(f"AVISO: no se pudo escribir el reporte: {exc}", file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
