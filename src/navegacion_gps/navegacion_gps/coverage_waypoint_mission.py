"""CLI ROS para generar y ejecutar una cobertura tipo cortadora de cesped."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Optional

from interfaces.srv import SetRouteMissionLL
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage

from navegacion_gps.coverage_waypoint_core import build_lawnmower_waypoints
from navegacion_gps.heading_math import normalize_yaw_deg
from navegacion_gps.heading_math import yaw_deg_from_quaternion_xyzw


class CoverageWaypointMissionNode(Node):
    """Resolver la referencia actual y enviar una ruta de cobertura geografica."""

    def __init__(self) -> None:
        super().__init__("coverage_waypoint_mission")
        self.gps_fix: Optional[NavSatFix] = None
        self.odom_local: Optional[Odometry] = None
        self.latest_map_odom_tf: Optional[dict[str, float]] = None

        self.create_subscription(NavSatFix, "/gps/fix", self._on_gps_fix, 10)
        self.create_subscription(Odometry, "/odometry/local", self._on_odom_local, 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 100)
        self.route_client = self.create_client(
            SetRouteMissionLL,
            "/route_executor/set_route_ll",
        )

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        self.gps_fix = msg

    def _on_odom_local(self, msg: Odometry) -> None:
        self.odom_local = msg

    def _on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.header.frame_id != "map" or transform.child_frame_id != "odom":
                continue
            rotation = transform.transform.rotation
            self.latest_map_odom_tf = {
                "x": float(transform.transform.translation.x),
                "y": float(transform.transform.translation.y),
                "yaw_deg": float(
                    yaw_deg_from_quaternion_xyzw(
                        rotation.x,
                        rotation.y,
                        rotation.z,
                        rotation.w,
                    )
                ),
            }

    def spin_until(self, predicate, *, timeout_s: float) -> bool:
        """Procesar callbacks hasta cumplir una condicion o vencer el timeout."""

        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def wait_for_bootstrap(self, *, timeout_s: float) -> bool:
        """Esperar el servicio de ruta y la referencia GPS/odom/TF."""

        if not self.route_client.wait_for_service(timeout_sec=float(timeout_s)):
            return False
        return self.spin_until(
            lambda: self.gps_fix is not None
            and self.odom_local is not None
            and self.latest_map_odom_tf is not None,
            timeout_s=float(timeout_s),
        )

    def current_reference(self) -> dict[str, float]:
        """Obtener latitud, longitud y yaw globales de la pose actual."""

        if self.gps_fix is None or self.odom_local is None or self.latest_map_odom_tf is None:
            raise RuntimeError("la referencia GPS/odom/TF todavia no esta disponible")
        latitude = float(self.gps_fix.latitude)
        longitude = float(self.gps_fix.longitude)
        if not math.isfinite(latitude) or not math.isfinite(longitude):
            raise RuntimeError("el fix GPS actual no contiene coordenadas finitas")

        orientation = self.odom_local.pose.pose.orientation
        odom_yaw_deg = yaw_deg_from_quaternion_xyzw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        return {
            "lat": latitude,
            "lon": longitude,
            "yaw_deg": float(
                normalize_yaw_deg(
                    float(self.latest_map_odom_tf["yaw_deg"]) + float(odom_yaw_deg)
                )
            ),
        }

    def send_route(
        self,
        *,
        waypoints: list[dict[str, object]],
        leg_spacing_m: float,
        chunk_span_m: float,
        chunk_max_waypoints: int,
        timeout_s: float,
    ) -> dict[str, object]:
        """Enviar waypoints al ejecutor normal de rutas."""

        request = SetRouteMissionLL.Request()
        request.lats = [float(item["lat"]) for item in waypoints]
        request.lons = [float(item["lon"]) for item in waypoints]
        request.yaws_deg = [float(item["yaw_deg"]) for item in waypoints]
        request.waypoint_action_jsons = []
        request.waypoint_roles = []
        request.loop = False
        request.leg_spacing_m = float(leg_spacing_m)
        request.chunk_span_m = float(chunk_span_m)
        request.chunk_max_waypoints = int(chunk_max_waypoints)

        future = self.route_client.call_async(request)
        if not self.spin_until(lambda: future.done(), timeout_s=float(timeout_s)):
            raise RuntimeError("timeout esperando /route_executor/set_route_ll")
        response = future.result()
        if response is None:
            raise RuntimeError("el servicio /route_executor/set_route_ll no respondio")
        return {
            "ok": bool(response.ok),
            "error": str(response.error),
            "input_waypoint_count": int(response.input_waypoint_count),
            "expanded_waypoint_count": int(response.expanded_waypoint_count),
        }


def _write_plan(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Genera pasadas paralelas con cabeceras Dubins y las envia como "
            "waypoints al route_executor."
        )
    )
    parser.add_argument("--field-length-m", type=float, default=20.0)
    parser.add_argument("--field-width-m", type=float, default=8.0)
    parser.add_argument("--cutter-width-m", type=float, default=2.0)
    parser.add_argument("--overlap-ratio", type=float, default=0.15)
    parser.add_argument("--min-turning-radius-m", type=float, default=4.0)
    parser.add_argument("--waypoint-spacing-m", type=float, default=2.0)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    # Sin esta bandera las pasadas se recorren de a una, igual que el perfil de
    # simulacion. Prenderla permite saltear pasadas para achicar la cabecera.
    parser.add_argument("--allow-row-skipping", action="store_true")
    parser.add_argument("--chunk-span-m", type=float, default=60.0)
    parser.add_argument("--chunk-max-waypoints", type=int, default=25)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--send-route", action="store_true")
    parser.add_argument("--print-waypoints", action="store_true")
    parser.add_argument("--output", type=str, default="")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    """Ejecutar la generacion y, opcionalmente, iniciar la ruta."""

    args = _build_parser().parse_args(argv)
    rclpy.init(args=None)
    node = CoverageWaypointMissionNode()
    try:
        if not node.wait_for_bootstrap(timeout_s=float(args.timeout_s)):
            raise RuntimeError("timeout esperando GPS, odometria, TF y route_executor")

        reference = node.current_reference()
        plan, waypoints = build_lawnmower_waypoints(
            start_lat=float(reference["lat"]),
            start_lon=float(reference["lon"]),
            start_yaw_deg=float(reference["yaw_deg"]),
            field_length_m=float(args.field_length_m),
            field_width_m=float(args.field_width_m),
            cutter_width_m=float(args.cutter_width_m),
            overlap_ratio=float(args.overlap_ratio),
            min_turning_radius_m=float(args.min_turning_radius_m),
            waypoint_spacing_m=float(args.waypoint_spacing_m),
            side=str(args.side),
            allow_row_skipping=bool(args.allow_row_skipping),
        )
        # Solo los extremos de pasada viajan al route_executor. Los puntos
        # intermedios de las curvas convierten cada cabecera en una cadena de
        # metas separadas por menos que el radio minimo, y ahi el planner Dubins
        # resuelve cada tramo dando una vuelta completa en lugar de seguir el arco.
        route_waypoints = [item for item in waypoints if bool(item.get("key"))]
        turn_separations = list(plan.turn_separations_m)
        summary: dict[str, object] = {
            "reference": reference,
            "field_length_m": float(args.field_length_m),
            "field_width_m": float(args.field_width_m),
            "side": str(args.side),
            "row_count": int(plan.row_count),
            "lane_spacing_m": float(plan.lane_spacing_m),
            "cutter_width_m": float(plan.cutter_width_m),
            "overlap_ratio": float(plan.overlap_ratio),
            "min_turning_radius_m": float(plan.min_turning_radius_m),
            "row_visit_order": [int(index) for index in plan.row_visit_order],
            "headland_turns": {
                "count": len(turn_separations),
                "clean_uturns": int(plan.clean_uturn_count),
                "omega_turns": len(turn_separations) - int(plan.clean_uturn_count),
                "min_separation_m": min(turn_separations) if turn_separations else 0.0,
                "separation_needed_for_uturn_m": 2.0 * float(plan.min_turning_radius_m),
            },
            "sampled_waypoint_count": len(waypoints),
            "route_waypoint_count": len(route_waypoints),
            "estimated_path_length_m": float(plan.estimated_path_length_m),
            "required_headland_m": {
                "before": float(plan.headland_before_m),
                "after": float(plan.headland_after_m),
                "lateral_centerline_overflow": float(plan.lateral_overflow_m),
            },
        }
        output_payload = {**summary, "waypoints": waypoints}

        if args.output:
            output_path = Path(str(args.output)).expanduser()
            _write_plan(output_path, output_payload)
            summary["output"] = str(output_path)

        if bool(args.send_route):
            # El route_executor interpola puntos sinteticos cada leg_spacing_m en
            # linea recta entre waypoints. Sobre una cabecera esa recta corta por
            # dentro del giro, asi que el espaciado tiene que superar tanto el
            # largo de pasada como la separacion lateral mas grande.
            leg_spacing_m = max(
                5.0,
                float(args.field_length_m),
                max(turn_separations) if turn_separations else 0.0,
            ) + 1.0
            route_result = node.send_route(
                waypoints=route_waypoints,
                leg_spacing_m=leg_spacing_m,
                chunk_span_m=max(20.0, float(args.chunk_span_m)),
                chunk_max_waypoints=max(2, int(args.chunk_max_waypoints)),
                timeout_s=float(args.timeout_s),
            )
            summary["leg_spacing_m"] = float(leg_spacing_m)
            summary["set_route_result"] = route_result
            if not bool(route_result["ok"]):
                raise RuntimeError(f"set_route_ll rechazo la ruta: {route_result['error']}")

        if bool(args.print_waypoints):
            summary["waypoints"] = waypoints
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
