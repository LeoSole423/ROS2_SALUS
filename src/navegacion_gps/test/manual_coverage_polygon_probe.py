"""
Mide por separado surcos y cabeceras de CAMPO contra un octogono.

Uso dentro del contenedor:

    source /opt/ros/humble/setup.bash
    source /opt/salus_coverage_ws/install/setup.bash
    source /ros2_ws/install/setup.bash
    python3 /ros2_ws/src/navegacion_gps/test/manual_coverage_polygon_probe.py

El servicio es de preview: este probe no inicia ni mueve el vehiculo.
"""

import argparse
import math
from typing import List, Sequence, Tuple

import rclpy
from rclpy.node import Node

from interfaces.msg import GeoRing, NoGoPoint
from interfaces.srv import GenerateCoveragePlanLL


Point = Tuple[float, float]


def _segment_distance(point: Point, start: Point, end: Point) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    squared = (dx * dx) + (dy * dy)
    if squared <= 1.0e-12:
        return math.hypot(px - ax, py - ay)
    fraction = max(
        0.0,
        min(1.0, (((px - ax) * dx) + ((py - ay) * dy)) / squared),
    )
    return math.hypot(px - ax - (fraction * dx), py - ay - (fraction * dy))


def _edge_distance(point: Point, polygon: Sequence[Point]) -> float:
    return min(
        _segment_distance(
            point,
            polygon[index],
            polygon[(index + 1) % len(polygon)],
        )
        for index in range(len(polygon))
    )


def _inside(point: Point, polygon: Sequence[Point]) -> bool:
    if _edge_distance(point, polygon) <= 1.0e-6:
        return True
    x, y = point
    inside = False
    previous = len(polygon) - 1
    for index, (current_x, current_y) in enumerate(polygon):
        previous_x, previous_y = polygon[previous]
        if (current_y > y) != (previous_y > y):
            crossing_x = (
                ((previous_x - current_x) * (y - current_y))
                / (previous_y - current_y)
            ) + current_x
            if x < crossing_x:
                inside = not inside
        previous = index
    return inside


def _print_summary(
    label: str,
    points: Sequence[Tuple[int, Point]],
    polygon: Sequence[Point],
) -> None:
    outside = sorted(
        (
            (_edge_distance(point, polygon), index, point)
            for index, point in points
            if not _inside(point, polygon)
        ),
        reverse=True,
    )
    maximum = outside[0][0] if outside else 0.0
    print(
        f"{label}: total={len(points)} afuera={len(outside)} "
        f"max_distancia={maximum:.3f}m"
    )
    for distance_m, index, point in outside[:10]:
        print(
            f"  idx={index} distancia={distance_m:.3f}m "
            f"xy=({point[0]:.3f},{point[1]:.3f})"
        )


def main() -> int:
    """Solicita el preview y reporta los puntos fuera del octogono."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--lat", type=float, default=-31.485802)
    parser.add_argument("--lon", type=float, default=-64.241050)
    parser.add_argument("--radius", type=float, default=25.0)
    parser.add_argument("--turn-radius", type=float, default=4.0)
    parser.add_argument("--cutter-width", type=float, default=2.0)
    parser.add_argument("--overlap", type=float, default=0.15)
    args = parser.parse_args()

    meters_lat = 111_320.0
    meters_lon = meters_lat * math.cos(math.radians(args.lat))

    def to_ll(point: Point) -> NoGoPoint:
        x, y = point
        return NoGoPoint(
            lat=args.lat + (y / meters_lat),
            lon=args.lon + (x / meters_lon),
        )

    def to_xy(lat: float, lon: float) -> Point:
        return (
            (float(lon) - args.lon) * meters_lon,
            (float(lat) - args.lat) * meters_lat,
        )

    polygon = [
        (
            args.radius * math.cos(2.0 * math.pi * index / 8.0),
            args.radius * math.sin(2.0 * math.pi * index / 8.0),
        )
        for index in range(8)
    ]

    rclpy.init()
    node = Node("probe_campo_octogono")
    client = node.create_client(
        GenerateCoveragePlanLL,
        "/route_executor/generate_coverage_plan_ll",
    )
    try:
        if not client.wait_for_service(timeout_sec=20.0):
            print(
                "ERROR: /route_executor/generate_coverage_plan_ll "
                "no disponible"
            )
            return 2

        request = GenerateCoveragePlanLL.Request()
        request.start_lat = float(args.lat)
        request.start_lon = float(args.lon)
        request.start_yaw_deg = 0.0
        request.start_is_field_corner = False
        request.field_length_m = 2.0 * float(args.radius)
        request.field_width_m = 2.0 * float(args.radius)
        request.cutter_width_m = float(args.cutter_width)
        request.overlap_ratio = float(args.overlap)
        request.min_turning_radius_m = float(args.turn_radius)
        request.waypoint_spacing_m = 2.0
        request.side = "left"
        ring = GeoRing()
        ring.vertices = [to_ll(point) for point in polygon]
        request.coverage_polygon = ring

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=120.0)
        response = future.result()
        if response is None:
            print("ERROR: timeout esperando el plan")
            return 3
        print(
            f"ok={response.ok} error={response.error!r} "
            f"field_mode={response.field_mode!r} rows={response.row_count} "
            f"lane_spacing={response.lane_spacing_m:.3f}m "
            f"no_go={response.nogo_polygon_count} "
            f"descartados={response.nogo_dropped_count} "
            f"rodeos={response.nogo_detour_count}"
        )
        if not response.ok:
            return 4

        grouped: dict[str, List[Tuple[int, Point]]] = {
            "row": [],
            "turn": [],
            "other": [],
        }
        for index, (lat, lon, phase) in enumerate(
            zip(
                response.sampled_lats,
                response.sampled_lons,
                response.sampled_phases,
            )
        ):
            key = str(phase) if str(phase) in grouped else "other"
            grouped[key].append((index, to_xy(lat, lon)))
        for key in ("row", "turn", "other"):
            _print_summary(key.upper(), grouped[key], polygon)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
