import math
import threading
import time
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from interfaces.msg import NavTelemetry
from interfaces.srv import (
    CancelNavGoal,
    CancelRouteMission,
    GetRouteMissionState,
    SetNavGoalLL,
    SetRouteMissionLL,
)


@dataclass(frozen=True)
class RouteWaypoint:
    lat: float
    lon: float
    yaw_deg: float


@dataclass(frozen=True)
class PreparedRouteMission:
    waypoints: List[RouteWaypoint]
    start_index: int
    skipped_waypoints: int
    rotated: bool
    note: str


@dataclass(frozen=True)
class RouteSegmentMatch:
    segment_index: int
    next_index: int
    distance_m: float
    ratio: float


def _normalize_yaw_deg(yaw_deg: float) -> float:
    yaw = float(yaw_deg)
    while yaw <= -180.0:
        yaw += 360.0
    while yaw > 180.0:
        yaw -= 360.0
    return float(yaw)


def _distance_m(start_lat: float, start_lon: float, end_lat: float, end_lon: float) -> float:
    meters_per_deg_lat = 111_320.0
    avg_lat_rad = math.radians((float(start_lat) + float(end_lat)) * 0.5)
    meters_per_deg_lon = meters_per_deg_lat * max(1.0e-6, abs(math.cos(avg_lat_rad)))
    north_m = (float(end_lat) - float(start_lat)) * meters_per_deg_lat
    east_m = (float(end_lon) - float(start_lon)) * meters_per_deg_lon
    return float(math.hypot(north_m, east_m))


def _bearing_deg(start_lat: float, start_lon: float, end_lat: float, end_lon: float) -> float:
    meters_per_deg_lat = 111_320.0
    avg_lat_rad = math.radians((float(start_lat) + float(end_lat)) * 0.5)
    meters_per_deg_lon = meters_per_deg_lat * max(1.0e-6, abs(math.cos(avg_lat_rad)))
    north_m = (float(end_lat) - float(start_lat)) * meters_per_deg_lat
    east_m = (float(end_lon) - float(start_lon)) * meters_per_deg_lon
    if math.hypot(north_m, east_m) <= 1.0e-6:
        return 0.0
    return _normalize_yaw_deg(math.degrees(math.atan2(north_m, east_m)))


def _interpolate_waypoint(start: RouteWaypoint, end: RouteWaypoint, ratio: float) -> RouteWaypoint:
    return RouteWaypoint(
        lat=float(start.lat + ((end.lat - start.lat) * ratio)),
        lon=float(start.lon + ((end.lon - start.lon) * ratio)),
        yaw_deg=_bearing_deg(start.lat, start.lon, end.lat, end.lon),
    )


def _local_xy_m(
    lat: float,
    lon: float,
    *,
    origin_lat: float,
    origin_lon: float,
) -> Tuple[float, float]:
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = meters_per_deg_lat * max(
        1.0e-6, abs(math.cos(math.radians(float(origin_lat))))
    )
    x = (float(lon) - float(origin_lon)) * meters_per_deg_lon
    y = (float(lat) - float(origin_lat)) * meters_per_deg_lat
    return float(x), float(y)


def _closest_route_segment(
    route: Sequence[RouteWaypoint],
    *,
    robot_lat: float,
    robot_lon: float,
    loop: bool,
) -> Optional[RouteSegmentMatch]:
    route_list = list(route)
    if len(route_list) <= 1:
        return None

    robot_x, robot_y = _local_xy_m(
        robot_lat,
        robot_lon,
        origin_lat=robot_lat,
        origin_lon=robot_lon,
    )
    segment_count = len(route_list) if loop else len(route_list) - 1
    best: Optional[RouteSegmentMatch] = None

    for idx in range(segment_count):
        start = route_list[idx]
        next_index = (idx + 1) % len(route_list)
        end = route_list[next_index]
        start_x, start_y = _local_xy_m(
            start.lat,
            start.lon,
            origin_lat=robot_lat,
            origin_lon=robot_lon,
        )
        end_x, end_y = _local_xy_m(
            end.lat,
            end.lon,
            origin_lat=robot_lat,
            origin_lon=robot_lon,
        )
        seg_x = end_x - start_x
        seg_y = end_y - start_y
        seg_len_sq = (seg_x * seg_x) + (seg_y * seg_y)
        if seg_len_sq <= 1.0e-6:
            continue

        raw_ratio = (
            ((robot_x - start_x) * seg_x) + ((robot_y - start_y) * seg_y)
        ) / seg_len_sq
        ratio = min(1.0, max(0.0, float(raw_ratio)))
        proj_x = start_x + (seg_x * ratio)
        proj_y = start_y + (seg_y * ratio)
        distance_m = math.hypot(robot_x - proj_x, robot_y - proj_y)
        candidate = RouteSegmentMatch(
            segment_index=int(idx),
            next_index=int(next_index),
            distance_m=float(distance_m),
            ratio=float(ratio),
        )
        if best is None or (candidate.distance_m, candidate.segment_index) < (
            best.distance_m,
            best.segment_index,
        ):
            best = candidate

    return best


def _is_usable_segment_match(
    match: Optional[RouteSegmentMatch],
    *,
    segment_tolerance_m: float,
) -> bool:
    return bool(
        match is not None
        and match.distance_m <= segment_tolerance_m
        and match.ratio > 0.0
        and match.ratio < 1.0
    )


def _resolve_input_waypoints(
    lats: Sequence[float],
    lons: Sequence[float],
    yaws_deg: Sequence[float],
    loop: bool,
) -> List[RouteWaypoint]:
    resolved: List[RouteWaypoint] = []
    use_explicit_yaws = len(yaws_deg) == len(lats)
    for idx, (lat, lon) in enumerate(zip(lats, lons)):
        if use_explicit_yaws:
            yaw_deg = _normalize_yaw_deg(float(yaws_deg[idx]))
        elif idx + 1 < len(lats):
            yaw_deg = _bearing_deg(lat, lon, float(lats[idx + 1]), float(lons[idx + 1]))
        elif loop and len(lats) > 1:
            yaw_deg = _bearing_deg(lat, lon, float(lats[0]), float(lons[0]))
        elif idx > 0:
            yaw_deg = _bearing_deg(float(lats[idx - 1]), float(lons[idx - 1]), lat, lon)
        else:
            yaw_deg = 0.0
        resolved.append(RouteWaypoint(lat=float(lat), lon=float(lon), yaw_deg=float(yaw_deg)))
    return resolved


def expand_route_waypoints(
    base_waypoints: Sequence[RouteWaypoint],
    *,
    leg_spacing_m: float,
    loop: bool,
) -> List[RouteWaypoint]:
    base = list(base_waypoints)
    if len(base) <= 1:
        return base

    spacing = max(1.0, float(leg_spacing_m))
    expanded: List[RouteWaypoint] = [base[0]]
    segment_count = len(base) if loop else len(base) - 1

    for idx in range(segment_count):
        start = base[idx]
        end = base[(idx + 1) % len(base)]
        distance_m = _distance_m(start.lat, start.lon, end.lat, end.lon)
        split_count = max(1, int(math.ceil(distance_m / spacing)))
        for split_idx in range(1, split_count):
            expanded.append(_interpolate_waypoint(start, end, float(split_idx) / float(split_count)))
        if idx + 1 < len(base):
            expanded.append(base[idx + 1])

    return expanded


def prepare_route_waypoints(
    base_waypoints: Sequence[RouteWaypoint],
    *,
    loop: bool,
    robot_lat: Optional[float],
    robot_lon: Optional[float],
    waypoint_reached_tolerance_m: float,
    segment_start_tolerance_m: float = 5.0,
) -> Tuple[Optional[PreparedRouteMission], str]:
    route = list(base_waypoints)
    if not route:
        return PreparedRouteMission([], 0, 0, False, ""), ""

    if (
        robot_lat is None
        or robot_lon is None
        or (not np.isfinite(float(robot_lat)))
        or (not np.isfinite(float(robot_lon)))
    ):
        return PreparedRouteMission(route, 0, 0, False, ""), ""

    tolerance_m = max(0.05, float(waypoint_reached_tolerance_m))
    segment_tolerance_raw = float(segment_start_tolerance_m)
    segment_tolerance_m = (
        max(tolerance_m, segment_tolerance_raw)
        if np.isfinite(segment_tolerance_raw)
        else tolerance_m
    )
    distances_m = [
        _distance_m(float(robot_lat), float(robot_lon), waypoint.lat, waypoint.lon)
        for waypoint in route
    ]

    if loop:
        if all(distance_m <= tolerance_m for distance_m in distances_m):
            return None, (
                f"route already within {tolerance_m:.2f} m of robot; "
                "refine the patrol route before sending it"
            )

        if len(route) <= 1:
            return PreparedRouteMission(route, 0, 0, False, ""), ""

        reached_indexes = [
            idx for idx, distance_m in enumerate(distances_m) if distance_m <= tolerance_m
        ]
        if not reached_indexes:
            closest_segment = _closest_route_segment(
                route,
                robot_lat=float(robot_lat),
                robot_lon=float(robot_lon),
                loop=True,
            )
            if not _is_usable_segment_match(
                closest_segment,
                segment_tolerance_m=segment_tolerance_m,
            ):
                return PreparedRouteMission(route, 0, 0, False, ""), ""

            match = closest_segment
            if match is None:
                return PreparedRouteMission(route, 0, 0, False, ""), ""
            start_index = int(match.next_index)
            if start_index == 0:
                return PreparedRouteMission(route, 0, 0, False, ""), ""
            return (
                PreparedRouteMission(
                    route[start_index:] + route[:start_index],
                    int(start_index),
                    0,
                    True,
                    (
                        "loop joined nearest segment "
                        f"{match.segment_index + 1}->{match.next_index + 1}"
                    ),
                ),
                "",
            )

        anchor_index = min(reached_indexes, key=lambda idx: (distances_m[idx], idx))
        start_index = anchor_index
        for _ in range(len(route)):
            if distances_m[start_index] > tolerance_m:
                break
            start_index = (start_index + 1) % len(route)

        if start_index == 0:
            return PreparedRouteMission(route, 0, 0, False, ""), ""

        return (
            PreparedRouteMission(
                route[start_index:] + route[:start_index],
                int(start_index),
                0,
                True,
                f"loop rotated to waypoint {start_index + 1}",
            ),
            "",
        )

    if distances_m[-1] <= tolerance_m:
        return None, (
            f"final waypoint already within {tolerance_m:.2f} m of robot; "
            "reorder the non-loop route or enable loop mode"
        )

    start_index = 0
    while start_index < (len(route) - 1) and distances_m[start_index] <= tolerance_m:
        start_index += 1

    segment_note = ""
    if start_index == 0 and len(route) > 1:
        closest_segment = _closest_route_segment(
            route,
            robot_lat=float(robot_lat),
            robot_lon=float(robot_lon),
            loop=False,
        )
        if _is_usable_segment_match(
            closest_segment,
            segment_tolerance_m=segment_tolerance_m,
        ):
            match = closest_segment
            if match is None:
                return PreparedRouteMission(route, 0, 0, False, ""), ""
            start_index = max(start_index, int(match.next_index))
            segment_note = (
                "joined nearest segment "
                f"{match.segment_index + 1}->{match.next_index + 1}"
            )

    note = ""
    if segment_note:
        note = segment_note
    elif start_index > 0:
        suffix = "s" if start_index != 1 else ""
        note = f"skipped {start_index} reached waypoint{suffix}"

    return (
        PreparedRouteMission(
            route[start_index:],
            int(start_index),
            int(start_index),
            False,
            note,
        ),
        "",
    )


def build_chunk_waypoints(
    route: Sequence[RouteWaypoint],
    *,
    start_index: int,
    loop: bool,
    chunk_span_m: float,
    chunk_max_waypoints: int,
) -> Tuple[List[RouteWaypoint], int]:
    route_list = list(route)
    if not route_list:
        return [], 0

    total = len(route_list)
    requested_start = max(0, int(start_index))
    if not loop and requested_start >= total:
        return [], total
    start = requested_start % total if loop else requested_start
    max_points = max(1, int(chunk_max_waypoints))
    max_span_m = max(1.0, float(chunk_span_m))

    chunk = [route_list[start]]
    end_index = start
    visited_steps = 0
    cumulative_distance_m = 0.0

    while len(chunk) < max_points:
        next_index = end_index + 1
        if loop:
            next_index %= total
            if next_index == start:
                break
        elif next_index >= total:
            break

        step_distance_m = _distance_m(
            route_list[end_index].lat,
            route_list[end_index].lon,
            route_list[next_index].lat,
            route_list[next_index].lon,
        )
        should_stop = len(chunk) > 1 and (cumulative_distance_m + step_distance_m) > max_span_m
        if should_stop:
            break

        chunk.append(route_list[next_index])
        cumulative_distance_m += step_distance_m
        end_index = next_index
        visited_steps += 1
        if visited_steps >= max(0, total - 1):
            break

    if len(chunk) == 1 and total > 1:
        next_index = (start + 1) % total if loop else start + 1
        if next_index < total and next_index != start:
            chunk.append(route_list[next_index])
            end_index = next_index

    return chunk, end_index


def next_chunk_start_index(
    *,
    current_target_index: int,
    route_size: int,
    loop: bool,
) -> int:
    total = max(0, int(route_size))
    if total <= 0:
        return 0

    target_index = max(0, int(current_target_index))
    if loop and total > 1:
        return (target_index + 1) % total
    return min(target_index + 1, total)


def should_suppress_chunk_success_brake(
    *,
    current_target_index: int,
    route_size: int,
    loop: bool,
) -> bool:
    total = max(0, int(route_size))
    if bool(loop):
        return total > 1
    return int(current_target_index) < max(0, total - 1)


class RouteExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("route_executor")

        self.declare_parameter("nav_set_goal_service", "/nav_command_server/set_goal_ll")
        self.declare_parameter("nav_cancel_goal_service", "/nav_command_server/cancel_goal")
        self.declare_parameter("nav_telemetry_topic", "/nav_command_server/telemetry")
        self.declare_parameter("set_route_service", "/route_executor/set_route_ll")
        self.declare_parameter("cancel_route_service", "/route_executor/cancel_route")
        self.declare_parameter("get_state_service", "/route_executor/get_state")
        self.declare_parameter("request_timeout_s", 8.0)
        self.declare_parameter("default_leg_spacing_m", 35.0)
        self.declare_parameter("default_chunk_span_m", 120.0)
        self.declare_parameter("default_chunk_max_waypoints", 5)
        self.declare_parameter("min_leg_spacing_m", 5.0)
        self.declare_parameter("min_chunk_span_m", 20.0)
        self.declare_parameter("min_chunk_max_waypoints", 2)
        self.declare_parameter("route_waypoint_reached_tolerance_m", 1.2)
        self.declare_parameter("route_segment_start_tolerance_m", 5.0)

        self.nav_set_goal_service = str(self.get_parameter("nav_set_goal_service").value)
        self.nav_cancel_goal_service = str(self.get_parameter("nav_cancel_goal_service").value)
        self.nav_telemetry_topic = str(self.get_parameter("nav_telemetry_topic").value)
        self.set_route_service = str(self.get_parameter("set_route_service").value)
        self.cancel_route_service = str(self.get_parameter("cancel_route_service").value)
        self.get_state_service = str(self.get_parameter("get_state_service").value)
        self.request_timeout_s = max(0.5, float(self.get_parameter("request_timeout_s").value))
        self.default_leg_spacing_m = max(1.0, float(self.get_parameter("default_leg_spacing_m").value))
        self.default_chunk_span_m = max(5.0, float(self.get_parameter("default_chunk_span_m").value))
        self.default_chunk_max_waypoints = max(
            2, int(self.get_parameter("default_chunk_max_waypoints").value)
        )
        self.min_leg_spacing_m = max(1.0, float(self.get_parameter("min_leg_spacing_m").value))
        self.min_chunk_span_m = max(5.0, float(self.get_parameter("min_chunk_span_m").value))
        self.min_chunk_max_waypoints = max(
            2, int(self.get_parameter("min_chunk_max_waypoints").value)
        )
        self.route_waypoint_reached_tolerance_m = max(
            0.05, float(self.get_parameter("route_waypoint_reached_tolerance_m").value)
        )
        self.route_segment_start_tolerance_m = max(
            self.route_waypoint_reached_tolerance_m,
            float(self.get_parameter("route_segment_start_tolerance_m").value),
        )

        self._lock = threading.Lock()
        self._route_input: List[RouteWaypoint] = []
        self._route_expanded: List[RouteWaypoint] = []
        self._active_chunk: List[RouteWaypoint] = []
        self._mission_active = False
        self._mission_paused = False
        self._mission_loop = False
        self._mission_status = "idle"
        self._leg_spacing_m = float(self.default_leg_spacing_m)
        self._chunk_span_m = float(self.default_chunk_span_m)
        self._chunk_max_waypoints = int(self.default_chunk_max_waypoints)
        self._last_robot_pose: Optional[Tuple[float, float]] = None
        self._mission_note = ""
        self._current_start_index = 0
        self._current_target_index = 0
        self._awaiting_chunk_result = False
        self._last_nav_goal_active = False
        self._last_nav_result_status = int(GoalStatus.STATUS_UNKNOWN)
        self._last_nav_result_event_id = 0
        self._last_handled_nav_result_event_id = 0

        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        self._nav_set_goal_client = self.create_client(
            SetNavGoalLL,
            self.nav_set_goal_service,
            callback_group=self._client_group,
        )
        self._nav_cancel_goal_client = self.create_client(
            CancelNavGoal,
            self.nav_cancel_goal_service,
            callback_group=self._client_group,
        )

        self._set_route_srv = self.create_service(
            SetRouteMissionLL,
            self.set_route_service,
            self._on_set_route,
            callback_group=self._service_group,
        )
        self._cancel_route_srv = self.create_service(
            CancelRouteMission,
            self.cancel_route_service,
            self._on_cancel_route,
            callback_group=self._service_group,
        )
        self._get_state_srv = self.create_service(
            GetRouteMissionState,
            self.get_state_service,
            self._on_get_state,
            callback_group=self._service_group,
        )
        self._nav_telemetry_sub = self.create_subscription(
            NavTelemetry,
            self.nav_telemetry_topic,
            self._on_nav_telemetry,
            10,
            callback_group=self._client_group,
        )

        self.get_logger().info(
            "Route executor ready "
            f"(set_route={self.set_route_service}, cancel={self.cancel_route_service}, "
            f"get_state={self.get_state_service}, nav_goal={self.nav_set_goal_service})"
        )

    @staticmethod
    def _wait_for_future(future: Any, timeout_s: float) -> Optional[Any]:
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                return future.result()
            if (time.monotonic() - start) >= timeout_s:
                return None
            time.sleep(0.01)
        return None

    def _call_service(self, client: Any, request: Any, timeout_s: float) -> Optional[Any]:
        if not client.wait_for_service(timeout_sec=min(timeout_s, 2.0)):
            self.get_logger().warning(f"Service unavailable: {getattr(client, 'srv_name', '<unknown>')}")
            return None
        future = client.call_async(request)
        return self._wait_for_future(future, timeout_s)

    def _reset_mission_locked(self, status: str = "idle") -> None:
        self._route_input = []
        self._route_expanded = []
        self._active_chunk = []
        self._mission_active = False
        self._mission_paused = False
        self._mission_loop = False
        self._mission_status = str(status)
        self._mission_note = ""
        self._current_start_index = 0
        self._current_target_index = 0
        self._awaiting_chunk_result = False
        self._last_handled_nav_result_event_id = self._last_nav_result_event_id

    def _status_with_note_locked(self, status: str) -> str:
        note = str(self._mission_note).strip()
        if not note:
            return str(status)
        return f"{status} [{note}]"

    def _cancel_nav_goal(self) -> Tuple[bool, str]:
        res = self._call_service(
            self._nav_cancel_goal_client, CancelNavGoal.Request(), self.request_timeout_s
        )
        if res is None:
            return False, "cancel_goal timeout"
        return bool(res.ok), str(res.error)

    def _send_chunk(self, *, start_index: int) -> Tuple[bool, str]:
        with self._lock:
            route = list(self._route_expanded)
            loop_enabled = bool(self._mission_loop)
            chunk_span_m = float(self._chunk_span_m)
            chunk_max_waypoints = int(self._chunk_max_waypoints)

        chunk, end_index = build_chunk_waypoints(
            route,
            start_index=start_index,
            loop=loop_enabled,
            chunk_span_m=chunk_span_m,
            chunk_max_waypoints=chunk_max_waypoints,
        )
        if not chunk:
            return False, "empty route chunk"

        request = SetNavGoalLL.Request()
        request.lats = [float(entry.lat) for entry in chunk]
        request.lons = [float(entry.lon) for entry in chunk]
        request.yaws_deg = [float(entry.yaw_deg) for entry in chunk]
        request.loop = False
        request.suppress_success_brake = should_suppress_chunk_success_brake(
            current_target_index=end_index,
            route_size=len(route),
            loop=loop_enabled,
        )
        request.lat = float(request.lats[0])
        request.lon = float(request.lons[0])
        request.yaw_deg = float(request.yaws_deg[0])
        response = self._call_service(self._nav_set_goal_client, request, self.request_timeout_s)
        if response is None:
            return False, "set_goal_ll timeout"
        if not response.ok:
            return False, str(response.error)

        with self._lock:
            self._active_chunk = chunk
            self._current_start_index = int(start_index)
            self._current_target_index = int(end_index)
            self._awaiting_chunk_result = True
            self._mission_status = self._status_with_note_locked(
                f"route active ({self._current_start_index + 1}->{self._current_target_index + 1})"
            )
        self.get_logger().info(
            f"Route chunk dispatched (start={start_index}, end={end_index}, size={len(chunk)})"
        )
        return True, ""

    def _start_next_chunk_after_success(self) -> None:
        with self._lock:
            if not self._mission_active or self._mission_paused:
                return
            expanded_count = len(self._route_expanded)
            loop_enabled = bool(self._mission_loop)
            if expanded_count == 0:
                self._reset_mission_locked("route failed: empty expanded route")
                return
            next_start_index = next_chunk_start_index(
                current_target_index=self._current_target_index,
                route_size=expanded_count,
                loop=loop_enabled,
            )
            reached_end = next_start_index >= expanded_count
            if reached_end and (not loop_enabled):
                self._mission_active = False
                self._mission_paused = False
                self._awaiting_chunk_result = False
                self._active_chunk = []
                self._mission_status = self._status_with_note_locked("route completed")
                return

        ok, err = self._send_chunk(start_index=next_start_index)
        if ok:
            return
        with self._lock:
            self._mission_active = False
            self._mission_paused = False
            self._awaiting_chunk_result = False
            self._active_chunk = []
            self._mission_status = self._status_with_note_locked(f"route failed: {err}")

    def _on_nav_telemetry(self, msg: NavTelemetry) -> None:
        should_pause = False
        should_advance = False
        should_stop = False
        stop_reason = ""

        with self._lock:
            if np.isfinite(float(msg.robot_lat)) and np.isfinite(float(msg.robot_lon)):
                self._last_robot_pose = (float(msg.robot_lat), float(msg.robot_lon))
            self._last_nav_goal_active = bool(msg.goal_active)
            self._last_nav_result_status = int(msg.nav_result_status)
            self._last_nav_result_event_id = int(msg.nav_result_event_id)

            if not self._mission_active:
                return

            if bool(msg.manual_enabled) and (not self._mission_paused):
                self._mission_paused = True
                self._awaiting_chunk_result = False
                self._active_chunk = []
                self._mission_status = self._status_with_note_locked(
                    "route paused by manual takeover"
                )
                should_pause = True
            elif self._mission_paused:
                return

            terminal_result = (
                (not bool(msg.goal_active))
                and self._awaiting_chunk_result
                and int(msg.nav_result_event_id) > 0
                and int(msg.nav_result_event_id) != self._last_handled_nav_result_event_id
                and int(msg.nav_result_status)
                in (
                    int(GoalStatus.STATUS_SUCCEEDED),
                    int(GoalStatus.STATUS_ABORTED),
                    int(GoalStatus.STATUS_CANCELED),
                )
            )
            if not terminal_result:
                return

            self._last_handled_nav_result_event_id = int(msg.nav_result_event_id)
            self._awaiting_chunk_result = False
            status = int(msg.nav_result_status)
            if status == int(GoalStatus.STATUS_SUCCEEDED):
                should_advance = True
            elif status == int(GoalStatus.STATUS_CANCELED):
                self._mission_active = False
                self._active_chunk = []
                self._mission_status = self._status_with_note_locked("route cancelled")
                should_stop = True
                stop_reason = "cancelled"
            else:
                self._mission_active = False
                self._active_chunk = []
                self._mission_status = self._status_with_note_locked(
                    f"route failed: {str(msg.nav_result_text)}"
                )
                should_stop = True
                stop_reason = str(msg.nav_result_text)

        if should_pause:
            self.get_logger().warning("Route mission paused by manual takeover")
            return
        if should_advance:
            self._start_next_chunk_after_success()
            return
        if should_stop:
            self.get_logger().warning(f"Route mission stopped ({stop_reason})")

    def _validate_set_route_request(
        self, request: SetRouteMissionLL.Request
    ) -> Tuple[Optional[List[RouteWaypoint]], bool, float, float, int, str]:
        lats = [float(value) for value in request.lats]
        lons = [float(value) for value in request.lons]
        yaws = [float(value) for value in request.yaws_deg]
        if len(lats) == 0:
            return None, False, 0.0, 0.0, 0, "at least one waypoint is required"
        if len(lats) != len(lons):
            return None, False, 0.0, 0.0, 0, "lats and lons must have the same length"
        if len(yaws) not in (0, len(lats)):
            return None, False, 0.0, 0.0, 0, "yaws_deg must be empty or match lats length"
        for idx, (lat, lon) in enumerate(zip(lats, lons)):
            if (not np.isfinite(lat)) or (not np.isfinite(lon)):
                return None, False, 0.0, 0.0, 0, f"invalid waypoint values at index {idx}"
        for idx, yaw_deg in enumerate(yaws):
            if not np.isfinite(yaw_deg):
                return None, False, 0.0, 0.0, 0, f"invalid yaw_deg at index {idx}"

        leg_spacing_m = (
            float(request.leg_spacing_m)
            if np.isfinite(float(request.leg_spacing_m)) and float(request.leg_spacing_m) > 0.0
            else float(self.default_leg_spacing_m)
        )
        chunk_span_m = (
            float(request.chunk_span_m)
            if np.isfinite(float(request.chunk_span_m)) and float(request.chunk_span_m) > 0.0
            else float(self.default_chunk_span_m)
        )
        chunk_max_waypoints = int(request.chunk_max_waypoints) or int(self.default_chunk_max_waypoints)
        leg_spacing_m = max(float(self.min_leg_spacing_m), leg_spacing_m)
        chunk_span_m = max(float(self.min_chunk_span_m), chunk_span_m)
        chunk_max_waypoints = max(int(self.min_chunk_max_waypoints), chunk_max_waypoints)

        resolved = _resolve_input_waypoints(lats, lons, yaws, bool(request.loop))
        return (
            resolved,
            bool(request.loop),
            leg_spacing_m,
            chunk_span_m,
            chunk_max_waypoints,
            "",
        )

    def _on_set_route(
        self, request: SetRouteMissionLL.Request, response: SetRouteMissionLL.Response
    ) -> SetRouteMissionLL.Response:
        route_input, loop_enabled, leg_spacing_m, chunk_span_m, chunk_max_waypoints, error = (
            self._validate_set_route_request(request)
        )
        if route_input is None:
            response.ok = False
            response.error = error
            return response

        with self._lock:
            robot_pose = self._last_robot_pose
        prepared, prepare_error = prepare_route_waypoints(
            route_input,
            loop=loop_enabled,
            robot_lat=None if robot_pose is None else float(robot_pose[0]),
            robot_lon=None if robot_pose is None else float(robot_pose[1]),
            waypoint_reached_tolerance_m=self.route_waypoint_reached_tolerance_m,
            segment_start_tolerance_m=self.route_segment_start_tolerance_m,
        )
        if prepared is None:
            response.ok = False
            response.error = prepare_error
            response.input_waypoint_count = 0
            response.expanded_waypoint_count = 0
            return response

        expanded = expand_route_waypoints(
            prepared.waypoints,
            leg_spacing_m=leg_spacing_m,
            loop=loop_enabled,
        )

        with self._lock:
            had_mission = self._mission_active or self._mission_paused
        if had_mission:
            self._cancel_nav_goal()

        with self._lock:
            self._route_input = list(prepared.waypoints)
            self._route_expanded = list(expanded)
            self._active_chunk = []
            self._mission_active = True
            self._mission_paused = False
            self._mission_loop = bool(loop_enabled)
            self._mission_note = str(prepared.note)
            self._mission_status = self._status_with_note_locked("route starting")
            self._leg_spacing_m = float(leg_spacing_m)
            self._chunk_span_m = float(chunk_span_m)
            self._chunk_max_waypoints = int(chunk_max_waypoints)
            self._current_start_index = 0
            self._current_target_index = 0
            self._awaiting_chunk_result = False
            self._last_handled_nav_result_event_id = self._last_nav_result_event_id

        ok, err = self._send_chunk(start_index=0)
        response.input_waypoint_count = int(len(prepared.waypoints))
        response.expanded_waypoint_count = int(len(expanded))
        response.ok = bool(ok)
        response.error = "" if ok else str(err)
        if not ok:
            with self._lock:
                self._mission_active = False
                self._mission_paused = False
                self._active_chunk = []
                self._mission_status = self._status_with_note_locked(f"route failed: {err}")
        return response

    def _on_cancel_route(
        self, _request: CancelRouteMission.Request, response: CancelRouteMission.Response
    ) -> CancelRouteMission.Response:
        cancel_ok, cancel_err = self._cancel_nav_goal()
        with self._lock:
            self._reset_mission_locked("route cancelled")
        response.ok = bool(cancel_ok or cancel_err == "cancel_goal timeout")
        response.error = "" if response.ok else str(cancel_err)
        return response

    def _fill_route_state_response(
        self, response: GetRouteMissionState.Response
    ) -> GetRouteMissionState.Response:
        with self._lock:
            response.ok = True
            response.error = ""
            response.active = bool(self._mission_active)
            response.paused = bool(self._mission_paused)
            response.loop = bool(self._mission_loop)
            response.input_waypoint_count = int(len(self._route_input))
            response.expanded_waypoint_count = int(len(self._route_expanded))
            response.current_start_index = int(self._current_start_index)
            response.current_target_index = int(self._current_target_index)
            response.active_chunk_size = int(len(self._active_chunk))
            response.leg_spacing_m = float(self._leg_spacing_m)
            response.chunk_span_m = float(self._chunk_span_m)
            response.chunk_max_waypoints = int(self._chunk_max_waypoints)
            response.status = str(self._mission_status)
            response.mission_lats = [float(entry.lat) for entry in self._route_expanded]
            response.mission_lons = [float(entry.lon) for entry in self._route_expanded]
            response.mission_yaws_deg = [float(entry.yaw_deg) for entry in self._route_expanded]
            response.active_lats = [float(entry.lat) for entry in self._active_chunk]
            response.active_lons = [float(entry.lon) for entry in self._active_chunk]
            response.active_yaws_deg = [float(entry.yaw_deg) for entry in self._active_chunk]
        return response

    def _on_get_state(
        self, _request: GetRouteMissionState.Request, response: GetRouteMissionState.Response
    ) -> GetRouteMissionState.Response:
        return self._fill_route_state_response(response)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = RouteExecutorNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
