import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from interfaces.msg import NavSnapshotLayers
from interfaces.srv import GetNavSnapshot


class NavSnapshotServerNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_snapshot_server")

        self.declare_parameter("get_snapshot_service", "/nav_snapshot_server/get_nav_snapshot")
        self.declare_parameter("local_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("keepout_mask_topic", "/keepout_filter_mask")
        self.declare_parameter(
            "local_footprint_topic", "/local_costmap/published_footprint"
        )
        self.declare_parameter("stop_zone_topic", "/stop_zone")
        self.declare_parameter(
            "collision_polygons_topic", "/collision_monitor/polygons"
        )
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("snapshot_extent_m", 30.0)
        self.declare_parameter("snapshot_size_px", 512)
        self.declare_parameter("snapshot_global_inset_px", 160)
        self.declare_parameter("snapshot_timeout_ms", 500)
        self.declare_parameter("tf_timeout_s", 0.2)

        self.get_snapshot_service = str(self.get_parameter("get_snapshot_service").value)
        self.local_costmap_topic = str(self.get_parameter("local_costmap_topic").value)
        self.global_costmap_topic = str(
            self.get_parameter("global_costmap_topic").value
        )
        self.keepout_mask_topic = str(self.get_parameter("keepout_mask_topic").value)
        self.local_footprint_topic = str(
            self.get_parameter("local_footprint_topic").value
        )
        self.stop_zone_topic = str(self.get_parameter("stop_zone_topic").value)
        self.collision_polygons_topic = str(
            self.get_parameter("collision_polygons_topic").value
        )
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.snapshot_extent_m = max(
            5.0, float(self.get_parameter("snapshot_extent_m").value)
        )
        self.snapshot_size_px = max(
            128, int(self.get_parameter("snapshot_size_px").value)
        )
        self.snapshot_global_inset_px = max(
            80, int(self.get_parameter("snapshot_global_inset_px").value)
        )
        self.snapshot_timeout_ms = max(
            100, int(self.get_parameter("snapshot_timeout_ms").value)
        )
        self.tf_timeout_s = max(0.05, float(self.get_parameter("tf_timeout_s").value))

        self._lock = threading.Lock()
        self._local_costmap: Optional[OccupancyGrid] = None
        self._global_costmap: Optional[OccupancyGrid] = None
        self._keepout_mask: Optional[OccupancyGrid] = None
        self._local_footprint: Optional[PolygonStamped] = None
        self._stop_zone: Optional[PolygonStamped] = None
        self._collision_polygons: Optional[MarkerArray] = None
        self._scan: Optional[LaserScan] = None
        self._plan: Optional[Path] = None

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        latched_grid_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._get_snapshot_srv = self.create_service(
            GetNavSnapshot, self.get_snapshot_service, self._on_get_snapshot
        )

        self._local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic,
            self._on_local_costmap,
            latched_grid_qos,
        )
        self._global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.global_costmap_topic,
            self._on_global_costmap,
            latched_grid_qos,
        )
        self._keepout_mask_sub = self.create_subscription(
            OccupancyGrid,
            self.keepout_mask_topic,
            self._on_keepout_mask,
            latched_grid_qos,
        )
        self._local_footprint_sub = self.create_subscription(
            PolygonStamped, self.local_footprint_topic, self._on_local_footprint, 10
        )
        self._stop_zone_sub = self.create_subscription(
            PolygonStamped, self.stop_zone_topic, self._on_stop_zone, 10
        )
        self._collision_polygons_sub = self.create_subscription(
            MarkerArray,
            self.collision_polygons_topic,
            self._on_collision_polygons,
            10,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self._plan_sub = self.create_subscription(Path, self.plan_topic, self._on_plan, 10)

        self.get_logger().info(
            "nav_snapshot_server ready "
            f"(service={self.get_snapshot_service}, size={self.snapshot_size_px}px, "
            f"local={self.local_costmap_topic}, global={self.global_costmap_topic}, "
            f"keepout={self.keepout_mask_topic}, scan={self.scan_topic}, plan={self.plan_topic})"
        )

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._local_costmap = msg

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._global_costmap = msg

    def _on_keepout_mask(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._keepout_mask = msg

    def _on_local_footprint(self, msg: PolygonStamped) -> None:
        with self._lock:
            self._local_footprint = msg

    def _on_stop_zone(self, msg: PolygonStamped) -> None:
        with self._lock:
            self._stop_zone = msg

    def _on_collision_polygons(self, msg: MarkerArray) -> None:
        with self._lock:
            self._collision_polygons = msg

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._scan = msg

    def _on_plan(self, msg: Path) -> None:
        with self._lock:
            self._plan = msg

    def _on_get_snapshot(
        self,
        _request: GetNavSnapshot.Request,
        response: GetNavSnapshot.Response,
    ) -> GetNavSnapshot.Response:
        self.get_logger().info("GetNavSnapshot request received")
        started = time.perf_counter()
        ok, payload, err = self._build_snapshot_payload()
        elapsed_ms = (time.perf_counter() - started) * 1000.0

        if elapsed_ms > float(self.snapshot_timeout_ms):
            self.get_logger().warning(
                "Snapshot generation exceeded target "
                f"({elapsed_ms:.1f} ms > {self.snapshot_timeout_ms} ms)"
            )

        response.ok = bool(ok)
        response.error = "" if ok else str(err or "snapshot failed")
        if not ok:
            self.get_logger().warning(
                f"GetNavSnapshot failed (elapsed_ms={elapsed_ms:.1f}, error='{response.error}')"
            )
            response.mime = ""
            response.width = 0
            response.height = 0
            response.frame_id = payload.get("frame_id", "")
            response.stamp = self.get_clock().now().to_msg()
            response.layers = self._dict_to_layers(payload.get("layers", {}))
            response.image_png = []
            return response

        response.mime = str(payload["mime"])
        response.width = int(payload["width"])
        response.height = int(payload["height"])
        response.frame_id = str(payload["frame_id"])
        response.stamp = payload["stamp"]
        response.layers = self._dict_to_layers(payload["layers"])
        response.image_png = list(payload["image_png"])
        self.get_logger().info(
            "GetNavSnapshot ok "
            f"(elapsed_ms={elapsed_ms:.1f}, "
            f"frame={response.frame_id}, "
            f"size={response.width}x{response.height}, "
            f"png_bytes={len(response.image_png)})"
        )
        return response

    def _dict_to_layers(self, layers: Dict[str, bool]) -> NavSnapshotLayers:
        out = NavSnapshotLayers()
        out.local_costmap = bool(layers.get("local_costmap", False))
        out.global_costmap = bool(layers.get("global_costmap", False))
        out.keepout_mask = bool(layers.get("keepout_mask", False))
        out.footprint = bool(layers.get("footprint", False))
        out.stop_zone = bool(layers.get("stop_zone", False))
        out.scan = bool(layers.get("scan", False))
        out.plan = bool(layers.get("plan", False))
        out.collision_polygons = bool(layers.get("collision_polygons", False))
        out.global_inset = bool(layers.get("global_inset", False))
        return out

    def _build_snapshot_payload(
        self,
    ) -> Tuple[bool, Dict[str, Any], Optional[str]]:
        with self._lock:
            local_costmap = self._local_costmap
            global_costmap = self._global_costmap
            keepout_mask = self._keepout_mask
            local_footprint = self._local_footprint
            stop_zone = self._stop_zone
            collision_polygons = self._collision_polygons
            scan = self._scan
            plan = self._plan

        layers = {
            "local_costmap": local_costmap is not None,
            "global_costmap": global_costmap is not None,
            "keepout_mask": False,
            "footprint": False,
            "stop_zone": False,
            "collision_polygons": False,
            "scan": False,
            "plan": False,
            "global_inset": False,
        }

        if local_costmap is None:
            return False, {"layers": layers}, "missing local costmap"

        local_frame = local_costmap.header.frame_id or self.base_frame
        robot_in_local = self._resolve_robot_position(local_costmap, local_frame)
        if robot_in_local is None:
            return (
                False,
                {"layers": layers, "frame_id": local_frame},
                f"missing TF {self.base_frame}->{local_frame}",
            )

        center_x, center_y = robot_in_local
        window = {
            "frame_id": local_frame,
            "size_px": int(self.snapshot_size_px),
            "min_x": center_x - (self.snapshot_extent_m * 0.5),
            "max_x": center_x + (self.snapshot_extent_m * 0.5),
            "min_y": center_y - (self.snapshot_extent_m * 0.5),
            "max_y": center_y + (self.snapshot_extent_m * 0.5),
        }

        base_occ = self._sample_grid_to_window(local_costmap, window, border_value=-1.0)
        canvas = self._occupancy_to_color(base_occ)

        if keepout_mask is not None:
            keepout_cost = self._sample_grid_to_window(
                keepout_mask,
                window,
                border_value=0.0,
            )
            self._overlay_keepout(canvas, keepout_cost)
            layers["keepout_mask"] = bool(np.any(keepout_cost > 0.5))

        if local_footprint is not None:
            layers["footprint"] = self._draw_polygon_stamped(
                canvas, local_footprint, window, (0, 255, 0), 2
            )

        if stop_zone is not None:
            layers["stop_zone"] = self._draw_polygon_stamped(
                canvas, stop_zone, window, (0, 0, 255), 2
            )

        if collision_polygons is not None:
            layers["collision_polygons"] = self._draw_collision_markers(
                canvas, collision_polygons, window
            )

        if scan is not None:
            layers["scan"] = self._draw_scan(canvas, scan, window)

        if plan is not None:
            layers["plan"] = self._draw_path(canvas, plan, window, (64, 255, 64), 2)

        if global_costmap is not None:
            layers["global_inset"] = self._draw_global_inset(
                canvas, global_costmap, plan, keepout_mask
            )

        stamp = self.get_clock().now().to_msg()
        ok, png = cv2.imencode(".png", canvas)
        if not ok:
            return False, {"layers": layers, "frame_id": local_frame}, "png encode failed"

        png_bytes = png.tobytes()
        payload = {
            "mime": "image/png",
            "width": int(canvas.shape[1]),
            "height": int(canvas.shape[0]),
            "frame_id": local_frame,
            "stamp": stamp,
            "layers": layers,
            "image_png": png_bytes,
        }
        return True, payload, None

    def _resolve_robot_position(
        self, local_costmap: OccupancyGrid, local_frame: str
    ) -> Optional[Tuple[float, float]]:
        if local_frame == self.base_frame:
            return 0.0, 0.0
        if local_frame:
            tf = self._lookup_transform(local_frame, self.base_frame)
            if tf is not None:
                tx = float(tf.transform.translation.x)
                ty = float(tf.transform.translation.y)
                return tx, ty

        info = local_costmap.info
        x = float(info.origin.position.x) + (float(info.width) * float(info.resolution) * 0.5)
        y = float(info.origin.position.y) + (
            float(info.height) * float(info.resolution) * 0.5
        )
        return x, y

    def _lookup_transform(self, target_frame: str, source_frame: str) -> Optional[Any]:
        if (not target_frame) or (not source_frame):
            return None
        if target_frame == source_frame:
            return None
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        except Exception:
            return None

    def _transform_2d_from_tf(
        self, tf: Any, x: np.ndarray, y: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_out = float(t.x) + (cos_yaw * x) - (sin_yaw * y)
        y_out = float(t.y) + (sin_yaw * x) + (cos_yaw * y)
        return x_out, y_out

    def _grid_data_top_left(self, grid: OccupancyGrid) -> np.ndarray:
        arr = np.array(grid.data, dtype=np.float32)
        h = int(grid.info.height)
        w = int(grid.info.width)
        if arr.size != (h * w):
            return np.full((h, w), -1.0, dtype=np.float32)
        arr = arr.reshape((h, w))
        return np.flipud(arr)

    def _sample_grid_to_window(
        self, grid: OccupancyGrid, window: Dict[str, Any], border_value: float
    ) -> np.ndarray:
        src = self._grid_data_top_left(grid)
        h = int(grid.info.height)
        w = int(grid.info.width)
        res = float(grid.info.resolution)
        if h <= 0 or w <= 0 or res <= 0.0:
            return np.full(
                (int(window["size_px"]), int(window["size_px"])),
                border_value,
                dtype=np.float32,
            )

        size_px = int(window["size_px"])
        min_x = float(window["min_x"])
        max_x = float(window["max_x"])
        min_y = float(window["min_y"])
        max_y = float(window["max_y"])

        xs = np.linspace(min_x, max_x, size_px, dtype=np.float32)
        ys = np.linspace(max_y, min_y, size_px, dtype=np.float32)
        x_tgt, y_tgt = np.meshgrid(xs, ys)

        src_frame = grid.header.frame_id
        tgt_frame = str(window["frame_id"])
        if src_frame and tgt_frame and src_frame != tgt_frame:
            tf = self._lookup_transform(src_frame, tgt_frame)
            if tf is not None:
                x_src, y_src = self._transform_2d_from_tf(tf, x_tgt, y_tgt)
            else:
                return np.full((size_px, size_px), border_value, dtype=np.float32)
        else:
            x_src = x_tgt
            y_src = y_tgt

        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        top_y = origin_y + (float(h) * res)
        map_x = (x_src - origin_x) / res
        map_y = (top_y - y_src) / res
        return cv2.remap(
            src.astype(np.float32),
            map_x.astype(np.float32),
            map_y.astype(np.float32),
            interpolation=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=float(border_value),
        )

    def _sample_grid_to_reference(
        self, src_grid: OccupancyGrid, ref_grid: OccupancyGrid, border_value: float
    ) -> np.ndarray:
        src = self._grid_data_top_left(src_grid)
        src_h = int(src_grid.info.height)
        src_w = int(src_grid.info.width)
        src_res = float(src_grid.info.resolution)
        ref_h = int(ref_grid.info.height)
        ref_w = int(ref_grid.info.width)
        ref_res = float(ref_grid.info.resolution)

        if (
            src_h <= 0
            or src_w <= 0
            or src_res <= 0.0
            or ref_h <= 0
            or ref_w <= 0
            or ref_res <= 0.0
        ):
            return np.full((ref_h, ref_w), border_value, dtype=np.float32)

        ref_origin_x = float(ref_grid.info.origin.position.x)
        ref_origin_y = float(ref_grid.info.origin.position.y)
        cols = np.arange(ref_w, dtype=np.float32)
        rows = np.arange(ref_h, dtype=np.float32)
        xs = ref_origin_x + ((cols + 0.5) * ref_res)
        ys = ref_origin_y + ((float(ref_h) - (rows + 0.5)) * ref_res)
        x_tgt, y_tgt = np.meshgrid(xs, ys)

        src_frame = src_grid.header.frame_id
        ref_frame = ref_grid.header.frame_id
        if src_frame and ref_frame and src_frame != ref_frame:
            tf = self._lookup_transform(src_frame, ref_frame)
            if tf is not None:
                x_src, y_src = self._transform_2d_from_tf(tf, x_tgt, y_tgt)
            else:
                return np.full((ref_h, ref_w), border_value, dtype=np.float32)
        else:
            x_src = x_tgt
            y_src = y_tgt

        src_origin_x = float(src_grid.info.origin.position.x)
        src_origin_y = float(src_grid.info.origin.position.y)
        src_top_y = src_origin_y + (float(src_h) * src_res)
        map_x = ((x_src - src_origin_x) / src_res) - 0.5
        map_y = ((src_top_y - y_src) / src_res) - 0.5
        return cv2.remap(
            src.astype(np.float32),
            map_x.astype(np.float32),
            map_y.astype(np.float32),
            interpolation=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=float(border_value),
        )

    def _occupancy_to_color(self, occ: np.ndarray) -> np.ndarray:
        out = np.zeros((occ.shape[0], occ.shape[1], 3), dtype=np.uint8)
        out[:, :] = (120, 120, 120)
        known = occ >= 0.0
        occ_clip = np.clip(occ, 0.0, 100.0)
        gray = np.clip(255.0 - (occ_clip * 2.3), 0.0, 255.0).astype(np.uint8)
        out[known] = np.stack([gray, gray, gray], axis=-1)[known]
        return out

    def _overlay_keepout(self, canvas: np.ndarray, keepout_cost: np.ndarray) -> None:
        cost = np.clip(keepout_cost, 0.0, 100.0)
        if not np.any(cost > 0.0):
            return
        alpha = np.clip(cost / 100.0, 0.05, 0.70)
        overlay = canvas.copy().astype(np.float32)
        overlay[:, :, 2] = 255.0
        overlay[:, :, 1] *= (1.0 - alpha * 0.8)
        overlay[:, :, 0] *= (1.0 - alpha * 0.8)
        keep_mask = cost > 0.0
        canvas_float = canvas.astype(np.float32)
        canvas_float[keep_mask] = (
            (1.0 - alpha[keep_mask][:, None]) * canvas_float[keep_mask]
            + alpha[keep_mask][:, None] * overlay[keep_mask]
        )
        canvas[:, :] = np.clip(canvas_float, 0.0, 255.0).astype(np.uint8)

    def _world_to_px(
        self, x: float, y: float, window: Dict[str, Any]
    ) -> Optional[Tuple[int, int]]:
        min_x = float(window["min_x"])
        max_x = float(window["max_x"])
        min_y = float(window["min_y"])
        max_y = float(window["max_y"])
        size_px = int(window["size_px"])
        if x < min_x or x > max_x or y < min_y or y > max_y:
            return None
        u = int(round(((x - min_x) / (max_x - min_x)) * float(size_px - 1)))
        v = int(round(((max_y - y) / (max_y - min_y)) * float(size_px - 1)))
        return u, v

    def _world_to_px_unbounded(
        self, x: float, y: float, window: Dict[str, Any]
    ) -> Optional[Tuple[int, int]]:
        min_x = float(window["min_x"])
        max_x = float(window["max_x"])
        min_y = float(window["min_y"])
        max_y = float(window["max_y"])
        size_px = int(window["size_px"])
        if size_px <= 1:
            return None
        span_x = max_x - min_x
        span_y = max_y - min_y
        if span_x <= 0.0 or span_y <= 0.0:
            return None
        u = int(round(((x - min_x) / span_x) * float(size_px - 1)))
        v = int(round(((max_y - y) / span_y) * float(size_px - 1)))
        return u, v

    def _transform_points_2d(
        self, pts_xy: List[Tuple[float, float]], src_frame: str, tgt_frame: str
    ) -> Optional[List[Tuple[float, float]]]:
        if not pts_xy:
            return []
        if (not src_frame) or src_frame == tgt_frame:
            return pts_xy
        tf = self._lookup_transform(tgt_frame, src_frame)
        if tf is None:
            return None
        xs = np.array([p[0] for p in pts_xy], dtype=np.float32)
        ys = np.array([p[1] for p in pts_xy], dtype=np.float32)
        x_out, y_out = self._transform_2d_from_tf(tf, xs, ys)
        return list(zip(x_out.tolist(), y_out.tolist()))

    def _draw_polyline(
        self,
        canvas: np.ndarray,
        pts_xy: List[Tuple[float, float]],
        window: Dict[str, Any],
        color_bgr: Tuple[int, int, int],
        thickness: int,
        closed: bool = False,
    ) -> bool:
        px_pts: List[Tuple[int, int]] = []
        for x, y in pts_xy:
            px = self._world_to_px(float(x), float(y), window)
            if px is not None:
                px_pts.append(px)
        if len(px_pts) < 2:
            return False
        arr = np.array(px_pts, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [arr], closed, color_bgr, thickness, lineType=cv2.LINE_AA)
        return True

    def _draw_polygon_stamped(
        self,
        canvas: np.ndarray,
        poly_msg: PolygonStamped,
        window: Dict[str, Any],
        color_bgr: Tuple[int, int, int],
        thickness: int,
    ) -> bool:
        src_frame = poly_msg.header.frame_id or str(window["frame_id"])
        raw_pts = [(float(p.x), float(p.y)) for p in poly_msg.polygon.points]
        pts = self._transform_points_2d(raw_pts, src_frame, str(window["frame_id"]))
        if pts is None or len(pts) < 3:
            return False
        return self._draw_polyline(canvas, pts, window, color_bgr, thickness, closed=True)

    def _draw_collision_markers(
        self, canvas: np.ndarray, msg: MarkerArray, window: Dict[str, Any]
    ) -> bool:
        any_drawn = False
        tgt_frame = str(window["frame_id"])
        for marker in msg.markers:
            src_frame = marker.header.frame_id or tgt_frame
            b = int(np.clip(float(marker.color.b) * 255.0, 0.0, 255.0))
            g = int(np.clip(float(marker.color.g) * 255.0, 0.0, 255.0))
            r = int(np.clip(float(marker.color.r) * 255.0, 0.0, 255.0))
            color = (b, g, r) if (r + g + b) > 0 else (0, 200, 255)

            if marker.points:
                yaw = math.atan2(
                    2.0
                    * (
                        marker.pose.orientation.w * marker.pose.orientation.z
                        + marker.pose.orientation.x * marker.pose.orientation.y
                    ),
                    1.0
                    - 2.0
                    * (
                        marker.pose.orientation.y * marker.pose.orientation.y
                        + marker.pose.orientation.z * marker.pose.orientation.z
                    ),
                )
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                px = float(marker.pose.position.x)
                py = float(marker.pose.position.y)

                transformed_pts: List[Tuple[float, float]] = []
                for p in marker.points:
                    x_local = float(p.x)
                    y_local = float(p.y)
                    x_pose = px + (cos_yaw * x_local) - (sin_yaw * y_local)
                    y_pose = py + (sin_yaw * x_local) + (cos_yaw * y_local)
                    transformed_pts.append((x_pose, y_pose))

                pts = self._transform_points_2d(transformed_pts, src_frame, tgt_frame)
                if pts is None or len(pts) < 2:
                    continue

                if marker.type == Marker.LINE_LIST:
                    for idx in range(0, len(pts) - 1, 2):
                        seg = [pts[idx], pts[idx + 1]]
                        if self._draw_polyline(
                            canvas, seg, window, color, thickness=2, closed=False
                        ):
                            any_drawn = True
                else:
                    closed = marker.type == Marker.LINE_STRIP and (
                        len(pts) >= 3 and pts[0] != pts[-1]
                    )
                    if self._draw_polyline(
                        canvas, pts, window, color, thickness=2, closed=closed
                    ):
                        any_drawn = True
                continue

            center_pts = self._transform_points_2d(
                [(float(marker.pose.position.x), float(marker.pose.position.y))],
                src_frame,
                tgt_frame,
            )
            if center_pts is None or not center_pts:
                continue
            px_pt = self._world_to_px(center_pts[0][0], center_pts[0][1], window)
            if px_pt is None:
                continue
            radius_px = max(2, int(round(max(marker.scale.x, marker.scale.y) * 0.5)))
            cv2.circle(canvas, px_pt, radius_px, color, thickness=2, lineType=cv2.LINE_AA)
            any_drawn = True
        return any_drawn

    def _draw_scan(
        self, canvas: np.ndarray, scan: LaserScan, window: Dict[str, Any]
    ) -> bool:
        if scan.angle_increment == 0.0 or len(scan.ranges) == 0:
            return False
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = scan.angle_min + (
            np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
        )
        valid = np.isfinite(ranges)
        valid &= ranges >= float(scan.range_min)
        valid &= ranges <= float(scan.range_max)
        if not np.any(valid):
            return False

        rs = ranges[valid]
        th = angles[valid]
        xs = rs * np.cos(th)
        ys = rs * np.sin(th)
        pts = list(zip(xs.tolist(), ys.tolist()))
        src_frame = scan.header.frame_id or self.base_frame
        pts_tgt = self._transform_points_2d(pts, src_frame, str(window["frame_id"]))
        if pts_tgt is None:
            return False

        drawn = False
        for x, y in pts_tgt:
            px = self._world_to_px(float(x), float(y), window)
            if px is None:
                continue
            cv2.circle(canvas, px, 1, (0, 80, 255), thickness=-1, lineType=cv2.LINE_AA)
            drawn = True
        return drawn

    def _draw_path(
        self,
        canvas: np.ndarray,
        path: Path,
        window: Dict[str, Any],
        color_bgr: Tuple[int, int, int],
        thickness: int,
    ) -> bool:
        if len(path.poses) < 2:
            return False
        src_frame = path.header.frame_id or path.poses[0].header.frame_id or self.base_frame
        pts = [
            (float(ps.pose.position.x), float(ps.pose.position.y))
            for ps in path.poses
        ]
        pts_tgt = self._transform_points_2d(pts, src_frame, str(window["frame_id"]))
        if pts_tgt is None:
            return False
        return self._draw_path_segment_clipped(
            canvas, pts_tgt, window, color_bgr, thickness
        )

    def _draw_path_segment_clipped(
        self,
        canvas: np.ndarray,
        pts_xy: List[Tuple[float, float]],
        window: Dict[str, Any],
        color_bgr: Tuple[int, int, int],
        thickness: int,
    ) -> bool:
        if len(pts_xy) < 2:
            return False

        size_px = int(window["size_px"])
        if size_px <= 1:
            return False
        clip_rect = (0, 0, size_px, size_px)

        drawn = False
        for idx in range(len(pts_xy) - 1):
            x0, y0 = pts_xy[idx]
            x1, y1 = pts_xy[idx + 1]
            if not (
                np.isfinite(x0)
                and np.isfinite(y0)
                and np.isfinite(x1)
                and np.isfinite(y1)
            ):
                continue

            p0 = self._world_to_px_unbounded(float(x0), float(y0), window)
            p1 = self._world_to_px_unbounded(float(x1), float(y1), window)
            if p0 is None or p1 is None:
                continue

            ok, c0, c1 = cv2.clipLine(clip_rect, p0, p1)
            if not ok:
                continue

            cv2.line(
                canvas,
                c0,
                c1,
                color_bgr,
                thickness=thickness,
                lineType=cv2.LINE_AA,
            )
            drawn = True

        return drawn

    def _draw_global_inset(
        self,
        canvas: np.ndarray,
        global_costmap: OccupancyGrid,
        plan: Optional[Path],
        keepout_mask: Optional[OccupancyGrid],
    ) -> bool:
        inset_px = int(self.snapshot_global_inset_px)
        if inset_px <= 0:
            return False

        global_occ = self._grid_data_top_left(global_costmap)
        global_img = self._occupancy_to_color(global_occ)

        if keepout_mask is not None:
            keep = self._sample_grid_to_reference(
                keepout_mask, global_costmap, border_value=0.0
            )
            if keep.shape == global_occ.shape:
                self._overlay_keepout(global_img, keep)

        if plan is not None:
            inset_window = {
                "frame_id": global_costmap.header.frame_id or self.base_frame,
                "size_px": int(global_costmap.info.width),
                "min_x": float(global_costmap.info.origin.position.x),
                "max_x": float(global_costmap.info.origin.position.x)
                + float(global_costmap.info.width) * float(global_costmap.info.resolution),
                "min_y": float(global_costmap.info.origin.position.y),
                "max_y": float(global_costmap.info.origin.position.y)
                + float(global_costmap.info.height) * float(global_costmap.info.resolution),
            }
            self._draw_path(global_img, plan, inset_window, (96, 255, 96), 2)

        robot_tf = self._lookup_transform(
            global_costmap.header.frame_id or self.base_frame, self.base_frame
        )
        if robot_tf is not None:
            rx = float(robot_tf.transform.translation.x)
            ry = float(robot_tf.transform.translation.y)
            px = self._grid_world_to_pixel(global_costmap, rx, ry)
            if px is not None:
                cv2.circle(global_img, px, 4, (255, 0, 0), thickness=-1, lineType=cv2.LINE_AA)

        inset = cv2.resize(global_img, (inset_px, inset_px), interpolation=cv2.INTER_AREA)
        margin = 10
        y0 = margin
        x0 = canvas.shape[1] - inset_px - margin
        if x0 < 0 or y0 < 0:
            return False
        canvas[y0:y0 + inset_px, x0:x0 + inset_px] = inset
        cv2.rectangle(
            canvas,
            (x0, y0),
            (x0 + inset_px, y0 + inset_px),
            (180, 180, 180),
            thickness=1,
            lineType=cv2.LINE_AA,
        )
        return True

    def _grid_world_to_pixel(
        self, grid: OccupancyGrid, x: float, y: float
    ) -> Optional[Tuple[int, int]]:
        w = int(grid.info.width)
        h = int(grid.info.height)
        if w <= 0 or h <= 0:
            return None
        res = float(grid.info.resolution)
        if res <= 0.0:
            return None
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        col = int(math.floor((x - origin_x) / res))
        row = int(math.floor((y - origin_y) / res))
        if col < 0 or row < 0 or col >= w or row >= h:
            return None
        img_row = h - 1 - row
        return col, img_row


def main() -> None:
    rclpy.init()
    node = NavSnapshotServerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
