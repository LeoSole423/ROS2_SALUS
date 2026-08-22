import copy
import json
import os
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped, Pose, Quaternion
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import MapMetaData
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from robot_localization.srv import FromLL
from std_srvs.srv import Trigger
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener

from interfaces.srv import GetZonesState, SetZonesGeoJson

from .keepout_mask_utils import exponential_gradient_from_core
from .zones_geojson_utils import (
    feature_and_polygon_counts,
    iter_polygons,
    normalize_geojson_object,
    rasterize_polygons_trinary,
)


def compose_keepout_cost_mask(
    core_binary_mask: np.ndarray,
    resolution: float,
    degrade_enabled: bool,
    degrade_radius_m: float,
    degrade_edge_cost: int,
    degrade_min_cost: int,
    degrade_use_l2: bool,
) -> np.ndarray:
    core_mask = np.where(core_binary_mask > 0, 100, 0).astype(np.uint8)
    if (not degrade_enabled) or degrade_radius_m <= 0.0:
        return core_mask

    gradient_mask = exponential_gradient_from_core(
        core_mask=core_mask,
        resolution=resolution,
        radius_m=degrade_radius_m,
        edge_cost=degrade_edge_cost,
        min_cost=degrade_min_cost,
        use_l2=degrade_use_l2,
    )
    cost_mask = np.maximum(core_mask, gradient_mask)
    cost_mask[core_mask > 0] = 100
    return cost_mask


def cost_mask_to_scale_image(cost_mask: np.ndarray) -> np.ndarray:
    bounded = np.clip(cost_mask.astype(np.float32), 0.0, 100.0)
    return np.rint((100.0 - bounded) * (255.0 / 100.0)).astype(np.uint8)


def summarize_keepout_cost_mask(cost_mask: np.ndarray) -> Dict[str, int]:
    core_cells = int(np.sum(cost_mask == 100))
    halo_cells = int(np.sum((cost_mask > 0) & (cost_mask < 100)))
    free_cells = int(np.sum(cost_mask == 0))
    return {
        "core_cells": core_cells,
        "halo_cells": halo_cells,
        "free_cells": free_cells,
    }


def build_scale_mask_yaml_data(
    image_ref: str,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> Dict[str, Any]:
    return {
        "image": image_ref,
        "mode": "scale",
        "resolution": float(resolution),
        "origin": [float(origin_x), float(origin_y), 0.0],
        "negate": 0,
        "occupied_thresh": 1.0,
        "free_thresh": 0.0,
    }


class ZonesManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("zones_manager")

        self.declare_parameter("fromll_service", "/fromLL")
        self.declare_parameter("fromll_service_fallback", "/navsat_transform/fromLL")
        self.declare_parameter("fromll_wait_timeout_s", 2.0)
        self.declare_parameter("fromll_call_retries", 4)
        self.declare_parameter("fromll_retry_delay_s", 0.15)
        self.declare_parameter("fromll_frame", "odom")

        self.declare_parameter("load_map_service", "/keepout_filter_mask_server/load_map")
        self.declare_parameter("load_map_wait_timeout_s", 8.0)
        self.declare_parameter("clear_global_after_reload", True)
        self.declare_parameter(
            "clear_global_costmap_service", "/global_costmap/clear_entirely_global_costmap"
        )
        self.declare_parameter("clear_global_costmap_wait_timeout_s", 2.0)
        self.declare_parameter("set_geojson_service", "/zones_manager/set_geojson")
        self.declare_parameter("get_state_service", "/zones_manager/get_state")
        self.declare_parameter(
            "reload_from_disk_service", "/zones_manager/reload_from_disk"
        )

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("tf_lookup_timeout_s", 0.5)
        self.declare_parameter("buffer_margin_m", 0.0)
        self.declare_parameter("degrade_enabled", True)
        self.declare_parameter("degrade_radius_m", 1.5)
        self.declare_parameter("degrade_edge_cost", 12)
        self.declare_parameter("degrade_min_cost", 1)
        self.declare_parameter("degrade_use_l2", True)
        self.declare_parameter("geojson_file", "")
        self.declare_parameter("mask_image_file", "")
        self.declare_parameter("mask_yaml_file", "")

        self.declare_parameter("mask_origin_mode", "explicit")
        self.declare_parameter("mask_origin_x", -150.0)
        self.declare_parameter("mask_origin_y", -150.0)
        self.declare_parameter("mask_width", 3000)
        self.declare_parameter("mask_height", 3000)
        self.declare_parameter("mask_resolution", 0.1)

        self.fromll_service = str(self.get_parameter("fromll_service").value)
        self.fromll_service_fallback = str(
            self.get_parameter("fromll_service_fallback").value
        )
        self.fromll_wait_timeout_s = max(
            0.1, float(self.get_parameter("fromll_wait_timeout_s").value)
        )
        self.fromll_call_retries = max(
            1, int(self.get_parameter("fromll_call_retries").value)
        )
        self.fromll_retry_delay_s = max(
            0.0, float(self.get_parameter("fromll_retry_delay_s").value)
        )
        self.fromll_frame = str(self.get_parameter("fromll_frame").value).strip() or "odom"

        self.load_map_service = str(self.get_parameter("load_map_service").value)
        self.load_map_wait_timeout_s = max(
            0.5, float(self.get_parameter("load_map_wait_timeout_s").value)
        )
        self.clear_global_after_reload = bool(
            self.get_parameter("clear_global_after_reload").value
        )
        self.clear_global_costmap_service = str(
            self.get_parameter("clear_global_costmap_service").value
        )
        self.clear_global_costmap_wait_timeout_s = max(
            0.2, float(self.get_parameter("clear_global_costmap_wait_timeout_s").value)
        )
        self.set_geojson_service = str(self.get_parameter("set_geojson_service").value)
        self.get_state_service = str(self.get_parameter("get_state_service").value)
        self.reload_from_disk_service = str(
            self.get_parameter("reload_from_disk_service").value
        )

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.tf_lookup_timeout_s = max(
            0.05, float(self.get_parameter("tf_lookup_timeout_s").value)
        )
        self.buffer_margin_m = max(0.0, float(self.get_parameter("buffer_margin_m").value))
        self.degrade_enabled = bool(self.get_parameter("degrade_enabled").value)
        self.degrade_radius_m = float(self.get_parameter("degrade_radius_m").value)
        self.degrade_edge_cost = int(self.get_parameter("degrade_edge_cost").value)
        self.degrade_min_cost = int(self.get_parameter("degrade_min_cost").value)
        self.degrade_use_l2 = bool(self.get_parameter("degrade_use_l2").value)

        self.mask_origin_mode = str(self.get_parameter("mask_origin_mode").value).strip().lower()
        self.mask_origin_x = float(self.get_parameter("mask_origin_x").value)
        self.mask_origin_y = float(self.get_parameter("mask_origin_y").value)
        self.mask_width = int(self.get_parameter("mask_width").value)
        self.mask_height = int(self.get_parameter("mask_height").value)
        self.mask_resolution = float(self.get_parameter("mask_resolution").value)
        self._sanitize_degrade_params()
        self._sanitize_mask_grid_params()

        default_dir = self._resolve_default_config_dir()
        configured_geojson = str(self.get_parameter("geojson_file").value)
        configured_mask_image = str(self.get_parameter("mask_image_file").value)
        configured_mask_yaml = str(self.get_parameter("mask_yaml_file").value)

        self.geojson_file = (
            Path(configured_geojson)
            if configured_geojson
            else default_dir / "no_go_zones.geojson"
        )
        self.mask_image_file = (
            Path(configured_mask_image)
            if configured_mask_image
            else default_dir / "keepout_mask.pgm"
        )
        self.mask_yaml_file = (
            Path(configured_mask_yaml)
            if configured_mask_yaml
            else default_dir / "keepout_mask.yaml"
        )

        self._lock = threading.Lock()
        self._geojson_doc = self._empty_geojson_doc()
        self._geojson_text = json.dumps(self._geojson_doc, separators=(",", ":"))
        self._mask_ready = False
        self._mask_source = "none"

        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        self._fromll_client = self.create_client(
            FromLL, self.fromll_service, callback_group=self._client_group
        )
        self._fromll_fallback_client = None
        if self.fromll_service_fallback and (
            self.fromll_service_fallback != self.fromll_service
        ):
            self._fromll_fallback_client = self.create_client(
                FromLL,
                self.fromll_service_fallback,
                callback_group=self._client_group,
            )
        self._active_fromll_name: Optional[str] = None
        self._active_fromll_client: Optional[Any] = None
        self._last_fromll_error: Optional[str] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._load_map_client = self.create_client(
            LoadMap,
            self.load_map_service,
            callback_group=self._client_group,
        )
        self._clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.clear_global_costmap_service,
            callback_group=self._client_group,
        )

        self._set_geojson_srv = self.create_service(
            SetZonesGeoJson,
            self.set_geojson_service,
            self._on_set_geojson,
            callback_group=self._service_group,
        )
        self._get_state_srv = self.create_service(
            GetZonesState,
            self.get_state_service,
            self._on_get_state,
            callback_group=self._service_group,
        )
        self._reload_srv = self.create_service(
            Trigger,
            self.reload_from_disk_service,
            self._on_reload_from_disk,
            callback_group=self._service_group,
        )

        self._load_initial_state()
        self.get_logger().info(
            "Zones manager ready "
            f"(set_service={self.set_geojson_service}, get_service={self.get_state_service}, "
            f"reload_service={self.reload_from_disk_service}, "
            f"load_map_service={self.load_map_service})"
        )
        self.get_logger().info(
            "Global costmap clear config "
            f"(enabled={self.clear_global_after_reload}, "
            f"service={self.clear_global_costmap_service}, "
            f"timeout_s={self.clear_global_costmap_wait_timeout_s:.2f})"
        )
        self.get_logger().info(
            "Keepout degrade config "
            f"(enabled={self.degrade_enabled}, radius_m={self.degrade_radius_m}, "
            f"edge_cost={self.degrade_edge_cost}, min_cost={self.degrade_min_cost}, "
            f"use_l2={self.degrade_use_l2})"
        )

    def _empty_geojson_doc(self) -> Dict[str, Any]:
        return {"type": "FeatureCollection", "features": []}

    def _resolve_default_config_dir(self) -> Path:
        pkg_dir = Path(get_package_share_directory("navegacion_gps"))
        default_dir = pkg_dir / "config"
        try:
            workspace_root = pkg_dir.parents[3]
            source_dir = workspace_root / "src" / "navegacion_gps" / "config"
            if source_dir.exists():
                return source_dir
        except Exception:
            pass
        return default_dir

    def _sanitize_mask_grid_params(self) -> None:
        if self.mask_origin_mode != "explicit":
            self.get_logger().warning(
                f"mask_origin_mode='{self.mask_origin_mode}' unsupported; forcing 'explicit'"
            )
            self.mask_origin_mode = "explicit"
        if self.mask_width <= 0:
            self.get_logger().warning(f"mask_width={self.mask_width} invalid; forcing 3000")
            self.mask_width = 3000
        if self.mask_height <= 0:
            self.get_logger().warning(f"mask_height={self.mask_height} invalid; forcing 3000")
            self.mask_height = 3000
        if self.mask_resolution <= 0.0 or not np.isfinite(self.mask_resolution):
            self.get_logger().warning(
                f"mask_resolution={self.mask_resolution} invalid; forcing 0.1"
            )
            self.mask_resolution = 0.1

    def _effective_mask_origin(self) -> Tuple[float, float]:
        return (float(self.mask_origin_x), float(self.mask_origin_y))

    def _sanitize_degrade_params(self) -> None:
        if self.degrade_radius_m < 0.0:
            self.get_logger().warning(
                f"degrade_radius_m={self.degrade_radius_m} invalid; forcing 0.0"
            )
            self.degrade_radius_m = 0.0

        if self.degrade_edge_cost < 1 or self.degrade_edge_cost > 99:
            self.get_logger().warning(
                f"degrade_edge_cost={self.degrade_edge_cost} out of range [1,99]; clamping"
            )
            self.degrade_edge_cost = max(1, min(99, self.degrade_edge_cost))

        if self.degrade_min_cost < 1 or self.degrade_min_cost > 99:
            self.get_logger().warning(
                f"degrade_min_cost={self.degrade_min_cost} out of range [1,99]; clamping"
            )
            self.degrade_min_cost = max(1, min(99, self.degrade_min_cost))

        if self.degrade_min_cost > self.degrade_edge_cost:
            self.get_logger().warning(
                "degrade_min_cost is greater than degrade_edge_cost; forcing equality"
            )
            self.degrade_min_cost = self.degrade_edge_cost

    def _build_fixed_mask_metadata(self) -> MapMetaData:
        info = MapMetaData()
        info.map_load_time = self.get_clock().now().to_msg()
        info.resolution = float(self.mask_resolution)
        info.width = int(self.mask_width)
        info.height = int(self.mask_height)
        effective_origin_x, effective_origin_y = self._effective_mask_origin()

        origin = Pose()
        origin.position.x = float(effective_origin_x)
        origin.position.y = float(effective_origin_y)
        origin.position.z = 0.0
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        info.origin = origin
        return info

    def _wait_for_future(self, future: Any, timeout_sec: float) -> Optional[Any]:
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                return future.result()
            if (time.monotonic() - start) >= timeout_sec:
                return None
            time.sleep(0.01)
        return None

    def _maybe_log_active_fromll(self, service_name: str) -> None:
        if self._active_fromll_name == service_name:
            return
        self._active_fromll_name = service_name
        self.get_logger().info(f"Using fromLL service: {service_name}")

    def _resolve_fromll_client(self) -> Optional[Any]:
        candidates: List[Tuple[Any, str, float]] = []
        if self._active_fromll_client is not None and self._active_fromll_name is not None:
            candidates.append((self._active_fromll_client, self._active_fromll_name, 0.05))

        candidates.append((self._fromll_client, self.fromll_service, self.fromll_wait_timeout_s))
        if self._fromll_fallback_client is not None:
            candidates.append(
                (
                    self._fromll_fallback_client,
                    self.fromll_service_fallback,
                    self.fromll_wait_timeout_s,
                )
            )

        seen = set()
        for client, service_name, wait_s in candidates:
            key = (id(client), service_name)
            if key in seen:
                continue
            seen.add(key)
            if client.wait_for_service(timeout_sec=wait_s):
                self._active_fromll_client = client
                self._maybe_log_active_fromll(service_name)
                return client

        self._last_fromll_error = "fromLL service unavailable"
        self.get_logger().warning(
            "fromLL service unavailable "
            f"(tried '{self.fromll_service}'"
            + (
                f" and '{self.fromll_service_fallback}'"
                if self._fromll_fallback_client is not None
                else ""
            )
            + ")"
        )
        return None

    def _call_from_ll(self, lat: float, lon: float) -> Optional[Tuple[float, float]]:
        for attempt in range(self.fromll_call_retries):
            fromll_client = self._resolve_fromll_client()
            if fromll_client is None:
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue

            req = FromLL.Request()
            req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
            future = fromll_client.call_async(req)
            try:
                res = self._wait_for_future(future, timeout_sec=2.5)
            except Exception as exc:
                self._last_fromll_error = str(exc)
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue
            if res is None:
                self._last_fromll_error = "timeout waiting fromLL response"
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue

            transformed = self._transform_point_to_map(
                float(res.map_point.x), float(res.map_point.y)
            )
            if transformed is None:
                if attempt + 1 < self.fromll_call_retries and self.fromll_retry_delay_s > 0.0:
                    time.sleep(self.fromll_retry_delay_s)
                continue

            self._last_fromll_error = None
            return transformed

        return None

    def _transform_point_to_map(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        if self.fromll_frame == self.map_frame:
            return float(x), float(y)

        point = PointStamped()
        point.header.frame_id = self.fromll_frame
        point.header.stamp = Time().to_msg()
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = 0.0
        try:
            transformed = self._tf_buffer.transform(
                point,
                self.map_frame,
                timeout=Duration(seconds=self.tf_lookup_timeout_s),
            )
        except TransformException as exc:
            self._last_fromll_error = (
                f"tf transform failed ({self.fromll_frame}->{self.map_frame}): {exc}"
            )
            return None
        return float(transformed.point.x), float(transformed.point.y)

    def _parse_geojson_text(self, raw_text: str) -> Tuple[Optional[Dict[str, Any]], str]:
        try:
            raw = json.loads(raw_text)
        except Exception as exc:
            return None, f"invalid json: {exc}"
        try:
            normalized = normalize_geojson_object(raw)
        except Exception as exc:
            return None, f"invalid geojson: {exc}"
        return normalized, ""

    def _convert_geojson_to_xy(
        self, geojson_doc: Dict[str, Any]
    ) -> Tuple[List[Dict[str, Any]], List[str], int]:
        xy_polygons: List[Dict[str, Any]] = []
        failed_polygon_ids: List[str] = []
        enabled_polygon_count = 0
        cache: Dict[Tuple[float, float], Tuple[float, float]] = {}

        for polygon in iter_polygons(geojson_doc):
            if polygon.get("enabled", True) is False:
                continue
            enabled_polygon_count += 1

            polygon_id = str(polygon.get("id", ""))
            outer_ll = polygon.get("outer_ll", [])
            holes_ll = polygon.get("holes_ll", [])
            if len(outer_ll) < 4:
                failed_polygon_ids.append(polygon_id)
                continue

            failed = False
            outer_xy: List[Dict[str, float]] = []
            holes_xy: List[List[Dict[str, float]]] = []

            for lon, lat in outer_ll:
                key = (float(lat), float(lon))
                converted = cache.get(key)
                if converted is None:
                    converted = self._call_from_ll(float(lat), float(lon))
                    if converted is not None:
                        cache[key] = converted
                if converted is None:
                    failed = True
                    break
                x, y = converted
                outer_xy.append({"x": x, "y": y})

            if failed:
                failed_polygon_ids.append(polygon_id)
                continue

            for hole_ll in holes_ll:
                if len(hole_ll) < 4:
                    continue
                hole_xy: List[Dict[str, float]] = []
                for lon, lat in hole_ll:
                    key = (float(lat), float(lon))
                    converted = cache.get(key)
                    if converted is None:
                        converted = self._call_from_ll(float(lat), float(lon))
                        if converted is not None:
                            cache[key] = converted
                    if converted is None:
                        failed = True
                        break
                    x, y = converted
                    hole_xy.append({"x": x, "y": y})
                if failed:
                    break
                if len(hole_xy) >= 4:
                    holes_xy.append(hole_xy)

            if failed:
                failed_polygon_ids.append(polygon_id)
                continue

            xy_polygons.append(
                {
                    "id": polygon_id,
                    "enabled": True,
                    "outer_xy": outer_xy,
                    "holes_xy": holes_xy,
                }
            )

        return xy_polygons, failed_polygon_ids, enabled_polygon_count

    def _write_mask_files(self, image: np.ndarray) -> Tuple[bool, str]:
        try:
            self.mask_image_file.parent.mkdir(parents=True, exist_ok=True)
            self.mask_yaml_file.parent.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            return False, f"failed creating output dirs: {exc}"

        # map_server puede abrir el PGM durante el arranque al mismo tiempo que
        # zones_manager regenera la mascara inicial. Escribir directo al destino
        # deja durante unos milisegundos un PGM truncado y Magick falla con
        # ``Unexpected end-of-file``. El temporal vive en el mismo directorio,
        # asi que ``replace`` es atomico: el lector ve el archivo viejo completo
        # o el nuevo completo, nunca uno a medio escribir.
        image_fd, image_temp_name = tempfile.mkstemp(
            prefix=f".{self.mask_image_file.stem}.",
            suffix=self.mask_image_file.suffix,
            dir=str(self.mask_image_file.parent),
        )
        os.close(image_fd)
        image_temp = Path(image_temp_name)
        try:
            if not cv2.imwrite(str(image_temp), image):
                return False, f"failed writing image file: {self.mask_image_file}"
            image_temp.replace(self.mask_image_file)
        except Exception as exc:
            return False, f"failed writing image file: {exc}"
        finally:
            image_temp.unlink(missing_ok=True)

        try:
            image_ref = str(self.mask_image_file.relative_to(self.mask_yaml_file.parent))
        except Exception:
            image_ref = str(self.mask_image_file)
        effective_origin_x, effective_origin_y = self._effective_mask_origin()
        yaml_data = build_scale_mask_yaml_data(
            image_ref=image_ref,
            resolution=float(self.mask_resolution),
            origin_x=float(effective_origin_x),
            origin_y=float(effective_origin_y),
        )
        yaml_fd, yaml_temp_name = tempfile.mkstemp(
            prefix=f".{self.mask_yaml_file.stem}.",
            suffix=self.mask_yaml_file.suffix,
            dir=str(self.mask_yaml_file.parent),
        )
        os.close(yaml_fd)
        yaml_temp = Path(yaml_temp_name)
        try:
            with yaml_temp.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(yaml_data, handle, sort_keys=False)
            yaml_temp.replace(self.mask_yaml_file)
        except Exception as exc:
            return False, f"failed writing yaml file: {exc}"
        finally:
            yaml_temp.unlink(missing_ok=True)

        return True, ""

    def _call_load_map(self) -> Tuple[bool, str]:
        if not self._load_map_client.wait_for_service(
            timeout_sec=float(self.load_map_wait_timeout_s)
        ):
            return False, f"service unavailable: {self.load_map_service}"

        req = LoadMap.Request()
        # nav2_map_server in Humble accepts filesystem path here; using file:// URI
        # can be rejected by YAML::LoadFile ("bad file: file:///...").
        req.map_url = str(self.mask_yaml_file.resolve())
        future = self._load_map_client.call_async(req)
        res = self._wait_for_future(future, timeout_sec=3.0)
        if res is None:
            return False, f"timeout calling {self.load_map_service}"

        success_code = getattr(LoadMap.Response, "RESULT_SUCCESS", 0)
        if int(res.result) != int(success_code):
            return False, f"load_map returned result={int(res.result)}"
        return True, ""

    def _call_clear_global_costmap(self) -> Tuple[bool, str]:
        if not self.clear_global_after_reload:
            return True, ""
        if not self._clear_global_costmap_client.wait_for_service(
            timeout_sec=float(self.clear_global_costmap_wait_timeout_s)
        ):
            return False, f"service unavailable: {self.clear_global_costmap_service}"

        req = ClearEntireCostmap.Request()
        future = self._clear_global_costmap_client.call_async(req)
        res = self._wait_for_future(
            future, timeout_sec=float(self.clear_global_costmap_wait_timeout_s)
        )
        if res is None:
            return False, f"timeout calling {self.clear_global_costmap_service}"
        return True, ""

    def _save_geojson_to_disk(self, geojson_doc: Dict[str, Any]) -> Tuple[bool, str]:
        try:
            self.geojson_file.parent.mkdir(parents=True, exist_ok=True)
            with self.geojson_file.open("w", encoding="utf-8") as handle:
                json.dump(geojson_doc, handle, ensure_ascii=True, indent=2)
                handle.write("\n")
            return True, ""
        except Exception as exc:
            return False, f"failed writing geojson file: {exc}"

    def _apply_geojson(
        self,
        geojson_doc: Dict[str, Any],
        persist_geojson: bool,
    ) -> Tuple[bool, str, bool, int, int]:
        feature_count, polygon_count = feature_and_polygon_counts(geojson_doc)
        xy_polygons, failed_polygon_ids, enabled_polygon_count = self._convert_geojson_to_xy(
            geojson_doc
        )
        if enabled_polygon_count > 0 and len(xy_polygons) == 0:
            detail = self._last_fromll_error or "fromLL conversion failed"
            return (
                False,
                f"no valid polygons after fromLL conversion: {detail}",
                False,
                feature_count,
                polygon_count,
            )
        if failed_polygon_ids:
            self.get_logger().warning(
                "Some polygons failed LL->XY conversion: " + ", ".join(failed_polygon_ids)
            )

        info = self._build_fixed_mask_metadata()
        image, clipped_vertices, outside_polygon_ids = rasterize_polygons_trinary(
            polygons_xy=xy_polygons,
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            buffer_margin_m=float(self.buffer_margin_m),
        )
        for polygon_id, clipped_count in clipped_vertices.items():
            self.get_logger().warning(
                f"Polygon '{polygon_id}' clipped {clipped_count} vertices to mask bounds"
            )
        for polygon_id in outside_polygon_ids:
            self.get_logger().warning(
                f"Polygon '{polygon_id}' is outside mask bounds and was skipped"
            )

        core_binary_mask = np.where(image == 0, 1, 0).astype(np.uint8)
        cost_mask = compose_keepout_cost_mask(
            core_binary_mask=core_binary_mask,
            resolution=float(info.resolution),
            degrade_enabled=bool(self.degrade_enabled),
            degrade_radius_m=float(self.degrade_radius_m),
            degrade_edge_cost=int(self.degrade_edge_cost),
            degrade_min_cost=int(self.degrade_min_cost),
            degrade_use_l2=bool(self.degrade_use_l2),
        )
        mask_summary = summarize_keepout_cost_mask(cost_mask)
        scale_image = cost_mask_to_scale_image(cost_mask)
        ok_write, err_write = self._write_mask_files(scale_image)
        if not ok_write:
            return False, err_write, False, feature_count, polygon_count
        self.get_logger().info(
            "Keepout mask regenerated "
            f"(degrade_enabled={self.degrade_enabled}, radius_m={self.degrade_radius_m:.2f}, "
            f"core_cells={mask_summary['core_cells']}, halo_cells={mask_summary['halo_cells']}, "
            f"free_cells={mask_summary['free_cells']})"
        )

        map_reloaded = False
        global_cleared = False
        ok_reload, err_reload = self._call_load_map()
        if ok_reload:
            map_reloaded = True
            ok_clear, err_clear = self._call_clear_global_costmap()
            if ok_clear:
                global_cleared = True
            else:
                self.get_logger().warning(
                    "global costmap clear failed after load_map: " + str(err_clear)
                )
        else:
            self.get_logger().warning(f"load_map failed: {err_reload}")

        if persist_geojson:
            ok_save, err_save = self._save_geojson_to_disk(geojson_doc)
            if not ok_save:
                return False, err_save, map_reloaded, feature_count, polygon_count

        geojson_text = json.dumps(geojson_doc, ensure_ascii=True, separators=(",", ":"))
        with self._lock:
            self._geojson_doc = copy.deepcopy(geojson_doc)
            self._geojson_text = geojson_text
            applied = bool(map_reloaded and global_cleared)
            self._mask_ready = applied
            if applied and self.clear_global_after_reload:
                self._mask_source = "map_server_load_map+global_costmap_clear"
            elif map_reloaded:
                self._mask_source = "map_server_load_map"
            else:
                self._mask_source = "mask_files_only"

        if not map_reloaded:
            return (
                False,
                "mask files written but load_map failed",
                False,
                feature_count,
                polygon_count,
            )
        if self.clear_global_after_reload and not global_cleared:
            return (
                False,
                "mask loaded but global costmap clear failed",
                False,
                feature_count,
                polygon_count,
            )
        return True, "", True, feature_count, polygon_count

    def _load_geojson_from_disk(self) -> Tuple[Optional[Dict[str, Any]], str]:
        if not self.geojson_file.exists():
            return self._empty_geojson_doc(), ""

        try:
            with self.geojson_file.open("r", encoding="utf-8") as handle:
                raw_text = handle.read()
        except Exception as exc:
            return None, f"failed reading geojson file: {exc}"

        return self._parse_geojson_text(raw_text)

    def _load_initial_state(self) -> None:
        geojson_doc, err = self._load_geojson_from_disk()
        if geojson_doc is None:
            self.get_logger().warning(err)
            geojson_doc = self._empty_geojson_doc()

        persist = not self.geojson_file.exists()
        ok, apply_err, reloaded, feature_count, polygon_count = self._apply_geojson(
            geojson_doc, persist_geojson=persist
        )
        if ok:
            self.get_logger().info(
                "Initial zones loaded "
                f"(features={feature_count}, polygons={polygon_count}, map_reloaded={reloaded})"
            )
        else:
            self.get_logger().warning(f"Initial zones load failed: {apply_err}")

        self.get_logger().info(f"GeoJSON file path: {self.geojson_file}")
        self.get_logger().info(f"Mask image path: {self.mask_image_file}")
        self.get_logger().info(f"Mask yaml path: {self.mask_yaml_file}")

    def _on_set_geojson(
        self,
        request: SetZonesGeoJson.Request,
        response: SetZonesGeoJson.Response,
    ) -> SetZonesGeoJson.Response:
        parsed_doc, err = self._parse_geojson_text(str(request.geojson))
        if parsed_doc is None:
            response.ok = False
            response.error = err
            response.map_reloaded = False
            response.feature_count = 0
            response.polygon_count = 0
            return response

        ok, apply_err, reloaded, feature_count, polygon_count = self._apply_geojson(
            parsed_doc, persist_geojson=True
        )
        response.ok = bool(ok)
        response.error = "" if ok else str(apply_err)
        response.map_reloaded = bool(reloaded)
        response.feature_count = int(feature_count)
        response.polygon_count = int(polygon_count)
        return response

    def _on_get_state(
        self,
        _request: GetZonesState.Request,
        response: GetZonesState.Response,
    ) -> GetZonesState.Response:
        with self._lock:
            response.ok = True
            response.error = ""
            response.frame_id = str(self.map_frame)
            response.mask_ready = bool(self._mask_ready)
            response.mask_source = str(self._mask_source)
            response.geojson = str(self._geojson_text)
        return response

    def _on_reload_from_disk(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        geojson_doc, err = self._load_geojson_from_disk()
        if geojson_doc is None:
            response.success = False
            response.message = str(err)
            return response

        ok, apply_err, reloaded, feature_count, polygon_count = self._apply_geojson(
            geojson_doc, persist_geojson=False
        )
        response.success = bool(ok)
        if ok:
            response.message = (
                f"reloaded (features={feature_count}, polygons={polygon_count}, "
                f"map_reloaded={reloaded})"
            )
        else:
            response.message = str(apply_err)
        return response


def main() -> None:
    rclpy.init()
    node = ZonesManagerNode()
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
