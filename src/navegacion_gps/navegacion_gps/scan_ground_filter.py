"""Filtro de segmentacion de suelo estilo Autoware `scan_ground_filter`.

Port directo del algoritmo *non-grid* (ring-based) de
`autoware_ground_segmentation::ScanGroundFilterComponent::classifyPointCloud`
(autoware.universe v0.51.0, Apache-2.0). Se portó solo el algoritmo a un nodo
Python propio para evitar compilar todo Autoware (lanelet2/pointcloud_preprocessor),
manteniendo la misma lógica de clasificación y los mismos parámetros.

Interfaz limpia, igual que el componente original:
    PointCloud2 (`/scan_3d`)  ->  PointCloud2 sin suelo (`/scan_3d/no_ground`)

El suelo se elimina en 3D *antes* de aplanar a 2D con `pointcloud_to_laserscan`,
de modo que su `min_height` puede volver a bajar a ~0.10 m y recuperar obstáculos
bajos sin reintroducir puntos fantasma del piso.

Nota: el algoritmo asume que la nube está en un frame nivelado (z = altura sobre
el suelo). Como el RS16 va montado con pitch, la nube se transforma al
`target_frame` (por defecto `base_footprint`) con TF antes de clasificar.
"""

import math
import threading
from dataclasses import dataclass, replace
from enum import IntEnum
from typing import List, Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class PointLabel(IntEnum):
    INIT = 0
    GROUND = 1
    NON_GROUND = 2
    POINT_FOLLOW = 3
    UNKNOWN = 4
    VIRTUAL_GROUND = 5
    OUT_OF_RANGE = 6


@dataclass(frozen=True)
class ScanGroundFilterConfig:
    """Mismos parámetros y defaults que scan_ground_filter.param.yaml (RS16)."""

    # umbrales comunes
    global_slope_max_angle_deg: float = 10.0
    local_slope_max_angle_deg: float = 13.0
    radial_divider_angle_deg: float = 1.0
    split_points_distance_tolerance: float = 0.2
    # modo non-grid
    use_virtual_ground_point: bool = True
    split_height_distance: float = 0.2
    # punto de suelo virtual: centro de las ruedas delanteras
    vehicle_wheel_base_m: float = 0.90
    # recorte de entrada (no parte del algoritmo Autoware; evita iterar ruido lejano)
    range_max: float = 20.0


def _normalize_radian(angle: float, min_value: float = 0.0) -> float:
    """Equivalente a autoware_utils::normalize_radian: lleva a [min, min+2π)."""
    value = math.fmod(angle - min_value, 2.0 * math.pi)
    if value < 0.0:
        value += 2.0 * math.pi
    return value + min_value


class ScanGroundSegmenter:
    """Núcleo del algoritmo, sin ROS, para poder testearlo de forma aislada."""

    def __init__(self, config: ScanGroundFilterConfig) -> None:
        self.config = config
        self.radial_divider_angle_rad = math.radians(config.radial_divider_angle_deg)
        self.radial_dividers_num = int(
            math.ceil(2.0 * math.pi / self.radial_divider_angle_rad)
        )
        self.global_slope_max_ratio = math.tan(
            math.radians(config.global_slope_max_angle_deg)
        )
        self.local_slope_max_angle_rad = math.radians(config.local_slope_max_angle_deg)
        # calcVirtualGroundOrigin: x = wheel_base, y = 0, z = 0
        self.virtual_ground_x = config.vehicle_wheel_base_m

    def segment(self, points_xyz: np.ndarray) -> np.ndarray:
        """Devuelve los índices (sobre `points_xyz`) clasificados como NO-suelo.

        `points_xyz` es un array (N, 3) en un frame nivelado (z = altura).

        El binning por azimut y el orden por radio se hacen vectorizados; el lazo
        de clasificación (dependiente entre puntos del rayo) corre sobre floats
        Python puros, mucho más rápido que numpy escalar para 3-vectores.
        """
        n = points_xyz.shape[0]
        if n == 0:
            return np.empty(0, dtype=np.int64)

        x = points_xyz[:, 0]
        y = points_xyz[:, 1]
        radius = np.hypot(x, y)

        # convertPointcloud: agrupar por ángulo azimutal y ordenar por radio.
        # Autoware usa atan2(x, y) (no y, x) y normaliza a [0, 2π).
        theta = np.mod(np.arctan2(x, y), 2.0 * math.pi)  # [0, 2π)
        radial_div = np.floor(theta / self.radial_divider_angle_rad).astype(np.int64)
        np.clip(radial_div, 0, self.radial_dividers_num - 1, out=radial_div)

        # lexsort: clave primaria radial_div, secundaria radius -> cada bloque
        # contiguo de radial_div ya queda ordenado por radio ascendente.
        order = np.lexsort((radius, radial_div))
        div_sorted = radial_div[order]
        boundaries = np.flatnonzero(np.diff(div_sorted)) + 1
        group_starts = np.concatenate(([0], boundaries))
        group_ends = np.concatenate((boundaries, [n]))

        # a floats/ints Python para el hot loop
        xs = x.tolist()
        ys = y.tolist()
        zs = points_xyz[:, 2].tolist()
        rs = radius.tolist()
        order_list = order.tolist()

        no_ground: List[int] = []
        for gs, ge in zip(group_starts.tolist(), group_ends.tolist()):
            if ge > gs:
                self._classify_ray(order_list[gs:ge], xs, ys, zs, rs, no_ground)

        return np.asarray(no_ground, dtype=np.int64)

    def _classify_ray(
        self,
        ray_idx: List[int],
        xs: List[float],
        ys: List[float],
        zs: List[float],
        rs: List[float],
        no_ground: List[int],
    ) -> None:
        cfg = self.config
        radial_angle = self.radial_divider_angle_rad
        split_tol = cfg.split_points_distance_tolerance
        split_h = cfg.split_height_distance
        local_slope_max = self.local_slope_max_angle_rad
        global_slope_max_ratio = self.global_slope_max_ratio
        use_virtual = cfg.use_virtual_ground_point
        virtual_x = self.virtual_ground_x

        # etiquetas como ints locales (evita lookups de IntEnum en el lazo)
        INIT = int(PointLabel.INIT)
        GROUND = int(PointLabel.GROUND)
        NON_GROUND = int(PointLabel.NON_GROUND)
        POINT_FOLLOW = int(PointLabel.POINT_FOLLOW)

        prev_gnd_radius = 0.0
        prev_gnd_slope = 0.0
        prev_gnd_x = prev_gnd_y = prev_gnd_z = 0.0
        # acumuladores de cluster (suma de radio/altura y conteo)
        g_rsum = g_hsum = 0.0
        g_n = 0
        ng_hsum = 0.0
        ng_n = 0

        label_curr = INIT
        cx = cy = cz = 0.0

        for j, idx in enumerate(ray_idx):
            px, py, pz = cx, cy, cz
            label_prev = label_curr

            cx = xs[idx]
            cy = ys[idx]
            cz = zs[idx]
            pd_radius = rs[idx]
            label_curr = INIT

            if j == 0:
                if use_virtual and cx > virtual_x:
                    prev_gnd_x = virtual_x
                    prev_gnd_y = 0.0
                    prev_gnd_z = 0.0
                    prev_gnd_radius = virtual_x  # hypot(x, 0)
                else:
                    prev_gnd_x = 0.0
                    prev_gnd_y = 0.0
                    prev_gnd_z = 0.0
                    prev_gnd_radius = 0.0
                prev_gnd_slope = 0.0
                g_rsum = g_hsum = 0.0
                g_n = 0
                ng_hsum = 0.0
                ng_n = 0
                dx = cx - prev_gnd_x
                dy = cy - prev_gnd_y
                dz = cz - prev_gnd_z
                points_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            else:
                dx = cx - px
                dy = cy - py
                dz = cz - pz
                points_distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            radius_distance_from_gnd = pd_radius - prev_gnd_radius
            height_from_gnd = cz - prev_gnd_z
            height_from_obj = 0.0
            if ng_n > 0:
                height_from_obj = cz - ng_hsum / ng_n

            calculate_slope = True
            is_close = points_distance < (pd_radius * radial_angle + split_tol)
            if is_close and g_n > 0:
                height_from_gnd = cz - g_hsum / g_n
                radius_distance_from_gnd = pd_radius - g_rsum / g_n

            global_slope_ratio = cz / pd_radius if pd_radius > 0.0 else 0.0

            if global_slope_ratio > global_slope_max_ratio:
                label_curr = NON_GROUND
                calculate_slope = False
            elif (
                label_prev == NON_GROUND
                and ng_n > 0
                and abs(height_from_obj) >= split_h
            ):
                calculate_slope = True
            elif (
                label_prev == GROUND
                and is_close
                and abs(height_from_gnd) < split_h
            ):
                label_curr = POINT_FOLLOW
                calculate_slope = False

            if calculate_slope:
                local_slope = math.atan2(height_from_gnd, radius_distance_from_gnd)
                if local_slope - prev_gnd_slope > local_slope_max:
                    label_curr = NON_GROUND
                else:
                    label_curr = GROUND

            # el reset ocurre solo para GROUND directo (antes de POINT_FOLLOW->GROUND)
            if label_curr == GROUND:
                g_rsum = g_hsum = 0.0
                g_n = 0
                ng_hsum = 0.0
                ng_n = 0

            if label_curr == NON_GROUND:
                no_ground.append(idx)
            elif label_curr == POINT_FOLLOW:
                label_curr = GROUND

            if label_curr == GROUND:
                prev_gnd_radius = pd_radius
                prev_gnd_x = cx
                prev_gnd_z = cz
                g_rsum += pd_radius
                g_hsum += cz
                g_n += 1
                prev_gnd_slope = math.atan2(g_hsum / g_n, g_rsum / g_n)
            elif label_curr == NON_GROUND:
                ng_hsum += cz
                ng_n += 1


class ScanGroundFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_ground_filter")

        cfg = ScanGroundFilterConfig
        self.declare_parameter("input_topic", "/scan_3d")
        self.declare_parameter("output_topic", "/scan_3d/no_ground")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("global_slope_max_angle_deg", cfg.global_slope_max_angle_deg)
        self.declare_parameter("local_slope_max_angle_deg", cfg.local_slope_max_angle_deg)
        self.declare_parameter("radial_divider_angle_deg", cfg.radial_divider_angle_deg)
        self.declare_parameter(
            "split_points_distance_tolerance", cfg.split_points_distance_tolerance
        )
        self.declare_parameter("use_virtual_ground_point", cfg.use_virtual_ground_point)
        self.declare_parameter("split_height_distance", cfg.split_height_distance)
        self.declare_parameter("vehicle_wheel_base_m", cfg.vehicle_wheel_base_m)
        self.declare_parameter("range_max", cfg.range_max)

        config = ScanGroundFilterConfig(
            global_slope_max_angle_deg=self._p("global_slope_max_angle_deg"),
            local_slope_max_angle_deg=self._p("local_slope_max_angle_deg"),
            radial_divider_angle_deg=self._p("radial_divider_angle_deg"),
            split_points_distance_tolerance=self._p("split_points_distance_tolerance"),
            use_virtual_ground_point=self._p("use_virtual_ground_point"),
            split_height_distance=self._p("split_height_distance"),
            vehicle_wheel_base_m=self._p("vehicle_wheel_base_m"),
            range_max=self._p("range_max"),
        )
        self._config = config
        self._config_lock = threading.Lock()
        self.segmenter = ScanGroundSegmenter(config)
        self.range_max = float(config.range_max)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.target_frame = self._p("target_frame")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_matrix: Optional[np.ndarray] = None
        self._tf_source: Optional[str] = None

        self.publisher = self.create_publisher(
            PointCloud2, self._p("output_topic"), qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            PointCloud2, self._p("input_topic"), self._on_cloud, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"scan_ground_filter listo: {self._p('input_topic')} -> "
            f"{self._p('output_topic')} (frame {self.target_frame})"
        )

    def _p(self, name: str):
        return self.get_parameter(name).value

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        """Actualiza en bloque los umbrales que cambian con el perfil de navegación."""
        updates = {
            str(parameter.name): parameter.value
            for parameter in parameters
            if str(parameter.name)
            in {
                "global_slope_max_angle_deg",
                "local_slope_max_angle_deg",
                "split_height_distance",
            }
        }
        if not updates:
            return SetParametersResult(successful=True, reason="")

        try:
            global_slope = float(
                updates.get(
                    "global_slope_max_angle_deg",
                    self._config.global_slope_max_angle_deg,
                )
            )
            local_slope = float(
                updates.get(
                    "local_slope_max_angle_deg",
                    self._config.local_slope_max_angle_deg,
                )
            )
            split_height = float(
                updates.get(
                    "split_height_distance",
                    self._config.split_height_distance,
                )
            )
        except (TypeError, ValueError):
            return SetParametersResult(
                successful=False,
                reason="ground profile parameters must be numeric",
            )

        if not (0.0 < global_slope <= 35.0):
            return SetParametersResult(
                successful=False,
                reason="global_slope_max_angle_deg must be > 0 and <= 35",
            )
        if not (0.0 < local_slope <= 35.0):
            return SetParametersResult(
                successful=False,
                reason="local_slope_max_angle_deg must be > 0 and <= 35",
            )
        if not (0.0 < split_height <= 0.50):
            return SetParametersResult(
                successful=False,
                reason="split_height_distance must be > 0 and <= 0.50",
            )

        with self._config_lock:
            next_config = replace(
                self._config,
                global_slope_max_angle_deg=global_slope,
                local_slope_max_angle_deg=local_slope,
                split_height_distance=split_height,
            )
            self._config = next_config
            self.segmenter = ScanGroundSegmenter(next_config)
        self.get_logger().info(
            "scan_ground_filter profile updated "
            f"(global={global_slope:.1f}deg, local={local_slope:.1f}deg, "
            f"split_height={split_height:.2f}m)"
        )
        return SetParametersResult(successful=True, reason="")

    def _lookup_matrix(self, source_frame: str, stamp) -> Optional[np.ndarray]:
        """4x4 target<-source. Cacheado: el TF lidar->base es estático."""
        if not self.target_frame or source_frame == self.target_frame:
            return np.eye(4)
        if self._tf_matrix is not None and self._tf_source == source_frame:
            return self._tf_matrix
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"TF {self.target_frame}<-{source_frame} no disponible: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        mat = np.eye(4)
        mat[:3, :3] = _quat_to_rot(q.x, q.y, q.z, q.w)
        mat[:3, 3] = [t.x, t.y, t.z]
        self._tf_matrix = mat
        self._tf_source = source_frame
        return mat

    def _on_cloud(self, msg: PointCloud2) -> None:
        matrix = self._lookup_matrix(msg.header.frame_id, msg.header.stamp)
        if matrix is None:
            return

        records = point_cloud2.read_points(
            msg, field_names=None, skip_nans=True, reshape_organized_cloud=False
        )
        if records.shape[0] == 0:
            self._publish(msg, records[:0])
            return

        xyz = np.stack(
            [
                records["x"].astype(np.float64),
                records["y"].astype(np.float64),
                records["z"].astype(np.float64),
            ],
            axis=1,
        )
        finite = np.isfinite(xyz).all(axis=1)
        if not np.all(finite):
            records = records[finite]
            xyz = xyz[finite]
            if records.shape[0] == 0:
                self._publish(msg, records[:0])
                return
        # transformar al frame nivelado
        xyz_h = matrix[:3, :3] @ xyz.T + matrix[:3, 3:4]
        xyz_t = xyz_h.T

        # recorte de rango (evita iterar ruido lejano; no altera la clasificación)
        keep = np.hypot(xyz_t[:, 0], xyz_t[:, 1]) <= self.range_max
        records_keep = records[keep]
        xyz_keep = xyz_t[keep]

        with self._config_lock:
            segmenter = self.segmenter
        no_ground_idx = segmenter.segment(xyz_keep)

        out_records = records_keep[no_ground_idx].copy()
        # escribir las coordenadas transformadas (output queda en target_frame)
        sel = xyz_keep[no_ground_idx]
        out_records["x"] = sel[:, 0].astype(out_records["x"].dtype)
        out_records["y"] = sel[:, 1].astype(out_records["y"].dtype)
        out_records["z"] = sel[:, 2].astype(out_records["z"].dtype)
        self._publish(msg, out_records)

    def _publish(self, src: PointCloud2, records: np.ndarray) -> None:
        header = Header()
        header.stamp = src.header.stamp
        header.frame_id = self.target_frame if self.target_frame else src.header.frame_id
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = int(records.shape[0])
        out.fields = src.fields
        out.is_bigendian = src.is_bigendian
        out.point_step = src.point_step
        if records.dtype.itemsize != src.point_step:
            out.point_step = records.dtype.itemsize
        out.row_step = out.point_step * out.width
        out.is_dense = True
        out.data = records.tobytes()
        self.publisher.publish(out)


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanGroundFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
