import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class LidarObstacleFilterConfig:
    roi_x_min: float = -0.4
    roi_x_max: float = 12.0
    roi_y_min: float = -2.5
    roi_y_max: float = 2.5
    roi_z_min: float = -1.0
    roi_z_max: float = 2.0
    ground_distance_threshold: float = 0.18
    ground_candidate_percentile: float = 95.0
    min_obstacle_height: float = 0.22
    max_obstacle_height: float = 1.40
    voxel_size_x: float = 0.25
    voxel_size_y: float = 0.25
    voxel_size_z: float = 0.20
    min_voxel_points: int = 3
    angle_min: float = -1.57079632679
    angle_max: float = 1.57079632679
    angle_increment: float = 0.00872664626
    range_min: float = 0.4
    range_max: float = 12.0
    ransac_iterations: int = 64
    min_ground_points: int = 24


@dataclass(frozen=True)
class TiltGateConfig:
    enabled: bool = True
    nominal_roll_deg: float = 0.0
    nominal_pitch_deg: float = 0.0
    # El offset debe quedar por ENCIMA de la pendiente operativa maxima:
    # la inclinacion sostenida es compensable (IMU + RANSAC) y ademas el
    # collision_monitor usa este scan como unica fuente (source_timeout 1.0 s),
    # asi que un bloqueo sostenido lo deja ciego. Los transitorios los corta
    # el limite de salto, no este.
    max_offset_deg: float = 12.0
    max_jump_deg: float = 3.0


class TiltGate:
    """Rechaza frames capturados con el LiDAR fuera de su actitud nominal o
    cambiando demasiado rapido para confiar en la compensacion IMU."""

    def __init__(self, config: TiltGateConfig) -> None:
        self._config = config
        self._last_roll: Optional[float] = None
        self._last_pitch: Optional[float] = None

    def update(self, roll: float, pitch: float) -> bool:
        """Devuelve True si el frame se acepta. Llamar una vez por frame."""
        if not self._config.enabled:
            return True
        max_offset = math.radians(self._config.max_offset_deg)
        max_jump = math.radians(self._config.max_jump_deg)
        roll_offset = abs(roll - math.radians(self._config.nominal_roll_deg))
        pitch_offset = abs(
            pitch - math.radians(self._config.nominal_pitch_deg)
        )
        jump_ok = True
        if self._last_roll is not None and self._last_pitch is not None:
            jump_ok = (
                abs(roll - self._last_roll) <= max_jump
                and abs(pitch - self._last_pitch) <= max_jump
            )
        self._last_roll = roll
        self._last_pitch = pitch
        return (
            roll_offset <= max_offset
            and pitch_offset <= max_offset
            and jump_ok
        )


@dataclass(frozen=True)
class VoxelPersistenceConfig:
    enabled: bool = True
    min_hits: int = 2
    window: int = 3


class VoxelPersistenceFilter:
    """Solo deja pasar voxels vistos en al menos min_hits de los ultimos
    window frames: un fantasma de un solo frame nunca llega al scan."""

    def __init__(
        self,
        config: VoxelPersistenceConfig,
        voxel_size_x: float,
        voxel_size_y: float,
    ) -> None:
        self._config = config
        self._voxel_size = np.maximum(
            np.array([voxel_size_x, voxel_size_y], dtype=np.float32),
            1.0e-3,
        )
        self._history: deque = deque(maxlen=max(1, config.window))

    def filter(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
        if not self._config.enabled or self._config.min_hits <= 1:
            return points
        if len(points) == 0:
            self._history.append(frozenset())
            return points
        indices = np.floor(points[:, :2] / self._voxel_size).astype(np.int32)
        keys = [tuple(index) for index in indices]
        self._history.append(frozenset(keys))
        hits: dict = {}
        for frame_keys in self._history:
            for key in frame_keys:
                hits[key] = hits.get(key, 0) + 1
        keep = np.fromiter(
            (hits[key] >= self._config.min_hits for key in keys),
            dtype=bool,
            count=len(keys),
        )
        return points[keep]


def quaternion_to_roll_pitch(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch


def _rotation_roll_pitch(roll: float, pitch: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    roll_matrix = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ],
        dtype=np.float32,
    )
    pitch_matrix = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ],
        dtype=np.float32,
    )
    return pitch_matrix @ roll_matrix


def _rotation_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm < 1.0e-9:
        return np.eye(3, dtype=np.float32)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float32,
    )


def compensate_roll_pitch(
    points: np.ndarray,
    roll: float,
    pitch: float,
) -> np.ndarray:
    if points.size == 0:
        return points.reshape((-1, 3))
    rotation = _rotation_roll_pitch(-roll, -pitch)
    return points @ rotation.T


def _fit_plane_svd(points: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    if len(points) < 3:
        return None
    centroid = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1, :]
    norm = float(np.linalg.norm(normal))
    if norm < 1.0e-9:
        return None
    normal = normal / norm
    if normal[2] < 0.0:
        normal = -normal
    offset = -float(np.dot(normal, centroid))
    return normal.astype(np.float32), offset


def fit_ground_plane(
    points: np.ndarray,
    config: LidarObstacleFilterConfig,
) -> Optional[Tuple[np.ndarray, float]]:
    if len(points) < config.min_ground_points:
        return None

    candidate_percentile = float(
        np.clip(config.ground_candidate_percentile, 50.0, 100.0)
    )
    z_limit = float(np.percentile(points[:, 2], candidate_percentile))
    candidates = points[points[:, 2] <= z_limit]
    if len(candidates) < config.min_ground_points:
        candidates = points

    rng = np.random.default_rng(17)
    best_inliers: Optional[np.ndarray] = None
    best_count = 0
    sample_size = min(3, len(candidates))
    for _ in range(max(1, config.ransac_iterations)):
        sample_indices = rng.choice(
            len(candidates),
            size=sample_size,
            replace=False,
        )
        plane = _fit_plane_svd(candidates[sample_indices])
        if plane is None:
            continue
        normal, offset = plane
        if abs(float(normal[2])) < 0.55:
            continue
        distances = np.abs(candidates @ normal + offset)
        inliers = distances <= config.ground_distance_threshold
        count = int(np.count_nonzero(inliers))
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < config.min_ground_points:
        return _fit_plane_svd(candidates)

    return _fit_plane_svd(candidates[best_inliers])


def density_filter_points(
    points: np.ndarray,
    config: LidarObstacleFilterConfig,
) -> np.ndarray:
    if len(points) == 0 or config.min_voxel_points <= 1:
        return points.reshape((-1, 3))
    voxel_size = np.array(
        [config.voxel_size_x, config.voxel_size_y],
        dtype=np.float32,
    )
    voxel_size = np.maximum(voxel_size, 1.0e-3)
    indices = np.floor(points[:, :2] / voxel_size).astype(np.int32)
    _, inverse, counts = np.unique(
        indices,
        axis=0,
        return_inverse=True,
        return_counts=True,
    )
    return points[counts[inverse] >= config.min_voxel_points]


def filter_obstacle_points(
    points: np.ndarray,
    config: LidarObstacleFilterConfig,
    roll: float = 0.0,
    pitch: float = 0.0,
) -> np.ndarray:
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    finite_mask = np.isfinite(points).all(axis=1)
    points = points[finite_mask]
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)

    roi_mask = (
        (points[:, 0] >= config.roi_x_min)
        & (points[:, 0] <= config.roi_x_max)
        & (points[:, 1] >= config.roi_y_min)
        & (points[:, 1] <= config.roi_y_max)
        & (points[:, 2] >= config.roi_z_min)
        & (points[:, 2] <= config.roi_z_max)
    )
    roi_points = points[roi_mask]
    if len(roi_points) == 0:
        return np.empty((0, 3), dtype=np.float32)

    stabilized = compensate_roll_pitch(roi_points, roll=roll, pitch=pitch)
    plane = fit_ground_plane(stabilized, config)
    if plane is None:
        obstacle_mask = (
            (stabilized[:, 2] >= config.min_obstacle_height)
            & (stabilized[:, 2] <= config.max_obstacle_height)
        )
    else:
        normal, offset = plane
        height_above_ground = stabilized @ normal + offset
        obstacle_mask = (
            (height_above_ground >= config.min_obstacle_height)
            & (height_above_ground <= config.max_obstacle_height)
        )

    obstacles = roi_points[obstacle_mask]
    return density_filter_points(obstacles, config).astype(np.float32)


def points_to_laserscan(
    points: np.ndarray,
    header: Header,
    config: LidarObstacleFilterConfig,
) -> LaserScan:
    scan = LaserScan()
    scan.header = header
    scan.angle_min = float(config.angle_min)
    scan.angle_max = float(config.angle_max)
    scan.angle_increment = float(config.angle_increment)
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = float(config.range_min)
    scan.range_max = float(config.range_max)

    beam_count = (
        int(
            math.floor(
                (scan.angle_max - scan.angle_min) / scan.angle_increment
            )
        )
        + 1
    )
    ranges = np.full(beam_count, np.inf, dtype=np.float32)
    if points.size > 0:
        points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
        xy_ranges = np.hypot(points[:, 0], points[:, 1])
        angles = np.arctan2(points[:, 1], points[:, 0])
        valid = (
            np.isfinite(xy_ranges)
            & (xy_ranges >= scan.range_min)
            & (xy_ranges <= scan.range_max)
            & (angles >= scan.angle_min)
            & (angles <= scan.angle_max)
        )
        valid_indices = np.nonzero(valid)[0]
        for point_index in valid_indices:
            beam_index = int(
                round(
                    (angles[point_index] - scan.angle_min)
                    / scan.angle_increment
                )
            )
            if 0 <= beam_index < beam_count:
                ranges[beam_index] = min(
                    ranges[beam_index],
                    xy_ranges[point_index],
                )
    scan.ranges = [float(value) for value in ranges]
    return scan


def _points_from_cloud(msg: PointCloud2) -> np.ndarray:
    points = point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )
    if isinstance(points, np.ndarray):
        if points.dtype.names:
            xyz = np.column_stack(
                (points["x"], points["y"], points["z"])
            ).astype(np.float32)
        else:
            xyz = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    else:
        xyz = np.asarray(list(points), dtype=np.float32).reshape((-1, 3))
    return xyz[np.isfinite(xyz).all(axis=1)]


def _cloud_from_points(points: np.ndarray, header: Header) -> PointCloud2:
    return point_cloud2.create_cloud_xyz32(
        header,
        points.astype(np.float32).tolist(),
    )


class LidarObstacleFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_obstacle_filter")
        self.declare_parameter("cloud_topic", "/scan_3d")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("obstacles_cloud_topic", "/obstacles_cloud")
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("use_imu_compensation", True)
        self.declare_parameter("imu_max_age_s", 0.5)
        self.declare_parameter("roi_x_min", -0.4)
        self.declare_parameter("roi_x_max", 12.0)
        self.declare_parameter("roi_y_min", -2.5)
        self.declare_parameter("roi_y_max", 2.5)
        self.declare_parameter("roi_z_min", -1.0)
        self.declare_parameter("roi_z_max", 2.0)
        self.declare_parameter("ground_distance_threshold", 0.18)
        self.declare_parameter("ground_candidate_percentile", 95.0)
        self.declare_parameter("min_obstacle_height", 0.22)
        self.declare_parameter("max_obstacle_height", 1.40)
        self.declare_parameter("voxel_size_x", 0.25)
        self.declare_parameter("voxel_size_y", 0.25)
        self.declare_parameter("voxel_size_z", 0.20)
        self.declare_parameter("min_voxel_points", 3)
        self.declare_parameter("angle_min", -1.57079632679)
        self.declare_parameter("angle_max", 1.57079632679)
        self.declare_parameter("angle_increment", 0.00872664626)
        self.declare_parameter("range_min", 0.4)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("tilt_gate_enabled", True)
        self.declare_parameter("tilt_gate_nominal_roll_deg", 0.0)
        self.declare_parameter("tilt_gate_nominal_pitch_deg", 0.0)
        self.declare_parameter("tilt_gate_max_offset_deg", 12.0)
        self.declare_parameter("tilt_gate_max_jump_deg", 3.0)
        self.declare_parameter(
            "tilt_gate_state_topic",
            "/lidar_obstacle_filter/tilt_gate_blocked",
        )
        self.declare_parameter("persistence_enabled", True)
        self.declare_parameter("persistence_min_hits", 2)
        self.declare_parameter("persistence_window", 3)

        self._cloud_topic = str(self.get_parameter("cloud_topic").value)
        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._obstacles_cloud_topic = str(
            self.get_parameter("obstacles_cloud_topic").value
        )
        self._output_frame = str(self.get_parameter("output_frame").value)
        self._use_imu_compensation = bool(
            self.get_parameter("use_imu_compensation").value
        )
        self._imu_max_age_s = max(
            0.0,
            float(self.get_parameter("imu_max_age_s").value),
        )
        self._last_roll_pitch: Optional[Tuple[float, float]] = None
        self._last_imu_time_s: Optional[float] = None
        self._config = self._read_config()
        self._tilt_gate = TiltGate(
            TiltGateConfig(
                enabled=bool(self.get_parameter("tilt_gate_enabled").value),
                nominal_roll_deg=float(
                    self.get_parameter("tilt_gate_nominal_roll_deg").value
                ),
                nominal_pitch_deg=float(
                    self.get_parameter("tilt_gate_nominal_pitch_deg").value
                ),
                max_offset_deg=float(
                    self.get_parameter("tilt_gate_max_offset_deg").value
                ),
                max_jump_deg=float(
                    self.get_parameter("tilt_gate_max_jump_deg").value
                ),
            )
        )
        self._persistence = VoxelPersistenceFilter(
            VoxelPersistenceConfig(
                enabled=bool(self.get_parameter("persistence_enabled").value),
                min_hits=int(self.get_parameter("persistence_min_hits").value),
                window=int(self.get_parameter("persistence_window").value),
            ),
            voxel_size_x=self._config.voxel_size_x,
            voxel_size_y=self._config.voxel_size_y,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._scan_pub = self.create_publisher(
            LaserScan,
            self._scan_topic,
            output_qos,
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2,
            self._obstacles_cloud_topic,
            output_qos,
        )
        self._gate_state_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("tilt_gate_state_topic").value),
            output_qos,
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2,
            self._cloud_topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self._imu_sub = self.create_subscription(
            Imu,
            self._imu_topic,
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "lidar_obstacle_filter ready "
            f"(cloud={self._cloud_topic}, scan={self._scan_topic}, "
            f"obstacles={self._obstacles_cloud_topic})"
        )

    def _read_config(self) -> LidarObstacleFilterConfig:
        return LidarObstacleFilterConfig(
            roi_x_min=float(self.get_parameter("roi_x_min").value),
            roi_x_max=float(self.get_parameter("roi_x_max").value),
            roi_y_min=float(self.get_parameter("roi_y_min").value),
            roi_y_max=float(self.get_parameter("roi_y_max").value),
            roi_z_min=float(self.get_parameter("roi_z_min").value),
            roi_z_max=float(self.get_parameter("roi_z_max").value),
            ground_distance_threshold=float(
                self.get_parameter("ground_distance_threshold").value
            ),
            ground_candidate_percentile=float(
                self.get_parameter("ground_candidate_percentile").value
            ),
            min_obstacle_height=float(
                self.get_parameter("min_obstacle_height").value
            ),
            max_obstacle_height=float(
                self.get_parameter("max_obstacle_height").value
            ),
            voxel_size_x=float(self.get_parameter("voxel_size_x").value),
            voxel_size_y=float(self.get_parameter("voxel_size_y").value),
            voxel_size_z=float(self.get_parameter("voxel_size_z").value),
            min_voxel_points=int(self.get_parameter("min_voxel_points").value),
            angle_min=float(self.get_parameter("angle_min").value),
            angle_max=float(self.get_parameter("angle_max").value),
            angle_increment=float(self.get_parameter("angle_increment").value),
            range_min=float(self.get_parameter("range_min").value),
            range_max=float(self.get_parameter("range_max").value),
        )

    def _on_imu(self, msg: Imu) -> None:
        orientation = msg.orientation
        self._last_roll_pitch = quaternion_to_roll_pitch(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self._last_imu_time_s = self.get_clock().now().nanoseconds / 1.0e9

    def _current_roll_pitch(self) -> Tuple[float, float]:
        if not self._use_imu_compensation or self._last_roll_pitch is None:
            return 0.0, 0.0
        now_s = self.get_clock().now().nanoseconds / 1.0e9
        imu_is_stale = (
            self._last_imu_time_s is None
            or now_s - self._last_imu_time_s > self._imu_max_age_s
        )
        if imu_is_stale:
            return 0.0, 0.0
        return self._last_roll_pitch

    def _transform_points_to_output_frame(
        self,
        points: np.ndarray,
        msg: PointCloud2,
    ) -> Optional[np.ndarray]:
        source_frame = msg.header.frame_id
        if source_frame == self._output_frame or not source_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                self._output_frame,
                source_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warn(
                "failed to transform lidar cloud "
                f"from {source_frame} to {self._output_frame}: {exc}"
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotation_matrix = _rotation_from_quaternion(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        offset = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float32,
        )
        return (points @ rotation_matrix.T) + offset

    def _on_cloud(self, msg: PointCloud2) -> None:
        try:
            points = _points_from_cloud(msg)
        except Exception as exc:
            self.get_logger().warn(f"failed to decode cloud: {exc}")
            return

        points = self._transform_points_to_output_frame(points, msg)
        if points is None:
            return

        roll, pitch = self._current_roll_pitch()
        gate_open = self._tilt_gate.update(roll, pitch)
        self._gate_state_pub.publish(Bool(data=not gate_open))
        if not gate_open:
            # Frame capturado durante un transitorio de inclinacion: no se
            # publica nada (el costmap retiene la ultima observacion valida)
            # en lugar de insertar suelo como obstaculo o limpiar de mas.
            self.get_logger().warning(
                "tilt gate blocked frame "
                f"(roll={math.degrees(roll):.1f} deg, "
                f"pitch={math.degrees(pitch):.1f} deg)",
                throttle_duration_sec=5.0,
            )
            return

        obstacles = filter_obstacle_points(
            points,
            self._config,
            roll=roll,
            pitch=pitch,
        )
        obstacles = self._persistence.filter(obstacles)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self._output_frame
        self._cloud_pub.publish(_cloud_from_points(obstacles, header))
        self._scan_pub.publish(
            points_to_laserscan(obstacles, header, self._config)
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarObstacleFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
