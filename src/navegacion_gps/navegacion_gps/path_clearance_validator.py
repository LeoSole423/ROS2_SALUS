import math
import threading
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class CostmapView:
    frame_id: str
    stamp_sec: float
    resolution: float
    size_x: int
    size_y: int
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: np.ndarray


@dataclass(frozen=True)
class ClearanceCheckResult:
    is_valid: bool
    invalid_indices: List[int]
    max_cost: int
    checked_samples: int
    reason: str


def yaw_from_quaternion_z_w(z: float, w: float) -> float:
    return math.atan2(
        2.0 * float(w) * float(z),
        1.0 - (2.0 * float(z) * float(z)),
    )


def costmap_view_from_msg(
    msg: Costmap,
    *,
    received_time_sec: float,
) -> CostmapView:
    metadata = msg.metadata
    size_x = int(metadata.size_x)
    size_y = int(metadata.size_y)
    data = np.asarray(msg.data, dtype=np.uint8)
    if data.size != size_x * size_y:
        data = np.resize(data, size_x * size_y).astype(np.uint8)
    return CostmapView(
        frame_id=str(msg.header.frame_id or "map"),
        stamp_sec=float(received_time_sec),
        resolution=max(1.0e-6, float(metadata.resolution)),
        size_x=size_x,
        size_y=size_y,
        origin_x=float(metadata.origin.position.x),
        origin_y=float(metadata.origin.position.y),
        origin_yaw=yaw_from_quaternion_z_w(
            float(metadata.origin.orientation.z),
            float(metadata.origin.orientation.w),
        ),
        data=data.reshape((size_y, size_x)),
    )


def world_to_costmap_cell(
    costmap: CostmapView,
    x: float,
    y: float,
) -> Optional[Tuple[int, int]]:
    dx = float(x) - costmap.origin_x
    dy = float(y) - costmap.origin_y
    cos_yaw = math.cos(-costmap.origin_yaw)
    sin_yaw = math.sin(-costmap.origin_yaw)
    local_x = (dx * cos_yaw) - (dy * sin_yaw)
    local_y = (dx * sin_yaw) + (dy * cos_yaw)
    ix = int(math.floor(local_x / costmap.resolution))
    iy = int(math.floor(local_y / costmap.resolution))
    if ix < 0 or iy < 0 or ix >= costmap.size_x or iy >= costmap.size_y:
        return None
    return ix, iy


def _path_xy(path: Path) -> List[Tuple[float, float]]:
    return [
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in path.poses
        if np.isfinite(float(pose.pose.position.x))
        and np.isfinite(float(pose.pose.position.y))
    ]


def _iter_path_samples(
    points: Sequence[Tuple[float, float]],
    *,
    max_check_distance_m: float,
    sample_step_m: float,
) -> Iterable[Tuple[int, float, float, float]]:
    remaining = max(0.0, float(max_check_distance_m))
    step = max(0.05, float(sample_step_m))
    emitted_start = False

    for segment_index in range(max(0, len(points) - 1)):
        start_x, start_y = points[segment_index]
        end_x, end_y = points[segment_index + 1]
        dx = end_x - start_x
        dy = end_y - start_y
        length = math.hypot(dx, dy)
        if length <= 1.0e-6:
            continue
        yaw = math.atan2(dy, dx)
        distance = 0.0 if not emitted_start else step
        while distance <= length + 1.0e-9:
            if remaining < 0.0:
                return
            ratio = min(1.0, distance / length)
            yield (
                segment_index,
                start_x + (dx * ratio),
                start_y + (dy * ratio),
                yaw,
            )
            emitted_start = True
            remaining -= step
            if remaining < 0.0:
                return
            distance += step


def check_path_clearance(
    path: Path,
    costmap: CostmapView,
    *,
    max_check_distance_m: float,
    sample_step_m: float,
    high_cost_threshold: int,
    lethal_cost_threshold: int,
    min_consecutive_high_cost_samples: int,
    lateral_offsets_m: Sequence[float],
) -> ClearanceCheckResult:
    points = _path_xy(path)
    if len(points) < 2:
        return ClearanceCheckResult(True, [], 0, 0, "path_too_short")

    high_threshold = int(max(0, min(255, high_cost_threshold)))
    lethal_threshold = int(max(0, min(255, lethal_cost_threshold)))
    min_consecutive = max(1, int(min_consecutive_high_cost_samples))
    offsets = [float(offset) for offset in lateral_offsets_m] or [0.0]

    invalid_indices: List[int] = []
    checked_samples = 0
    max_cost = 0
    consecutive_high = 0

    for segment_index, x, y, yaw in _iter_path_samples(
        points,
        max_check_distance_m=max_check_distance_m,
        sample_step_m=sample_step_m,
    ):
        sample_cost = 0
        for offset in offsets:
            sample_x = x - (math.sin(yaw) * offset)
            sample_y = y + (math.cos(yaw) * offset)
            cell = world_to_costmap_cell(costmap, sample_x, sample_y)
            if cell is None:
                continue
            ix, iy = cell
            sample_cost = max(sample_cost, int(costmap.data[iy, ix]))

        checked_samples += 1
        max_cost = max(max_cost, sample_cost)
        if sample_cost >= lethal_threshold:
            invalid_indices.append(segment_index)
            return ClearanceCheckResult(
                False,
                invalid_indices,
                max_cost,
                checked_samples,
                "lethal_cost",
            )
        if sample_cost >= high_threshold:
            consecutive_high += 1
            invalid_indices.append(segment_index)
            if consecutive_high >= min_consecutive:
                return ClearanceCheckResult(
                    False,
                    invalid_indices,
                    max_cost,
                    checked_samples,
                    "sustained_high_cost",
                )
        else:
            consecutive_high = 0

    return ClearanceCheckResult(True, [], max_cost, checked_samples, "clear")


class PathClearanceValidatorNode(Node):
    def __init__(self) -> None:
        super().__init__("path_clearance_validator")
        self.declare_parameter("enabled", True)
        self.declare_parameter(
            "service_name",
            "/path_clearance_validator/is_path_clearance_valid",
        )
        self.declare_parameter(
            "global_costmap_topic",
            "/global_costmap/costmap_raw",
        )
        self.declare_parameter("max_check_distance_m", 12.0)
        self.declare_parameter("sample_step_m", 0.25)
        self.declare_parameter("high_cost_threshold", 100)
        self.declare_parameter("lethal_cost_threshold", 253)
        self.declare_parameter("min_consecutive_high_cost_samples", 3)
        self.declare_parameter("lateral_offsets_m", [0.0, 0.45, -0.45])
        self.declare_parameter("costmap_timeout_s", 1.5)
        self.declare_parameter("tf_timeout_s", 0.1)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.service_name = str(self.get_parameter("service_name").value)
        self.global_costmap_topic = str(
            self.get_parameter("global_costmap_topic").value
        )
        self.max_check_distance_m = max(
            0.1, float(self.get_parameter("max_check_distance_m").value)
        )
        self.sample_step_m = max(
            0.05,
            float(self.get_parameter("sample_step_m").value),
        )
        self.high_cost_threshold = int(
            self.get_parameter("high_cost_threshold").value
        )
        self.lethal_cost_threshold = int(
            self.get_parameter("lethal_cost_threshold").value
        )
        self.min_consecutive_high_cost_samples = max(
            1,
            int(
                self.get_parameter(
                    "min_consecutive_high_cost_samples"
                ).value
            ),
        )
        lateral_offsets = self.get_parameter("lateral_offsets_m").value
        self.lateral_offsets_m = [
            float(value) for value in lateral_offsets
        ]
        self.costmap_timeout_s = max(
            0.1,
            float(self.get_parameter("costmap_timeout_s").value),
        )
        self.tf_timeout_s = max(
            0.01,
            float(self.get_parameter("tf_timeout_s").value),
        )

        self._lock = threading.Lock()
        self._costmap: Optional[CostmapView] = None
        self._last_open_warning_s = 0.0

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._costmap_sub = self.create_subscription(
            Costmap,
            self.global_costmap_topic,
            self._on_costmap,
            costmap_qos,
        )
        self._service = self.create_service(
            IsPathValid,
            self.service_name,
            self._on_validate,
        )
        self.get_logger().info(
            "path_clearance_validator ready "
            f"(service={self.service_name}, "
            f"costmap={self.global_costmap_topic}, "
            f"threshold={self.high_cost_threshold}, "
            f"distance={self.max_check_distance_m:.1f}m)"
        )

    def _on_costmap(self, msg: Costmap) -> None:
        view = costmap_view_from_msg(
            msg,
            received_time_sec=self.get_clock().now().nanoseconds * 1.0e-9,
        )
        with self._lock:
            self._costmap = view

    def _warn_open(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._last_open_warning_s < 2.0:
            return
        self._last_open_warning_s = now
        self.get_logger().warning(
            f"Path clearance check failing open: {reason}"
        )

    def _transform_path_to_costmap_frame(
        self,
        path: Path,
        costmap_frame: str,
    ) -> Optional[Path]:
        path_frame = str(path.header.frame_id or costmap_frame)
        if path_frame == costmap_frame:
            return path

        try:
            transform = self._tf_buffer.lookup_transform(
                costmap_frame,
                path_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as exc:
            self._warn_open(
                f"tf unavailable {path_frame}->{costmap_frame}: {exc}"
            )
            return None

        transformed = Path()
        transformed.header = path.header
        transformed.header.frame_id = costmap_frame
        dx = float(transform.transform.translation.x)
        dy = float(transform.transform.translation.y)
        yaw = yaw_from_quaternion_z_w(
            float(transform.transform.rotation.z),
            float(transform.transform.rotation.w),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        for pose in path.poses:
            new_pose = PoseStamped()
            new_pose.header = pose.header
            new_pose.header.frame_id = costmap_frame
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            new_pose.pose = pose.pose
            new_pose.pose.position = Point(
                x=(x * cos_yaw) - (y * sin_yaw) + dx,
                y=(x * sin_yaw) + (y * cos_yaw) + dy,
                z=float(pose.pose.position.z),
            )
            transformed.poses.append(new_pose)
        return transformed

    def _on_validate(
        self,
        request: IsPathValid.Request,
        response: IsPathValid.Response,
    ) -> IsPathValid.Response:
        response.is_valid = True
        response.invalid_pose_indices = []
        if not self.enabled:
            return response

        with self._lock:
            costmap = self._costmap
        if costmap is None:
            self._warn_open("costmap unavailable")
            return response

        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if (now_sec - costmap.stamp_sec) > self.costmap_timeout_s:
            self._warn_open(
                f"costmap stale age={now_sec - costmap.stamp_sec:.2f}s"
            )
            return response

        path = self._transform_path_to_costmap_frame(
            request.path,
            costmap.frame_id,
        )
        if path is None:
            return response

        result = check_path_clearance(
            path,
            costmap,
            max_check_distance_m=self.max_check_distance_m,
            sample_step_m=self.sample_step_m,
            high_cost_threshold=self.high_cost_threshold,
            lethal_cost_threshold=self.lethal_cost_threshold,
            min_consecutive_high_cost_samples=(
                self.min_consecutive_high_cost_samples
            ),
            lateral_offsets_m=self.lateral_offsets_m,
        )
        response.is_valid = bool(result.is_valid)
        response.invalid_pose_indices = list(result.invalid_indices)
        if not result.is_valid:
            self.get_logger().warning(
                "Path clearance invalid "
                f"(reason={result.reason}, max_cost={result.max_cost}, "
                f"samples={result.checked_samples})"
            )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathClearanceValidatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
