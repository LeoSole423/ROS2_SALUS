import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class ScanNoiseFilterConfig:
    filter_range_min_m: float = 0.4
    filter_range_max_m: float = 20.0
    speckle_filter_window: int = 2
    speckle_max_range_m: float = 12.0
    speckle_max_deviation_m: float = 0.30


def _clone_scan_metadata(msg: LaserScan) -> LaserScan:
    filtered = LaserScan()
    filtered.header = msg.header
    filtered.angle_min = msg.angle_min
    filtered.angle_max = msg.angle_max
    filtered.angle_increment = msg.angle_increment
    filtered.time_increment = msg.time_increment
    filtered.scan_time = msg.scan_time
    filtered.range_min = msg.range_min
    filtered.range_max = msg.range_max
    filtered.intensities = list(msg.intensities)
    return filtered


def _effective_filter_range(msg: LaserScan, config: ScanNoiseFilterConfig):
    range_min = float(config.filter_range_min_m)
    range_max = float(config.filter_range_max_m)

    if math.isfinite(msg.range_min) and msg.range_min > 0.0:
        range_min = max(range_min, float(msg.range_min))
    if math.isfinite(msg.range_max) and msg.range_max > 0.0:
        range_max = min(range_max, float(msg.range_max))

    return range_min, range_max


def _has_supporting_neighbor(
    ranges,
    index: int,
    window: int,
    max_deviation_m: float,
) -> bool:
    reading = ranges[index]
    start = max(0, index - window)
    end = min(len(ranges), index + window + 1)
    for neighbor_index in range(start, end):
        if neighbor_index == index:
            continue
        neighbor = ranges[neighbor_index]
        if (
            math.isfinite(neighbor)
            and abs(neighbor - reading) <= max_deviation_m
        ):
            return True
    return False


def filter_scan_noise(
    msg: LaserScan,
    config: ScanNoiseFilterConfig,
) -> LaserScan:
    filtered = _clone_scan_metadata(msg)
    range_min, range_max = _effective_filter_range(msg, config)

    cleaned_ranges = []
    for reading in msg.ranges:
        value = float(reading)
        if not math.isfinite(value) or value < range_min or value > range_max:
            cleaned_ranges.append(float("inf"))
        else:
            cleaned_ranges.append(value)

    window = max(0, int(config.speckle_filter_window))
    if window > 0:
        speckle_max_range = float(config.speckle_max_range_m)
        speckle_max_deviation = max(0.0, float(config.speckle_max_deviation_m))
        speckle_filtered = list(cleaned_ranges)
        for index, reading in enumerate(cleaned_ranges):
            if not math.isfinite(reading) or reading > speckle_max_range:
                continue
            if not _has_supporting_neighbor(
                cleaned_ranges,
                index,
                window,
                speckle_max_deviation,
            ):
                speckle_filtered[index] = float("inf")
        cleaned_ranges = speckle_filtered

    filtered.ranges = cleaned_ranges
    return filtered


class ScanNoiseFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_noise_filter")

        self.declare_parameter("source_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_clean")
        self.declare_parameter("filter_range_min_m", 0.4)
        self.declare_parameter("filter_range_max_m", 20.0)
        self.declare_parameter("speckle_filter_window", 2)
        self.declare_parameter("speckle_max_range_m", 12.0)
        self.declare_parameter("speckle_max_deviation_m", 0.30)

        self._source_topic = str(self.get_parameter("source_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._config = ScanNoiseFilterConfig(
            filter_range_min_m=float(
                self.get_parameter("filter_range_min_m").value
            ),
            filter_range_max_m=float(
                self.get_parameter("filter_range_max_m").value
            ),
            speckle_filter_window=int(
                self.get_parameter("speckle_filter_window").value
            ),
            speckle_max_range_m=float(
                self.get_parameter("speckle_max_range_m").value
            ),
            speckle_max_deviation_m=float(
                self.get_parameter("speckle_max_deviation_m").value
            ),
        )

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(
            LaserScan,
            self._output_topic,
            output_qos,
        )
        self._subscription = self.create_subscription(
            LaserScan,
            self._source_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "scan_noise_filter ready "
            f"(source={self._source_topic}, output={self._output_topic}, "
            f"range={self._config.filter_range_min_m:.2f}.."
            f"{self._config.filter_range_max_m:.1f}m, "
            f"window={self._config.speckle_filter_window}, "
            f"max_deviation={self._config.speckle_max_deviation_m:.2f}m)"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._publisher.publish(filter_scan_noise(msg, self._config))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanNoiseFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
