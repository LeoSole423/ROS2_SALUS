import math

from sensor_msgs.msg import LaserScan

from navegacion_gps.scan_noise_filter import (
    ScanNoiseFilterConfig,
    filter_scan_noise,
)


def _scan(ranges):
    msg = LaserScan()
    msg.header.frame_id = "base_footprint"
    msg.angle_min = -1.0
    msg.angle_max = 1.0
    msg.angle_increment = 0.1
    msg.time_increment = 0.01
    msg.scan_time = 0.1
    msg.range_min = 0.4
    msg.range_max = 20.0
    msg.ranges = [float(value) for value in ranges]
    msg.intensities = [float(index) for index, _ in enumerate(ranges)]
    return msg


def _config(**overrides):
    values = {
        "filter_range_min_m": 0.4,
        "filter_range_max_m": 20.0,
        "speckle_filter_window": 2,
        "speckle_max_range_m": 12.0,
        "speckle_max_deviation_m": 0.30,
    }
    values.update(overrides)
    return ScanNoiseFilterConfig(**values)


def test_isolated_point_is_removed() -> None:
    msg = _scan([math.inf, math.inf, 4.0, math.inf, math.inf])

    filtered = filter_scan_noise(msg, _config())

    assert math.isinf(filtered.ranges[2])


def test_wide_cluster_is_preserved() -> None:
    msg = _scan([math.inf, 5.0, 5.1, 4.9, math.inf])

    filtered = filter_scan_noise(msg, _config())

    assert all(
        math.isclose(actual, expected, rel_tol=1.0e-6)
        for actual, expected in zip(filtered.ranges[1:4], [5.0, 5.1, 4.9])
    )


def test_nan_and_inf_ranges_do_not_break_filter() -> None:
    msg = _scan([math.nan, math.inf, 3.0, 3.1, -math.inf])

    filtered = filter_scan_noise(msg, _config())

    assert math.isinf(filtered.ranges[0])
    assert math.isinf(filtered.ranges[1])
    assert all(
        math.isclose(actual, expected, rel_tol=1.0e-6)
        for actual, expected in zip(filtered.ranges[2:4], [3.0, 3.1])
    )
    assert math.isinf(filtered.ranges[4])


def test_ranges_outside_min_max_are_cleaned() -> None:
    msg = _scan([0.2, 0.4, 19.9, 20.1, 21.0])

    filtered = filter_scan_noise(
        msg,
        _config(speckle_filter_window=0),
    )

    assert math.isinf(filtered.ranges[0])
    assert math.isclose(filtered.ranges[1], 0.4, rel_tol=1.0e-6)
    assert math.isclose(filtered.ranges[2], 19.9, rel_tol=1.0e-6)
    assert math.isinf(filtered.ranges[3])
    assert math.isinf(filtered.ranges[4])


def test_header_and_metadata_are_preserved() -> None:
    msg = _scan([2.0, 2.1])
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456

    filtered = filter_scan_noise(msg, _config())

    assert filtered.header == msg.header
    assert filtered.angle_min == msg.angle_min
    assert filtered.angle_max == msg.angle_max
    assert filtered.angle_increment == msg.angle_increment
    assert filtered.time_increment == msg.time_increment
    assert filtered.scan_time == msg.scan_time
    assert filtered.range_min == msg.range_min
    assert filtered.range_max == msg.range_max
    assert filtered.intensities == msg.intensities
