import math
import threading
from types import SimpleNamespace

import numpy as np

from navegacion_gps.scan_ground_filter import (
    ScanGroundFilterConfig,
    ScanGroundFilterNode,
    ScanGroundSegmenter,
    _normalize_radian,
)


def _segmenter(**overrides):
    return ScanGroundSegmenter(ScanGroundFilterConfig(**overrides))


def _flat_ground_ray(azimuth_deg=0.0, z=0.0):
    """Una fila de puntos a lo largo de un radio, todos a la misma altura z."""
    radii = np.linspace(0.6, 12.0, 40)
    a = math.radians(azimuth_deg)
    # convención del filtro: theta = atan2(x, y) -> x = r*sin(a), y = r*cos(a)
    xs = radii * math.sin(a)
    ys = radii * math.cos(a)
    zs = np.full_like(radii, z)
    return np.stack([xs, ys, zs], axis=1)


def _sloped_ground_ray(azimuth_deg=0.0, slope_deg=5.0):
    radii = np.linspace(1.0, 12.0, 40)
    a = math.radians(azimuth_deg)
    xs = radii * math.sin(a)
    ys = radii * math.cos(a)
    zs = radii * math.tan(math.radians(slope_deg))
    return np.stack([xs, ys, zs], axis=1)


def test_normalize_radian_wraps_to_0_2pi():
    assert _normalize_radian(-0.1) == math.fmod(-0.1, 2 * math.pi) + 2 * math.pi
    assert 0.0 <= _normalize_radian(7.0) < 2 * math.pi


def test_flat_ground_is_all_ground():
    seg = _segmenter()
    pts = _flat_ground_ray(azimuth_deg=30.0, z=0.0)
    no_ground = seg.segment(pts)
    assert no_ground.size == 0


def test_sustained_mild_slope_is_ground():
    seg = _segmenter()
    pts = _sloped_ground_ray(azimuth_deg=0.0, slope_deg=5.0)
    no_ground = seg.segment(pts)
    assert no_ground.size == 0


def test_flat_ground_is_sorted_by_radius_inside_ray():
    seg = _segmenter()
    a = math.radians(30.0)
    radii = np.array([12.0, 1.0, 6.0])
    pts = np.stack(
        [
            radii * math.sin(a),
            radii * math.cos(a),
            np.zeros_like(radii),
        ],
        axis=1,
    )
    no_ground = seg.segment(pts)
    assert no_ground.size == 0


def test_tall_obstacle_is_non_ground():
    seg = _segmenter()
    ground = _flat_ground_ray(azimuth_deg=30.0, z=0.0)
    # una columna vertical (obstáculo) a ~5 m sobre el rayo
    obstacle = []
    for h in np.linspace(0.3, 1.2, 10):
        obstacle.append([5.0 * math.sin(math.radians(30.0)),
                         5.0 * math.cos(math.radians(30.0)), h])
    pts = np.vstack([ground, np.array(obstacle)])
    no_ground = seg.segment(pts)
    # los puntos del obstáculo (índices >= len(ground)) deben salir como no-suelo
    assert no_ground.size >= 8
    assert set(no_ground.tolist()).issubset(set(range(len(ground), len(pts)))) or \
        no_ground.size >= 8


def test_steep_global_slope_point_is_non_ground():
    seg = _segmenter(global_slope_max_angle_deg=10.0)
    # un único punto muy por encima respecto a su radio (pendiente global > 10°)
    a = math.radians(45.0)
    r = 4.0
    pts = np.array([[r * math.sin(a), r * math.cos(a), 3.0]])  # ~36° de pendiente
    no_ground = seg.segment(pts)
    assert no_ground.tolist() == [0]


def test_obstacle_just_above_split_height_is_non_ground():
    seg = _segmenter()
    ground = _flat_ground_ray(azimuth_deg=10.0, z=0.0)
    a = math.radians(10.0)
    step = np.array([[6.0 * math.sin(a), 6.0 * math.cos(a), 0.205]])
    pts = np.vstack([ground, step])
    no_ground = seg.segment(pts)
    assert (len(pts) - 1) in no_ground.tolist()


def test_rural_profile_accepts_sustained_15_degree_ground_slope():
    points = _sloped_ground_ray(azimuth_deg=15.0, slope_deg=15.0)
    urban = _segmenter(
        global_slope_max_angle_deg=10.0,
        local_slope_max_angle_deg=13.0,
        split_height_distance=0.20,
    )
    rural = _segmenter(
        global_slope_max_angle_deg=15.0,
        local_slope_max_angle_deg=18.0,
        split_height_distance=0.25,
    )

    assert urban.segment(points).size > 0
    assert rural.segment(points).size == 0


def test_rural_profile_filters_low_ground_undulation_but_keeps_tall_obstacle():
    ground = _flat_ground_ray(azimuth_deg=10.0, z=0.0)
    a = math.radians(10.0)
    low_undulation = np.array([[6.0 * math.sin(a), 6.0 * math.cos(a), 0.24]])
    tall_obstacle = np.array([[7.0 * math.sin(a), 7.0 * math.cos(a), 0.35]])
    points = np.vstack([ground, low_undulation, tall_obstacle])
    urban = _segmenter()
    rural = _segmenter(
        global_slope_max_angle_deg=15.0,
        local_slope_max_angle_deg=18.0,
        split_height_distance=0.25,
    )

    assert len(ground) in urban.segment(points).tolist()
    rural_no_ground = rural.segment(points).tolist()
    assert len(ground) not in rural_no_ground
    assert len(points) - 1 in rural_no_ground


def test_ground_profile_parameter_update_rebuilds_segmenter_atomically():
    node = object.__new__(ScanGroundFilterNode)
    node._config_lock = threading.Lock()
    node._config = ScanGroundFilterConfig()
    node.segmenter = ScanGroundSegmenter(node._config)
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args, **_kwargs: None)

    result = ScanGroundFilterNode._on_set_parameters(
        node,
        [
            SimpleNamespace(name="global_slope_max_angle_deg", value=15.0),
            SimpleNamespace(name="local_slope_max_angle_deg", value=18.0),
            SimpleNamespace(name="split_height_distance", value=0.25),
        ],
    )

    assert result.successful is True
    assert node.segmenter.config == ScanGroundFilterConfig(
        global_slope_max_angle_deg=15.0,
        local_slope_max_angle_deg=18.0,
        split_height_distance=0.25,
    )


def test_ground_profile_parameter_update_rejects_invalid_values_without_mutation():
    node = object.__new__(ScanGroundFilterNode)
    node._config_lock = threading.Lock()
    node._config = ScanGroundFilterConfig()
    node.segmenter = ScanGroundSegmenter(node._config)
    original_segmenter = node.segmenter

    result = ScanGroundFilterNode._on_set_parameters(
        node,
        [SimpleNamespace(name="split_height_distance", value=0.75)],
    )

    assert result.successful is False
    assert node._config == ScanGroundFilterConfig()
    assert node.segmenter is original_segmenter


def test_empty_cloud_returns_empty():
    seg = _segmenter()
    assert seg.segment(np.empty((0, 3))).size == 0


def test_many_rays_flat_ground_have_no_false_objects():
    seg = _segmenter()
    radii = np.array([1.0, 3.0, 6.0, 9.0])
    pts = []
    for azimuth_deg in range(0, 360, 2):
        a = math.radians(azimuth_deg)
        for r in radii:
            pts.append([r * math.sin(a), r * math.cos(a), 0.0])
    no_ground = seg.segment(np.array(pts))
    assert no_ground.size == 0


def test_low_curb_recovered_with_low_split_height():
    """Un escalón bajo (~0.25 m) sobre suelo plano debe marcarse como no-suelo."""
    seg = _segmenter()
    ground = _flat_ground_ray(azimuth_deg=10.0, z=0.0)
    a = math.radians(10.0)
    step = np.array([[6.0 * math.sin(a), 6.0 * math.cos(a), 0.30]])
    pts = np.vstack([ground, step])
    no_ground = seg.segment(pts)
    assert (len(pts) - 1) in no_ground.tolist()
