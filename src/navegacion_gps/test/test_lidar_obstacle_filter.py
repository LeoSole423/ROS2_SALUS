import math

import numpy as np
from std_msgs.msg import Header

from navegacion_gps.lidar_obstacle_filter import (
    LidarObstacleFilterConfig,
    TiltGate,
    TiltGateConfig,
    VoxelPersistenceConfig,
    VoxelPersistenceFilter,
    filter_obstacle_points,
    points_to_laserscan,
    quaternion_to_roll_pitch,
)


def _config(**overrides):
    values = {
        "min_ground_points": 10,
        "ransac_iterations": 32,
    }
    values.update(overrides)
    return LidarObstacleFilterConfig(**values)


def _ground_grid(slope_x=0.0, slope_y=0.0):
    xs = np.linspace(0.5, 8.0, 16)
    ys = np.linspace(-2.0, 2.0, 9)
    points = []
    for x in xs:
        for y in ys:
            points.append([x, y, (slope_x * x) + (slope_y * y)])
    return np.asarray(points, dtype=np.float32)


def test_flat_ground_does_not_generate_obstacles() -> None:
    obstacles = filter_obstacle_points(_ground_grid(), _config())

    assert obstacles.shape == (0, 3)


def test_sloped_ground_does_not_generate_obstacles() -> None:
    obstacles = filter_obstacle_points(_ground_grid(slope_x=0.10), _config())

    assert obstacles.shape == (0, 3)


def test_lateral_sloped_ground_does_not_generate_obstacles() -> None:
    obstacles = filter_obstacle_points(_ground_grid(slope_y=0.12), _config())

    assert obstacles.shape == (0, 3)


def test_obstacle_cluster_above_ground_is_preserved() -> None:
    ground = _ground_grid()
    obstacle = np.asarray(
        [
            [3.0, 0.08, 0.35],
            [3.02, 0.10, 0.42],
            [3.04, 0.12, 0.55],
            [3.05, 0.13, 0.68],
        ],
        dtype=np.float32,
    )

    obstacles = filter_obstacle_points(
        np.vstack([ground, obstacle]),
        _config(),
    )

    assert len(obstacles) >= 2
    assert np.all(obstacles[:, 2] > 0.22)


def test_isolated_point_is_removed_by_voxel_density() -> None:
    isolated = np.asarray([[3.0, 0.0, 0.50]], dtype=np.float32)

    obstacles = filter_obstacle_points(
        np.vstack([_ground_grid(), isolated]),
        _config(),
    )

    assert obstacles.shape == (0, 3)


def test_points_to_laserscan_projects_nearest_obstacle() -> None:
    config = _config(angle_increment=math.radians(1.0), range_max=12.0)
    header = Header()
    header.frame_id = "base_footprint"
    points = np.asarray([[4.0, 0.0, 0.5], [3.0, 0.0, 0.6]], dtype=np.float32)

    scan = points_to_laserscan(points, header, config)
    center_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))

    assert scan.header.frame_id == "base_footprint"
    assert math.isclose(scan.ranges[center_index], 3.0, rel_tol=1.0e-6)


def test_quaternion_to_roll_pitch_recovers_pitch() -> None:
    pitch = math.radians(8.0)
    qy = math.sin(pitch / 2.0)
    qw = math.cos(pitch / 2.0)

    roll_out, pitch_out = quaternion_to_roll_pitch(0.0, qy, 0.0, qw)

    assert math.isclose(roll_out, 0.0, abs_tol=1.0e-6)
    assert math.isclose(pitch_out, pitch, abs_tol=1.0e-6)


def _gate(**overrides) -> TiltGate:
    return TiltGate(TiltGateConfig(**overrides))


def _gate_sequence(gate: TiltGate, pitches_deg) -> list:
    return [gate.update(0.0, math.radians(pitch)) for pitch in pitches_deg]


def test_tilt_gate_accepts_steady_nominal_attitude() -> None:
    assert _gate_sequence(_gate(), [0.0, 0.5, -0.5, 1.0]) == [
        True,
        True,
        True,
        True,
    ]


def test_tilt_gate_rejects_offset_beyond_limit() -> None:
    gate = _gate(max_offset_deg=7.0, max_jump_deg=90.0)

    assert _gate_sequence(gate, [0.0, 8.0, 0.0]) == [True, False, True]


def test_tilt_gate_rejects_sudden_jumps_then_recovers() -> None:
    gate = _gate(max_offset_deg=7.0, max_jump_deg=3.0)

    # Cada transicion brusca cuesta exactamente un frame rechazado; una vez
    # estabilizada la actitud (aunque sea distinta), se vuelve a aceptar.
    assert _gate_sequence(gate, [0.0, 5.0, 5.0, 0.0, 0.0]) == [
        True,
        False,
        True,
        False,
        True,
    ]


def test_tilt_gate_honors_nominal_pitch() -> None:
    gate = _gate(nominal_pitch_deg=10.0, max_jump_deg=90.0)

    assert _gate_sequence(gate, [10.0, 12.0, 0.0]) == [True, True, False]


def test_tilt_gate_disabled_accepts_everything() -> None:
    gate = _gate(enabled=False)

    assert _gate_sequence(gate, [0.0, 45.0, -45.0]) == [True, True, True]


def test_tilt_gate_rejects_roll_offset() -> None:
    gate = _gate(max_offset_deg=7.0, max_jump_deg=90.0)

    assert gate.update(math.radians(8.0), 0.0) is False


def _persistence(**overrides) -> VoxelPersistenceFilter:
    return VoxelPersistenceFilter(
        VoxelPersistenceConfig(**overrides),
        voxel_size_x=0.25,
        voxel_size_y=0.25,
    )


def test_persistence_suppresses_single_frame_ghost() -> None:
    persistence = _persistence()
    ghost = np.asarray([[3.0, 0.0, 0.5]], dtype=np.float32)
    empty = np.empty((0, 3), dtype=np.float32)

    assert persistence.filter(ghost).shape == (0, 3)
    assert persistence.filter(empty).shape == (0, 3)
    assert persistence.filter(empty).shape == (0, 3)


def test_persistence_passes_repeated_obstacle() -> None:
    persistence = _persistence(min_hits=2, window=3)
    obstacle = np.asarray(
        [[3.0, 0.0, 0.5], [3.05, 0.05, 0.6]],
        dtype=np.float32,
    )

    first = persistence.filter(obstacle)
    second = persistence.filter(obstacle)

    assert first.shape == (0, 3)
    assert len(second) == 2


def test_persistence_forgets_stale_voxels() -> None:
    persistence = _persistence(min_hits=2, window=3)
    obstacle = np.asarray([[3.0, 0.0, 0.5]], dtype=np.float32)
    empty = np.empty((0, 3), dtype=np.float32)

    persistence.filter(obstacle)
    persistence.filter(empty)
    persistence.filter(empty)
    # La unica observacion vieja ya salio de la ventana: volver a aparecer
    # cuenta como primera vez y se suprime de nuevo.
    assert persistence.filter(obstacle).shape == (0, 3)


def test_persistence_disabled_is_passthrough() -> None:
    persistence = _persistence(enabled=False)
    ghost = np.asarray([[3.0, 0.0, 0.5]], dtype=np.float32)

    assert len(persistence.filter(ghost)) == 1
