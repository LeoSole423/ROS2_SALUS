import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Costmap, CostmapMetaData
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path

from navegacion_gps.path_clearance_validator import (
    CachedValidationResult,
    PathClearanceValidatorNode,
    check_path_clearance,
    costmap_view_from_msg,
    _path_signature,
)


class _FakeClock:
    class _Now:
        nanoseconds = int(10.0e9)

    def now(self):
        return self._Now()


def _costmap(*, width=20, height=20, resolution=1.0, costs=None):
    msg = Costmap()
    msg.header.frame_id = "map"
    msg.metadata = CostmapMetaData()
    msg.metadata.resolution = float(resolution)
    msg.metadata.size_x = int(width)
    msg.metadata.size_y = int(height)
    msg.metadata.origin.position.x = 0.0
    msg.metadata.origin.position.y = 0.0
    msg.metadata.origin.orientation.w = 1.0
    data = [0] * (width * height)
    for (x, y), value in (costs or {}).items():
        data[int(y) * width + int(x)] = int(value)
    msg.data = data
    return costmap_view_from_msg(msg, received_time_sec=10.0)


def _path(points):
    path = Path()
    path.header.frame_id = "map"
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def _check(path, costmap, **kwargs):
    defaults = {
        "max_check_distance_m": 12.0,
        "sample_step_m": 1.0,
        "high_cost_threshold": 100,
        "lethal_cost_threshold": 253,
        "min_consecutive_high_cost_samples": 3,
        "lateral_offsets_m": [0.0],
    }
    defaults.update(kwargs)
    return check_path_clearance(path, costmap, **defaults)


def test_clear_path_on_empty_costmap_is_valid():
    result = _check(_path([(1, 1), (10, 1)]), _costmap())

    assert result.is_valid is True
    assert result.reason == "clear"


def test_path_crossing_lethal_cost_is_invalid():
    result = _check(
        _path([(1, 1), (10, 1)]),
        _costmap(costs={(4, 1): 254}),
    )

    assert result.is_valid is False
    assert result.reason == "lethal_cost"
    assert result.invalid_indices == [0]


def test_path_crossing_sustained_high_inflation_is_invalid():
    result = _check(
        _path([(1, 1), (10, 1)]),
        _costmap(costs={(4, 1): 120, (5, 1): 125, (6, 1): 130}),
    )

    assert result.is_valid is False
    assert result.reason == "sustained_high_cost"
    assert len(result.invalid_indices) >= 3


def test_single_gray_sample_does_not_invalidate_path():
    result = _check(
        _path([(1, 1), (10, 1)]),
        _costmap(costs={(4, 1): 140}),
    )

    assert result.is_valid is True


def test_far_gray_samples_beyond_check_distance_do_not_invalidate_path():
    result = _check(
        _path([(1, 1), (25, 1)]),
        _costmap(width=30, costs={(16, 1): 140, (17, 1): 140, (18, 1): 140}),
        max_check_distance_m=8.0,
    )

    assert result.is_valid is True


def test_lateral_offsets_detect_nearby_inflated_cost():
    result = _check(
        _path([(1, 1), (10, 1)]),
        _costmap(costs={(4, 2): 140, (5, 2): 140, (6, 2): 140}),
        lateral_offsets_m=[0.0, 1.0, -1.0],
    )

    assert result.is_valid is False
    assert result.reason == "sustained_high_cost"


def test_validator_fails_open_without_costmap():
    node = object.__new__(PathClearanceValidatorNode)
    node.enabled = True
    node._lock = threading.Lock()
    node._costmap = None
    node._warn_open = lambda _reason: None

    response = PathClearanceValidatorNode._on_validate(
        node,
        IsPathValid.Request(path=_path([(1, 1), (10, 1)])),
        IsPathValid.Response(),
    )

    assert response.is_valid is True
    assert list(response.invalid_pose_indices) == []


def test_validator_fails_open_with_stale_costmap():
    node = object.__new__(PathClearanceValidatorNode)
    node.enabled = True
    node._lock = threading.Lock()
    node._costmap = _costmap()
    node._costmap = type(node._costmap)(
        **{**node._costmap.__dict__, "stamp_sec": 1.0}
    )
    node.costmap_timeout_s = 1.5
    node.get_clock = lambda: _FakeClock()
    node._warn_open = lambda _reason: None

    response = PathClearanceValidatorNode._on_validate(
        node,
        IsPathValid.Request(path=_path([(1, 1), (10, 1)])),
        IsPathValid.Response(),
    )

    assert response.is_valid is True
    assert list(response.invalid_pose_indices) == []


def test_validator_uses_recent_cache_for_same_path_and_costmap():
    path = _path([(1, 1), (10, 1)])
    costmap = _costmap(costs={(4, 1): 254})
    node = object.__new__(PathClearanceValidatorNode)
    node.enabled = True
    node._lock = threading.Lock()
    node._costmap = costmap
    node.min_validation_period_s = 1.0
    node._last_cache = CachedValidationResult(
        path_signature=_path_signature(path),
        costmap_stamp_sec=costmap.stamp_sec,
        checked_at_monotonic_s=time.monotonic(),
        is_valid=True,
        invalid_indices=(),
    )

    response = PathClearanceValidatorNode._on_validate(
        node,
        IsPathValid.Request(path=path),
        IsPathValid.Response(),
    )

    assert response.is_valid is True
    assert list(response.invalid_pose_indices) == []


def test_validator_dynamic_parameters_update_runtime_values():
    class _Logger:
        def info(self, _message):
            pass

    node = object.__new__(PathClearanceValidatorNode)
    node._lock = threading.Lock()
    node._last_cache = object()
    node.get_logger = lambda: _Logger()

    result = PathClearanceValidatorNode._on_set_parameters(
        node,
        [
            SimpleNamespace(name="enabled", value=False),
            SimpleNamespace(name="costmap_timeout_s", value=4.0),
            SimpleNamespace(name="min_validation_period_s", value=0.75),
            SimpleNamespace(name="lateral_offsets_m", value=[0.0, 0.5]),
        ],
    )

    assert result.successful is True
    assert node.enabled is False
    assert node.costmap_timeout_s == 4.0
    assert node.min_validation_period_s == 0.75
    assert node.lateral_offsets_m == [0.0, 0.5]
    assert node._last_cache is None
