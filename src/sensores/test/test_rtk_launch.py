"""Evaluate launch conditions only: never execute hardware nodes."""
import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
launch_ros = pytest.importorskip("launch_ros.actions")


@pytest.mark.parametrize("enabled,manager,wanted", [
    ("True", "true", ["rtk_source_manager", "rtk_bridge"]),
    ("true", "false", ["rtk_bridge"]),
    ("false", "true", []),
    ("1", "1", ["rtk_source_manager", "rtk_bridge"]),
])
def test_rtk_chain_is_exclusive_and_case_insensitive(enabled, manager, wanted):
    path = Path(__file__).resolve().parents[1] / "launch/mavros.launch.py"
    spec = importlib.util.spec_from_file_location("rtk_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = launch.LaunchContext()
    context.launch_configurations.update(enable_rtk=enabled, enable_rtk_source_manager=manager)
    selected = []
    for action in description.entities:
        if not isinstance(action, launch_ros.Node):
            continue
        name = action.node_executable
        if name in {"rtk_source_manager", "rtk_bridge"} and action.condition.evaluate(context):
            selected.append(name)
    assert selected == wanted
