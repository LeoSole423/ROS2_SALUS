import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("stop_real", Path(__file__).with_name("stop_real_global_v2.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LaunchScopeTests(unittest.TestCase):
    def test_supported_real_launches(self):
        for filename in module.LAUNCHES:
            self.assertTrue(module.is_real_launch(["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch", "/ros2_ws/src/navegacion_gps/launch/" + filename]))
            self.assertTrue(module.is_real_launch(["ros2", "launch", "navegacion_gps", filename]))

    def test_does_not_stop_simulation_diagnostics_or_unrelated_processes(self):
        for args in [
            ["ros2", "launch", "navegacion_gps", "sim_global_v2.launch.py"],
            ["ros2", "launch", "navegacion_gps", "real_global_v2.launch.py", "--show-args"],
            ["python3", "real_global_v2.launch.py"],
            ["bash", "-c", "ros2 launch real_global_v2.launch.py"],
            ["ros2", "run", "sensores", "rtk_source_manager"],
        ]:
            self.assertFalse(module.is_real_launch(args))


if __name__ == "__main__":
    unittest.main()
