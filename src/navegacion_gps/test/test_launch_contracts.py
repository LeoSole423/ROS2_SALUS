from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_ROOT / "launch"


def _read_launch_file(name: str) -> str:
    return (LAUNCH_DIR / name).read_text(encoding="utf-8")


def test_simulation_realism_mode_reuses_nav2_only_launch() -> None:
    simulation_launch = _read_launch_file("simulacion.launch.py")

    assert '"realism_mode"' in simulation_launch
    assert 'default_value="True"' in simulation_launch
    assert '"gps_profile"' in simulation_launch
    assert "nav2_only.launch.py" in simulation_launch
    assert "collision_monitor_lidar_only.yaml" in simulation_launch
    assert "else ('m8n'" in simulation_launch


def test_real_and_simulation_share_navigation_contracts() -> None:
    simulation_launch = _read_launch_file("simulacion.launch.py")
    real_launch = _read_launch_file("real.launch.py")

    assert "dual_ekf_navsat_params.yaml" in simulation_launch
    assert "dual_ekf_navsat_params.yaml" in real_launch
    assert 'default_value="/gps/fix"' in simulation_launch
    assert 'default_value="/gps/fix"' in real_launch
    assert '"odometry/local"' in simulation_launch
    assert '"odometry/local"' in real_launch
    assert '"/gps/rtk_status"' in simulation_launch


def test_simulation_launch_exposes_localization_profiles() -> None:
    simulation_launch = _read_launch_file("simulacion.launch.py")

    assert '"sim_localization_profile"' in simulation_launch
    assert '"sim_localization_params_file"' in simulation_launch
    assert "dual_ekf_navsat_params.sim_navsat_imu_heading.yaml" in simulation_launch
    assert "dual_ekf_navsat_params.sim_decouple_global_yaw.yaml" in simulation_launch
    assert "dual_ekf_navsat_params.sim_decouple_global_twist_only.yaml" in simulation_launch
    assert (
        "dual_ekf_navsat_params.sim_decouple_global_linear_twist_only.yaml"
        in simulation_launch
    )
    assert "dual_ekf_navsat_params.sim_gps_only_global.yaml" not in simulation_launch


def test_global_v2_lidar_scan_topic_contract_is_reversible() -> None:
    real_global = _read_launch_file("real_global_v2.launch.py")
    sim_global = _read_launch_file("sim_global_v2.launch.py")
    nav_global = _read_launch_file("nav_global_v2.launch.py")
    lidar_filter_default_arg = (
        'DeclareLaunchArgument("enable_lidar_obstacle_filter", '
        'default_value="False")'
    )
    scan_filter_default_arg = (
        'DeclareLaunchArgument("enable_scan_noise_filter", default_value="True")'
    )
    scan_filter_output_arg = (
        'DeclareLaunchArgument("scan_noise_filter_output", '
        'default_value="/scan_clean")'
    )

    for launch_contents in (real_global, sim_global):
        assert lidar_filter_default_arg in launch_contents
        assert scan_filter_default_arg in launch_contents
        assert scan_filter_output_arg in launch_contents
        assert "' if '" in launch_contents
        assert "enable_lidar_obstacle_filter" in launch_contents
        assert "'.lower() == 'true' else ('" in launch_contents
        assert "scan_noise_filter_output" in launch_contents
        assert "enable_scan_noise_filter" in launch_contents
        assert "'.lower() == 'true' else '/scan')" in launch_contents
        assert 'condition=IfCondition(enable_legacy_scan_noise_filter)' in launch_contents
        assert '"source_topic": "/scan"' in launch_contents
        assert '"output_topic": scan_noise_filter_output' in launch_contents
        assert '"scan_topic": lidar_scan_topic' in launch_contents
        assert '"lidar_scan_topic": effective_lidar_scan_topic' in launch_contents

    assert (
        "local_costmap.local_costmap.ros__parameters."
        "voxel_layer.scan_marking.topic"
    ) in nav_global
    assert (
        "local_costmap.local_costmap.ros__parameters."
        "voxel_layer.scan_clearing.topic"
    ) in nav_global
    assert "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic" in nav_global
    assert "collision_monitor.ros__parameters.scan.topic" in nav_global
    assert 'executable="path_clearance_validator"' in nav_global
    assert 'name="path_clearance_validator"' in nav_global
    assert "DeclareLaunchArgument(\"lidar_scan_topic\", default_value=\"/scan\")" in nav_global
