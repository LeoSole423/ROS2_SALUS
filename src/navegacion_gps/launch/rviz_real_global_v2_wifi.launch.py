import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_config_file_path(package_share_dir: str, filename: str) -> str:
    package_share_path = Path(package_share_dir)
    default_path = package_share_path / "config" / filename
    try:
        workspace_root = package_share_path.parents[3]
        source_path = workspace_root / "src" / "navegacion_gps" / "config" / filename
        if source_path.exists():
            return str(source_path)
    except IndexError:
        pass
    return str(default_path)


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_rviz = _resolve_config_file_path(gps_wpf_dir, "rviz_global_v2_wifi.rviz")
    default_urdf = os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("custom_urdf", default_value=default_urdf),
            DeclareLaunchArgument("launch_robot_state_publisher", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "rviz_real_global_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "custom_urdf": LaunchConfiguration("custom_urdf"),
                    "launch_robot_state_publisher": LaunchConfiguration(
                        "launch_robot_state_publisher"
                    ),
                }.items(),
            ),
        ]
    )
