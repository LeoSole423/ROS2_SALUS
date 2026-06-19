"""Medición de KPIs del scan_ground_filter en el robot REAL.

A diferencia de `validate_scan_ground.launch.py` (que levanta Gazebo con la rampa
`slope_lidar.world`), este launch NO levanta ningún stack: asume que el robot real
ya está corriendo (p. ej. `real_local_v2.launch.py`) y solo agrega el nodo de
medición `scan_ground_validation` con `use_sim_time:=False`. Mide FP en el costmap
local y eventos de freno falsos (`/cmd_vel` vs `/cmd_vel_safe`) durante
`duration_s` y escribe un JSON.

A/B en el robot real (prendiendo/apagando el filtro en el stack):

    # baseline (stack sin filtro)
    ros2 launch navegacion_gps real_local_v2.launch.py \
        enable_scan_ground_filter:=False
    # en otra terminal, mientras navegás una meta:
    ros2 launch navegacion_gps validate_scan_ground_real.launch.py \
        label:=real_baseline output_path:=/tmp/real_baseline.json duration_s:=60

    # con filtro (relanzar el stack con el flag en True)
    ros2 launch navegacion_gps real_local_v2.launch.py \
        enable_scan_ground_filter:=True
    ros2 launch navegacion_gps validate_scan_ground_real.launch.py \
        label:=real_filtered output_path:=/tmp/real_filtered.json duration_s:=60

Nota: los eventos de freno solo se cuentan si se comanda avance; para medirlos,
mandá una meta con nav_command_server durante la corrida.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="En el robot real va en False (reloj de sistema).",
            ),
            DeclareLaunchArgument("label", default_value="real_run"),
            DeclareLaunchArgument("duration_s", default_value="60.0"),
            DeclareLaunchArgument("output_path", default_value=""),
            DeclareLaunchArgument("costmap_topic", default_value="/local_costmap/costmap"),
            DeclareLaunchArgument(
                "costmap_updates_topic",
                default_value="/local_costmap/costmap_updates",
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("cmd_vel_safe_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("occupied_threshold", default_value="100"),
            Node(
                package="navegacion_gps",
                executable="scan_ground_validation",
                name="scan_ground_validation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "label": LaunchConfiguration("label"),
                        "duration_s": ParameterValue(
                            LaunchConfiguration("duration_s"), value_type=float
                        ),
                        "output_path": LaunchConfiguration("output_path"),
                        "costmap_topic": LaunchConfiguration("costmap_topic"),
                        "costmap_updates_topic": LaunchConfiguration(
                            "costmap_updates_topic"
                        ),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "cmd_vel_safe_topic": LaunchConfiguration("cmd_vel_safe_topic"),
                        "occupied_threshold": ParameterValue(
                            LaunchConfiguration("occupied_threshold"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
