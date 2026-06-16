"""Escenario de validación del scan_ground_filter en la rampa.

Levanta el stack de sim local completo (Nav2 + collision_monitor) sobre
`slope_lidar.world` con el URDF plano (`cuatri_real.urdf`) y agrega el nodo de
medición `scan_ground_validation`, que mide FP en el costmap local y eventos de
freno falsos durante `duration_s` y escribe un JSON.

Uso (A/B):
    # baseline (sin filtro)
    ros2 launch navegacion_gps validate_scan_ground.launch.py \
        enable_scan_ground_filter:=False label:=baseline \
        output_path:=/tmp/baseline.json duration_s:=60

    # con filtro
    ros2 launch navegacion_gps validate_scan_ground.launch.py \
        enable_scan_ground_filter:=True label:=filtered \
        output_path:=/tmp/filtered.json duration_s:=60

El script `scripts/run_scan_ground_validation.sh` encadena ambas y compara.

Nota: los FP del costmap aparecen aun con el robot quieto (el piso de la rampa
entra como obstáculo). Los eventos de freno solo se cuentan si se comanda avance;
para medirlos, mandá una meta con nav_command_server durante la corrida.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    slope_world = os.path.join(gps_wpf_dir, "worlds", "slope_lidar.world")
    flat_urdf = os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf")
    default_bt_xml = os.path.join(
        gps_wpf_dir,
        "config",
        "navigate_to_pose_w_replanning_and_recovery_no_spin.xml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("enable_scan_ground_filter", default_value="False"),
            DeclareLaunchArgument("scan_ground_min_height", default_value="0.10"),
            DeclareLaunchArgument("scan_ground_max_height", default_value="2.50"),
            DeclareLaunchArgument("enable_lidar_obstacle_filter", default_value="False"),
            DeclareLaunchArgument(
                "lidar_obstacle_ground_distance_threshold", default_value="0.05"
            ),
            DeclareLaunchArgument("use_rviz", default_value="False"),
            DeclareLaunchArgument(
                "publish_static_map_odom",
                default_value="True",
                description="Publica un TF estático map->odom (identidad). El "
                "escenario de rampa solo trae EKF local (sin frame map), así que "
                "sin esto el planner global aborta y el BT de Nav2 hace BackUp en "
                "loop ('retrocede de la nada'). Con esto se puede navegar con "
                "metas. Poné False si vas a correr localización global real.",
            ),
            DeclareLaunchArgument(
                "nav_to_pose_bt_xml",
                default_value=default_bt_xml,
                description="BT de NavigateToPose. Default = el de main (con "
                "BackUp de recovery). Pasá el '..._no_backup.xml' para correr la "
                "rampa sin retroceso.",
            ),
            DeclareLaunchArgument("label", default_value="run"),
            DeclareLaunchArgument("duration_s", default_value="60.0"),
            DeclareLaunchArgument("output_path", default_value=""),
            DeclareLaunchArgument("world", default_value=slope_world),
            DeclareLaunchArgument("custom_urdf", default_value=flat_urdf),
            DeclareLaunchArgument("collision_backup_recovery_enabled", default_value="False"),
            DeclareLaunchArgument("collision_backup_distance_m", default_value="0.50"),
            DeclareLaunchArgument("collision_backup_speed_mps", default_value="0.25"),
            DeclareLaunchArgument("collision_backup_cooldown_s", default_value="8.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "sim_local_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "world": LaunchConfiguration("world"),
                    "custom_urdf": LaunchConfiguration("custom_urdf"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "use_keepout": "False",
                    "gz_args": "-s -r ",
                    "nav_to_pose_bt_xml": LaunchConfiguration("nav_to_pose_bt_xml"),
                    "enable_scan_ground_filter": LaunchConfiguration(
                        "enable_scan_ground_filter"
                    ),
                    "scan_ground_min_height": LaunchConfiguration(
                        "scan_ground_min_height"
                    ),
                    "scan_ground_max_height": LaunchConfiguration(
                        "scan_ground_max_height"
                    ),
                    "enable_lidar_obstacle_filter": LaunchConfiguration(
                        "enable_lidar_obstacle_filter"
                    ),
                    "lidar_obstacle_ground_distance_threshold": LaunchConfiguration(
                        "lidar_obstacle_ground_distance_threshold"
                    ),
                    "collision_backup_recovery_enabled": LaunchConfiguration(
                        "collision_backup_recovery_enabled"
                    ),
                    "collision_backup_distance_m": LaunchConfiguration(
                        "collision_backup_distance_m"
                    ),
                    "collision_backup_speed_mps": LaunchConfiguration(
                        "collision_backup_speed_mps"
                    ),
                    "collision_backup_cooldown_s": LaunchConfiguration(
                        "collision_backup_cooldown_s"
                    ),
                }.items(),
            ),
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
                    }
                ],
            ),
            # El escenario de rampa solo trae EKF local (sin frame map). Sin un
            # map->odom, el planner global aborta y el BT de Nav2 entra en BackUp
            # loop ("retrocede de la nada"). Publicamos identidad para poder
            # navegar con metas en este escenario de prueba.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_map_odom_validate",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("publish_static_map_odom")
                ),
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
            ),
        ]
    )
