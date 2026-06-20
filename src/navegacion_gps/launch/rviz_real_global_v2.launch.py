import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_file(path):
    with open(path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


def _build_robot_state_publisher(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    custom_urdf = LaunchConfiguration("custom_urdf").perform(context)
    use_sim_time_bool = use_sim_time == "True"

    robot_description = _read_file(custom_urdf)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time_bool,
                    "robot_description": robot_description,
                }
            ],
        )
    ]


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_rviz = os.path.join(gps_wpf_dir, "config", "rviz_global_v2.rviz")
    default_urdf = os.path.join(gps_wpf_dir, "models", "cuatri_real_v2.urdf")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    launch_robot_state_publisher = LaunchConfiguration("launch_robot_state_publisher")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="Path to the RViz config file",
            ),
            DeclareLaunchArgument(
                "custom_urdf",
                default_value=default_urdf,
                description="Path to custom URDF for RViz RobotModel",
            ),
            DeclareLaunchArgument(
                "launch_robot_state_publisher",
                default_value="false",
                description=(
                    "Launch a local robot_state_publisher for RViz "
                    "when no other publisher is running"
                ),
            ),
            OpaqueFunction(
                function=_build_robot_state_publisher,
                condition=IfCondition(launch_robot_state_publisher),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    rviz_config,
                    "--ros-args",
                    "-p",
                    PythonExpression(["'use_sim_time:=' + str(", use_sim_time, ")"]),
                ],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            ),
        ]
    )
