import os
import re
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


def _resolve_config_file_path(package_share_dir: str, filename: str) -> str:
    package_share_path = Path(package_share_dir)
    default_path = package_share_path / "config" / filename
    try:
        workspace_root = package_share_path.parents[3]
        source_path = workspace_root / "src" / "navegacion_gps" / "config" / filename
        if source_path.parent.exists():
            return str(source_path)
    except IndexError:
        pass
    return str(default_path)


def _extract_world_name_from_sdf(world_path: str) -> str:
    try:
        with open(world_path, "r", encoding="utf-8") as file_handle:
            world_contents = file_handle.read()
        match = re.search(r"<world\s+name\s*=\s*['\"]([^'\"]+)['\"]", world_contents)
        if match:
            return match.group(1)
    except OSError:
        pass
    return ""


def _materialize_bridge_config_for_world(bridge_config_path: str, world_name: str) -> str:
    with open(bridge_config_path, "r", encoding="utf-8") as file_handle:
        bridge_config_contents = file_handle.read()
    patched_contents = re.sub(
        r"/world/[^/\s]+/",
        f"/world/{world_name}/",
        bridge_config_contents,
    )
    tmp_file = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".yaml",
        prefix="bridge_config_v2_",
        delete=False,
    )
    with tmp_file:
        tmp_file.write(patched_contents)
    return tmp_file.name


def _spawn_robot(context):
    custom_urdf = LaunchConfiguration("custom_urdf").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "True"
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_roll = LaunchConfiguration("spawn_roll").perform(context)
    spawn_pitch = LaunchConfiguration("spawn_pitch").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    robot_description = _read_file(custom_urdf)

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_description,
                }
            ],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-name",
                model_name,
                "-file",
                custom_urdf,
                "-x",
                spawn_x,
                "-y",
                spawn_y,
                "-z",
                spawn_z,
                "-R",
                spawn_roll,
                "-P",
                spawn_pitch,
                "-Y",
                spawn_yaw,
            ],
        ),
    ]


def _build_gz_bridge(context, *, bridge_config: str):
    world_path = LaunchConfiguration("world").perform(context)
    world_name = _extract_world_name_from_sdf(world_path) or LaunchConfiguration(
        "world_name"
    ).perform(context)
    bridge_config_path = _materialize_bridge_config_for_world(bridge_config, world_name)
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config_path}],
        )
    ]


def _build_joint_state_bridge(context):
    world_path = LaunchConfiguration("world").perform(context)
    world_name = _extract_world_name_from_sdf(world_path) or LaunchConfiguration(
        "world_name"
    ).perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    joint_state_topic = (
        f"/world/{world_name}/model/{model_name}/joint_state"
        "@sensor_msgs/msg/JointState[gz.msgs.Model"
    )
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[joint_state_topic],
            remappings=[
                (f"/world/{world_name}/model/{model_name}/joint_state", "/joint_states")
            ],
        )
    ]


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(gps_wpf_dir, "worlds", "vacio.world")
    bridge_config = _resolve_config_file_path(gps_wpf_dir, "bridge_config_v2.yaml")
    lidar_to_scan_params = _resolve_config_file_path(
        gps_wpf_dir, "pointcloud_to_laserscan.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument(
                "custom_urdf",
                default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf"),
            ),
            DeclareLaunchArgument("world", default_value=world_path),
            DeclareLaunchArgument("world_name", default_value="vacio"),
            DeclareLaunchArgument(
                "model_name", default_value="quad_ackermann_viewer_safe"
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={"gz_args": [TextSubstitution(text="-r "), world]}.items(),
            ),
            OpaqueFunction(
                function=_build_gz_bridge, kwargs={"bridge_config": bridge_config}
            ),
            OpaqueFunction(function=_build_joint_state_bridge),
            OpaqueFunction(function=_spawn_robot),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[
                    lidar_to_scan_params,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"output_qos": "sensor_data"},
                ],
                remappings=[("cloud_in", "/scan_3d"), ("scan", "/scan")],
            ),
        ]
    )
