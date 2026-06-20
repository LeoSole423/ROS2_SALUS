from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


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


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_nav2_params = _resolve_config_file_path(gps_wpf_dir, "nav2_global_v2_params.yaml")
    default_collision_monitor_params = _resolve_config_file_path(
        gps_wpf_dir, "collision_monitor_v2.yaml"
    )
    bt_xml = _resolve_config_file_path(
        gps_wpf_dir, "navigate_to_pose_w_replanning_and_recovery_no_spin.xml"
    )
    bt_through_poses_xml = _resolve_config_file_path(
        gps_wpf_dir, "navigate_through_poses_w_replanning_and_recovery_no_spin.xml"
    )
    keepout_launch = str(Path(gps_wpf_dir) / "launch" / "keepout_filters_v2.launch.py")
    default_keepout_mask = _resolve_config_file_path(gps_wpf_dir, "keepout_mask.yaml")
    default_keepout_overrides = _resolve_config_file_path(
        gps_wpf_dir, "nav2_local_v2_keepout_overrides.yaml"
    )
    default_no_keepout_overrides = _resolve_config_file_path(
        gps_wpf_dir, "nav2_local_v2_no_keepout_overrides.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_keepout = LaunchConfiguration("use_keepout")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params_file")
    keepout_mask_yaml = LaunchConfiguration("keepout_mask_yaml")
    lidar_scan_topic = LaunchConfiguration("lidar_scan_topic")
    nav_through_poses_bt_xml = LaunchConfiguration("nav_through_poses_bt_xml")
    selected_nav2_overrides_file = PythonExpression(
        [
            "'",
            default_keepout_overrides,
            "' if '",
            use_keepout,
            "' == 'True' else '",
            default_no_keepout_overrides,
            "'",
        ]
    )
    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key="",
            param_rewrites={
                "default_nav_to_pose_bt_xml": bt_xml,
                "default_nav_through_poses_bt_xml": nav_through_poses_bt_xml,
                "local_costmap.local_costmap.ros__parameters.voxel_layer.scan_marking.topic": (
                    lidar_scan_topic
                ),
                "local_costmap.local_costmap.ros__parameters.voxel_layer.scan_clearing.topic": (
                    lidar_scan_topic
                ),
                "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic": (
                    lidar_scan_topic
                ),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    configured_collision_monitor_params = ParameterFile(
        RewrittenYaml(
            source_file=collision_monitor_params_file,
            root_key="",
            param_rewrites={
                "collision_monitor.ros__parameters.scan.topic": lidar_scan_topic,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    configured_nav2_overrides = ParameterFile(
        selected_nav2_overrides_file,
        allow_substs=True,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    lifecycle_node_names = [
        "planner_server",
        "controller_server",
        "smoother_server",
        "bt_navigator",
        "behavior_server",
        "waypoint_follower",
    ]
    collision_monitor_lifecycle_node_names = [
        "collision_monitor",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("use_keepout", default_value="True"),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument(
                "collision_monitor_params_file",
                default_value=default_collision_monitor_params,
            ),
            DeclareLaunchArgument("keepout_mask_yaml", default_value=default_keepout_mask),
            DeclareLaunchArgument("lidar_scan_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "nav_through_poses_bt_xml",
                default_value=bt_through_poses_xml,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(keepout_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "keepout_mask_yaml": keepout_mask_yaml,
                    "keepout_mask_frame": "map",
                }.items(),
                condition=IfCondition(use_keepout),
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[
                    configured_nav2_params,
                    configured_nav2_overrides,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    configured_nav2_params,
                    configured_nav2_overrides,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[
                    configured_nav2_params,
                    configured_nav2_overrides,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    configured_nav2_params,
                    configured_nav2_overrides,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=remappings,
            ),
            Node(
                package="navegacion_gps",
                executable="path_clearance_validator",
                name="path_clearance_validator",
                output="screen",
                parameters=[
                    configured_nav2_params,
                    configured_nav2_overrides,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[
                    configured_nav2_params,
                    configured_nav2_overrides,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[
                    configured_nav2_params,
                    configured_nav2_overrides,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=remappings,
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[
                    configured_collision_monitor_params,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="polygon_stamped_republisher",
                name="stop_zone_republisher",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"input_topic": "/stop_zone_raw"},
                    {"output_topic": "/stop_zone"},
                    {"republish_period_s": 1.0},
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="polygon_stamped_republisher",
                name="critical_slow_zone_republisher",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"input_topic": "/critical_slow_zone_raw"},
                    {"output_topic": "/critical_slow_zone"},
                    {"republish_period_s": 1.0},
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="polygon_stamped_republisher",
                name="slow_zone_republisher",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {"input_topic": "/slow_zone_raw"},
                    {"output_topic": "/slow_zone"},
                    {"republish_period_s": 1.0},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_global_navigation_v2",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "autostart": True,
                        "node_names": lifecycle_node_names,
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="collision_monitor_lifecycle_manager_global_v2",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "autostart": True,
                        "node_names": collision_monitor_lifecycle_node_names,
                    }
                ],
            ),
        ]
    )
