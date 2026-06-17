import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_rviz = os.path.join(gps_wpf_dir, "config", "rviz_local_v2.rviz")
    keepout_mask_yaml = os.path.join(gps_wpf_dir, "config", "keepout_mask.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    invert_measured_steer_sign = LaunchConfiguration("invert_measured_steer_sign")
    nav_start_delay_s = LaunchConfiguration("nav_start_delay_s")
    use_keepout = LaunchConfiguration("use_keepout")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    vx_deadband_mps = LaunchConfiguration("vx_deadband_mps")
    vx_min_effective_mps = LaunchConfiguration("vx_min_effective_mps")
    invert_steer_from_cmd_vel = LaunchConfiguration("invert_steer_from_cmd_vel")
    localization_params_file = LaunchConfiguration("localization_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params_file")
    keepout_mask_yaml_arg = LaunchConfiguration("keepout_mask_yaml")
    nav_to_pose_bt_xml = LaunchConfiguration("nav_to_pose_bt_xml")
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")
    gps_profile = LaunchConfiguration("gps_profile")
    collision_backup_recovery_enabled = LaunchConfiguration(
        "collision_backup_recovery_enabled"
    )
    collision_backup_distance_m = LaunchConfiguration("collision_backup_distance_m")
    collision_backup_speed_mps = LaunchConfiguration("collision_backup_speed_mps")
    collision_backup_cooldown_s = LaunchConfiguration("collision_backup_cooldown_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument("invert_measured_steer_sign", default_value="True"),
            DeclareLaunchArgument("nav_start_delay_s", default_value="4.0"),
            DeclareLaunchArgument("use_keepout", default_value="True"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("vx_deadband_mps", default_value="0.01"),
            DeclareLaunchArgument("vx_min_effective_mps", default_value="0.5"),
            DeclareLaunchArgument("invert_steer_from_cmd_vel", default_value="True"),
            DeclareLaunchArgument(
                "localization_params_file",
                default_value=os.path.join(gps_wpf_dir, "config", "localization_v2.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(gps_wpf_dir, "config", "nav2_local_v2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "collision_monitor_params_file",
                default_value=os.path.join(gps_wpf_dir, "config", "collision_monitor_v2.yaml"),
            ),
            DeclareLaunchArgument("keepout_mask_yaml", default_value=keepout_mask_yaml),
            DeclareLaunchArgument(
                "nav_to_pose_bt_xml",
                default_value=os.path.join(
                    gps_wpf_dir,
                    "config",
                    "navigate_to_pose_w_replanning_and_recovery_no_spin.xml",
                ),
                description="BT de NavigateToPose reenviado a nav_local_v2. Default "
                "= el de main (con BackUp). Pasá el '..._no_backup.xml' para sim "
                "sin retroceso.",
            ),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.1"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.01"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.1"),
            # Shared simulated GPS profiles:
            # - ideal: architecture/smoke tests
            # - f9p_rtk: RTK-fixed approximation
            # - m8n: degraded single-band GPS
            DeclareLaunchArgument("gps_profile", default_value="ideal"),
            # Escenario/pipeline LiDAR (se reenvían a sim_v2_base). Defaults =
            # comportamiento actual (mundo vacío, URDF plano, filtro de suelo off).
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(gps_wpf_dir, "worlds", "vacio.world"),
            ),
            DeclareLaunchArgument(
                "custom_urdf",
                default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf"),
            ),
            DeclareLaunchArgument("enable_scan_ground_filter", default_value="False"),
            DeclareLaunchArgument("scan_ground_min_height", default_value="0.10"),
            DeclareLaunchArgument("scan_ground_max_height", default_value="2.50"),
            DeclareLaunchArgument("enable_lidar_obstacle_filter", default_value="False"),
            DeclareLaunchArgument(
                "lidar_obstacle_ground_distance_threshold", default_value="0.05"
            ),
            DeclareLaunchArgument("collision_backup_recovery_enabled", default_value="False"),
            DeclareLaunchArgument("collision_backup_distance_m", default_value="0.50"),
            DeclareLaunchArgument("collision_backup_speed_mps", default_value="0.25"),
            DeclareLaunchArgument("collision_backup_cooldown_s", default_value="8.0"),
            DeclareLaunchArgument("gz_args", default_value="-r "),
            Node(
                package="navegacion_gps",
                executable="sim_sensor_normalizer_v2",
                name="sim_sensor_normalizer_v2",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "gps_profile": gps_profile,
                        "gps_rtk_status_topic": "/gps/rtk_status",
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "sim_v2_base.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "world": LaunchConfiguration("world"),
                    "custom_urdf": LaunchConfiguration("custom_urdf"),
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
                    "gz_args": LaunchConfiguration("gz_args"),
                }.items(),
            ),
            Node(
                package="controller_server",
                executable="controller_server_node",
                name="vehicle_controller_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "transport_backend": "sim_gazebo",
                        "serial_port": "/dev/null",
                        "serial_baud": 115200,
                        "serial_tx_hz": 50.0,
                        "max_reverse_mps": 1.30,
                        "max_abs_angular_z": 0.4,
                        "wheelbase_m": 0.94,
                        "steering_limit_rad": 0.5235987756,
                        "operational_steering_limit_rad": 0.3141592654,
                        "vx_deadband_mps": ParameterValue(
                            vx_deadband_mps, value_type=float
                        ),
                        "vx_min_effective_mps": ParameterValue(
                            vx_min_effective_mps, value_type=float
                        ),
                        "invert_steer_from_cmd_vel": ParameterValue(
                            invert_steer_from_cmd_vel, value_type=bool
                        ),
                        "sim_cmd_vel_topic": "/cmd_vel_gazebo",
                        "sim_odom_topic": "/odom_raw",
                        "sim_joint_states_topic": "/joint_states",
                        "sim_front_left_steer_joint": "front_left_steer_joint",
                        "sim_front_right_steer_joint": "front_right_steer_joint",
                        "sim_wheelbase_m": 0.94,
                        "sim_track_width_m": 0.75,
                        "sim_max_steering_angle_rad": 0.5235987756,
                        "sim_telemetry_timeout_s": 0.5,
                        "sim_invert_actuation_steer_sign": True,
                        "sim_invert_measured_steer_sign": True,
                        "sim_max_joint_odom_steer_delta_deg": 5.0,
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="nav_command_server",
                name="nav_command_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "fromll_service": "/fromLL",
                        "fromll_service_fallback": "/navsat_transform/fromLL",
                        "fromll_wait_timeout_s": 2.0,
                        "fromll_frame": "odom",
                        "map_frame": "odom",
                        "gps_topic": "/gps/fix",
                        "cmd_vel_raw_topic": "/cmd_vel",
                        "cmd_vel_safe_topic": "/cmd_vel_safe",
                        "cmd_vel_final_topic": "/cmd_vel_final",
                        "forward_cmd_vel_safe_without_goal": True,
                        "brake_topic": "/cmd_vel_safe",
                        "manual_cmd_topic": "/cmd_vel_safe",
                        "teleop_cmd_topic": "/cmd_vel_teleop",
                        "brake_publish_count": 5,
                        "brake_publish_interval_s": 0.1,
                        "collision_backup_recovery_enabled": ParameterValue(
                            collision_backup_recovery_enabled, value_type=bool
                        ),
                        "collision_backup_distance_m": ParameterValue(
                            collision_backup_distance_m, value_type=float
                        ),
                        "collision_backup_speed_mps": ParameterValue(
                            collision_backup_speed_mps, value_type=float
                        ),
                        "collision_backup_cooldown_s": ParameterValue(
                            collision_backup_cooldown_s, value_type=float
                        ),
                        "manual_cmd_timeout_s": 0.4,
                        "manual_watchdog_hz": 10.0,
                        "nav_telemetry_hz": 5.0,
                        "telemetry_topic": "/nav_command_server/telemetry",
                        "event_topic": "/nav_command_server/events",
                        "set_goal_service": "/nav_command_server/set_goal_ll",
                        "cancel_goal_service": "/nav_command_server/cancel_goal",
                        "brake_service": "/nav_command_server/brake",
                        "set_manual_mode_service": "/nav_command_server/set_manual_mode",
                        "get_state_service": "/nav_command_server/get_state",
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "localization_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "wheelbase_m": wheelbase_m,
                    "invert_measured_steer_sign": invert_measured_steer_sign,
                    "localization_params_file": localization_params_file,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                }.items(),
            ),
            TimerAction(
                period=nav_start_delay_s,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(gps_wpf_dir, "launch", "nav_local_v2.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "use_keepout": use_keepout,
                            "nav2_params_file": nav2_params_file,
                            "collision_monitor_params_file": collision_monitor_params_file,
                            "keepout_mask_yaml": keepout_mask_yaml_arg,
                            "nav_to_pose_bt_xml": nav_to_pose_bt_xml,
                        }.items(),
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
