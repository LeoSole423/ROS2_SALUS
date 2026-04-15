import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    map_tools_dir = get_package_share_directory("map_tools")
    keepout_mask_yaml = _resolve_config_file_path(gps_wpf_dir, "keepout_mask.yaml")
    default_nav2_params_file = _resolve_config_file_path(
        gps_wpf_dir, "nav2_global_v2_sim_rolling_params.yaml"
    )
    default_collision_monitor_params_file = _resolve_config_file_path(
        gps_wpf_dir, "collision_monitor_v2.yaml"
    )
    default_global_localization_params_file = _resolve_config_file_path(
        gps_wpf_dir, "localization_global_v2.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    invert_measured_steer_sign = LaunchConfiguration("invert_measured_steer_sign")
    nav_start_delay_s = LaunchConfiguration("nav_start_delay_s")
    use_keepout = LaunchConfiguration("use_keepout")
    vx_deadband_mps = LaunchConfiguration("vx_deadband_mps")
    vx_min_effective_mps = LaunchConfiguration("vx_min_effective_mps")
    invert_steer_from_cmd_vel = LaunchConfiguration("invert_steer_from_cmd_vel")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params_file")
    keepout_mask_yaml_arg = LaunchConfiguration("keepout_mask_yaml")
    global_localization_params_file = LaunchConfiguration("global_localization_params_file")
    custom_urdf = LaunchConfiguration("custom_urdf")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    model_name = LaunchConfiguration("model_name")
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")
    datum_lat = LaunchConfiguration("datum_lat")
    datum_lon = LaunchConfiguration("datum_lon")
    datum_yaw_deg = LaunchConfiguration("datum_yaw_deg")
    datum_setter = LaunchConfiguration("datum_setter")
    enable_map_gps_absolute_measurement = LaunchConfiguration(
        "enable_map_gps_absolute_measurement"
    )
    map_gps_absolute_topic = LaunchConfiguration("map_gps_absolute_topic")
    map_gps_pose_covariance_xy = LaunchConfiguration("map_gps_pose_covariance_xy")
    map_gps_fromll_service = LaunchConfiguration("map_gps_fromll_service")
    map_gps_fromll_service_fallback = LaunchConfiguration("map_gps_fromll_service_fallback")
    map_gps_fromll_wait_timeout_s = LaunchConfiguration("map_gps_fromll_wait_timeout_s")
    enable_gps_course_heading = LaunchConfiguration("enable_gps_course_heading")
    gps_course_heading_min_distance_m = LaunchConfiguration(
        "gps_course_heading_min_distance_m"
    )
    gps_course_heading_min_speed_mps = LaunchConfiguration("gps_course_heading_min_speed_mps")
    gps_course_heading_max_abs_steer_deg = LaunchConfiguration(
        "gps_course_heading_max_abs_steer_deg"
    )
    gps_course_heading_max_abs_yaw_rate_rps = LaunchConfiguration(
        "gps_course_heading_max_abs_yaw_rate_rps"
    )
    gps_course_heading_invalid_hold_s = LaunchConfiguration(
        "gps_course_heading_invalid_hold_s"
    )
    gps_course_heading_max_sample_dt_s = LaunchConfiguration(
        "gps_course_heading_max_sample_dt_s"
    )
    gps_course_heading_publish_hz = LaunchConfiguration("gps_course_heading_publish_hz")
    gps_course_heading_yaw_variance_rad2 = LaunchConfiguration(
        "gps_course_heading_yaw_variance_rad2"
    )
    gps_course_heading_hold_yaw_variance_multiplier = LaunchConfiguration(
        "gps_course_heading_hold_yaw_variance_multiplier"
    )
    gps_profile = LaunchConfiguration("gps_profile")
    launch_web_app = LaunchConfiguration("launch_web_app")
    ws_host = LaunchConfiguration("ws_host")
    web_app_port = LaunchConfiguration("web_app_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument("invert_measured_steer_sign", default_value="True"),
            DeclareLaunchArgument("nav_start_delay_s", default_value="4.0"),
            DeclareLaunchArgument("use_keepout", default_value="True"),
            DeclareLaunchArgument("vx_deadband_mps", default_value="0.01"),
            DeclareLaunchArgument("vx_min_effective_mps", default_value="0.5"),
            DeclareLaunchArgument("invert_steer_from_cmd_vel", default_value="True"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params_file,
            ),
            DeclareLaunchArgument(
                "collision_monitor_params_file",
                default_value=default_collision_monitor_params_file,
            ),
            DeclareLaunchArgument("keepout_mask_yaml", default_value=keepout_mask_yaml),
            DeclareLaunchArgument(
                "global_localization_params_file",
                default_value=default_global_localization_params_file,
            ),
            DeclareLaunchArgument(
                "custom_urdf",
                default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf"),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(gps_wpf_dir, "worlds", "vacio.world"),
            ),
            DeclareLaunchArgument("world_name", default_value="vacio"),
            DeclareLaunchArgument("model_name", default_value="quad_ackermann_viewer_safe"),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.1"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.01"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.1"),
            DeclareLaunchArgument("datum_lat", default_value="-31.4858037"),
            DeclareLaunchArgument("datum_lon", default_value="-64.2410570"),
            # Convencion fija operativa para `global v2`: por default el robot
            # arranca mirando al Este (`datum_yaw_deg = 0.0` en ROS ENU).
            DeclareLaunchArgument("datum_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument("datum_setter", default_value="false"),
            DeclareLaunchArgument("enable_map_gps_absolute_measurement", default_value="true"),
            DeclareLaunchArgument("map_gps_absolute_topic", default_value="/gps/odometry_map"),
            DeclareLaunchArgument("map_gps_pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("map_gps_fromll_service", default_value="/fromLL"),
            DeclareLaunchArgument(
                "map_gps_fromll_service_fallback",
                default_value="/navsat_transform/fromLL",
            ),
            DeclareLaunchArgument("map_gps_fromll_wait_timeout_s", default_value="0.2"),
            DeclareLaunchArgument("enable_gps_course_heading", default_value="true"),
            # Con GPS RTK simulado podemos cerrar el heading por avance antes y
            # con más confianza que con el perfil ideal/m8n anterior.
            DeclareLaunchArgument("gps_course_heading_min_distance_m", default_value="1.0"),
            DeclareLaunchArgument("gps_course_heading_min_speed_mps", default_value="0.4"),
            # En curvas el heading inferido por desplazamiento GPS deja de ser
            # una buena referencia del cuerpo Ackermann. Endurecemos el gating
            # en simulacion para aceptarlo solo en tramos claramente rectos.
            DeclareLaunchArgument("gps_course_heading_max_abs_steer_deg", default_value="3.0"),
            DeclareLaunchArgument("gps_course_heading_max_abs_yaw_rate_rps", default_value="0.06"),
            # Cuando el vehiculo entra en una curva leve, dejar caer el heading
            # en un solo ciclo hace que el EKF global reoriente `map->odom`
            # demasiado brusco. Mantenemos el ultimo yaw valido por una ventana
            # corta y con menor confianza para suavizar esa transicion.
            DeclareLaunchArgument("gps_course_heading_invalid_hold_s", default_value="0.8"),
            # Limita cuan viejo puede ser el segmento GPS usado para inferir
            # el heading. En curvas largas, usar una cuerda demasiado antigua
            # reinyecta un yaw que ya no representa la tangente actual.
            DeclareLaunchArgument("gps_course_heading_max_sample_dt_s", default_value="2.5"),
            DeclareLaunchArgument("gps_course_heading_publish_hz", default_value="10.0"),
            DeclareLaunchArgument("gps_course_heading_yaw_variance_rad2", default_value="0.05"),
            DeclareLaunchArgument(
                "gps_course_heading_hold_yaw_variance_multiplier",
                default_value="4.0",
            ),
            # Sim global defaults to the ideal profile so LL/map debugging is not
            # polluted by GNSS noise unless the operator opts into RTK/M8N.
            DeclareLaunchArgument("gps_profile", default_value="ideal"),
            DeclareLaunchArgument("launch_web_app", default_value="True"),
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_app_port", default_value="8766"),
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
                        # En simulacion global mantenemos el fix RTK congelado
                        # cuando el vehiculo esta quieto para que el EKF global
                        # no amplifique el jitter estacionario del GPS.
                        "gps_hold_when_stationary": True,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "sim_v2_base.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "custom_urdf": custom_urdf,
                    "world": world,
                    "world_name": world_name,
                    "model_name": model_name,
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
                        "approx_fromll_fallback_enabled": True,
                        "approx_fromll_datum_lat": ParameterValue(datum_lat, value_type=float),
                        "approx_fromll_datum_lon": ParameterValue(datum_lon, value_type=float),
                        "approx_fromll_datum_yaw_deg": ParameterValue(
                            datum_yaw_deg, value_type=float
                        ),
                        "approx_fromll_zero_threshold_m": 1.0e-3,
                        "approx_fromll_min_distance_for_fallback_m": 0.5,
                        "fromll_frame": "map",
                        "map_frame": "map",
                        "gps_topic": "/gps/fix",
                        "cmd_vel_safe_topic": "/cmd_vel_safe",
                        "cmd_vel_final_topic": "/cmd_vel_final",
                        "forward_cmd_vel_safe_without_goal": True,
                        "brake_topic": "/cmd_vel_safe",
                        "manual_cmd_topic": "/cmd_vel_safe",
                        "teleop_cmd_topic": "/cmd_vel_teleop",
                        "brake_publish_count": 5,
                        "brake_publish_interval_s": 0.1,
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
            Node(
                package="navegacion_gps",
                executable="gps_course_heading",
                name="gps_course_heading",
                output="screen",
                condition=IfCondition(enable_gps_course_heading),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "gps_topic": "/gps/fix",
                        "odom_topic": "/odometry/local",
                        "drive_telemetry_topic": "/controller/drive_telemetry",
                        "output_topic": "/gps/course_heading",
                        "debug_topic": "/gps/course_heading/debug",
                        "base_frame": "base_footprint",
                        "min_distance_m": ParameterValue(
                            gps_course_heading_min_distance_m, value_type=float
                        ),
                        "min_speed_mps": ParameterValue(
                            gps_course_heading_min_speed_mps, value_type=float
                        ),
                        "max_abs_steer_deg": ParameterValue(
                            gps_course_heading_max_abs_steer_deg, value_type=float
                        ),
                        "max_abs_yaw_rate_rps": ParameterValue(
                            gps_course_heading_max_abs_yaw_rate_rps, value_type=float
                        ),
                        "invalid_hold_s": ParameterValue(
                            gps_course_heading_invalid_hold_s, value_type=float
                        ),
                        "max_sample_dt_s": ParameterValue(
                            gps_course_heading_max_sample_dt_s, value_type=float
                        ),
                        "publish_hz": ParameterValue(
                            gps_course_heading_publish_hz, value_type=float
                        ),
                        "yaw_variance_rad2": ParameterValue(
                            gps_course_heading_yaw_variance_rad2, value_type=float
                        ),
                        "hold_yaw_variance_multiplier": ParameterValue(
                            gps_course_heading_hold_yaw_variance_multiplier,
                            value_type=float,
                        ),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "localization_global_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "drive_telemetry_topic": "/controller/drive_telemetry",
                    "imu_topic": "/imu/data",
                    "gps_topic": "/gps/fix",
                    "wheelbase_m": wheelbase_m,
                    "invert_measured_steer_sign": invert_measured_steer_sign,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                    "global_localization_params_file": global_localization_params_file,
                    "enable_map_gps_absolute_measurement": enable_map_gps_absolute_measurement,
                    "map_gps_absolute_topic": map_gps_absolute_topic,
                    "map_gps_pose_covariance_xy": map_gps_pose_covariance_xy,
                    "map_gps_fromll_service": map_gps_fromll_service,
                    "map_gps_fromll_service_fallback": map_gps_fromll_service_fallback,
                    "map_gps_fromll_wait_timeout_s": map_gps_fromll_wait_timeout_s,
                    # Simulacion global: con `gps_course_heading` activo dejamos
                    # `navsat_transform` desacoplado del yaw local para no mezclar
                    # dos fuentes distintas de heading global.
                    "navsat_use_odometry_yaw": "false",
                    "enable_gps_course_heading": enable_gps_course_heading,
                    "gps_course_heading_topic": "/gps/course_heading",
                    "datum_lat": datum_lat,
                    "datum_lon": datum_lon,
                    "datum_yaw_deg": datum_yaw_deg,
                    "datum_setter": datum_setter,
                }.items(),
            ),
            TimerAction(
                period=nav_start_delay_s,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(gps_wpf_dir, "launch", "nav_global_v2.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "use_keepout": use_keepout,
                            "nav2_params_file": nav2_params_file,
                            "collision_monitor_params_file": collision_monitor_params_file,
                            "keepout_mask_yaml": keepout_mask_yaml_arg,
                        }.items(),
                    )
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(map_tools_dir, "launch", "no_go_editor.launch.py")
                ),
                launch_arguments={
                    "ws_host": ws_host,
                    "ws_port": web_app_port,
                    "gps_topic": "/gps/fix",
                    "odom_topic": "/odometry/global",
                    "map_frame": "map",
                    "launch_nav_command_server": "false",
                    "sensor_bridge_enabled": "false",
                    "fixed_datum_lat": datum_lat,
                    "fixed_datum_lon": datum_lon,
                    "fixed_datum_yaw_deg": datum_yaw_deg,
                    "fixed_datum_source": "sim_global_v2_fixed",
                }.items(),
                condition=IfCondition(launch_web_app),
            ),
        ]
    )
