import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from navegacion_gps.datum_profile_resolver import resolve_selected_datum


def _read_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


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


def _build_robot_state_publisher(context):
    custom_urdf = LaunchConfiguration("custom_urdf").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
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
        )
    ]


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    map_tools_dir = get_package_share_directory("map_tools")
    sensores_dir = get_package_share_directory("sensores")

    default_rviz = _resolve_config_file_path(gps_wpf_dir, "rviz_global_v2.rviz")
    lidar_to_scan_params = _resolve_config_file_path(
        gps_wpf_dir, "pointcloud_to_laserscan_real.yaml"
    )
    default_global_localization_params = _resolve_config_file_path(
        gps_wpf_dir, "localization_global_v2.yaml"
    )
    default_nav2_params = _resolve_config_file_path(
        gps_wpf_dir, "nav2_global_v2_real_rolling_params.yaml"
    )
    default_collision_monitor_params = _resolve_config_file_path(
        gps_wpf_dir, "collision_monitor_v2.yaml"
    )
    default_keepout_mask = _resolve_config_file_path(gps_wpf_dir, "keepout_mask.yaml")
    default_datum_lat, default_datum_lon, default_datum_yaw_deg, default_datums_file = (
        resolve_selected_datum(gps_wpf_dir)
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    invert_measured_steer_sign = LaunchConfiguration("invert_measured_steer_sign")
    enable_rtk = LaunchConfiguration("enable_rtk")
    lidar_config_path = LaunchConfiguration("lidar_config_path")
    fcu_url = LaunchConfiguration("fcu_url")
    use_cyclone_dds = LaunchConfiguration("use_cyclone_dds")
    nav_start_delay_s = LaunchConfiguration("nav_start_delay_s")
    use_keepout = LaunchConfiguration("use_keepout")
    launch_web_app = LaunchConfiguration("launch_web_app")
    ws_host = LaunchConfiguration("ws_host")
    web_app_port = LaunchConfiguration("web_app_port")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    enable_scan_wifi_debug = LaunchConfiguration("enable_scan_wifi_debug")
    scan_wifi_debug_topic = LaunchConfiguration("scan_wifi_debug_topic")
    scan_wifi_debug_publish_hz = LaunchConfiguration("scan_wifi_debug_publish_hz")
    scan_wifi_debug_beam_stride = LaunchConfiguration("scan_wifi_debug_beam_stride")
    scan_wifi_debug_range_max_m = LaunchConfiguration("scan_wifi_debug_range_max_m")
    vx_deadband_mps = LaunchConfiguration("vx_deadband_mps")
    vx_min_effective_mps = LaunchConfiguration("vx_min_effective_mps")
    invert_steer_from_cmd_vel = LaunchConfiguration("invert_steer_from_cmd_vel")
    global_localization_params_file = LaunchConfiguration("global_localization_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params_file")
    keepout_mask_yaml = LaunchConfiguration("keepout_mask_yaml")
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")
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
    gps_course_heading_require_rtk = LaunchConfiguration("gps_course_heading_require_rtk")
    gps_course_heading_allowed_rtk_statuses = LaunchConfiguration(
        "gps_course_heading_allowed_rtk_statuses"
    )
    gps_course_heading_rtk_status_max_age_s = LaunchConfiguration(
        "gps_course_heading_rtk_status_max_age_s"
    )
    gps_rtk_status_topic = LaunchConfiguration("gps_rtk_status_topic")
    enable_map_gps_absolute_measurement = LaunchConfiguration(
        "enable_map_gps_absolute_measurement"
    )
    map_gps_absolute_topic = LaunchConfiguration("map_gps_absolute_topic")
    map_gps_pose_covariance_xy = LaunchConfiguration("map_gps_pose_covariance_xy")
    map_gps_fromll_service = LaunchConfiguration("map_gps_fromll_service")
    map_gps_fromll_service_fallback = LaunchConfiguration("map_gps_fromll_service_fallback")
    map_gps_fromll_wait_timeout_s = LaunchConfiguration("map_gps_fromll_wait_timeout_s")
    navsat_use_odometry_yaw = LaunchConfiguration("navsat_use_odometry_yaw")
    datum_lat = LaunchConfiguration("datum_lat")
    datum_lon = LaunchConfiguration("datum_lon")
    datum_yaw_deg = LaunchConfiguration("datum_yaw_deg")
    datums_file = LaunchConfiguration("datums_file")
    effective_enable_rtk = PythonExpression(
        [
            "'true' if ('",
            enable_rtk,
            "'.lower() == 'true' or ('",
            enable_gps_course_heading,
            "'.lower() == 'true' and '",
            gps_course_heading_require_rtk,
            "'.lower() == 'true')) else 'false'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            # En `real_global_v2` la cadena RTK debe quedar observable por
            # default para diagnosticar el GNSS aunque el heading GPS siga
            # deshabilitado.
            DeclareLaunchArgument("enable_rtk", default_value="True"),
            DeclareLaunchArgument(
                "invert_measured_steer_sign",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "custom_urdf",
                default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf"),
            ),
            DeclareLaunchArgument(
                "lidar_config_path",
                default_value=os.path.join(sensores_dir, "config", "rs16.yaml"),
            ),
            DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM0:921600"),
            DeclareLaunchArgument("use_cyclone_dds", default_value="false"),
            DeclareLaunchArgument("nav_start_delay_s", default_value="4.0"),
            # Perfil operativo actual: keepout deshabilitado por default
            # mientras se estabiliza la navegación global con el costmap de
            # 300 x 300 m. Se puede reactivar explícitamente por launch arg.
            DeclareLaunchArgument("use_keepout", default_value="False"),
            DeclareLaunchArgument("launch_web_app", default_value="True"),
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_app_port", default_value="8766"),
            DeclareLaunchArgument("use_rviz", default_value="False"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("enable_scan_wifi_debug", default_value="True"),
            DeclareLaunchArgument(
                "scan_wifi_debug_topic", default_value="/scan_wifi_debug"
            ),
            DeclareLaunchArgument(
                "scan_wifi_debug_publish_hz", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "scan_wifi_debug_beam_stride", default_value="4"
            ),
            DeclareLaunchArgument(
                "scan_wifi_debug_range_max_m", default_value="12.0"
            ),
            DeclareLaunchArgument("vx_deadband_mps", default_value="0.01"),
            DeclareLaunchArgument("vx_min_effective_mps", default_value="0.5"),
            DeclareLaunchArgument("invert_steer_from_cmd_vel", default_value="True"),
            DeclareLaunchArgument(
                "global_localization_params_file",
                default_value=default_global_localization_params,
            ),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument(
                "collision_monitor_params_file",
                default_value=default_collision_monitor_params,
            ),
            DeclareLaunchArgument("keepout_mask_yaml", default_value=default_keepout_mask),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.1"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.01"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.1"),
            DeclareLaunchArgument(
                "enable_map_gps_absolute_measurement",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "map_gps_absolute_topic",
                default_value="/gps/odometry_map",
            ),
            DeclareLaunchArgument("map_gps_pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument(
                "map_gps_fromll_service",
                default_value="/fromLL",
            ),
            DeclareLaunchArgument(
                "map_gps_fromll_service_fallback",
                default_value="/navsat_transform/fromLL",
            ),
            DeclareLaunchArgument("map_gps_fromll_wait_timeout_s", default_value="0.2"),
            DeclareLaunchArgument("navsat_use_odometry_yaw", default_value="false"),
            DeclareLaunchArgument("enable_gps_course_heading", default_value="True"),
            DeclareLaunchArgument("gps_course_heading_min_distance_m", default_value="2.0"),
            DeclareLaunchArgument("gps_course_heading_min_speed_mps", default_value="0.8"),
            DeclareLaunchArgument("gps_course_heading_max_abs_steer_deg", default_value="3.0"),
            DeclareLaunchArgument(
                "gps_course_heading_max_abs_yaw_rate_rps",
                default_value="0.05",
            ),
            # En real conviene mantener el ultimo yaw GPS valido por una
            # ventana breve cuando el RTK sigue sano pero el vehiculo entra
            # en una curva suave, para evitar que `map->odom` cambie bruscamente.
            DeclareLaunchArgument("gps_course_heading_invalid_hold_s", default_value="0.8"),
            # Evita reutilizar una cuerda GPS demasiado vieja; en curvas largas
            # termina representando una tangente pasada y no el heading actual.
            DeclareLaunchArgument("gps_course_heading_max_sample_dt_s", default_value="2.5"),
            DeclareLaunchArgument("gps_course_heading_publish_hz", default_value="5.0"),
            DeclareLaunchArgument(
                "gps_course_heading_yaw_variance_rad2",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "gps_course_heading_hold_yaw_variance_multiplier",
                default_value="4.0",
            ),
            DeclareLaunchArgument("gps_course_heading_require_rtk", default_value="True"),
            DeclareLaunchArgument(
                "gps_course_heading_allowed_rtk_statuses",
                # `/gps/rtk_status_mavros` puede reportar `rtcm_ok` mientras
                # ya hay correcciones frescas pero MAVROS todavia no elevo la
                # solucion a `rtk_float`/`rtk_fixed`.
                default_value="RTK_FIXED,RTK_FIX,RTK_FLOAT,RTCM_OK",
            ),
            DeclareLaunchArgument(
                "gps_course_heading_rtk_status_max_age_s",
                default_value="2.5",
            ),
            DeclareLaunchArgument(
                "gps_rtk_status_topic",
                default_value="/gps/rtk_status_mavros",
            ),
            DeclareLaunchArgument("datum_lat", default_value=str(default_datum_lat)),
            DeclareLaunchArgument("datum_lon", default_value=str(default_datum_lon)),
            # Convencion fija operativa para `global v2`: por default el robot
            # arranca mirando al Este (`datum_yaw_deg = 0.0` en ROS ENU).
            DeclareLaunchArgument("datum_yaw_deg", default_value=str(default_datum_yaw_deg)),
            DeclareLaunchArgument("datums_file", default_value=default_datums_file),
            OpaqueFunction(function=_build_robot_state_publisher),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensores_dir, "launch", "mavros.launch.py")
                ),
                launch_arguments={
                    "launch_web": launch_web_app,
                    "launch_legacy_compat": "false",
                    # El bridge RTK queda activo por default en este perfil.
                    # Si el operador desactiva RTK pero luego habilita
                    # `gps_course_heading` en modo RTK-obligatorio, esta
                    # expresion vuelve a encender la cadena necesaria para
                    # evitar una activacion a medias del heading global.
                    "enable_rtk": effective_enable_rtk,
                    "rtk_status_topic": gps_rtk_status_topic,
                    "fcu_url": fcu_url,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sensores_dir, "launch", "rs16.launch.py")
                ),
                launch_arguments={
                    "config_path": lidar_config_path,
                    "use_cyclone_dds": use_cyclone_dds,
                }.items(),
            ),
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
            Node(
                package="controller_server",
                executable="controller_server_node",
                name="vehicle_controller_server",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/serial0",
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
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="scan_wifi_debug",
                name="scan_wifi_debug",
                output="screen",
                condition=IfCondition(enable_scan_wifi_debug),
                parameters=[
                    {
                        "source_topic": "/scan",
                        "output_topic": scan_wifi_debug_topic,
                        "publish_hz": ParameterValue(
                            scan_wifi_debug_publish_hz, value_type=float
                        ),
                        "beam_stride": ParameterValue(
                            scan_wifi_debug_beam_stride, value_type=int
                        ),
                        "crop_angle_min_rad": -1.57079632679,
                        "crop_angle_max_rad": 1.57079632679,
                        "output_range_max_m": ParameterValue(
                            scan_wifi_debug_range_max_m, value_type=float
                        ),
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
                        "gps_topic": "/global_position/raw/fix",
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
                        "rtk_status_topic": gps_rtk_status_topic,
                        "require_rtk": ParameterValue(
                            gps_course_heading_require_rtk, value_type=bool
                        ),
                        "allowed_rtk_statuses": gps_course_heading_allowed_rtk_statuses,
                        "rtk_status_max_age_s": ParameterValue(
                            gps_course_heading_rtk_status_max_age_s, value_type=float
                        ),
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
                        "gps_topic": "/global_position/raw/fix",
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
                executable="route_executor",
                name="route_executor",
                output="screen",
                parameters=[
                    {
                        "nav_set_goal_service": "/nav_command_server/set_goal_ll",
                        "nav_cancel_goal_service": "/nav_command_server/cancel_goal",
                        "nav_telemetry_topic": "/nav_command_server/telemetry",
                        "set_route_service": "/route_executor/set_route_ll",
                        "cancel_route_service": "/route_executor/cancel_route",
                        "get_state_service": "/route_executor/get_state",
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
                    "gps_topic": "/global_position/raw/fix",
                    "wheelbase_m": wheelbase_m,
                    "invert_measured_steer_sign": invert_measured_steer_sign,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                    "enable_map_gps_absolute_measurement": enable_map_gps_absolute_measurement,
                    "map_gps_absolute_topic": map_gps_absolute_topic,
                    "map_gps_pose_covariance_xy": map_gps_pose_covariance_xy,
                    "map_gps_fromll_service": map_gps_fromll_service,
                    "map_gps_fromll_service_fallback": map_gps_fromll_service_fallback,
                    "map_gps_fromll_wait_timeout_s": map_gps_fromll_wait_timeout_s,
                    "navsat_use_odometry_yaw": navsat_use_odometry_yaw,
                    "global_localization_params_file": global_localization_params_file,
                    "enable_gps_course_heading": enable_gps_course_heading,
                    "gps_course_heading_topic": "/gps/course_heading",
                    "datum_setter": "false",
                    "datum_lat": datum_lat,
                    "datum_lon": datum_lon,
                    "datum_yaw_deg": datum_yaw_deg,
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
                            "keepout_mask_yaml": keepout_mask_yaml,
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
                    "gps_topic": "/global_position/raw/fix",
                    "odom_topic": "/odometry/global",
                    "map_frame": "map",
                    "launch_nav_command_server": "false",
                    "launch_route_executor": "false",
                    "teleop_cmd_topic": "/cmd_vel_teleop",
                    "gps_status_topic": gps_rtk_status_topic,
                    "sensor_bridge_enabled": launch_web_app,
                    "sensor_bridge_http_url": "http://127.0.0.1:8000/data",
                    "fixed_datum_lat": datum_lat,
                    "fixed_datum_lon": datum_lon,
                    "fixed_datum_yaw_deg": datum_yaw_deg,
                    "fixed_datum_source": "real_global_v2_fixed",
                    "datums_file": datums_file,
                }.items(),
                condition=IfCondition(launch_web_app),
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
