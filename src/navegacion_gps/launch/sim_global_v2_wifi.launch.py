import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from navegacion_gps.datum_profile_resolver import resolve_selected_datum


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
    default_nav2_params_file = _resolve_config_file_path(
        gps_wpf_dir, "nav2_global_v2_sim_rolling_wifi_params.yaml"
    )
    default_collision_monitor_params_file = _resolve_config_file_path(
        gps_wpf_dir, "collision_monitor_v2.yaml"
    )
    default_keepout_mask_yaml = _resolve_config_file_path(gps_wpf_dir, "keepout_mask.yaml")
    default_global_localization_params_file = _resolve_config_file_path(
        gps_wpf_dir, "localization_global_v2.yaml"
    )
    default_datum_lat, default_datum_lon, default_datum_yaw_deg, default_datums_file = (
        resolve_selected_datum(gps_wpf_dir)
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
    keepout_mask_yaml = LaunchConfiguration("keepout_mask_yaml")
    global_localization_params_file = LaunchConfiguration("global_localization_params_file")
    lidar_to_scan_params_file = LaunchConfiguration("lidar_to_scan_params_file")
    custom_urdf = LaunchConfiguration("custom_urdf")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    model_name = LaunchConfiguration("model_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")
    datum_lat = LaunchConfiguration("datum_lat")
    datum_lon = LaunchConfiguration("datum_lon")
    datum_yaw_deg = LaunchConfiguration("datum_yaw_deg")
    datums_file = LaunchConfiguration("datums_file")
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
    gps_course_heading_require_rtk = LaunchConfiguration("gps_course_heading_require_rtk")
    gps_course_heading_allowed_rtk_statuses = LaunchConfiguration(
        "gps_course_heading_allowed_rtk_statuses"
    )
    gps_course_heading_rtk_status_max_age_s = LaunchConfiguration(
        "gps_course_heading_rtk_status_max_age_s"
    )
    gps_rtk_status_topic = LaunchConfiguration("gps_rtk_status_topic")
    enable_sim_compass = LaunchConfiguration("enable_sim_compass")
    sim_compass_hdg_topic = LaunchConfiguration("sim_compass_hdg_topic")
    sim_compass_noise_stddev_deg = LaunchConfiguration("sim_compass_noise_stddev_deg")
    sim_compass_bias_deg = LaunchConfiguration("sim_compass_bias_deg")
    sim_compass_publish_hz = LaunchConfiguration("sim_compass_publish_hz")
    sim_compass_seed = LaunchConfiguration("sim_compass_seed")
    enable_compass_heading = LaunchConfiguration("enable_compass_heading")
    enable_compass_initial_guess = LaunchConfiguration("enable_compass_initial_guess")
    compass_hdg_topic = LaunchConfiguration("compass_hdg_topic")
    compass_heading_topic = LaunchConfiguration("compass_heading_topic")
    compass_heading_debug_topic = LaunchConfiguration("compass_heading_debug_topic")
    enable_compass_heading_fusion = LaunchConfiguration("enable_compass_heading_fusion")
    compass_heading_yaw_variance_rad2 = LaunchConfiguration(
        "compass_heading_yaw_variance_rad2"
    )
    gps_profile = LaunchConfiguration("gps_profile")
    launch_web_app = LaunchConfiguration("launch_web_app")
    ws_host = LaunchConfiguration("ws_host")
    web_app_port = LaunchConfiguration("web_app_port")
    enable_scan_wifi_debug = LaunchConfiguration("enable_scan_wifi_debug")
    scan_wifi_debug_topic = LaunchConfiguration("scan_wifi_debug_topic")
    scan_wifi_debug_publish_hz = LaunchConfiguration("scan_wifi_debug_publish_hz")
    scan_wifi_debug_beam_stride = LaunchConfiguration("scan_wifi_debug_beam_stride")
    scan_wifi_debug_range_max_m = LaunchConfiguration("scan_wifi_debug_range_max_m")
    enable_lidar_obstacle_filter = LaunchConfiguration("enable_lidar_obstacle_filter")
    lidar_scan_topic = LaunchConfiguration("lidar_scan_topic")
    enable_scan_noise_filter = LaunchConfiguration("enable_scan_noise_filter")
    scan_noise_filter_output = LaunchConfiguration("scan_noise_filter_output")
    scan_noise_filter_range_min_m = LaunchConfiguration("scan_noise_filter_range_min_m")
    scan_noise_filter_range_max_m = LaunchConfiguration("scan_noise_filter_range_max_m")
    scan_noise_filter_speckle_window = LaunchConfiguration(
        "scan_noise_filter_speckle_window"
    )
    scan_noise_filter_speckle_max_range_m = LaunchConfiguration(
        "scan_noise_filter_speckle_max_range_m"
    )
    scan_noise_filter_speckle_max_deviation_m = LaunchConfiguration(
        "scan_noise_filter_speckle_max_deviation_m"
    )
    lidar_filter_roi_x_min = LaunchConfiguration("lidar_filter_roi_x_min")
    lidar_filter_roi_x_max = LaunchConfiguration("lidar_filter_roi_x_max")
    lidar_filter_roi_y_min = LaunchConfiguration("lidar_filter_roi_y_min")
    lidar_filter_roi_y_max = LaunchConfiguration("lidar_filter_roi_y_max")
    lidar_filter_roi_z_min = LaunchConfiguration("lidar_filter_roi_z_min")
    lidar_filter_roi_z_max = LaunchConfiguration("lidar_filter_roi_z_max")
    lidar_filter_ground_distance_threshold = LaunchConfiguration(
        "lidar_filter_ground_distance_threshold"
    )
    lidar_filter_min_obstacle_height = LaunchConfiguration(
        "lidar_filter_min_obstacle_height"
    )
    lidar_filter_max_obstacle_height = LaunchConfiguration(
        "lidar_filter_max_obstacle_height"
    )
    lidar_filter_min_voxel_points = LaunchConfiguration("lidar_filter_min_voxel_points")
    effective_lidar_scan_topic = PythonExpression(
        [
            "'",
            lidar_scan_topic,
            "' if '",
            enable_lidar_obstacle_filter,
            "'.lower() == 'true' else ('",
            scan_noise_filter_output,
            "' if '",
            enable_scan_noise_filter,
            "'.lower() == 'true' else '/scan')",
        ]
    )
    nav_snapshot_scan_topic = PythonExpression(
        [
            "'",
            scan_wifi_debug_topic,
            "' if '",
            enable_scan_wifi_debug,
            "'.lower() == 'true' else '",
            effective_lidar_scan_topic,
            "'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument("invert_measured_steer_sign", default_value="True"),
            DeclareLaunchArgument("nav_start_delay_s", default_value="3.0"),
            DeclareLaunchArgument("use_keepout", default_value="False"),
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
            DeclareLaunchArgument("keepout_mask_yaml", default_value=default_keepout_mask_yaml),
            DeclareLaunchArgument(
                "global_localization_params_file",
                default_value=default_global_localization_params_file,
            ),
            DeclareLaunchArgument(
                "lidar_to_scan_params_file",
                default_value=_resolve_config_file_path(
                    gps_wpf_dir, "pointcloud_to_laserscan.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "custom_urdf",
                default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real_v2.urdf"),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(gps_wpf_dir, "worlds", "vacio.world"),
            ),
            DeclareLaunchArgument("world_name", default_value="vacio"),
            DeclareLaunchArgument("model_name", default_value="quad_ackermann_viewer_safe"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.1"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.01"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.1"),
            DeclareLaunchArgument("datum_lat", default_value=str(default_datum_lat)),
            DeclareLaunchArgument("datum_lon", default_value=str(default_datum_lon)),
            DeclareLaunchArgument("datum_yaw_deg", default_value=str(default_datum_yaw_deg)),
            DeclareLaunchArgument("datums_file", default_value=default_datums_file),
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
            DeclareLaunchArgument("gps_course_heading_min_distance_m", default_value="2.0"),
            DeclareLaunchArgument("gps_course_heading_min_speed_mps", default_value="0.8"),
            DeclareLaunchArgument("gps_course_heading_max_abs_steer_deg", default_value="3.0"),
            DeclareLaunchArgument(
                "gps_course_heading_max_abs_yaw_rate_rps", default_value="0.05"
            ),
            DeclareLaunchArgument("gps_course_heading_invalid_hold_s", default_value="0.8"),
            DeclareLaunchArgument("gps_course_heading_max_sample_dt_s", default_value="2.5"),
            DeclareLaunchArgument("gps_course_heading_publish_hz", default_value="5.0"),
            DeclareLaunchArgument("gps_course_heading_yaw_variance_rad2", default_value="0.05"),
            DeclareLaunchArgument(
                "gps_course_heading_hold_yaw_variance_multiplier",
                default_value="4.0",
            ),
            DeclareLaunchArgument("gps_course_heading_require_rtk", default_value="True"),
            DeclareLaunchArgument(
                "gps_course_heading_allowed_rtk_statuses",
                default_value="RTK_FIXED,RTK_FIX,RTK_FLOAT,RTCM_OK",
            ),
            DeclareLaunchArgument(
                "gps_course_heading_rtk_status_max_age_s", default_value="2.5"
            ),
            DeclareLaunchArgument("gps_rtk_status_topic", default_value="/gps/rtk_status"),
            DeclareLaunchArgument("enable_sim_compass", default_value="false"),
            DeclareLaunchArgument("sim_compass_hdg_topic", default_value="/sim/compass_hdg"),
            DeclareLaunchArgument("sim_compass_noise_stddev_deg", default_value="0.0"),
            DeclareLaunchArgument("sim_compass_bias_deg", default_value="0.0"),
            DeclareLaunchArgument("sim_compass_publish_hz", default_value="5.0"),
            DeclareLaunchArgument("sim_compass_seed", default_value="1"),
            DeclareLaunchArgument("enable_compass_heading", default_value="false"),
            DeclareLaunchArgument("enable_compass_initial_guess", default_value="false"),
            DeclareLaunchArgument("compass_hdg_topic", default_value="/mavros_node/compass_hdg"),
            DeclareLaunchArgument("compass_heading_topic", default_value="/imu/compass_heading"),
            DeclareLaunchArgument(
                "compass_heading_debug_topic",
                default_value="/imu/compass_heading/debug",
            ),
            DeclareLaunchArgument("enable_compass_heading_fusion", default_value="false"),
            DeclareLaunchArgument("compass_heading_yaw_variance_rad2", default_value="1.0"),
            DeclareLaunchArgument("gps_profile", default_value="f9p_rtk"),
            DeclareLaunchArgument("launch_web_app", default_value="True"),
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_app_port", default_value="8766"),
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
            DeclareLaunchArgument("enable_lidar_obstacle_filter", default_value="False"),
            DeclareLaunchArgument("enable_scan_ground_filter", default_value="True"),
            DeclareLaunchArgument("scan_ground_min_height", default_value="0.10"),
            DeclareLaunchArgument("scan_ground_max_height", default_value="2.50"),
            DeclareLaunchArgument("lidar_scan_topic", default_value="/scan_filtered"),
            DeclareLaunchArgument("enable_scan_noise_filter", default_value="True"),
            DeclareLaunchArgument("scan_noise_filter_output", default_value="/scan_clean"),
            DeclareLaunchArgument("scan_noise_filter_range_min_m", default_value="0.4"),
            DeclareLaunchArgument("scan_noise_filter_range_max_m", default_value="20.0"),
            DeclareLaunchArgument("scan_noise_filter_speckle_window", default_value="2"),
            DeclareLaunchArgument(
                "scan_noise_filter_speckle_max_range_m",
                default_value="12.0",
            ),
            DeclareLaunchArgument(
                "scan_noise_filter_speckle_max_deviation_m",
                default_value="0.30",
            ),
            DeclareLaunchArgument("lidar_filter_roi_x_min", default_value="-0.4"),
            DeclareLaunchArgument("lidar_filter_roi_x_max", default_value="12.0"),
            DeclareLaunchArgument("lidar_filter_roi_y_min", default_value="-2.5"),
            DeclareLaunchArgument("lidar_filter_roi_y_max", default_value="2.5"),
            DeclareLaunchArgument("lidar_filter_roi_z_min", default_value="-1.0"),
            DeclareLaunchArgument("lidar_filter_roi_z_max", default_value="2.0"),
            DeclareLaunchArgument(
                "lidar_filter_ground_distance_threshold",
                default_value="0.18",
            ),
            DeclareLaunchArgument(
                "lidar_filter_min_obstacle_height",
                default_value="0.22",
            ),
            DeclareLaunchArgument(
                "lidar_filter_max_obstacle_height",
                default_value="1.40",
            ),
            DeclareLaunchArgument("lidar_filter_min_voxel_points", default_value="3"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gps_wpf_dir, "launch", "sim_global_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "wheelbase_m": wheelbase_m,
                    "invert_measured_steer_sign": invert_measured_steer_sign,
                    "nav_start_delay_s": nav_start_delay_s,
                    "use_keepout": use_keepout,
                    "vx_deadband_mps": vx_deadband_mps,
                    "vx_min_effective_mps": vx_min_effective_mps,
                    "invert_steer_from_cmd_vel": invert_steer_from_cmd_vel,
                    "nav2_params_file": nav2_params_file,
                    "collision_monitor_params_file": collision_monitor_params_file,
                    "keepout_mask_yaml": keepout_mask_yaml,
                    "global_localization_params_file": global_localization_params_file,
                    "lidar_to_scan_params_file": lidar_to_scan_params_file,
                    "custom_urdf": custom_urdf,
                    "world": world,
                    "world_name": world_name,
                    "model_name": model_name,
                    "spawn_x": spawn_x,
                    "spawn_y": spawn_y,
                    "spawn_z": spawn_z,
                    "spawn_roll": spawn_roll,
                    "spawn_pitch": spawn_pitch,
                    "spawn_yaw": spawn_yaw,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                    "datum_lat": datum_lat,
                    "datum_lon": datum_lon,
                    "datum_yaw_deg": datum_yaw_deg,
                    "datums_file": datums_file,
                    "datum_setter": datum_setter,
                    "enable_map_gps_absolute_measurement": enable_map_gps_absolute_measurement,
                    "map_gps_absolute_topic": map_gps_absolute_topic,
                    "map_gps_pose_covariance_xy": map_gps_pose_covariance_xy,
                    "map_gps_fromll_service": map_gps_fromll_service,
                    "map_gps_fromll_service_fallback": map_gps_fromll_service_fallback,
                    "map_gps_fromll_wait_timeout_s": map_gps_fromll_wait_timeout_s,
                    "enable_gps_course_heading": enable_gps_course_heading,
                    "gps_course_heading_min_distance_m": gps_course_heading_min_distance_m,
                    "gps_course_heading_min_speed_mps": gps_course_heading_min_speed_mps,
                    "gps_course_heading_max_abs_steer_deg": gps_course_heading_max_abs_steer_deg,
                    "gps_course_heading_max_abs_yaw_rate_rps": gps_course_heading_max_abs_yaw_rate_rps,
                    "gps_course_heading_invalid_hold_s": gps_course_heading_invalid_hold_s,
                    "gps_course_heading_max_sample_dt_s": gps_course_heading_max_sample_dt_s,
                    "gps_course_heading_publish_hz": gps_course_heading_publish_hz,
                    "gps_course_heading_yaw_variance_rad2": gps_course_heading_yaw_variance_rad2,
                    "gps_course_heading_hold_yaw_variance_multiplier": gps_course_heading_hold_yaw_variance_multiplier,
                    "gps_course_heading_require_rtk": gps_course_heading_require_rtk,
                    "gps_course_heading_allowed_rtk_statuses": gps_course_heading_allowed_rtk_statuses,
                    "gps_course_heading_rtk_status_max_age_s": gps_course_heading_rtk_status_max_age_s,
                    "gps_rtk_status_topic": gps_rtk_status_topic,
                    "enable_sim_compass": enable_sim_compass,
                    "sim_compass_hdg_topic": sim_compass_hdg_topic,
                    "sim_compass_noise_stddev_deg": sim_compass_noise_stddev_deg,
                    "sim_compass_bias_deg": sim_compass_bias_deg,
                    "sim_compass_publish_hz": sim_compass_publish_hz,
                    "sim_compass_seed": sim_compass_seed,
                    "enable_compass_heading": enable_compass_heading,
                    "enable_compass_initial_guess": enable_compass_initial_guess,
                    "compass_hdg_topic": compass_hdg_topic,
                    "compass_heading_topic": compass_heading_topic,
                    "compass_heading_debug_topic": compass_heading_debug_topic,
                    "enable_compass_heading_fusion": enable_compass_heading_fusion,
                    "compass_heading_yaw_variance_rad2": compass_heading_yaw_variance_rad2,
                    "gps_profile": gps_profile,
                    "launch_web_app": launch_web_app,
                    "ws_host": ws_host,
                    "web_app_port": web_app_port,
                    "nav_snapshot_scan_topic": nav_snapshot_scan_topic,
                    "enable_lidar_obstacle_filter": enable_lidar_obstacle_filter,
                    "enable_scan_ground_filter": LaunchConfiguration(
                        "enable_scan_ground_filter"
                    ),
                    "scan_ground_min_height": LaunchConfiguration(
                        "scan_ground_min_height"
                    ),
                    "scan_ground_max_height": LaunchConfiguration(
                        "scan_ground_max_height"
                    ),
                    "lidar_scan_topic": lidar_scan_topic,
                    "enable_scan_noise_filter": enable_scan_noise_filter,
                    "scan_noise_filter_output": scan_noise_filter_output,
                    "scan_noise_filter_range_min_m": scan_noise_filter_range_min_m,
                    "scan_noise_filter_range_max_m": scan_noise_filter_range_max_m,
                    "scan_noise_filter_speckle_window": (
                        scan_noise_filter_speckle_window
                    ),
                    "scan_noise_filter_speckle_max_range_m": (
                        scan_noise_filter_speckle_max_range_m
                    ),
                    "scan_noise_filter_speckle_max_deviation_m": (
                        scan_noise_filter_speckle_max_deviation_m
                    ),
                    "lidar_filter_roi_x_min": lidar_filter_roi_x_min,
                    "lidar_filter_roi_x_max": lidar_filter_roi_x_max,
                    "lidar_filter_roi_y_min": lidar_filter_roi_y_min,
                    "lidar_filter_roi_y_max": lidar_filter_roi_y_max,
                    "lidar_filter_roi_z_min": lidar_filter_roi_z_min,
                    "lidar_filter_roi_z_max": lidar_filter_roi_z_max,
                    "lidar_filter_ground_distance_threshold": (
                        lidar_filter_ground_distance_threshold
                    ),
                    "lidar_filter_min_obstacle_height": (
                        lidar_filter_min_obstacle_height
                    ),
                    "lidar_filter_max_obstacle_height": (
                        lidar_filter_max_obstacle_height
                    ),
                    "lidar_filter_min_voxel_points": lidar_filter_min_voxel_points,
                }.items(),
            ),
            Node(
                package="navegacion_gps",
                executable="scan_wifi_debug",
                name="scan_wifi_debug",
                output="screen",
                condition=IfCondition(enable_scan_wifi_debug),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "source_topic": effective_lidar_scan_topic,
                        "output_topic": scan_wifi_debug_topic,
                        "publish_hz": ParameterValue(scan_wifi_debug_publish_hz, value_type=float),
                        "beam_stride": ParameterValue(scan_wifi_debug_beam_stride, value_type=int),
                        "crop_angle_min_rad": -1.57079632679,
                        "crop_angle_max_rad": 1.57079632679,
                        "output_range_max_m": ParameterValue(
                            scan_wifi_debug_range_max_m, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
