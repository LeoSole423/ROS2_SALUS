from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_sim_global_v2_launch_reuses_current_sim_stack_without_rviz() -> None:
    launch_contents = _read("launch/sim_global_v2.launch.py")
    map_gps_enable_arg = (
        'DeclareLaunchArgument("enable_map_gps_absolute_measurement", default_value="true")'
    )
    map_gps_topic_arg = (
        'DeclareLaunchArgument("map_gps_absolute_topic", default_value="/gps/odometry_map")'
    )
    map_gps_cov_arg = (
        'DeclareLaunchArgument("map_gps_pose_covariance_xy", default_value="0.05")'
    )
    map_gps_fromll_arg = (
        'DeclareLaunchArgument("map_gps_fromll_service", default_value="/fromLL")'
    )
    map_gps_param_ref = (
        '"enable_map_gps_absolute_measurement": enable_map_gps_absolute_measurement'
    )
    gps_heading_enable_arg = (
        'DeclareLaunchArgument("enable_gps_course_heading", default_value="true")'
    )
    gps_heading_distance_arg = (
        'DeclareLaunchArgument("gps_course_heading_min_distance_m", default_value="2.0")'
    )
    gps_heading_speed_arg = (
        'DeclareLaunchArgument("gps_course_heading_min_speed_mps", default_value="0.8")'
    )
    gps_heading_steer_arg = (
        'DeclareLaunchArgument("gps_course_heading_max_abs_steer_deg", default_value="3.0")'
    )
    gps_heading_yaw_rate_arg = (
        'DeclareLaunchArgument("gps_course_heading_max_abs_yaw_rate_rps", default_value="0.05")'
    )
    gps_heading_hold_arg = (
        'DeclareLaunchArgument("gps_course_heading_invalid_hold_s", default_value="0.8")'
    )
    gps_heading_max_sample_dt_arg = (
        'DeclareLaunchArgument("gps_course_heading_max_sample_dt_s", default_value="2.5")'
    )
    gps_heading_publish_arg = (
        'DeclareLaunchArgument("gps_course_heading_publish_hz", default_value="5.0")'
    )
    gps_heading_variance_arg = (
        'DeclareLaunchArgument("gps_course_heading_yaw_variance_rad2", default_value="0.05")'
    )
    gps_heading_hold_variance_arg = (
        'DeclareLaunchArgument(\n                "gps_course_heading_hold_yaw_variance_multiplier",'
    )
    approx_lat_arg = '"approx_fromll_datum_lat": ParameterValue(datum_lat, value_type=float)'
    approx_lon_arg = '"approx_fromll_datum_lon": ParameterValue(datum_lon, value_type=float)'

    assert "sim_v2_base.launch.py" in launch_contents
    assert "localization_global_v2.launch.py" in launch_contents
    assert "nav_global_v2.launch.py" in launch_contents
    assert "no_go_editor.launch.py" in launch_contents
    assert '"fromll_frame": "map"' in launch_contents
    assert '"map_frame": "map"' in launch_contents
    assert '"approx_fromll_fallback_enabled": True' in launch_contents
    assert 'DeclareLaunchArgument("datum_lat", default_value=str(default_datum_lat))' in launch_contents
    assert 'DeclareLaunchArgument("datum_lon", default_value=str(default_datum_lon))' in launch_contents
    assert (
        'DeclareLaunchArgument("datum_yaw_deg", default_value=str(default_datum_yaw_deg))'
        in launch_contents
    )
    assert map_gps_enable_arg in launch_contents
    assert map_gps_topic_arg in launch_contents
    assert map_gps_cov_arg in launch_contents
    assert map_gps_fromll_arg in launch_contents
    assert 'DeclareLaunchArgument(' in launch_contents
    assert map_gps_param_ref in launch_contents
    assert '"map_gps_absolute_topic": map_gps_absolute_topic' in launch_contents
    assert '"map_gps_fromll_service_fallback": map_gps_fromll_service_fallback' in launch_contents
    assert gps_heading_enable_arg in launch_contents
    assert gps_heading_distance_arg in launch_contents
    assert gps_heading_speed_arg in launch_contents
    assert gps_heading_steer_arg in launch_contents
    assert gps_heading_yaw_rate_arg in launch_contents
    assert gps_heading_hold_arg in launch_contents
    assert gps_heading_max_sample_dt_arg in launch_contents
    assert gps_heading_publish_arg in launch_contents
    assert gps_heading_variance_arg in launch_contents
    assert gps_heading_hold_variance_arg in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_require_rtk", default_value="True")' in launch_contents
    assert 'default_value="RTK_FIXED,RTK_FIX,RTK_FLOAT,RTCM_OK"' in launch_contents
    assert 'DeclareLaunchArgument("gps_rtk_status_topic", default_value="/gps/rtk_status")' in launch_contents
    assert 'DeclareLaunchArgument("gps_profile", default_value="f9p_rtk")' in launch_contents
    assert 'executable="gps_course_heading"' in launch_contents
    assert '"gps_profile": gps_profile' in launch_contents
    assert approx_lat_arg in launch_contents
    assert approx_lon_arg in launch_contents
    assert '"approx_fromll_datum_yaw_deg": ParameterValue(' in launch_contents
    assert '"navsat_use_odometry_yaw": "false"' in launch_contents
    assert '"enable_gps_course_heading": enable_gps_course_heading' in launch_contents
    assert '"gps_course_heading_topic": "/gps/course_heading"' in launch_contents
    assert '"invalid_hold_s": ParameterValue(' in launch_contents
    assert '"max_sample_dt_s": ParameterValue(' in launch_contents
    assert '"publish_hz": ParameterValue(' in launch_contents
    assert '"hold_yaw_variance_multiplier": ParameterValue(' in launch_contents
    assert '"rtk_status_topic": gps_rtk_status_topic' in launch_contents
    assert '"require_rtk": ParameterValue(' in launch_contents
    assert '"allowed_rtk_statuses": gps_course_heading_allowed_rtk_statuses' in launch_contents
    assert '"rtk_status_max_age_s": ParameterValue(' in launch_contents
    assert "nav2_global_v2_sim_rolling_params.yaml" in launch_contents
    assert 'DeclareLaunchArgument("launch_web_app", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_x", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_y", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_z", default_value="0.2")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_yaw", default_value="0.0")' in launch_contents
    assert '"spawn_x": spawn_x' in launch_contents
    assert '"spawn_y": spawn_y' in launch_contents
    assert '"spawn_z": spawn_z' in launch_contents
    assert '"spawn_yaw": spawn_yaw' in launch_contents
    assert '"odom_topic": "/odometry/global"' in launch_contents
    assert '"launch_nav_command_server": "false"' in launch_contents
    assert 'executable="rviz2"' not in launch_contents


def test_localization_global_v2_launch_adds_map_filter_and_navsat_support() -> None:
    launch_contents = _read("launch/localization_global_v2.launch.py")
    navsat_yaw_arg = 'DeclareLaunchArgument("navsat_use_odometry_yaw", default_value="false")'
    odom_gate_arg = 'DeclareLaunchArgument(\n                "enable_global_odom_stationary_gate"'
    imu_gate_arg = 'DeclareLaunchArgument(\n                "enable_global_imu_stationary_gate"'
    yaw_hold_arg = 'DeclareLaunchArgument(\n                "enable_global_stationary_yaw_hold"'
    yaw_hold_topic_arg = (
        'DeclareLaunchArgument(\n                "global_stationary_yaw_hold_topic"'
    )
    map_gps_arg = 'DeclareLaunchArgument(\n                "enable_map_gps_absolute_measurement"'
    gps_heading_arg = 'DeclareLaunchArgument("enable_gps_course_heading", default_value="false")'
    gps_heading_topic_arg = (
        'DeclareLaunchArgument("gps_course_heading_topic", default_value="/gps/course_heading")'
    )

    assert "localization_v2.launch.py" in launch_contents
    assert 'name="global_odom_stationary_gate"' in launch_contents
    assert 'name="global_imu_stationary_gate"' in launch_contents
    assert 'name="global_yaw_stationary_hold"' in launch_contents
    assert 'name="map_gps_absolute_measurement"' in launch_contents
    assert 'name="ekf_filter_node_map"' in launch_contents
    assert 'name="navsat_transform"' in launch_contents
    assert 'DeclareLaunchArgument("datum_setter", default_value="false")' in launch_contents
    assert navsat_yaw_arg in launch_contents
    assert odom_gate_arg in launch_contents
    assert 'DeclareLaunchArgument(\n                "global_odom_gated_topic"' in launch_contents
    assert imu_gate_arg in launch_contents
    assert 'DeclareLaunchArgument(\n                "global_imu_gated_topic"' in launch_contents
    assert yaw_hold_arg in launch_contents
    assert yaw_hold_topic_arg in launch_contents
    assert map_gps_arg in launch_contents
    assert 'DeclareLaunchArgument(\n                "map_gps_absolute_topic"' in launch_contents
    assert gps_heading_arg in launch_contents
    assert gps_heading_topic_arg in launch_contents
    assert '"use_odometry_yaw": navsat_use_odometry_yaw' in launch_contents
    assert '"input_odom_topic": "/odometry/local"' in launch_contents
    assert '"output_odom_topic": global_odom_gated_topic' in launch_contents
    assert '"drive_telemetry_topic": drive_telemetry_topic' in launch_contents
    assert '{"odom0": map_filter_odom_topic}' in launch_contents
    assert '"input_imu_topic": imu_topic' in launch_contents
    assert '"output_imu_topic": global_imu_gated_topic' in launch_contents
    assert '{"imu0": map_filter_imu_topic}' in launch_contents
    assert '"output_odom_topic": global_stationary_yaw_hold_topic' in launch_contents
    assert '"odom2": global_stationary_yaw_hold_topic' in launch_contents
    assert '"odom2_config": [' in launch_contents
    assert '"output_topic": map_gps_absolute_topic' in launch_contents
    assert '"fromll_service": map_gps_fromll_service' in launch_contents
    assert '{"odom1": map_gps_absolute_topic}' in launch_contents
    assert '"imu1": gps_course_heading_topic' in launch_contents
    assert '"imu1_config": [' in launch_contents
    assert '("odometry/filtered", "/odometry/local")' in launch_contents
    assert '("odometry/gps", "/odometry/gps")' in launch_contents


def test_nav2_global_params_switch_global_frame_to_map() -> None:
    params_contents = _read("config/nav2_global_v2_params.yaml")

    assert "global_frame: map" in params_contents
    assert "local_frame: odom" in params_contents
    assert "odom_topic: /odometry/local" in params_contents


def test_sim_nav2_global_params_enable_rolling_global_costmap() -> None:
    params_contents = _read("config/nav2_global_v2_sim_rolling_params.yaml")

    assert "rolling_window: true" in params_contents
    assert "global_frame: map" in params_contents
    assert "width: 300" in params_contents
    assert "height: 300" in params_contents
    assert 'filters: ["keepout_filter"]' in params_contents
    assert "waypoint_follower:" in params_contents


def test_rviz_global_config_and_launch_target_map() -> None:
    rviz_contents = _read("config/rviz_global_v2.rviz")
    launch_contents = _read("launch/rviz_sim_global_v2.launch.py")

    assert "Fixed Frame: map" in rviz_contents
    assert "/odometry/global" in rviz_contents
    assert "/gps/odometry_map" in rviz_contents
    assert "GPS Map Odom" in rviz_contents
    assert "rviz_global_v2.rviz" in launch_contents


def test_sim_global_v2_wifi_launch_wraps_base_and_enables_scan_reduction() -> None:
    launch_contents = _read("launch/sim_global_v2_wifi.launch.py")

    assert "sim_global_v2.launch.py" in launch_contents
    assert 'nav2_global_v2_sim_rolling_wifi_params.yaml' in launch_contents
    assert 'DeclareLaunchArgument("enable_scan_wifi_debug", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_min_distance_m", default_value="2.0")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_min_speed_mps", default_value="0.8")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_publish_hz", default_value="5.0")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_require_rtk", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("gps_rtk_status_topic", default_value="/gps/rtk_status")' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_topic", default_value="/scan_wifi_debug"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_publish_hz", default_value="2.0"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_beam_stride", default_value="4"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_range_max_m", default_value="12.0"' in launch_contents
    assert 'executable="scan_wifi_debug"' in launch_contents
    assert 'condition=IfCondition(enable_scan_wifi_debug)' in launch_contents
    assert 'DeclareLaunchArgument("enable_lidar_obstacle_filter", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("lidar_scan_topic", default_value="/scan_filtered")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_x", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_y", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_z", default_value="0.2")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_yaw", default_value="0.0")' in launch_contents
    assert '"enable_lidar_obstacle_filter": enable_lidar_obstacle_filter' in launch_contents
    assert '"lidar_scan_topic": lidar_scan_topic' in launch_contents
    assert '"spawn_x": spawn_x' in launch_contents
    assert '"spawn_y": spawn_y' in launch_contents
    assert '"spawn_z": spawn_z' in launch_contents
    assert '"spawn_yaw": spawn_yaw' in launch_contents
    assert '"source_topic": effective_lidar_scan_topic' in launch_contents
    assert '"output_topic": scan_wifi_debug_topic' in launch_contents
    assert '"crop_angle_min_rad": -1.57079632679' in launch_contents
    assert '"crop_angle_max_rad": 1.57079632679' in launch_contents


def test_sim_v2_base_spawn_pose_is_launch_configurable() -> None:
    launch_contents = _read("launch/sim_v2_base.launch.py")

    assert 'DeclareLaunchArgument("spawn_x", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_y", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_z", default_value="0.2")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_roll", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_pitch", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_yaw", default_value="0.0")' in launch_contents
    assert 'spawn_x = LaunchConfiguration("spawn_x").perform(context)' in launch_contents
    assert 'spawn_y = LaunchConfiguration("spawn_y").perform(context)' in launch_contents
    assert 'spawn_z = LaunchConfiguration("spawn_z").perform(context)' in launch_contents
    assert 'spawn_roll = LaunchConfiguration("spawn_roll").perform(context)' in launch_contents
    assert 'spawn_pitch = LaunchConfiguration("spawn_pitch").perform(context)' in launch_contents
    assert 'spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)' in launch_contents
    assert '"-R",' in launch_contents
    assert '"-P",' in launch_contents
    assert '"-Y",' in launch_contents


def test_nav_global_v2_keeps_global_costmap_window_in_profile_yaml() -> None:
    launch_contents = _read("launch/nav_global_v2.launch.py")

    assert "global_costmap_width" not in launch_contents
    assert "global_costmap_height" not in launch_contents
    assert "global_costmap_resolution" not in launch_contents


def test_sim_global_v2_wifi_rviz_and_params_match_remote_profile() -> None:
    launch_contents = _read("launch/rviz_sim_global_v2_wifi.launch.py")
    rviz_contents = _read("config/rviz_global_v2_wifi.rviz")
    nav2_params_contents = _read("config/nav2_global_v2_sim_rolling_wifi_params.yaml")

    assert "rviz_sim_global_v2.launch.py" in launch_contents
    assert "rviz_global_v2_wifi.rviz" in launch_contents
    assert "Fixed Frame: map" in rviz_contents
    assert "Frame Rate: 15" in rviz_contents
    assert "Value: /scan_wifi_debug" in rviz_contents
    assert "Value: /odometry/global" in rviz_contents
    assert "Value: /local_costmap/costmap" in rviz_contents
    assert "Value: /global_costmap/costmap" in rviz_contents
    assert "Value: /plan" in rviz_contents
    assert "Value: /stop_zone" in rviz_contents
    assert "/gps/odometry_map" not in rviz_contents
    assert "/odometry/local" not in rviz_contents
    assert "/local_nav_v2/path_tracking_debug" not in rviz_contents
    assert "/scan_3d" not in rviz_contents
    assert "publish_frequency: 1.0" in nav2_params_contents
    assert "publish_frequency: 0.5" in nav2_params_contents
    assert "publish_voxel_map: False" in nav2_params_contents
    assert "always_send_full_costmap: false" in nav2_params_contents
    assert 'filters: ["keepout_filter"]' in nav2_params_contents
    assert "waypoint_follower:" in nav2_params_contents


def test_collision_monitor_v2_keeps_contact_stop_and_uses_approach_zone() -> None:
    params_contents = _read("config/collision_monitor_v2.yaml")

    assert "footprint:" in params_contents
    assert 'action_type: "stop"' in params_contents
    assert "stop_zone:" in params_contents
    assert 'action_type: "approach"' in params_contents
    assert "time_before_collision: 2.0" in params_contents
    assert "simulation_time_step: 0.1" in params_contents
    assert "critical_slow_zone:" in params_contents
    assert "slow_zone:" in params_contents
    assert 'action_type: "slowdown"' in params_contents


def test_recovery_behavior_uses_intermediate_ackermann_backup() -> None:
    through_poses_bt = _read("config/navigate_through_poses_w_replanning_and_recovery_no_spin.xml")
    to_pose_bt = _read("config/navigate_to_pose_w_replanning_and_recovery_no_spin.xml")
    real_wifi_params = _read("config/nav2_global_v2_real_rolling_wifi_params.yaml")
    sim_wifi_params = _read("config/nav2_global_v2_sim_rolling_wifi_params.yaml")

    assert '<BackUp backup_dist="1.0" backup_speed="0.7"/>' in through_poses_bt
    assert '<BackUp backup_dist="1.0" backup_speed="0.7" />' in to_pose_bt
    assert '<RateController hz="0.666" name="RateControllerComputePathToPose">' in to_pose_bt
    assert "<ComputePathToPose goal=\"{goal}\" path=\"{path}\" planner_id=\"GridBased\" />" in to_pose_bt
    assert '<FollowPath path="{path}" controller_id="FollowPath" />' in to_pose_bt
    assert '<FollowPath path="{path}" controller_id="FollowPath"/>' in through_poses_bt
    assert "SmoothPath" not in to_pose_bt
    assert "SmoothPath" not in through_poses_bt
    assert "smoothed_path" not in to_pose_bt
    assert "smoothed_path" not in through_poses_bt
    assert "IsPathValid" not in to_pose_bt
    assert "spin" not in through_poses_bt.lower()
    assert "spin" not in to_pose_bt.lower()
    assert "simulate_ahead_time: 2.0" in real_wifi_params
    assert "simulate_ahead_time: 2.0" in sim_wifi_params


def test_global_v2_profiles_bias_paths_away_from_obstacles() -> None:
    profile_paths = [
        "config/nav2_global_v2_real_rolling_params.yaml",
        "config/nav2_global_v2_sim_rolling_params.yaml",
        "config/nav2_global_v2_real_rolling_wifi_params.yaml",
        "config/nav2_global_v2_sim_rolling_wifi_params.yaml",
    ]

    for profile_path in profile_paths:
        params_contents = _read(profile_path)

        assert "optimizer_costmap_weight: 10.0" in params_contents
        assert "cost_penalty: 3.5" in params_contents
        assert "inflation_cost_scaling_factor: 1.3" in params_contents
        assert "inflation_radius: 1.4" in params_contents
        assert "cost_scaling_factor: 1.3" in params_contents
        assert "inflation_radius: 1.5" in params_contents
        assert "cost_scaling_factor: 1.4" in params_contents


def test_wifi_nav2_costmaps_keep_longer_lidar_forward_vision() -> None:
    real_wifi_params = _read("config/nav2_global_v2_real_rolling_wifi_params.yaml")
    sim_wifi_params = _read("config/nav2_global_v2_sim_rolling_wifi_params.yaml")

    for params_contents in (real_wifi_params, sim_wifi_params):
        assert "width: 30" in params_contents
        assert "height: 30" in params_contents
        assert params_contents.count("obstacle_max_range: 15.0") >= 2
        assert "raytrace_max_range: 20.0" in params_contents


def test_global_v2_local_costmaps_split_lidar_marking_from_clearing() -> None:
    profile_paths = [
        "config/nav2_global_v2_params.yaml",
        "config/nav2_global_v2_real_rolling_params.yaml",
        "config/nav2_global_v2_sim_rolling_params.yaml",
        "config/nav2_global_v2_real_rolling_wifi_params.yaml",
        "config/nav2_global_v2_sim_rolling_wifi_params.yaml",
    ]

    for profile_path in profile_paths:
        params_contents = _read(profile_path)

        assert "observation_sources: scan_marking scan_clearing" in params_contents
        assert "scan_marking:" in params_contents
        assert "scan_clearing:" in params_contents
        assert "inf_is_valid: False" in params_contents
        assert "clearing: False" in params_contents
        assert "marking: False" in params_contents
