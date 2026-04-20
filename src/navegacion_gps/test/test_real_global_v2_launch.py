from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def _assert_real_wifi_wrapper_parity(launch_contents: str) -> None:
    assert 'DeclareLaunchArgument("use_keepout", default_value="False")' in launch_contents
    assert 'DeclareLaunchArgument("enable_gps_course_heading", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_min_distance_m", default_value="2.0")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_min_speed_mps", default_value="0.8")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_invalid_hold_s", default_value="0.8")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_max_sample_dt_s", default_value="2.5")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_publish_hz", default_value="5.0")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_require_rtk", default_value="True")' in launch_contents
    assert 'default_value="RTK_FIXED,RTK_FIX,RTK_FLOAT,RTCM_OK"' in launch_contents
    assert 'default_value="/gps/rtk_status_mavros"' in launch_contents
    assert '"gps_course_heading_rtk_status_max_age_s"' in launch_contents
    assert 'default_value="2.5"' in launch_contents
    assert 'DeclareLaunchArgument("navsat_use_odometry_yaw", default_value="false")' in launch_contents
    assert 'DeclareLaunchArgument("enable_rtk", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("enable_scan_wifi_debug", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_topic", default_value="/scan_wifi_debug"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_publish_hz", default_value="2.0"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_beam_stride", default_value="4"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_range_max_m", default_value="12.0"' in launch_contents
    assert 'source_path.exists()' in launch_contents
    assert "source_path.parent.exists()" not in launch_contents


def test_real_global_v2_launch_reuses_real_stack_with_global_navigation() -> None:
    launch_contents = _read("launch/real_global_v2.launch.py")
    map_gps_enable_arg = (
        'DeclareLaunchArgument(\n                "enable_map_gps_absolute_measurement",'
    )
    map_gps_topic_arg = (
        'DeclareLaunchArgument(\n                "map_gps_absolute_topic",'
    )
    map_gps_cov_arg = (
        'DeclareLaunchArgument("map_gps_pose_covariance_xy", default_value="0.05")'
    )
    map_gps_fromll_arg = (
        'DeclareLaunchArgument(\n                "map_gps_fromll_service",'
    )
    map_gps_param_ref = (
        '"enable_map_gps_absolute_measurement": enable_map_gps_absolute_measurement'
    )

    assert "mavros.launch.py" in launch_contents
    assert "rs16.launch.py" in launch_contents
    assert 'executable="pointcloud_to_laserscan_node"' in launch_contents
    assert 'executable="controller_server_node"' in launch_contents
    assert "localization_global_v2.launch.py" in launch_contents
    assert "nav_global_v2.launch.py" in launch_contents
    assert "no_go_editor.launch.py" in launch_contents
    assert '"fromll_frame": "map"' in launch_contents
    assert '"map_frame": "map"' in launch_contents
    assert '"approx_fromll_fallback_enabled": True' in launch_contents
    assert '"odom_topic": "/odometry/global"' in launch_contents
    assert '"launch_zones_manager": "false"' not in launch_contents
    assert '"launch_nav_command_server": "false"' in launch_contents
    assert '"launch_nav_snapshot_server": "false"' not in launch_contents
    assert '"launch_route_executor": "false"' in launch_contents
    assert 'DeclareLaunchArgument("datum_lat"' in launch_contents
    assert 'DeclareLaunchArgument("datum_lon"' in launch_contents
    assert 'DeclareLaunchArgument("datum_yaw_deg", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("use_rviz", default_value="False")' in launch_contents
    assert 'DeclareLaunchArgument("rviz_config", default_value=default_rviz)' in launch_contents
    assert 'DeclareLaunchArgument("enable_scan_wifi_debug", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_topic", default_value="/scan_wifi_debug"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_publish_hz", default_value="2.0"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_beam_stride", default_value="4"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_range_max_m", default_value="12.0"' in launch_contents
    assert "rviz_global_v2.rviz" in launch_contents
    assert 'DeclareLaunchArgument("enable_rtk", default_value="True")' in launch_contents
    assert map_gps_enable_arg in launch_contents
    assert map_gps_topic_arg in launch_contents
    assert map_gps_cov_arg in launch_contents
    assert map_gps_fromll_arg in launch_contents
    assert map_gps_param_ref in launch_contents
    assert '"map_gps_absolute_topic": map_gps_absolute_topic' in launch_contents
    assert '"map_gps_pose_covariance_xy": map_gps_pose_covariance_xy' in launch_contents
    assert '"map_gps_fromll_service": map_gps_fromll_service' in launch_contents
    assert '"map_gps_fromll_service_fallback": map_gps_fromll_service_fallback' in launch_contents
    assert '"map_gps_fromll_wait_timeout_s": map_gps_fromll_wait_timeout_s' in launch_contents
    assert 'DeclareLaunchArgument("navsat_use_odometry_yaw", default_value="false")' in launch_contents
    assert "nav2_global_v2_real_rolling_params.yaml" in launch_contents
    assert '"navsat_use_odometry_yaw": navsat_use_odometry_yaw' in launch_contents
    assert 'DeclareLaunchArgument("enable_gps_course_heading", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_min_distance_m", default_value="2.0")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_min_speed_mps", default_value="0.8")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_max_abs_steer_deg", default_value="3.0")' in launch_contents
    assert 'default_value="0.05"' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_invalid_hold_s", default_value="0.8")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_max_sample_dt_s", default_value="2.5")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_publish_hz", default_value="5.0")' in launch_contents
    assert 'default_value="4.0"' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_require_rtk", default_value="True")' in launch_contents
    assert 'default_value="RTK_FIXED,RTK_FIX,RTK_FLOAT,RTCM_OK"' in launch_contents
    assert "effective_enable_rtk = PythonExpression(" in launch_contents
    assert "'enable_rtk': effective_enable_rtk".replace("'", '"') in launch_contents
    assert 'DeclareLaunchArgument(\n                "gps_rtk_status_topic",' in launch_contents
    assert 'default_value="/gps/rtk_status_mavros"' in launch_contents
    assert 'executable="gps_course_heading"' in launch_contents
    assert 'executable="scan_wifi_debug"' in launch_contents
    assert 'condition=IfCondition(enable_scan_wifi_debug)' in launch_contents
    assert '"source_topic": "/scan"' in launch_contents
    assert '"output_topic": scan_wifi_debug_topic' in launch_contents
    assert '"publish_hz": ParameterValue(' in launch_contents
    assert '"beam_stride": ParameterValue(' in launch_contents
    assert '"output_range_max_m": ParameterValue(' in launch_contents
    assert 'condition=IfCondition(enable_gps_course_heading)' in launch_contents
    assert '"rtk_status_topic": gps_rtk_status_topic' in launch_contents
    assert '"gps_status_topic": gps_rtk_status_topic' in launch_contents
    assert '"invalid_hold_s": ParameterValue(' in launch_contents
    assert '"max_sample_dt_s": ParameterValue(' in launch_contents
    assert '"hold_yaw_variance_multiplier": ParameterValue(' in launch_contents
    assert '"require_rtk": ParameterValue(' in launch_contents
    assert '"allowed_rtk_statuses": gps_course_heading_allowed_rtk_statuses' in launch_contents
    assert '"enable_gps_course_heading": enable_gps_course_heading' in launch_contents
    assert '"gps_course_heading_topic": "/gps/course_heading"' in launch_contents
    _assert_real_wifi_wrapper_parity(launch_contents)


def test_localization_global_v2_launch_supports_datum_overrides() -> None:
    launch_contents = _read("launch/localization_global_v2.launch.py")
    gated_arg = 'DeclareLaunchArgument(\n                "enable_global_odom_stationary_gate"'
    imu_gate_arg = 'DeclareLaunchArgument(\n                "enable_global_imu_stationary_gate"'
    yaw_hold_arg = 'DeclareLaunchArgument(\n                "enable_global_stationary_yaw_hold"'
    map_gps_arg = 'DeclareLaunchArgument(\n                "enable_map_gps_absolute_measurement"'
    navsat_yaw_arg = 'DeclareLaunchArgument("navsat_use_odometry_yaw", default_value="false")'
    gps_heading_arg = 'DeclareLaunchArgument("enable_gps_course_heading", default_value="false")'
    gps_heading_topic_arg = (
        'DeclareLaunchArgument("gps_course_heading_topic", default_value="/gps/course_heading")'
    )

    assert 'DeclareLaunchArgument("datum_lat"' in launch_contents
    assert 'DeclareLaunchArgument("datum_lon"' in launch_contents
    assert 'DeclareLaunchArgument("datum_yaw_deg"' in launch_contents
    assert gated_arg in launch_contents
    assert imu_gate_arg in launch_contents
    assert yaw_hold_arg in launch_contents
    assert map_gps_arg in launch_contents
    assert 'name="global_odom_stationary_gate"' in launch_contents
    assert 'name="global_imu_stationary_gate"' in launch_contents
    assert 'name="global_yaw_stationary_hold"' in launch_contents
    assert 'name="map_gps_absolute_measurement"' in launch_contents
    assert "OpaqueFunction(function=_build_navsat_transform)" in launch_contents
    assert '"wait_for_datum": True' in launch_contents
    assert navsat_yaw_arg in launch_contents
    assert gps_heading_arg in launch_contents
    assert gps_heading_topic_arg in launch_contents
    assert '"use_odometry_yaw": navsat_use_odometry_yaw' in launch_contents
    assert 'DeclareLaunchArgument(\n                "map_gps_absolute_topic"' in launch_contents
    assert '{"odom1": map_gps_absolute_topic}' in launch_contents
    assert '"odom2": global_stationary_yaw_hold_topic' in launch_contents
    assert '"odom2_config": [' in launch_contents
    assert '"imu1": gps_course_heading_topic' in launch_contents
    assert '"datum": [datum_lat, datum_lon, datum_yaw_rad]' in launch_contents
    assert "math.radians(datum_yaw_deg)" in launch_contents


def test_rviz_real_global_v2_launch_targets_global_config_for_local_pc() -> None:
    launch_contents = _read("launch/rviz_real_global_v2.launch.py")
    launch_rsp_arg = (
        'DeclareLaunchArgument(\n                "launch_robot_state_publisher"'
    )

    assert "rviz_global_v2.rviz" in launch_contents
    assert 'DeclareLaunchArgument(\n                "use_sim_time"' in launch_contents
    assert 'default_value="False"' in launch_contents
    assert launch_rsp_arg in launch_contents
    assert "condition=IfCondition(launch_robot_state_publisher)" in launch_contents
    assert 'executable="rviz2"' in launch_contents


def test_real_nav2_global_params_enable_rolling_global_costmap() -> None:
    params_contents = _read("config/nav2_global_v2_real_rolling_params.yaml")

    assert "rolling_window: true" in params_contents
    assert "global_frame: map" in params_contents
    assert "width: 300" in params_contents
    assert "height: 300" in params_contents
    assert "origin_x" not in params_contents
    assert "origin_y" not in params_contents
    assert 'filters: ["keepout_filter"]' in params_contents


def test_rviz_global_v2_prefers_wifi_debug_scan_for_remote_use() -> None:
    rviz_contents = _read("config/rviz_global_v2.rviz")

    assert "Name: Lidar Points" in rviz_contents
    assert "Value: /scan_3d" in rviz_contents
    assert "Enabled: false" in rviz_contents
    assert "Name: LaserScan" in rviz_contents
    assert "Value: /scan_wifi_debug" in rviz_contents


def test_real_global_v2_wifi_launch_wraps_base_without_local_rviz() -> None:
    launch_contents = _read("launch/real_global_v2_wifi.launch.py")

    assert "real_global_v2.launch.py" in launch_contents
    assert "nav2_global_v2_real_rolling_wifi_params.yaml" in launch_contents
    assert '"use_rviz": "false"' in launch_contents
    assert '"enable_scan_wifi_debug": enable_scan_wifi_debug' in launch_contents
    assert '"scan_wifi_debug_topic": scan_wifi_debug_topic' in launch_contents
    assert '"scan_wifi_debug_publish_hz": scan_wifi_debug_publish_hz' in launch_contents
    assert '"scan_wifi_debug_beam_stride": scan_wifi_debug_beam_stride' in launch_contents
    assert '"scan_wifi_debug_range_max_m": scan_wifi_debug_range_max_m' in launch_contents
    _assert_real_wifi_wrapper_parity(launch_contents)


def test_real_global_v2_wifi_rviz_and_params_match_remote_profile() -> None:
    launch_contents = _read("launch/rviz_real_global_v2_wifi.launch.py")
    rviz_contents = _read("config/rviz_global_v2_wifi.rviz")
    nav2_params_contents = _read("config/nav2_global_v2_real_rolling_wifi_params.yaml")

    assert "rviz_real_global_v2.launch.py" in launch_contents
    assert "rviz_global_v2_wifi.rviz" in launch_contents
    assert "Value: /scan_wifi_debug" in rviz_contents
    assert "Value: /odometry/global" in rviz_contents
    assert "Value: /local_costmap/costmap" in rviz_contents
    assert "Value: /global_costmap/costmap" in rviz_contents
    assert "Value: /plan" in rviz_contents
    assert "/gps/odometry_map" not in rviz_contents
    assert "/odometry/local" not in rviz_contents
    assert "/local_nav_v2/path_tracking_debug" not in rviz_contents
    assert "/scan_3d" not in rviz_contents
    assert "publish_frequency: 1.0" in nav2_params_contents
    assert "publish_frequency: 0.5" in nav2_params_contents
    assert "publish_voxel_map: False" in nav2_params_contents
    assert "always_send_full_costmap: false" in nav2_params_contents
