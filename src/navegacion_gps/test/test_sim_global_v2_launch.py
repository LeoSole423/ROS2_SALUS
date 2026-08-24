from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LIDAR_FILTER_DEFAULT_ARG = (
    'DeclareLaunchArgument("enable_lidar_obstacle_filter", default_value="False")'
)
SCAN_FILTER_DEFAULT_ARG = (
    'DeclareLaunchArgument("enable_scan_noise_filter", default_value="True")'
)
SCAN_FILTER_OUTPUT_ARG = (
    'DeclareLaunchArgument("scan_noise_filter_output", default_value="/scan_clean")'
)


def _read(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def _read_repo(relative_path: str) -> str:
    return (PACKAGE_ROOT.parents[1] / relative_path).read_text(encoding="utf-8")


def test_coverage_turning_radius_is_consistent_across_profiles() -> None:
    sim_launch = _read("launch/sim_global_v2.launch.py")
    real_launch = _read("launch/real_global_v2.launch.py")
    sim_params = _read("config/nav2_global_v2_sim_rolling_params.yaml")
    sim_wifi_params = _read("config/nav2_global_v2_sim_rolling_wifi_params.yaml")
    real_params = _read("config/nav2_global_v2_real_rolling_params.yaml")
    real_wifi_params = _read("config/nav2_global_v2_real_rolling_wifi_params.yaml")

    # El plan de cobertura no puede pedir menos que el Smac que lo sigue, asi
    # que el piso del executor, Coverage Server y Smac se mueven juntos.
    assert '"coverage_planner_min_turning_radius_m": 4.0' in sim_launch
    assert '"coverage_allow_headland_conflicts": True' in sim_launch
    # La cobertura avanza fila por fila. Los giros cerrados se resuelven con la
    # omega hacia adelante y no salteando pasadas.
    assert '"coverage_allow_row_skipping": False' in sim_launch
    assert '"coverage_nogo_enabled": True' in sim_launch
    assert '"coverage_f2c_route_type": "BOUSTROPHEDON"' in sim_launch
    assert '"coverage_start_max_distance_m": "50.0"' in sim_launch
    # La ejecucion usa los extremos de surco sobre el borde. Las guias de
    # cabecera quedan fuera de la ruta ejecutable porque pueden traer una
    # accion coverage_backup; el enlace lo resuelve Nav2 hacia adelante.
    assert '"coverage_use_headland_guides": "false"' in sim_launch
    # Tanto quien envia el goal como quien dibuja mission_path tienen que
    # interpretar el Point sin header de /fromLL en el mismo frame. Si el
    # executor usa odom, aplica odom->map una segunda vez y RViz queda corrido.
    # El tercero es zones_manager: rasteriza la mascara keepout en map, asi que
    # con el default odom la zona no-go queda corrida el offset map->odom y
    # Nav2 frena por una colision donde el GeoJSON no tiene nada.
    assert sim_launch.count('"fromll_frame": "map"') == 3
    assert '"path_frame": "map"' in sim_launch
    # 25 grados dan un radio efectivo de 2.02 m: deja margen de correccion
    # respecto de los 4.0 m que traza el planner.
    assert '"operational_steering_limit_rad": 0.4363323130' in sim_launch
    assert sim_params.count("minimum_turning_radius: 4.0") == 2
    assert sim_wifi_params.count("minimum_turning_radius: 4.0") == 2

    # CAMPO corre igual en real que en sim: mismo planificador, mismo piso de
    # radio y el mismo orden adyacente de pasadas.
    assert '"coverage_planner_min_turning_radius_m": 4.0' in real_launch
    assert '"coverage_allow_headland_conflicts": True' in real_launch
    assert '"coverage_allow_row_skipping": False' in real_launch
    assert '"coverage_nogo_enabled": True' in real_launch
    assert '"coverage_planner": _COVERAGE_PLANNER' in real_launch
    assert '"coverage_f2c_route_type": "BOUSTROPHEDON"' in real_launch
    assert '"coverage_f2c_swath_angle_deg": 0.0' in real_launch
    # El Coverage Server se agrega solo si el overlay esta instalado: en el
    # robot, un overlay faltante no puede voltear la navegacion entera.
    assert "_COVERAGE_SERVER_OK" in real_launch
    # El preflight de arranque. El default del nodo son 5.0 m y describe al
    # planificador propio: ahi la primera pasada nace bajo el vehiculo, asi que
    # estar lejos de ella era estar lejos del lote. Con Fields2Cover la primera
    # pasada la elige la forma del lote y puede caer en el extremo opuesto, de
    # modo que el default rechazaba arranques legitimos: CAMPO planificaba bien
    # y no arrancaba nunca en el robot. Ahora la distancia se mide contra el
    # poligono del lote y el umbral describe "estoy en el lote".
    #
    # El real tiene que pasarlo -sin esto vuelve al default de 5 m- y tiene que
    # seguir siendo mas estricto que sim, que es lo que la version anterior de
    # este test queria proteger cuando pedia que el real no lo copiara.
    assert '"coverage_start_max_distance_m": "25.0"' in real_launch
    real_max_start_m = float(
        real_launch.split('"coverage_start_max_distance_m": "')[1].split('"')[0]
    )
    sim_max_start_m = float(
        sim_launch.split('"coverage_start_max_distance_m": "')[1].split('"')[0]
    )
    assert real_max_start_m < sim_max_start_m
    # Medir contra el lote es lo que vuelve util al umbral: contra la pasada
    # sorteada no hay numero que sirva sin ser mas ancho que la diagonal.
    servidor_web = (
        PACKAGE_ROOT.parent / "map_tools" / "map_tools" / "web_zone_server.py"
    ).read_text(encoding="utf-8")
    assert "_distance_to_ring_m" in servidor_web
    assert '"distance_scope"' in servidor_web
    # Las guias de cabecera si siguen sin copiarse: son el default del editor.
    assert "coverage_use_headland_guides" not in real_launch
    assert real_launch.count('"fromll_frame": "map"') == 3

    # El editor tiene que aceptar el frame y bajarselo a zones_manager. Sin
    # este cableado los dos perfiles global v2 vuelven al default odom y la
    # mascara keepout se corre respecto del GeoJSON.
    editor_launch = (
        PACKAGE_ROOT.parent / "map_tools" / "launch" / "no_go_editor.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(' in editor_launch
    assert '"fromll_frame",' in editor_launch
    assert '"fromll_frame": fromll_frame,' in editor_launch
    assert '"path_frame": "map"' in real_launch
    assert '"operational_steering_limit_rad": 0.3141592654' in real_launch
    assert real_params.count("minimum_turning_radius: 4.0") == 2
    assert real_wifi_params.count("minimum_turning_radius: 4.0") == 2

    # El mismo lookahead amortiguado se usa en los cuatro perfiles rolling.
    # Con 1.0--2.2 m la direccion alternaba de signo aun sobre filas rectas.
    for contents in (sim_params, sim_wifi_params, real_params, real_wifi_params):
        assert contents.count("bt_loop_duration: 50") == 1
        assert contents.count("default_server_timeout: 200") == 1
        assert contents.count("lookahead_dist: 3.0") == 1
        assert contents.count("min_lookahead_dist: 2.3") == 1
        assert contents.count("max_lookahead_dist: 4.5") == 1
        assert contents.count("lookahead_time: 2.2") == 1
        assert contents.count("angle_quantization_bins: 48") == 1
        assert contents.count('plugin: "nav2_controller::PositionGoalChecker"') == 1
        assert 'plugin: "nav2_controller::SimpleGoalChecker"' not in contents


def test_backup_collision_check_uses_the_local_costmap_frame() -> None:
    for relative_path in (
        "config/nav2_global_v2_sim_rolling_params.yaml",
        "config/nav2_global_v2_sim_rolling_wifi_params.yaml",
        "config/nav2_global_v2_real_rolling_params.yaml",
        "config/nav2_global_v2_real_rolling_wifi_params.yaml",
    ):
        contents = _read(relative_path)
        behavior = contents.split("behavior_server:", 1)[1].split(
            "# ================================", 1
        )[0]
        assert "costmap_topic: local_costmap/costmap_raw" in behavior
        assert "footprint_topic: local_costmap/published_footprint" in behavior
        assert "global_frame: odom" in behavior
        assert "global_frame: map" not in behavior


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
    assert 'default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real_v2.urdf")' in launch_contents
    assert "localization_global_v2.launch.py" in launch_contents
    assert "nav_global_v2.launch.py" in launch_contents
    assert 'DeclareLaunchArgument("enable_nav_trace", default_value="True")' in launch_contents
    assert 'executable="nav_trace_recorder"' in launch_contents
    assert '"nav_through_poses_bt_xml": selected_through_poses_bt' in launch_contents
    assert "no_go_editor.launch.py" in launch_contents
    assert 'executable="sim_battery_publisher"' not in launch_contents
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
    assert 'DeclareLaunchArgument("use_keepout", default_value="True")' in launch_contents
    assert LIDAR_FILTER_DEFAULT_ARG in launch_contents
    assert SCAN_FILTER_DEFAULT_ARG in launch_contents
    assert SCAN_FILTER_OUTPUT_ARG in launch_contents
    assert 'executable="scan_noise_filter"' in launch_contents
    assert 'condition=IfCondition(enable_legacy_scan_noise_filter)' in launch_contents
    assert '"lidar_scan_topic": effective_lidar_scan_topic' in launch_contents
    assert 'DeclareLaunchArgument("spawn_x", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_y", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_z", default_value="0.2")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_yaw", default_value="0.0")' in launch_contents
    assert '"spawn_x": spawn_x' in launch_contents
    assert '"spawn_y": spawn_y' in launch_contents
    assert '"spawn_z": spawn_z' in launch_contents
    assert '"spawn_yaw": spawn_yaw' in launch_contents
    assert "sim_compass_initial_yaw_offset_deg = PythonExpression(" in launch_contents
    assert '"initial_yaw_offset_deg": ParameterValue(' in launch_contents
    assert 'DeclareLaunchArgument("enable_compass_initial_guess", default_value="false")' in launch_contents
    assert "effective_enable_compass_heading = PythonExpression(" in launch_contents
    assert 'condition=IfCondition(effective_enable_compass_heading)' in launch_contents
    assert '"initial_guess_only": ParameterValue(' in launch_contents
    assert '"enable_compass_initial_guess": enable_compass_initial_guess' in launch_contents
    assert '"odom_topic": "/odometry/global"' in launch_contents
    assert '"launch_nav_command_server": "false"' in launch_contents
    assert '"battery_guard_topic": "/battery_mission_guard"' in launch_contents
    assert 'executable="rviz2"' not in launch_contents


def test_localization_global_v2_launch_adds_map_filter_and_navsat_support() -> None:
    launch_contents = _read("launch/localization_global_v2.launch.py")
    localization_params = _read("config/localization_global_v2.yaml")
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
    compass_heading_enable_arg = (
        'DeclareLaunchArgument("enable_compass_heading", default_value="false")'
    )
    compass_heading_fusion_arg = (
        'DeclareLaunchArgument("enable_compass_heading_fusion", default_value="false")'
    )
    compass_initial_guess_arg = (
        'DeclareLaunchArgument("enable_compass_initial_guess", default_value="false")'
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
    assert compass_heading_enable_arg in launch_contents
    assert 'DeclareLaunchArgument("compass_heading_topic", default_value="/imu/compass_heading")' in launch_contents
    assert compass_heading_fusion_arg in launch_contents
    assert compass_initial_guess_arg in launch_contents
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
    assert '"imu2": compass_heading_topic' in launch_contents
    assert '"imu2_config": [' in launch_contents
    assert "if enable_compass_heading_fusion or enable_compass_initial_guess:" in launch_contents
    assert '("odometry/filtered", "/odometry/local")' in launch_contents
    assert '("odometry/gps", "/odometry/gps")' in launch_contents
    # El EKF map->odom corre a 30 Hz. Se fecha el TF a futuro para que el
    # controlador no aborte entre dos publicaciones por extrapolacion de ~33 ms.
    assert "transform_time_offset: 0.1" in localization_params


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
    assert 'default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real_v2.urdf")' in launch_contents
    assert 'nav2_global_v2_sim_rolling_wifi_params.yaml' in launch_contents
    assert 'DeclareLaunchArgument("enable_scan_wifi_debug", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_min_distance_m", default_value="2.0")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_min_speed_mps", default_value="0.8")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_publish_hz", default_value="5.0")' in launch_contents
    assert 'DeclareLaunchArgument("gps_course_heading_require_rtk", default_value="True")' in launch_contents
    assert 'DeclareLaunchArgument("gps_rtk_status_topic", default_value="/gps/rtk_status")' in launch_contents
    assert 'DeclareLaunchArgument("enable_sim_compass", default_value="false")' in launch_contents
    assert 'DeclareLaunchArgument("sim_compass_hdg_topic", default_value="/sim/compass_hdg")' in launch_contents
    assert 'DeclareLaunchArgument("enable_compass_heading", default_value="false")' in launch_contents
    assert 'DeclareLaunchArgument("enable_compass_heading_fusion", default_value="false")' in launch_contents
    assert 'DeclareLaunchArgument("enable_compass_initial_guess", default_value="false")' in launch_contents
    assert 'DeclareLaunchArgument("enable_scan_ground_filter", default_value="True")' in launch_contents
    assert "spawn_yaw" in launch_contents
    assert '"enable_sim_compass": enable_sim_compass' in launch_contents
    assert '"sim_compass_hdg_topic": sim_compass_hdg_topic' in launch_contents
    assert '"enable_compass_heading": enable_compass_heading' in launch_contents
    assert '"enable_compass_heading_fusion": enable_compass_heading_fusion' in launch_contents
    assert '"enable_compass_initial_guess": enable_compass_initial_guess' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_topic", default_value="/scan_wifi_debug"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_publish_hz", default_value="2.0"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_beam_stride", default_value="4"' in launch_contents
    assert 'DeclareLaunchArgument(\n                "scan_wifi_debug_range_max_m", default_value="12.0"' in launch_contents
    assert 'executable="scan_wifi_debug"' in launch_contents
    assert 'condition=IfCondition(enable_scan_wifi_debug)' in launch_contents
    assert LIDAR_FILTER_DEFAULT_ARG in launch_contents
    assert 'DeclareLaunchArgument("lidar_scan_topic", default_value="/scan_filtered")' in launch_contents
    assert SCAN_FILTER_DEFAULT_ARG in launch_contents
    assert SCAN_FILTER_OUTPUT_ARG in launch_contents
    assert 'DeclareLaunchArgument("spawn_x", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_y", default_value="0.0")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_z", default_value="0.2")' in launch_contents
    assert 'DeclareLaunchArgument("spawn_yaw", default_value="0.0")' in launch_contents
    assert '"enable_lidar_obstacle_filter": enable_lidar_obstacle_filter' in launch_contents
    assert '"lidar_scan_topic": lidar_scan_topic' in launch_contents
    assert '"enable_scan_noise_filter": enable_scan_noise_filter' in launch_contents
    assert '"scan_noise_filter_output": scan_noise_filter_output' in launch_contents
    assert '"scan_noise_filter_range_min_m": scan_noise_filter_range_min_m' in launch_contents
    assert '"scan_noise_filter_range_max_m": scan_noise_filter_range_max_m' in launch_contents
    assert '"scan_noise_filter_speckle_window": (' in launch_contents
    assert '"scan_noise_filter_speckle_max_range_m": (' in launch_contents
    assert '"scan_noise_filter_speckle_max_deviation_m": (' in launch_contents
    assert '"spawn_x": spawn_x' in launch_contents
    assert '"spawn_y": spawn_y' in launch_contents
    assert '"spawn_z": spawn_z' in launch_contents
    assert '"spawn_yaw": spawn_yaw' in launch_contents
    assert '"source_topic": effective_lidar_scan_topic' in launch_contents
    assert '"output_topic": scan_wifi_debug_topic' in launch_contents
    assert '"crop_angle_min_rad": -1.57079632679' in launch_contents
    assert '"crop_angle_max_rad": 1.57079632679' in launch_contents


def test_cuatri_real_v2_wrapper_keeps_current_urdf_entrypoint() -> None:
    script_contents = _read_repo("tools/launch_sim_global_v2_wifi_cuatri_real_v2.sh")

    assert 'URDF_PATH="/ros2_ws/src/navegacion_gps/models/cuatri_real_v2.urdf"' in script_contents
    assert 'MODEL_NAME="cuatri_real_v2"' in script_contents
    assert 'custom_urdf:="${URDF_PATH}"' in script_contents
    assert "enable_sim_compass:=true" in script_contents
    assert "enable_compass_initial_guess:=true" in script_contents


def test_global_v2_rviz_defaults_use_realistic_cuatri_model() -> None:
    launch_paths = [
        "launch/rviz_sim_global_v2.launch.py",
        "launch/rviz_sim_global_v2_wifi.launch.py",
    ]

    for launch_path in launch_paths:
        launch_contents = _read(launch_path)
        assert 'models", "cuatri_real_v2.urdf"' in launch_contents
        assert 'models", "cuatri_real.urdf"' not in launch_contents


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


def test_recovery_behavior_uses_clearance_guard_without_backup() -> None:
    through_poses_bt = _read("config/navigate_through_poses_w_replanning_and_recovery_no_spin.xml")
    to_pose_bt = _read("config/navigate_to_pose_w_replanning_and_recovery_no_spin.xml")
    real_wifi_params = _read("config/nav2_global_v2_real_rolling_wifi_params.yaml")
    sim_wifi_params = _read("config/nav2_global_v2_sim_rolling_wifi_params.yaml")

    assert "BackUp" not in through_poses_bt
    assert "BackUp" not in to_pose_bt
    assert '<Sequence name="WaitAndReplan">' in through_poses_bt
    assert '<Sequence name="WaitAndReplan">' in to_pose_bt
    # 1.2 m es la misma tolerancia de llegada que usa el resto del stack
    # (route_waypoint_reached_tolerance_m y xy_goal_tolerance). Con los 2.5 m
    # anteriores el radio superaba la separacion entre pasadas de cobertura
    # (1.5-2.4 m): al llegar al final de una pasada, la meta de inicio de la
    # siguiente caia dentro del radio, Nav2 la borraba y el vehiculo cruzaba el
    # lote en diagonal en vez de hacer la cabecera.
    assert '<RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="1.2"/>' in through_poses_bt
    assert 'radius="2.5"' not in through_poses_bt
    trace_bt = _read("config/navigate_through_poses_trace.xml")
    assert 'radius="2.5"' not in trace_bt
    assert trace_bt.count('radius="1.2"') == 3
    assert '<Sequence name="PruneAndComputeClearPathThroughPoses">' in through_poses_bt
    assert '<RateController hz="0.333" name="RateControllerComputePathToPose">' in to_pose_bt
    assert '<RateController hz="0.333" name="RateController">' in through_poses_bt
    assert '<Fallback name="FallbackComputePathToPose">' in to_pose_bt
    assert '<GlobalUpdatedGoal />' in to_pose_bt
    assert '<IsPathValid path="{path}" />' in to_pose_bt
    assert '<IsPathClearanceValid path="{path}" service_name="/path_clearance_validator/is_path_clearance_valid" server_timeout="1000" />' in to_pose_bt
    assert '<Sequence name="ComputeClearPathToPose">' in to_pose_bt
    assert '<Sequence name="ComputeClearRecoveryPathToPose">' in to_pose_bt
    assert '<Fallback name="FallbackComputePathThroughPoses">' in through_poses_bt
    assert '<GlobalUpdatedGoal/>' in through_poses_bt
    assert '<IsPathValid path="{path}"/>' in through_poses_bt
    assert '<IsPathClearanceValid path="{path}" service_name="/path_clearance_validator/is_path_clearance_valid" server_timeout="1000"/>' in through_poses_bt
    assert '<Sequence name="PruneAndComputeClearPathThroughPoses">' in through_poses_bt
    assert through_poses_bt.index('<Fallback name="FallbackComputePathThroughPoses">') < (
        through_poses_bt.index('<RemovePassedGoals')
    )
    assert through_poses_bt.index('<RemovePassedGoals') < through_poses_bt.index(
        '<ComputePathThroughPoses'
    )
    assert "<ComputePathToPose goal=\"{goal}\" path=\"{path}\" planner_id=\"GridBased\" />" in to_pose_bt
    assert '<FollowPath path="{path}" controller_id="FollowPath" />' in to_pose_bt
    assert '<FollowPath path="{path}" controller_id="FollowPath"/>' in through_poses_bt
    assert 'server_timeout="5000"' in through_poses_bt
    assert "SmoothPath" not in to_pose_bt
    assert "SmoothPath" not in through_poses_bt
    assert "smoothed_path" not in to_pose_bt
    assert "smoothed_path" not in through_poses_bt
    assert "spin" not in through_poses_bt.lower()
    assert "spin" not in to_pose_bt.lower()
    assert "simulate_ahead_time: 2.0" in real_wifi_params
    assert "simulate_ahead_time: 2.0" in sim_wifi_params


def test_sim_trace_bt_reports_explicit_replan_causes_without_changing_real_bt() -> None:
    trace_bt = _read("config/navigate_through_poses_trace.xml")
    production_bt = _read("config/navigate_through_poses_w_replanning_and_recovery_no_spin.xml")
    sim_params = _read("config/nav2_global_v2_sim_rolling_params.yaml")
    real_params = _read("config/nav2_global_v2_real_rolling_params.yaml")

    assert '<TraceReplan reason="goal_updated"' in trace_bt
    assert '<TraceReplan reason="path_invalid"' in trace_bt
    assert '<TraceReplan reason="clearance_invalid"' in trace_bt
    assert "TraceReplan" not in production_bt
    assert "nav2_trace_replan_decorator_bt_node" in sim_params
    assert "nav2_trace_replan_decorator_bt_node" not in real_params


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
        assert "path_clearance_validator:" in params_contents
        assert "max_check_distance_m: 12.0" in params_contents
        assert "sample_step_m: 0.25" in params_contents
        assert "high_cost_threshold: 100" in params_contents
        assert "lethal_cost_threshold: 253" in params_contents
        assert "min_consecutive_high_cost_samples: 3" in params_contents
        assert "lateral_offsets_m: [0.0, 0.45, -0.45]" in params_contents
        assert "costmap_timeout_s: 4.0" in params_contents
        assert "min_validation_period_s: 0.75" in params_contents
        assert "slow_check_warning_s: 0.1" in params_contents
        assert "nav2_is_path_clearance_valid_condition_bt_node" in params_contents


def test_wifi_nav2_costmaps_use_long_lidar_marking_ranges() -> None:
    real_wifi_params = _read("config/nav2_global_v2_real_rolling_wifi_params.yaml")
    sim_wifi_params = _read("config/nav2_global_v2_sim_rolling_wifi_params.yaml")

    for params_contents in (real_wifi_params, sim_wifi_params):
        assert "width: 30" in params_contents
        assert "height: 30" in params_contents
        assert "raytrace_max_range: 20.0" in params_contents

    assert real_wifi_params.count("obstacle_max_range: 15.0") >= 3
    assert sim_wifi_params.count("obstacle_max_range: 15.0") >= 2


def test_wifi_rpp_lookahead_is_smoother_and_sim_real_parity() -> None:
    real_wifi_params = _read("config/nav2_global_v2_real_rolling_wifi_params.yaml")
    sim_wifi_params = _read("config/nav2_global_v2_sim_rolling_wifi_params.yaml")

    for params_contents in (real_wifi_params, sim_wifi_params):
        assert "use_final_approach_orientation: false" in params_contents
        assert "keep_goal_orientation: true" in params_contents
        assert "desired_linear_vel: 1.6" in params_contents
        assert "lookahead_dist: 3.0" in params_contents
        assert "min_lookahead_dist: 2.3" in params_contents
        assert "max_lookahead_dist: 4.5" in params_contents
        assert "lookahead_time: 2.2" in params_contents


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


def test_la_simulacion_es_forward_only_de_punta_a_punta() -> None:
    """El vehiculo simulado no puede retroceder por ningun camino.

    Son cuatro fuentes posibles de marcha atras y las cuatro tienen que estar
    cerradas a la vez: la cabecera de tres puntos de la cobertura, el planner
    global, el smoother/controlador y los behaviors de Nav2. Alcanza con que
    una quede abierta para que aparezca una velocidad lineal negativa.
    """
    sim_launch = _read("launch/sim_global_v2.launch.py")
    sim_params = _read("config/nav2_global_v2_sim_rolling_params.yaml")
    sim_wifi_params = _read("config/nav2_global_v2_sim_rolling_wifi_params.yaml")

    # 1. Cobertura: sin cabecera de tres puntos, o sea sin backup_m ni accion
    #    coverage_backup. La transicion corta se arma como omega.
    assert '"coverage_f2c_allow_reverse": False' in sim_launch

    for params in (sim_params, sim_wifi_params):
        # 2. Planner global: Dubins, que no tiene primitivas hacia atras.
        #    Reeds-Shepp si las tiene y por eso no se usa.
        assert 'motion_model_for_search: "DUBIN"' in params
        assert "REEDS_SHEPP" not in params
        # 3. Smoother y controlador.
        assert "reversing_enabled: false" in params
        assert "allow_reversing: false" in params
        # 4. Behaviors: BackUp ni se carga en el behavior server.
        assert 'behavior_plugins: ["drive_on_heading", "wait"]' in params
        assert "nav2_behaviors/BackUp" not in params


def test_la_cobertura_del_perfil_real_no_planifica_marcha_atras() -> None:
    """Automatico no retrocede; la recuperacion de Nav2 es otra cosa.

    Son dos mecanismos distintos y solo uno esta prohibido. `BackUp` es una
    recuperacion: la dispara Nav2 cuando el vehiculo queda trabado, y en el
    robot real se conserva. `coverage_f2c_allow_reverse` es otra cosa: decide
    si un PLAN de cobertura puede pedir marcha atras mientras trabaja el lote,
    sin nadie mirando. Eso no se permite.

    Apagarlo ademas es lo que hace que la curva llegue al robot. Con reversa,
    una separacion menor que el diametro de giro se resuelve con la cabecera de
    tres puntos, cuya fase es "headland"; el filtro de guias apagadas conserva
    forward_turn y las fases nogo_*, pero no "headland", asi que el giro se
    calculaba y se tiraba. Sin reversa la maniobra es la omega hacia adelante,
    con fase forward_turn, que si sobrevive.
    """
    real_launch = _read("launch/real_global_v2.launch.py")
    real_params = _read("config/nav2_global_v2_real_rolling_params.yaml")
    real_wifi_params = _read("config/nav2_global_v2_real_rolling_wifi_params.yaml")

    assert '"coverage_f2c_allow_reverse": False' in real_launch
    assert '"coverage_f2c_allow_reverse": True' not in real_launch
    # La recuperacion de Nav2 se conserva: es la diferencia con simulacion.
    for params in (real_params, real_wifi_params):
        assert 'plugin: "nav2_behaviors/BackUp"' in params


def test_el_filtro_de_guias_conserva_la_fase_que_produce_cada_perfil() -> None:
    """La fase del giro tiene que estar en la lista que el filtro respeta.

    Este es el eslabon que fallaba: el planificador producia la maniobra y la
    ruta ejecutable la descartaba, dejando al robot con los extremos de surco
    pelados. Si alguien vuelve a permitir la marcha atras en cobertura, o saca
    forward_turn de la lista, esto tiene que romperse.
    """
    servidor = (
        PACKAGE_ROOT.parent / "map_tools" / "map_tools" / "web_zone_server.py"
    ).read_text(encoding="utf-8")
    planificador = _read("navegacion_gps/coverage_fields2cover.py")

    # Las dos fases de giro que existen, segun con que se resuelva la cabecera.
    assert '"forward_turn"' in planificador
    assert '"headland"' in planificador
    # El filtro solo respeta una de las dos.
    assert '"forward_turn",' in servidor
    assert '"headland",' not in servidor
    # Por eso los dos perfiles tienen que planificar hacia adelante.
    for perfil in ("launch/sim_global_v2.launch.py", "launch/real_global_v2.launch.py"):
        assert '"coverage_f2c_allow_reverse": False' in _read(perfil)


def test_los_arboles_de_comportamiento_de_la_simulacion_no_traen_backup() -> None:
    """Ni el BT ni la lista de plugins pueden invocar BackUp."""
    sim_launch = _read("launch/sim_global_v2.launch.py")
    sim_params = _read("config/nav2_global_v2_sim_rolling_params.yaml")

    assert "navigate_through_poses_w_replanning_and_recovery_no_spin.xml" in sim_launch
    assert "nav2_back_up_action_bt_node" not in sim_params
    for nombre in (
        "config/navigate_through_poses_w_replanning_and_recovery_no_spin.xml",
        "config/navigate_to_pose_w_replanning_and_recovery_no_spin.xml",
        "config/navigate_to_pose_w_replanning_and_recovery_no_spin_no_backup.xml",
    ):
        assert "BackUp" not in _read(nombre)
