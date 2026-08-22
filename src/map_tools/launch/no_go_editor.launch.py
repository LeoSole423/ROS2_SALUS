from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")
    gps_topic = LaunchConfiguration("gps_topic")
    gps_status_topic = LaunchConfiguration("gps_status_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    map_frame = LaunchConfiguration("map_frame")
    fromll_frame = LaunchConfiguration("fromll_frame")
    launch_zones_manager = LaunchConfiguration("launch_zones_manager")
    launch_nav_command_server = LaunchConfiguration("launch_nav_command_server")
    launch_nav_snapshot_server = LaunchConfiguration("launch_nav_snapshot_server")
    launch_route_executor = LaunchConfiguration("launch_route_executor")

    zones_set_geojson_service = LaunchConfiguration("zones_set_geojson_service")
    zones_get_state_service = LaunchConfiguration("zones_get_state_service")
    zones_reload_service = LaunchConfiguration("zones_reload_service")

    nav_set_goal_service = LaunchConfiguration("nav_set_goal_service")
    nav_cancel_goal_service = LaunchConfiguration("nav_cancel_goal_service")
    nav_brake_service = LaunchConfiguration("nav_brake_service")
    nav_set_manual_mode_service = LaunchConfiguration("nav_set_manual_mode_service")
    nav_get_state_service = LaunchConfiguration("nav_get_state_service")
    route_set_service = LaunchConfiguration("route_set_service")
    generate_coverage_plan_service = LaunchConfiguration(
        "generate_coverage_plan_service"
    )
    route_cancel_service = LaunchConfiguration("route_cancel_service")
    route_get_state_service = LaunchConfiguration("route_get_state_service")
    patrol_set_service = LaunchConfiguration("patrol_set_service")
    patrol_cancel_service = LaunchConfiguration("patrol_cancel_service")
    patrol_get_state_service = LaunchConfiguration("patrol_get_state_service")
    patrol_return_home_service = LaunchConfiguration("patrol_return_home_service")
    teleop_cmd_topic = LaunchConfiguration("teleop_cmd_topic")

    nav_snapshot_service = LaunchConfiguration("nav_snapshot_service")
    nav_snapshot_scan_topic = LaunchConfiguration("nav_snapshot_scan_topic")
    nav_telemetry_topic = LaunchConfiguration("nav_telemetry_topic")
    camera_pan_service = LaunchConfiguration("camera_pan_service")
    camera_zoom_toggle_service = LaunchConfiguration("camera_zoom_toggle_service")
    camera_status_service = LaunchConfiguration("camera_status_service")
    camera_ptz_service = LaunchConfiguration("camera_ptz_service")
    camera_preset_service = LaunchConfiguration("camera_preset_service")
    camera_save_preset_service = LaunchConfiguration("camera_save_preset_service")
    camera_ptz_state_service = LaunchConfiguration("camera_ptz_state_service")
    enable_control_lock = LaunchConfiguration("enable_control_lock")
    control_lock_start_locked = LaunchConfiguration("control_lock_start_locked")
    sensor_bridge_enabled = LaunchConfiguration("sensor_bridge_enabled")
    sensor_bridge_http_url = LaunchConfiguration("sensor_bridge_http_url")
    datum_get_service = LaunchConfiguration("datum_get_service")
    fixed_datum_lat = LaunchConfiguration("fixed_datum_lat")
    fixed_datum_lon = LaunchConfiguration("fixed_datum_lon")
    fixed_datum_yaw_deg = LaunchConfiguration("fixed_datum_yaw_deg")
    fixed_datum_source = LaunchConfiguration("fixed_datum_source")
    datums_file = LaunchConfiguration("datums_file")

    request_timeout_s = LaunchConfiguration("request_timeout_s")
    snapshot_request_timeout_s = LaunchConfiguration("snapshot_request_timeout_s")
    set_zones_timeout_s = LaunchConfiguration("set_zones_timeout_s")
    set_goal_timeout_s = LaunchConfiguration("set_goal_timeout_s")
    coverage_plan_timeout_s = LaunchConfiguration("coverage_plan_timeout_s")
    coverage_planner_min_turning_radius_m = LaunchConfiguration(
        "coverage_planner_min_turning_radius_m"
    )
    coverage_topology_audit_spacing_m = LaunchConfiguration(
        "coverage_topology_audit_spacing_m"
    )
    coverage_reference_max_age_s = LaunchConfiguration(
        "coverage_reference_max_age_s"
    )
    coverage_start_max_distance_m = LaunchConfiguration(
        "coverage_start_max_distance_m"
    )
    coverage_start_max_heading_error_deg = LaunchConfiguration(
        "coverage_start_max_heading_error_deg"
    )
    coverage_use_headland_guides = LaunchConfiguration(
        "coverage_use_headland_guides"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("ws_port", default_value="8766"),
            DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("gps_status_topic", default_value="/gps/rtk_status"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/local"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument(
                # Marco en el que /fromLL devuelve los puntos, que depende del
                # world_frame del perfil de localizacion: `odom` en los perfiles
                # locales y `map` en global v2. zones_manager rasteriza la
                # mascara keepout en map_frame, asi que si este valor no
                # coincide con el perfil aplica una transformada de mas y deja
                # la zona no-go corrida respecto de donde esta el GeoJSON.
                "fromll_frame",
                default_value="odom",
            ),
            DeclareLaunchArgument("launch_zones_manager", default_value="true"),
            DeclareLaunchArgument("launch_nav_command_server", default_value="true"),
            DeclareLaunchArgument("launch_nav_snapshot_server", default_value="true"),
            DeclareLaunchArgument("launch_route_executor", default_value="true"),
            DeclareLaunchArgument(
                "zones_set_geojson_service", default_value="/zones_manager/set_geojson"
            ),
            DeclareLaunchArgument(
                "zones_get_state_service", default_value="/zones_manager/get_state"
            ),
            DeclareLaunchArgument(
                "zones_reload_service", default_value="/zones_manager/reload_from_disk"
            ),
            DeclareLaunchArgument(
                "nav_set_goal_service", default_value="/nav_command_server/set_goal_ll"
            ),
            DeclareLaunchArgument(
                "nav_cancel_goal_service", default_value="/nav_command_server/cancel_goal"
            ),
            DeclareLaunchArgument(
                "nav_brake_service", default_value="/nav_command_server/brake"
            ),
            DeclareLaunchArgument(
                "nav_set_manual_mode_service",
                default_value="/nav_command_server/set_manual_mode",
            ),
            DeclareLaunchArgument(
                "teleop_cmd_topic",
                default_value="/cmd_vel_teleop",
            ),
            DeclareLaunchArgument(
                "nav_get_state_service", default_value="/nav_command_server/get_state"
            ),
            DeclareLaunchArgument(
                "route_set_service", default_value="/route_executor/set_route_ll"
            ),
            DeclareLaunchArgument(
                "generate_coverage_plan_service",
                default_value="/route_executor/generate_coverage_plan_ll",
            ),
            DeclareLaunchArgument(
                "route_cancel_service", default_value="/route_executor/cancel_route"
            ),
            DeclareLaunchArgument(
                "route_get_state_service", default_value="/route_executor/get_state"
            ),
            DeclareLaunchArgument(
                "patrol_set_service", default_value="/route_executor/set_patrol_ll"
            ),
            DeclareLaunchArgument(
                "patrol_cancel_service", default_value="/route_executor/cancel_patrol"
            ),
            DeclareLaunchArgument(
                "patrol_get_state_service", default_value="/route_executor/get_patrol_state"
            ),
            DeclareLaunchArgument(
                "patrol_return_home_service",
                default_value="/route_executor/request_return_home",
            ),
            DeclareLaunchArgument(
                "nav_snapshot_service",
                default_value="/nav_snapshot_server/get_nav_snapshot",
            ),
            DeclareLaunchArgument(
                "nav_snapshot_scan_topic",
                default_value="/scan",
            ),
            DeclareLaunchArgument(
                "nav_telemetry_topic",
                default_value="/nav_command_server/telemetry",
            ),
            DeclareLaunchArgument(
                "camera_pan_service",
                default_value="/camara/camera_pan",
            ),
            DeclareLaunchArgument(
                "camera_zoom_toggle_service",
                default_value="/camara/camera_zoom_toggle",
            ),
            DeclareLaunchArgument(
                "camera_status_service",
                default_value="/camara/camera_status",
            ),
            DeclareLaunchArgument(
                "camera_ptz_service",
                default_value="/camara/camera_ptz",
            ),
            DeclareLaunchArgument(
                "camera_preset_service",
                default_value="/camara/camera_preset",
            ),
            DeclareLaunchArgument(
                "camera_save_preset_service",
                default_value="/camara/camera_save_preset",
            ),
            DeclareLaunchArgument(
                "camera_ptz_state_service",
                default_value="/camara/camera_ptz_state",
            ),
            DeclareLaunchArgument("enable_control_lock", default_value="false"),
            DeclareLaunchArgument("control_lock_start_locked", default_value="true"),
            DeclareLaunchArgument("sensor_bridge_enabled", default_value="false"),
            DeclareLaunchArgument(
                "sensor_bridge_http_url",
                default_value="http://127.0.0.1:8000/data",
            ),
            DeclareLaunchArgument(
                "datum_get_service",
                default_value="/datum_setter/get_datum",
            ),
            DeclareLaunchArgument("fixed_datum_lat", default_value="nan"),
            DeclareLaunchArgument("fixed_datum_lon", default_value="nan"),
            DeclareLaunchArgument("fixed_datum_yaw_deg", default_value="0.0"),
            DeclareLaunchArgument(
                "fixed_datum_source",
                default_value="global_v2_fixed",
            ),
            DeclareLaunchArgument("datums_file", default_value=""),
            DeclareLaunchArgument("request_timeout_s", default_value="5.0"),
            DeclareLaunchArgument("snapshot_request_timeout_s", default_value="5.0"),
            DeclareLaunchArgument("set_zones_timeout_s", default_value="12.0"),
            DeclareLaunchArgument("set_goal_timeout_s", default_value="12.0"),
            DeclareLaunchArgument("coverage_plan_timeout_s", default_value="5.0"),
            DeclareLaunchArgument(
                "coverage_planner_min_turning_radius_m",
                default_value="4.0",
            ),
            DeclareLaunchArgument(
                "coverage_topology_audit_spacing_m",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "coverage_reference_max_age_s",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "coverage_start_max_distance_m",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "coverage_start_max_heading_error_deg",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "coverage_use_headland_guides",
                default_value="false",
                description=(
                    "Use one non-key exterior guide per coverage headland. "
                    "Leave off: splitting the turn leaves a leg at the exact "
                    "planner minimum radius, which Smac closes with a full "
                    "25 m loop instead of the nominal arc"
                ),
            ),
            Node(
                package="navegacion_gps",
                executable="zones_manager",
                name="zones_manager",
                output="screen",
                condition=IfCondition(launch_zones_manager),
                parameters=[
                    {
                        "map_frame": map_frame,
                        "fromll_frame": fromll_frame,
                        "set_geojson_service": zones_set_geojson_service,
                        "get_state_service": zones_get_state_service,
                        "reload_from_disk_service": zones_reload_service,
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="nav_command_server",
                name="nav_command_server",
                output="screen",
                condition=IfCondition(launch_nav_command_server),
                parameters=[
                    {
                        "map_frame": map_frame,
                        "gps_topic": gps_topic,
                        "telemetry_topic": nav_telemetry_topic,
                        "teleop_cmd_topic": teleop_cmd_topic,
                        "set_goal_service": nav_set_goal_service,
                        "cancel_goal_service": nav_cancel_goal_service,
                        "brake_service": nav_brake_service,
                        "set_manual_mode_service": nav_set_manual_mode_service,
                        "get_state_service": nav_get_state_service,
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="nav_snapshot_server",
                name="nav_snapshot_server",
                output="screen",
                condition=IfCondition(launch_nav_snapshot_server),
                parameters=[
                    {
                        "get_snapshot_service": nav_snapshot_service,
                        "scan_topic": nav_snapshot_scan_topic,
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="route_executor",
                name="route_executor",
                output="screen",
                condition=IfCondition(launch_route_executor),
                parameters=[
                    {
                        "nav_set_goal_service": nav_set_goal_service,
                        "nav_cancel_goal_service": nav_cancel_goal_service,
                        "nav_telemetry_topic": nav_telemetry_topic,
                        "set_route_service": route_set_service,
                        "generate_coverage_plan_service": generate_coverage_plan_service,
                        "coverage_planner_min_turning_radius_m": (
                            coverage_planner_min_turning_radius_m
                        ),
                        "coverage_topology_audit_spacing_m": (
                            coverage_topology_audit_spacing_m
                        ),
                        "cancel_route_service": route_cancel_service,
                        "get_state_service": route_get_state_service,
                    }
                ],
            ),
            Node(
                package="map_tools",
                executable="web_zone_server",
                name="web_zone_server",
                output="screen",
                parameters=[
                    {
                        "ws_host": ws_host,
                        "ws_port": ws_port,
                        "gps_topic": gps_topic,
                        "gps_status_topic": gps_status_topic,
                        "odom_topic": odom_topic,
                        "map_frame": map_frame,
                        "zones_set_geojson_service": zones_set_geojson_service,
                        "zones_get_state_service": zones_get_state_service,
                        "zones_reload_service": zones_reload_service,
                        "nav_set_goal_service": nav_set_goal_service,
                        "nav_cancel_goal_service": nav_cancel_goal_service,
                        "nav_brake_service": nav_brake_service,
                        "nav_set_manual_mode_service": nav_set_manual_mode_service,
                        "nav_get_state_service": nav_get_state_service,
                        "route_set_service": route_set_service,
                        "coverage_plan_service": generate_coverage_plan_service,
                        "route_cancel_service": route_cancel_service,
                        "route_get_state_service": route_get_state_service,
                        "patrol_set_service": patrol_set_service,
                        "patrol_cancel_service": patrol_cancel_service,
                        "patrol_get_state_service": patrol_get_state_service,
                        "patrol_return_home_service": patrol_return_home_service,
                        "teleop_cmd_topic": teleop_cmd_topic,
                        "nav_snapshot_service": nav_snapshot_service,
                        "nav_telemetry_topic": nav_telemetry_topic,
                        "camera_pan_service": camera_pan_service,
                        "camera_zoom_toggle_service": camera_zoom_toggle_service,
                        "camera_status_service": camera_status_service,
                        "camera_ptz_service": camera_ptz_service,
                        "camera_preset_service": camera_preset_service,
                        "camera_save_preset_service": camera_save_preset_service,
                        "camera_ptz_state_service": camera_ptz_state_service,
                        "enable_control_lock": enable_control_lock,
                        "control_lock_start_locked": control_lock_start_locked,
                        "sensor_bridge_enabled": sensor_bridge_enabled,
                        "sensor_bridge_http_url": sensor_bridge_http_url,
                        "datum_get_service": datum_get_service,
                        "fixed_datum_lat": fixed_datum_lat,
                        "fixed_datum_lon": fixed_datum_lon,
                        "fixed_datum_yaw_deg": fixed_datum_yaw_deg,
                        "fixed_datum_source": fixed_datum_source,
                        "datums_file": datums_file,
                        "request_timeout_s": request_timeout_s,
                        "snapshot_request_timeout_s": snapshot_request_timeout_s,
                        "set_zones_timeout_s": set_zones_timeout_s,
                        "set_goal_timeout_s": set_goal_timeout_s,
                        "coverage_plan_timeout_s": coverage_plan_timeout_s,
                        "coverage_reference_max_age_s": coverage_reference_max_age_s,
                        "coverage_start_max_distance_m": coverage_start_max_distance_m,
                        "coverage_start_max_heading_error_deg": (
                            coverage_start_max_heading_error_deg
                        ),
                        "coverage_use_headland_guides": ParameterValue(
                            coverage_use_headland_guides,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
