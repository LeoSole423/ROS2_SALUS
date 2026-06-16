import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_DATUM_LAT = -31.4858037
DEFAULT_DATUM_LON = -64.2410570
# Convencion fija operativa para `global v2`: arranca mirando al Este.
# En ROS ENU eso corresponde a `datum_yaw_deg = 0.0`.
DEFAULT_DATUM_YAW_DEG = 0.0


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


def _build_navsat_transform(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    gps_topic = LaunchConfiguration("gps_topic").perform(context)
    navsat_use_odometry_yaw = (
        LaunchConfiguration("navsat_use_odometry_yaw").perform(context).lower() == "true"
    )
    global_localization_params_file = LaunchConfiguration(
        "global_localization_params_file"
    ).perform(context)
    datum_lat = float(LaunchConfiguration("datum_lat").perform(context))
    datum_lon = float(LaunchConfiguration("datum_lon").perform(context))
    datum_yaw_deg = float(LaunchConfiguration("datum_yaw_deg").perform(context))
    datum_yaw_rad = math.radians(datum_yaw_deg)

    return [
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                global_localization_params_file,
                {
                    "use_sim_time": use_sim_time,
                    "wait_for_datum": True,
                    "use_odometry_yaw": navsat_use_odometry_yaw,
                    "datum": [datum_lat, datum_lon, datum_yaw_rad],
                },
            ],
            remappings=[
                ("imu/data", imu_topic),
                ("gps/fix", gps_topic),
                ("odometry/filtered", "/odometry/local"),
                ("odometry/gps", "/odometry/gps"),
            ],
        )
    ]


def _build_map_ekf(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    global_localization_params_file = LaunchConfiguration(
        "global_localization_params_file"
    ).perform(context)
    enable_global_odom_stationary_gate = (
        LaunchConfiguration("enable_global_odom_stationary_gate").perform(context).lower()
        == "true"
    )
    global_odom_gated_topic = LaunchConfiguration("global_odom_gated_topic").perform(context)
    enable_global_imu_stationary_gate = (
        LaunchConfiguration("enable_global_imu_stationary_gate").perform(context).lower()
        == "true"
    )
    global_imu_gated_topic = LaunchConfiguration("global_imu_gated_topic").perform(context)
    enable_global_stationary_yaw_hold = (
        LaunchConfiguration("enable_global_stationary_yaw_hold").perform(context).lower()
        == "true"
    )
    global_stationary_yaw_hold_topic = LaunchConfiguration(
        "global_stationary_yaw_hold_topic"
    ).perform(context)
    enable_map_gps_absolute_measurement = (
        LaunchConfiguration("enable_map_gps_absolute_measurement").perform(context).lower()
        == "true"
    )
    map_gps_absolute_topic = LaunchConfiguration("map_gps_absolute_topic").perform(context)
    enable_gps_course_heading = (
        LaunchConfiguration("enable_gps_course_heading").perform(context).lower() == "true"
    )
    gps_course_heading_topic = LaunchConfiguration("gps_course_heading_topic").perform(context)
    enable_compass_heading_fusion = (
        LaunchConfiguration("enable_compass_heading_fusion").perform(context).lower()
        == "true"
    )
    enable_compass_initial_guess = (
        LaunchConfiguration("enable_compass_initial_guess").perform(context).lower()
        == "true"
    )
    compass_heading_topic = LaunchConfiguration("compass_heading_topic").perform(context)
    map_filter_odom_topic = (
        global_odom_gated_topic if enable_global_odom_stationary_gate else "/odometry/local"
    )
    map_filter_imu_topic = (
        global_imu_gated_topic if enable_global_imu_stationary_gate else imu_topic
    )

    parameters = [
        global_localization_params_file,
        {"use_sim_time": use_sim_time},
        {"odom0": map_filter_odom_topic},
        {"imu0": map_filter_imu_topic},
    ]
    if enable_global_stationary_yaw_hold:
        parameters.append(
            {
                "odom2": global_stationary_yaw_hold_topic,
                "odom2_config": [
                    False,
                    False,
                    False,
                    False,
                    False,
                    True,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                ],
                "odom2_queue_size": 10,
                "odom2_differential": False,
                "odom2_relative": False,
            }
        )
    if enable_map_gps_absolute_measurement:
        parameters.append({"odom1": map_gps_absolute_topic})
    if enable_gps_course_heading:
        parameters.append(
            {
                "imu1": gps_course_heading_topic,
                "imu1_config": [
                    False,
                    False,
                    False,
                    False,
                    False,
                    True,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                ],
                "imu1_queue_size": 10,
                "imu1_differential": False,
                "imu1_relative": False,
                "imu1_remove_gravitational_acceleration": False,
            }
        )
    if enable_compass_heading_fusion or enable_compass_initial_guess:
        parameters.append(
            {
                "imu2": compass_heading_topic,
                "imu2_config": [
                    False,
                    False,
                    False,
                    False,
                    False,
                    True,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                    False,
                ],
                "imu2_queue_size": 10,
                "imu2_differential": False,
                "imu2_relative": False,
                "imu2_remove_gravitational_acceleration": False,
            }
        )

    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=parameters,
            remappings=[
                ("imu/data", imu_topic),
                ("odometry/filtered", "/odometry/global"),
            ],
        )
    ]


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    default_global_params_file = _resolve_config_file_path(
        gps_wpf_dir, "localization_global_v2.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    drive_telemetry_topic = LaunchConfiguration("drive_telemetry_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    gps_topic = LaunchConfiguration("gps_topic")
    wheelbase_m = LaunchConfiguration("wheelbase_m")
    invert_measured_steer_sign = LaunchConfiguration("invert_measured_steer_sign")
    pose_covariance_xy = LaunchConfiguration("pose_covariance_xy")
    pose_covariance_yaw = LaunchConfiguration("pose_covariance_yaw")
    twist_covariance_vx = LaunchConfiguration("twist_covariance_vx")
    twist_covariance_vy = LaunchConfiguration("twist_covariance_vy")
    twist_covariance_yaw_rate = LaunchConfiguration("twist_covariance_yaw_rate")
    enable_global_odom_stationary_gate = LaunchConfiguration(
        "enable_global_odom_stationary_gate"
    )
    global_odom_gated_topic = LaunchConfiguration("global_odom_gated_topic")
    global_odom_stationary_gate_speed_threshold_mps = LaunchConfiguration(
        "global_odom_stationary_gate_speed_threshold_mps"
    )
    global_odom_stationary_gate_telemetry_timeout_s = LaunchConfiguration(
        "global_odom_stationary_gate_telemetry_timeout_s"
    )
    enable_global_imu_stationary_gate = LaunchConfiguration(
        "enable_global_imu_stationary_gate"
    )
    global_imu_gated_topic = LaunchConfiguration("global_imu_gated_topic")
    global_imu_stationary_gate_speed_threshold_mps = LaunchConfiguration(
        "global_imu_stationary_gate_speed_threshold_mps"
    )
    global_imu_stationary_gate_telemetry_timeout_s = LaunchConfiguration(
        "global_imu_stationary_gate_telemetry_timeout_s"
    )
    enable_global_stationary_yaw_hold = LaunchConfiguration(
        "enable_global_stationary_yaw_hold"
    )
    global_stationary_yaw_hold_topic = LaunchConfiguration("global_stationary_yaw_hold_topic")
    global_stationary_yaw_hold_speed_threshold_mps = LaunchConfiguration(
        "global_stationary_yaw_hold_speed_threshold_mps"
    )
    global_stationary_yaw_hold_telemetry_timeout_s = LaunchConfiguration(
        "global_stationary_yaw_hold_telemetry_timeout_s"
    )
    global_stationary_yaw_hold_variance_rad2 = LaunchConfiguration(
        "global_stationary_yaw_hold_variance_rad2"
    )
    enable_map_gps_absolute_measurement = LaunchConfiguration(
        "enable_map_gps_absolute_measurement"
    )
    map_gps_absolute_topic = LaunchConfiguration("map_gps_absolute_topic")
    map_gps_pose_covariance_xy = LaunchConfiguration("map_gps_pose_covariance_xy")
    map_gps_fromll_service = LaunchConfiguration("map_gps_fromll_service")
    map_gps_fromll_service_fallback = LaunchConfiguration("map_gps_fromll_service_fallback")
    map_gps_fromll_wait_timeout_s = LaunchConfiguration("map_gps_fromll_wait_timeout_s")
    compass_heading_topic = LaunchConfiguration("compass_heading_topic")
    enable_compass_heading_fusion = LaunchConfiguration("enable_compass_heading_fusion")
    enable_compass_initial_guess = LaunchConfiguration("enable_compass_initial_guess")
    # LEGACY: dynamic datum setting is intentionally disabled in current
    # global profiles. Keep the launch switch only for historical tooling.
    datum_setter = LaunchConfiguration("datum_setter")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument(
                "drive_telemetry_topic",
                default_value="/controller/drive_telemetry",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="/global_position/raw/fix",
            ),
            DeclareLaunchArgument("navsat_use_odometry_yaw", default_value="false"),
            DeclareLaunchArgument(
                "enable_global_odom_stationary_gate",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "global_odom_gated_topic",
                default_value="/odometry/local_global",
            ),
            DeclareLaunchArgument(
                "global_odom_stationary_gate_speed_threshold_mps",
                default_value="0.03",
            ),
            DeclareLaunchArgument(
                "global_odom_stationary_gate_telemetry_timeout_s",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "enable_global_imu_stationary_gate",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "global_imu_gated_topic",
                default_value="/imu/data_global",
            ),
            DeclareLaunchArgument(
                "global_imu_stationary_gate_speed_threshold_mps",
                default_value="0.03",
            ),
            DeclareLaunchArgument(
                "global_imu_stationary_gate_telemetry_timeout_s",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "enable_global_stationary_yaw_hold",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "global_stationary_yaw_hold_topic",
                default_value="/odometry/local_yaw_hold",
            ),
            DeclareLaunchArgument(
                "global_stationary_yaw_hold_speed_threshold_mps",
                default_value="0.03",
            ),
            DeclareLaunchArgument(
                "global_stationary_yaw_hold_telemetry_timeout_s",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "global_stationary_yaw_hold_variance_rad2",
                default_value="0.01",
            ),
            DeclareLaunchArgument(
                "enable_map_gps_absolute_measurement",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "map_gps_absolute_topic",
                default_value="/gps/odometry_map",
            ),
            DeclareLaunchArgument(
                "map_gps_pose_covariance_xy",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "map_gps_fromll_service",
                default_value="/fromLL",
            ),
            DeclareLaunchArgument(
                "map_gps_fromll_service_fallback",
                default_value="/navsat_transform/fromLL",
            ),
            DeclareLaunchArgument(
                "map_gps_fromll_wait_timeout_s",
                default_value="0.2",
            ),
            DeclareLaunchArgument("enable_gps_course_heading", default_value="false"),
            DeclareLaunchArgument("gps_course_heading_topic", default_value="/gps/course_heading"),
            DeclareLaunchArgument("enable_compass_heading", default_value="false"),
            DeclareLaunchArgument("compass_heading_topic", default_value="/imu/compass_heading"),
            DeclareLaunchArgument("enable_compass_heading_fusion", default_value="false"),
            DeclareLaunchArgument("enable_compass_initial_guess", default_value="false"),
            DeclareLaunchArgument("wheelbase_m", default_value="0.94"),
            DeclareLaunchArgument(
                "invert_measured_steer_sign",
                default_value="False",
            ),
            DeclareLaunchArgument("pose_covariance_xy", default_value="0.05"),
            DeclareLaunchArgument("pose_covariance_yaw", default_value="0.1"),
            DeclareLaunchArgument("twist_covariance_vx", default_value="0.05"),
            DeclareLaunchArgument("twist_covariance_vy", default_value="0.01"),
            DeclareLaunchArgument("twist_covariance_yaw_rate", default_value="0.1"),
            DeclareLaunchArgument(
                "global_localization_params_file",
                default_value=default_global_params_file,
            ),
            DeclareLaunchArgument("datum_setter", default_value="false"),
            DeclareLaunchArgument("datum_lat", default_value=str(DEFAULT_DATUM_LAT)),
            DeclareLaunchArgument("datum_lon", default_value=str(DEFAULT_DATUM_LON)),
            DeclareLaunchArgument("datum_yaw_deg", default_value=str(DEFAULT_DATUM_YAW_DEG)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(gps_wpf_dir) / "launch" / "localization_v2.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "drive_telemetry_topic": drive_telemetry_topic,
                    "imu_topic": imu_topic,
                    "wheelbase_m": wheelbase_m,
                    "invert_measured_steer_sign": invert_measured_steer_sign,
                    "pose_covariance_xy": pose_covariance_xy,
                    "pose_covariance_yaw": pose_covariance_yaw,
                    "twist_covariance_vx": twist_covariance_vx,
                    "twist_covariance_vy": twist_covariance_vy,
                    "twist_covariance_yaw_rate": twist_covariance_yaw_rate,
                }.items(),
            ),
            Node(
                package="navegacion_gps",
                executable="global_odom_stationary_gate",
                name="global_odom_stationary_gate",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", enable_global_odom_stationary_gate, "'.lower() == 'true'"]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "input_odom_topic": "/odometry/local",
                        "output_odom_topic": global_odom_gated_topic,
                        "drive_telemetry_topic": drive_telemetry_topic,
                        "stationary_speed_threshold_mps": ParameterValue(
                            global_odom_stationary_gate_speed_threshold_mps,
                            value_type=float,
                        ),
                        "drive_telemetry_timeout_s": ParameterValue(
                            global_odom_stationary_gate_telemetry_timeout_s,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="global_imu_stationary_gate",
                name="global_imu_stationary_gate",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", enable_global_imu_stationary_gate, "'.lower() == 'true'"]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "input_imu_topic": imu_topic,
                        "output_imu_topic": global_imu_gated_topic,
                        "drive_telemetry_topic": drive_telemetry_topic,
                        "stationary_speed_threshold_mps": ParameterValue(
                            global_imu_stationary_gate_speed_threshold_mps,
                            value_type=float,
                        ),
                        "drive_telemetry_timeout_s": ParameterValue(
                            global_imu_stationary_gate_telemetry_timeout_s,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="global_yaw_stationary_hold",
                name="global_yaw_stationary_hold",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", enable_global_stationary_yaw_hold, "'.lower() == 'true'"]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "input_odom_topic": "/odometry/local",
                        "output_odom_topic": global_stationary_yaw_hold_topic,
                        "drive_telemetry_topic": drive_telemetry_topic,
                        "stationary_speed_threshold_mps": ParameterValue(
                            global_stationary_yaw_hold_speed_threshold_mps,
                            value_type=float,
                        ),
                        "drive_telemetry_timeout_s": ParameterValue(
                            global_stationary_yaw_hold_telemetry_timeout_s,
                            value_type=float,
                        ),
                        "yaw_variance_rad2": ParameterValue(
                            global_stationary_yaw_hold_variance_rad2,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="navegacion_gps",
                executable="map_gps_absolute_measurement",
                name="map_gps_absolute_measurement",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", enable_map_gps_absolute_measurement, "'.lower() == 'true'"]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "gps_topic": gps_topic,
                        "output_topic": map_gps_absolute_topic,
                        "map_frame": "map",
                        "pose_covariance_xy": ParameterValue(
                            map_gps_pose_covariance_xy,
                            value_type=float,
                        ),
                        "fromll_service": map_gps_fromll_service,
                        "fromll_service_fallback": map_gps_fromll_service_fallback,
                        "fromll_wait_timeout_s": ParameterValue(
                            map_gps_fromll_wait_timeout_s,
                            value_type=float,
                        ),
                    }
                ],
            ),
            OpaqueFunction(function=_build_map_ekf),
            OpaqueFunction(function=_build_navsat_transform),
            Node(
                package="navegacion_gps",
                executable="datum_setter",
                name="datum_setter",
                output="screen",
                # LEGACY: do not enable in normal operation. Datum is fixed by
                # site launch arguments so maps, goals and keepout stay stable.
                condition=IfCondition(
                    PythonExpression(["'", datum_setter, "'.lower() == 'true'"])
                ),
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "gps_topic": gps_topic,
                        "imu_topic": imu_topic,
                        "rtk_status_topic": "/gps/rtk_status",
                        "set_datum_service": "/datum_setter/set_datum",
                        "get_datum_service": "/datum_setter/get_datum",
                        "datum_service": "/datum",
                        "datum_service_fallback": "/navsat_transform/datum",
                        "imu_yaw_max_age_s": 1.0,
                        "datum_wait_timeout_s": 2.0,
                        "datum_call_timeout_s": 2.5,
                        "datum_call_retries": 3,
                        "datum_retry_delay_s": 0.15,
                    }
                ],
            ),
        ]
    )
