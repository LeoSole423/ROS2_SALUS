import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sensores_share_dir = get_package_share_directory("sensores")
    mavros_share_dir = get_package_share_directory("mavros")

    launch_web = LaunchConfiguration("launch_web")
    launch_legacy_compat = LaunchConfiguration("launch_legacy_compat")
    enable_rtk = LaunchConfiguration("enable_rtk")
    enable_rtcm_tcp = LaunchConfiguration("enable_rtcm_tcp")
    enable_rtk_source_manager = LaunchConfiguration("enable_rtk_source_manager")
    rtcm_tcp_host = LaunchConfiguration("rtcm_tcp_host")
    rtcm_tcp_port = LaunchConfiguration("rtcm_tcp_port")
    rtcm_topic = LaunchConfiguration("rtcm_topic")
    send_rtcm_topic = LaunchConfiguration("send_rtcm_topic")
    rtk_status_topic = LaunchConfiguration("rtk_status_topic")
    gps_raw_topic = LaunchConfiguration("gps_raw_topic")
    gps_rtk_topic = LaunchConfiguration("gps_rtk_topic")
    rtk_sources_config = LaunchConfiguration("rtk_sources_config")
    active_rtk_source_id = LaunchConfiguration("active_rtk_source_id")
    rtk_source_select_topic = LaunchConfiguration("rtk_source_select_topic")
    rtk_source_manage_topic = LaunchConfiguration("rtk_source_manage_topic")
    rtk_sources_topic = LaunchConfiguration("rtk_sources_topic")
    rtk_source_status_topic = LaunchConfiguration("rtk_source_status_topic")
    gps_topic = LaunchConfiguration("gps_topic")
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    tgt_system = LaunchConfiguration("tgt_system")
    tgt_component = LaunchConfiguration("tgt_component")
    namespace = LaunchConfiguration("namespace")
    fcu_protocol = LaunchConfiguration("fcu_protocol")
    pluginlists_yaml = LaunchConfiguration("pluginlists_yaml")
    apm_config_yaml = LaunchConfiguration("apm_config_yaml")
    config_yaml = LaunchConfiguration("config_yaml")

    default_pluginlists_yaml = os.path.join(
        sensores_share_dir, "config", "mavros_sensor_only_pluginlists.yaml"
    )
    default_apm_config_yaml = os.path.join(mavros_share_dir, "launch", "apm_config.yaml")
    default_config_yaml = os.path.join(
        sensores_share_dir, "config", "mavros_apm_overrides.yaml"
    )
    source_tree_rtk_sources_yaml = os.path.join(
        "/ros2_ws", "src", "sensores", "config", "rtk_sources.yaml"
    )
    default_rtk_sources_yaml = (
        source_tree_rtk_sources_yaml
        if os.path.exists(source_tree_rtk_sources_yaml)
        else os.path.join(sensores_share_dir, "config", "rtk_sources.yaml")
    )
    local_sources = os.path.join("/ros2_ws", "src", "sensores", "config", "rtk_sources.local.yaml")
    if os.path.isfile(local_sources):
        default_rtk_sources_yaml = local_sources

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_web",
                default_value="false",
                description="Launch the sensores_web node",
            ),
            DeclareLaunchArgument(
                "launch_legacy_compat",
                default_value="true",
                description="Republish MAVROS native topics to the legacy contract",
            ),
            DeclareLaunchArgument(
                "enable_rtk",
                default_value="false",
                description="Launch the RTK bridge that feeds RTCM to the MAVROS gps_rtk plugin",
            ),
            DeclareLaunchArgument(
                "enable_rtcm_tcp",
                default_value="true",
                description="Read RTCM corrections from a TCP source",
            ),
            DeclareLaunchArgument(
                "enable_rtk_source_manager",
                default_value="false",
                description="Launch the NTRIP source manager so the web UI can switch RTCM bases",
            ),
            DeclareLaunchArgument(
                "rtcm_tcp_host",
                default_value="127.0.0.1",
                description="Host for the incoming RTCM TCP stream",
            ),
            DeclareLaunchArgument(
                "rtcm_tcp_port",
                default_value="2102",
                description="Port for the incoming RTCM TCP stream",
            ),
            DeclareLaunchArgument(
                "rtcm_topic",
                default_value="/rtcm",
                description="Optional ROS topic carrying RTCM corrections",
            ),
            DeclareLaunchArgument(
                "send_rtcm_topic",
                default_value="/mavros_node/send_rtcm",
                description="Topic consumed by the MAVROS gps_rtk plugin; override if mavros_node runs in a namespace",
            ),
            DeclareLaunchArgument(
                "rtk_status_topic",
                default_value="/gps/rtk_status",
                description="Topic where the RTK bridge publishes consolidated RTK status",
            ),
            DeclareLaunchArgument(
                "gps_raw_topic",
                default_value="/mavros_node/gps1/raw",
                description="Raw MAVROS GPS topic used to classify RTK float/fixed states",
            ),
            DeclareLaunchArgument(
                "gps_rtk_topic",
                default_value="/mavros_node/gps1/rtk",
                description="MAVROS GPS RTK diagnostics topic used by the RTK bridge",
            ),
            DeclareLaunchArgument(
                "rtk_sources_config",
                default_value=default_rtk_sources_yaml,
                description="YAML file with the available RTK bases for the source manager",
            ),
            DeclareLaunchArgument(
                "active_rtk_source_id",
                default_value="",
                description="Initial RTK source id; empty uses the saved selection, then the first entry",
            ),
            DeclareLaunchArgument(
                "rtk_source_select_topic",
                default_value="/gps/rtk_source/select",
                description="Topic used by the web UI to request a different RTK source",
            ),
            DeclareLaunchArgument(
                "rtk_source_manage_topic",
                default_value="/gps/rtk_source/manage_json",
                description="Topic used by the web UI to add, update or delete RTK sources",
            ),
            DeclareLaunchArgument(
                "rtk_sources_topic",
                default_value="/gps/rtk_sources/json",
                description="Topic where the RTK source manager publishes the available source catalog",
            ),
            DeclareLaunchArgument(
                "rtk_source_status_topic",
                default_value="/gps/rtk_source/status_json",
                description="Topic where the RTK source manager publishes the current source state",
            ),
            DeclareLaunchArgument(
                "gps_topic",
                default_value="/global_position/raw/fix",
                description="Native GPS topic used for RTK diagnostics and the web dashboard",
            ),
            DeclareLaunchArgument(
                "fcu_url",
                default_value="/dev/ttyACM0:921600",
                description="FCU connection URL used by MAVROS",
            ),
            DeclareLaunchArgument(
                "gcs_url",
                default_value="",
                description="Optional GCS connection URL used by MAVROS",
            ),
            DeclareLaunchArgument(
                "tgt_system",
                default_value="1",
                description="MAVLink target system id",
            ),
            DeclareLaunchArgument(
                "tgt_component",
                default_value="1",
                description="MAVLink target component id",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for mavros_node; empty keeps canonical root-level topics",
            ),
            DeclareLaunchArgument(
                "fcu_protocol",
                default_value="v2.0",
                description="MAVLink protocol version",
            ),
            DeclareLaunchArgument(
                "pluginlists_yaml",
                default_value=default_pluginlists_yaml,
                description="Path to MAVROS plugin allow/deny list config",
            ),
            DeclareLaunchArgument(
                "config_yaml",
                default_value=default_config_yaml,
                description="Path to MAVROS runtime config",
            ),
            DeclareLaunchArgument(
                "apm_config_yaml",
                default_value=default_apm_config_yaml,
                description="Path to base MAVROS APM config",
            ),
            Node(
                package="mavros",
                executable="mavros_node",
                name="mavros_node",
                namespace=namespace,
                output="screen",
                remappings=[
                    ("mavros_node/data", "/imu/data"),
                    ("mavros_node/raw/fix", "/global_position/raw/fix"),
                    ("mavros_node/velocity_local", "/local_position/velocity_local"),
                    ("mavros_node/odom", "/local_position/odom"),
                ],
                parameters=[
                    pluginlists_yaml,
                    apm_config_yaml,
                    config_yaml,
                    {
                        "fcu_url": fcu_url,
                        "gcs_url": gcs_url,
                        "tgt_system": tgt_system,
                        "tgt_component": tgt_component,
                        "fcu_protocol": fcu_protocol,
                    },
                ],
            ),
            Node(
                package="sensores",
                executable="mavros_compat_bridge",
                name="mavros_compat_bridge",
                output="screen",
                condition=IfCondition(launch_legacy_compat),
            ),
            Node(
                package="sensores",
                executable="sensores_web",
                name="sensores_web",
                output="screen",
                condition=IfCondition(launch_web),
                parameters=[
                    {"imu_topic": "/imu/data"},
                    {"gps_topic": gps_topic},
                    {"gps_raw_topic": "/mavros_node/gps1/raw"},
                    {"rtk_sources_config": rtk_sources_config},
                    {"rtk_sources_topic": rtk_sources_topic},
                    {"rtk_source_status_topic": rtk_source_status_topic},
                    {"rtk_source_select_topic": rtk_source_select_topic},
                    {"rtk_source_manage_topic": rtk_source_manage_topic},
                    {"velocity_topic": "/local_position/velocity_local"},
                    {"odom_topic": "/local_position/odom"},
                ],
            ),
            Node(
                package="sensores",
                executable="rtk_source_manager",
                name="rtk_source_manager",
                output="screen",
                condition=IfCondition(PythonExpression([
                    "'", enable_rtk, "'.lower() in ('true', '1') and '",
                    enable_rtk_source_manager, "'.lower() in ('true', '1')",
                ])),
                parameters=[
                    {"sources_config": rtk_sources_config},
                    {"active_source_id": active_rtk_source_id},
                    {"rtcm_topic": rtcm_topic},
                    {"source_select_topic": rtk_source_select_topic},
                    {"source_manage_topic": rtk_source_manage_topic},
                    {"sources_topic": rtk_sources_topic},
                    {"source_status_topic": rtk_source_status_topic},
                ],
            ),
            Node(
                package="sensores",
                executable="rtk_bridge",
                name="rtk_bridge",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            enable_rtk,
                            "'.lower() in ('true', '1') and '",
                            enable_rtk_source_manager,
                            "'.lower() in ('true', '1')",
                        ]
                    )
                ),
                parameters=[
                    {"enable_rtcm_tcp": False},
                    {"rtcm_tcp_host": rtcm_tcp_host},
                    {"rtcm_tcp_port": rtcm_tcp_port},
                    {"rtcm_topic": rtcm_topic},
                    {"send_rtcm_topic": send_rtcm_topic},
                    {"gps_topic": gps_topic},
                    {"status_topic": rtk_status_topic},
                    {"gps_raw_topic": gps_raw_topic},
                    {"gps_rtk_topic": gps_rtk_topic},
                ],
            ),
            Node(
                package="sensores",
                executable="rtk_bridge",
                name="rtk_bridge",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            enable_rtk,
                            "'.lower() in ('true', '1') and '",
                            enable_rtk_source_manager,
                            "'.lower() not in ('true', '1')",
                        ]
                    )
                ),
                parameters=[
                    {"enable_rtcm_tcp": enable_rtcm_tcp},
                    {"rtcm_tcp_host": rtcm_tcp_host},
                    {"rtcm_tcp_port": rtcm_tcp_port},
                    {"rtcm_topic": rtcm_topic},
                    {"send_rtcm_topic": send_rtcm_topic},
                    {"gps_topic": gps_topic},
                    {"status_topic": rtk_status_topic},
                    {"gps_raw_topic": gps_raw_topic},
                    {"gps_rtk_topic": gps_rtk_topic},
                ],
            ),
        ]
    )
