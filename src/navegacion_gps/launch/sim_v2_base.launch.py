import os
import re
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _read_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


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


def _extract_world_name_from_sdf(world_path: str) -> str:
    try:
        with open(world_path, "r", encoding="utf-8") as file_handle:
            world_contents = file_handle.read()
        match = re.search(r"<world\s+name\s*=\s*['\"]([^'\"]+)['\"]", world_contents)
        if match:
            return match.group(1)
    except OSError:
        pass
    return ""


def _materialize_bridge_config_for_world(bridge_config_path: str, world_name: str) -> str:
    with open(bridge_config_path, "r", encoding="utf-8") as file_handle:
        bridge_config_contents = file_handle.read()
    patched_contents = re.sub(
        r"/world/[^/\s]+/",
        f"/world/{world_name}/",
        bridge_config_contents,
    )
    tmp_file = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".yaml",
        prefix="bridge_config_v2_",
        delete=False,
    )
    with tmp_file:
        tmp_file.write(patched_contents)
    return tmp_file.name


def _spawn_robot(context):
    custom_urdf = LaunchConfiguration("custom_urdf").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "True"
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_roll = LaunchConfiguration("spawn_roll").perform(context)
    spawn_pitch = LaunchConfiguration("spawn_pitch").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
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
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-name",
                model_name,
                "-file",
                custom_urdf,
                "-x",
                spawn_x,
                "-y",
                spawn_y,
                "-z",
                spawn_z,
                "-R",
                spawn_roll,
                "-P",
                spawn_pitch,
                "-Y",
                spawn_yaw,
            ],
        ),
    ]


def _build_gz_bridge(context, *, bridge_config: str):
    world_path = LaunchConfiguration("world").perform(context)
    world_name = _extract_world_name_from_sdf(world_path) or LaunchConfiguration(
        "world_name"
    ).perform(context)
    bridge_config_path = _materialize_bridge_config_for_world(bridge_config, world_name)
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config_path}],
        )
    ]


def _build_joint_state_bridge(context):
    world_path = LaunchConfiguration("world").perform(context)
    world_name = _extract_world_name_from_sdf(world_path) or LaunchConfiguration(
        "world_name"
    ).perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    joint_state_topic = (
        f"/world/{world_name}/model/{model_name}/joint_state"
        "@sensor_msgs/msg/JointState[gz.msgs.Model"
    )
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[joint_state_topic],
            remappings=[
                (f"/world/{world_name}/model/{model_name}/joint_state", "/joint_states")
            ],
        )
    ]


def _build_lidar_pipeline(context, *, scan_ground_params_file: str):
    """Arma scan_ground_filter (opcional) + pointcloud_to_laserscan.

    Con `enable_scan_ground_filter:=True` se intercala el filtro de suelo entre
    `/scan_3d` y `pointcloud_to_laserscan` (que pasa a consumir
    `/scan_3d/no_ground`) y se ajusta la ventana `min_height`/`max_height` del
    proyector 3D->2D: con el piso ya quitado en 3D se puede bajar `min_height`
    (obstáculos bajos) y subir `max_height` (obstáculos altos sobre terreno
    inclinado) sin reintroducir puntos fantasma del suelo.
    """
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "True"
    p2l_params_file = LaunchConfiguration("lidar_to_scan_params_file").perform(context)
    enabled = LaunchConfiguration("enable_scan_ground_filter").perform(context).lower() in (
        "true",
        "1",
    )
    lof_enabled = LaunchConfiguration(
        "enable_lidar_obstacle_filter"
    ).perform(context).lower() in ("true", "1")

    # Mutuamente excluyentes: con los dos activos se mediría el filtro equivocado
    # en silencio (el obstacle filter gana). Fallar temprano.
    if enabled and lof_enabled:
        raise RuntimeError(
            "enable_scan_ground_filter y enable_lidar_obstacle_filter son "
            "mutuamente excluyentes: elegí uno solo."
        )

    # lidar_obstacle_filter tiene precedencia: publica /scan él mismo (compensa
    # cabeceo con la IMU), así que reemplaza al pointcloud_to_laserscan.
    if lof_enabled:
        ground_thr = float(
            LaunchConfiguration(
                "lidar_obstacle_ground_distance_threshold"
            ).perform(context)
        )
        return [
            Node(
                package="navegacion_gps",
                executable="lidar_obstacle_filter",
                name="lidar_obstacle_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "cloud_topic": "/scan_3d",
                        "imu_topic": "/imu/data",
                        "obstacles_cloud_topic": "/obstacles_cloud",
                        "scan_topic": "/scan",
                        "output_frame": "base_footprint",
                        "ground_distance_threshold": ground_thr,
                    }
                ],
            )
        ]

    nodes = []
    cloud_in = "/scan_3d"
    p2l_overrides = []

    if enabled:
        cloud_in = "/scan_3d/no_ground"
        min_height = float(
            LaunchConfiguration("scan_ground_min_height").perform(context)
        )
        max_height = float(
            LaunchConfiguration("scan_ground_max_height").perform(context)
        )
        p2l_overrides = [{"min_height": min_height, "max_height": max_height}]
        nodes.append(
            Node(
                package="navegacion_gps",
                executable="scan_ground_filter",
                name="scan_ground_filter",
                output="screen",
                parameters=[
                    scan_ground_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            )
        )

    nodes.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[
                p2l_params_file,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                {"output_qos": "sensor_data"},
                *p2l_overrides,
            ],
            remappings=[("cloud_in", cloud_in), ("scan", "/scan")],
        )
    )
    return nodes


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("navegacion_gps")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(gps_wpf_dir, "worlds", "vacio.world")
    bridge_config = _resolve_config_file_path(gps_wpf_dir, "bridge_config_v2.yaml")
    default_lidar_to_scan_params = _resolve_config_file_path(
        gps_wpf_dir, "pointcloud_to_laserscan.yaml"
    )
    scan_ground_params = _resolve_config_file_path(
        gps_wpf_dir, "scan_ground_filter.param.yaml"
    )

    world = LaunchConfiguration("world")
    ign_partition = LaunchConfiguration("ign_partition")
    ign_ip = LaunchConfiguration("ign_ip")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            # Evita que una instancia Gazebo previa (incluso fuera de este
            # contenedor) comparta /clock y el servicio /world/*/create. Sin
            # aislamiento el robot puede crearse en otro servidor y Nav2 queda
            # esperando para siempre el TF map -> base_footprint.
            DeclareLaunchArgument("ign_partition", default_value="salus_sim_v2"),
            # El host puede anunciar Gazebo por una interfaz VPN. Para esta
            # simulacion local se fuerza loopback: servidor, GUI, bridge y
            # spawner se ven entre si, pero no se mezclan con otro host.
            DeclareLaunchArgument("ign_ip", default_value="127.0.0.1"),
            SetEnvironmentVariable(name="IGN_PARTITION", value=ign_partition),
            SetEnvironmentVariable(name="IGN_IP", value=ign_ip),
            DeclareLaunchArgument(
                "lidar_to_scan_params_file",
                default_value=default_lidar_to_scan_params,
            ),
            DeclareLaunchArgument(
                "custom_urdf",
                default_value=os.path.join(gps_wpf_dir, "models", "cuatri_real.urdf"),
            ),
            DeclareLaunchArgument("world", default_value=world_path),
            DeclareLaunchArgument("world_name", default_value="vacio"),
            DeclareLaunchArgument(
                "model_name", default_value="quad_ackermann_viewer_safe"
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("spawn_roll", default_value="0.0"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("gz_args", default_value="-r "),
            DeclareLaunchArgument(
                "enable_scan_ground_filter",
                default_value="False",
                description="Intercala el scan_ground_filter (estilo Autoware) "
                "entre /scan_3d y pointcloud_to_laserscan.",
            ),
            DeclareLaunchArgument(
                "scan_ground_min_height",
                default_value="0.10",
                description="min_height del pointcloud_to_laserscan cuando el "
                "filtro de suelo está activo (el piso ya se quitó en 3D).",
            ),
            DeclareLaunchArgument(
                "scan_ground_max_height",
                default_value="2.50",
                description="max_height del pointcloud_to_laserscan cuando el "
                "filtro de suelo está activo. Más alto que el 1.60 base para "
                "recuperar obstáculos sobre terreno inclinado, seguro porque el "
                "piso ya no está.",
            ),
            DeclareLaunchArgument(
                "enable_lidar_obstacle_filter",
                default_value="False",
                description="Usa el lidar_obstacle_filter (compensación IMU + "
                "RANSAC + tilt gate + persistencia) que publica /scan directo, "
                "en lugar de pointcloud_to_laserscan. Tiene precedencia sobre "
                "enable_scan_ground_filter. Es el mismo nodo cableado en real.",
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_ground_distance_threshold",
                default_value="0.05",
                description="ground_distance_threshold (RANSAC) del "
                "lidar_obstacle_filter; 0.05 ganó el barrido en la rampa sim.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={
                    "gz_args": [LaunchConfiguration("gz_args"), world]
                }.items(),
            ),
            OpaqueFunction(
                function=_build_gz_bridge, kwargs={"bridge_config": bridge_config}
            ),
            OpaqueFunction(function=_build_joint_state_bridge),
            OpaqueFunction(function=_spawn_robot),
            OpaqueFunction(
                function=_build_lidar_pipeline,
                kwargs={"scan_ground_params_file": scan_ground_params},
            ),
        ]
    )
