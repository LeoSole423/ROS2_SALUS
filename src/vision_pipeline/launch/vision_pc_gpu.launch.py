"""
vision_pc_gpu.launch.py — pipeline de visión para PC con GPU.

Lanza ip_camera_publisher + yolo_onnx_detector (GPU).
El cockpit consume /camera/image_raw y /detections para mostrar la visión.

Uso:
  ros2 launch vision_pipeline vision_pc_gpu.launch.py \
    stream_url:='rtsp://admin:PASS@192.168.1.64:554/Streaming/Channels/101' \
    model_path:='/home/user/models/yolo11n.onnx' \
    execution_provider:=cuda
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('vision_pipeline')
    default_detector_params = PathJoinSubstitution(
        [package_share, 'config', 'yolo_detector.yaml']
    )

    return LaunchDescription([
        # ── Cámara ──────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'stream_url',
            description='RTSP o MJPEG URL de la cámara IP.',
        ),
        DeclareLaunchArgument(
            'target_fps',
            default_value='15.0',
            description='FPS objetivo para ip_camera_publisher.',
        ),
        DeclareLaunchArgument(
            'width',
            default_value='640',
            description='Ancho de captura en píxeles.',
        ),
        DeclareLaunchArgument(
            'height',
            default_value='360',
            description='Alto de captura en píxeles.',
        ),
        # ── Detector ────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'model_path',
            description='Ruta absoluta al modelo YOLO ONNX en la PC.',
        ),
        DeclareLaunchArgument(
            'execution_provider',
            default_value='auto',
            description=(
                'Backend de ONNX Runtime: auto | cuda | tensorrt | openvino | cpu. '
                '"auto" intenta TensorRT → CUDA → CPU en orden.'
            ),
        ),
        DeclareLaunchArgument(
            'intra_op_threads',
            default_value='4',
            description='Threads intra-operación para ONNX Runtime.',
        ),
        DeclareLaunchArgument(
            'inter_op_threads',
            default_value='2',
            description='Threads inter-operación para ONNX Runtime.',
        ),
        DeclareLaunchArgument(
            'detector_params_file',
            default_value=default_detector_params,
            description='YAML con parámetros adicionales del detector.',
        ),
        # ── Nodos ───────────────────────────────────────────────────────────
        Node(
            package='vision_pipeline',
            executable='ip_camera_publisher',
            name='ip_camera_publisher',
            parameters=[{
                'stream_url': LaunchConfiguration('stream_url'),
                'image_topic': '/camera/image_raw',
                'target_fps': LaunchConfiguration('target_fps'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
            }],
            output='screen',
        ),
        Node(
            package='vision_pipeline',
            executable='yolo_onnx_detector',
            name='yolo_onnx_detector',
            parameters=[
                LaunchConfiguration('detector_params_file'),
                {
                    'model_path': LaunchConfiguration('model_path'),
                    'execution_provider': LaunchConfiguration('execution_provider'),
                    'intra_op_threads': LaunchConfiguration('intra_op_threads'),
                    'inter_op_threads': LaunchConfiguration('inter_op_threads'),
                },
            ],
            output='screen',
        ),
    ])
