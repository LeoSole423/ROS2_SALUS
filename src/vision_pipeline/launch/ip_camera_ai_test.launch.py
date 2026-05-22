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
        DeclareLaunchArgument(
            'stream_url',
            description='RTSP or MJPEG URL of the IP camera.',
        ),
        DeclareLaunchArgument(
            'model_path',
            description='Absolute path to the YOLO ONNX model file.',
        ),
        DeclareLaunchArgument(
            'target_fps',
            default_value='15.0',
        ),
        DeclareLaunchArgument(
            'width',
            default_value='640',
        ),
        DeclareLaunchArgument(
            'height',
            default_value='360',
        ),
        DeclareLaunchArgument(
            'detector_params_file',
            default_value=default_detector_params,
        ),
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
                {'model_path': LaunchConfiguration('model_path')},
            ],
            output='screen',
        ),
    ])
