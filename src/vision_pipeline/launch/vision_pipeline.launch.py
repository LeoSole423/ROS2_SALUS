from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('vision_pipeline')
    default_camera_params = PathJoinSubstitution(
        [package_share, 'config', 'v4l2_camera_low_latency.yaml']
    )
    default_detector_params = PathJoinSubstitution(
        [package_share, 'config', 'yolo_detector.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            description='Absolute path to the YOLO ONNX model file.',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Linux V4L2 device for the USB camera.',
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
            description='Namespace for the camera node so the topic becomes /camera/image_raw.',
        ),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=default_camera_params,
        ),
        DeclareLaunchArgument(
            'detector_params_file',
            default_value=default_detector_params,
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            namespace=LaunchConfiguration('camera_namespace'),
            parameters=[
                LaunchConfiguration('camera_params_file'),
                {'video_device': LaunchConfiguration('video_device')},
            ],
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
