from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    controller_serial_port = LaunchConfiguration("controller_serial_port")
    controller_serial_baud = LaunchConfiguration("controller_serial_baud")
    controller_serial_tx_hz = LaunchConfiguration("controller_serial_tx_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("controller_serial_port", default_value="auto"),
            DeclareLaunchArgument("controller_serial_baud", default_value="115200"),
            DeclareLaunchArgument("controller_serial_tx_hz", default_value="50.0"),
            Node(
                package="controller_server",
                executable="controller_server_node",
                name="controller_server",
                output="screen",
                parameters=[
                    {
                        "serial_port": controller_serial_port,
                        "serial_baud": ParameterValue(
                            controller_serial_baud, value_type=int
                        ),
                        "serial_tx_hz": ParameterValue(
                            controller_serial_tx_hz, value_type=float
                        ),
                        "max_reverse_mps": 1.30,
                        "max_abs_angular_z": 0.4,
                        "wheelbase_m": 0.94,
                        "steering_limit_rad": 0.5235987756,
                        "operational_steering_limit_rad": 0.3141592654,
                        "manual_operational_steering_limit_rad": 0.5235987756,
                        "vx_deadband_mps": 0.10,
                        "vx_min_effective_mps": 0.75,
                        "invert_steer_from_cmd_vel": True,
                    }
                ],
            ),
        ]
    )
