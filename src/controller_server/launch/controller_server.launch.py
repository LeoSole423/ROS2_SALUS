from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="controller_server",
                executable="controller_server_node",
                name="controller_server",
                output="screen",
                parameters=[
                    {
                        "serial_port": "/dev/serial0",
                        "serial_baud": 115200,
                        "serial_tx_hz": 50.0,
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
