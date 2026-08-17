from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navegacion_gps'


def regular_files(pattern: str):
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), regular_files('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), regular_files('config/*')),
        (os.path.join('share', package_name, 'models'), regular_files('models/*')),
        (os.path.join('share', package_name, 'worlds'), regular_files('worlds/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps'),
         regular_files('models/turtlebot_waffle_gps/*')),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='Demo package for following GPS waypoints with nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_odometry = navegacion_gps.ackermann_odometry:main',
            'cmd_vel_ackermann_bridge_v2 = navegacion_gps.cmd_vel_ackermann_bridge_v2:main',
            'compass_calibration_recorder = navegacion_gps.compass_calibration_recorder:main',
            'compass_heading_gate = navegacion_gps.compass_heading_gate:main',
            'coverage_waypoint_mission = navegacion_gps.coverage_waypoint_mission:main',
            # LEGACY: dynamic datum setter. Current global profiles use fixed
            # site datum from launch/configuration.
            'datum_setter = navegacion_gps.datum_setter:main',
            'gazebo_utils = navegacion_gps.gazebo_utils:main',
            'goal_pose_to_follow_path_v2 = navegacion_gps.goal_pose_to_follow_path_v2:main',
            'global_odom_stationary_gate = navegacion_gps.global_odom_stationary_gate:main',
            'global_imu_stationary_gate = navegacion_gps.global_imu_stationary_gate:main',
            'global_yaw_stationary_hold = navegacion_gps.global_yaw_stationary_hold:main',
            'lidar_obstacle_filter = navegacion_gps.lidar_obstacle_filter:main',
            'loop_waypoint_benchmark = navegacion_gps.loop_waypoint_benchmark:main',
            'map_gps_absolute_measurement = navegacion_gps.map_gps_absolute_measurement:main',
            'zones_manager = navegacion_gps.zones_manager:main',
            'nav_command_server = navegacion_gps.nav_command_server:main',
            'nav_benchmark_report = navegacion_gps.nav_benchmark_report:main',
            'nav_benchmark_runner = navegacion_gps.nav_benchmark_runner:main',
            'nav_snapshot_server = navegacion_gps.nav_snapshot_server:main',
            'nav_trace_recorder = navegacion_gps.nav_trace_recorder:main',
            'nav_trace_report = navegacion_gps.nav_trace_report:main',
            'nav_observability = navegacion_gps.nav_observability:main',
            'path_clearance_validator = navegacion_gps.path_clearance_validator:main',
            'polygon_stamped_republisher = navegacion_gps.polygon_stamped_republisher:main',
            'replay_localization_compare = navegacion_gps.replay_localization_compare:main',
            'route_executor = navegacion_gps.route_executor:main',
            'scan_ground_filter = navegacion_gps.scan_ground_filter:main',
            'scan_ground_validation = navegacion_gps.scan_ground_validation:main',
            'scan_noise_filter = navegacion_gps.scan_noise_filter:main',
            'scan_wifi_debug = navegacion_gps.scan_wifi_debug:main',
            'sim_battery_publisher = navegacion_gps.sim_battery_publisher:main',
            'sim_drive_telemetry = navegacion_gps.sim_drive_telemetry:main',
            'sim_compass_hdg = navegacion_gps.sim_compass_hdg:main',
            'sim_sensor_normalizer_v2 = navegacion_gps.sim_sensor_normalizer_v2:main',
            'sim_localization_benchmark = navegacion_gps.sim_localization_benchmark:main',
            'sim_global_straight_benchmark = navegacion_gps.sim_global_straight_benchmark:main',
            'startup_heading_diagnosis = navegacion_gps.startup_heading_diagnosis:main',
            'gps_course_heading = navegacion_gps.gps_course_heading:main',
        ],
    },
)
