from collections import defaultdict
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'sensores'


def _wsdl_data_files():
    wsdl_root = Path('wsdl')
    grouped = defaultdict(list)
    if wsdl_root.exists():
        for path in wsdl_root.rglob('*'):
            if not path.is_file():
                continue
            rel_parent = path.parent.relative_to(wsdl_root)
            target = Path('share') / package_name / 'wsdl' / rel_parent
            grouped[str(target)].append(str(path))
    return sorted(grouped.items())

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['pixhawk_dashboard.html']),
        ('share/' + package_name + '/launch', [
            'launch/camara.launch.py',
            'launch/mavros.launch.py',
            'launch/pixhawk.launch.py',
            'launch/rs16.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/mavros_apm_overrides.yaml',
            'config/mavros_sensor_only_pluginlists.yaml',
            'config/rtk_sources.yaml',
            'config/rtk_sources.example.yaml',
            'config/rs16.yaml',
        ]),
    ] + _wsdl_data_files(),
    install_requires=['setuptools', 'websockets', 'requests', 'PyYAML', 'pymavlink>=2.4.43'],
    zip_safe=True,
    description='ROS 2 sensor integration for Pixhawk telemetry, MAVROS and web tools',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixhawk_driver = sensores.pixhawk_driver:main',
            'mavros_compat_bridge = sensores.mavros_compat_bridge:main',
            'rtk_bridge = sensores.rtk_bridge:main',
            'rtk_source_manager = sensores.rtk_source_manager:main',
            'sensores_web = sensores.web_server:main',
            'camara = sensores.camara:main',
        ],
    },
)
