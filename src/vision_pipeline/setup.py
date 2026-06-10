import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'vision_pipeline'


def regular_files(pattern: str):
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), regular_files('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), regular_files('config/*')),
        (os.path.join('share', package_name, 'web'), regular_files('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@example.com',
    description='Low-latency ROS 2 vision pipeline for realtime robot perception.',
    license='MIT',
    tests_require=['pytest'],
        entry_points={
        'console_scripts': [
            'ip_camera_publisher = vision_pipeline.ip_camera_publisher:main',
            'yolo_onnx_detector = vision_pipeline.yolo_onnx_detector:main',
            'vision_web_server = vision_pipeline.vision_web_server:main',
        ],
    },
)
