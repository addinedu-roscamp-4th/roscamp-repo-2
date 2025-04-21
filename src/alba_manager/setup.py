from setuptools import setup
import os
from glob import glob

package_name = 'alba_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='Alba Planner ROS2 ↔ TCP bridge for RoboDine',
    license='Apache-2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/alba_manager']),
        ('share/alba_manager', ['package.xml']),
        (os.path.join('share', package_name), glob('resource/*')),
    ],
    entry_points={
        'console_scripts': [
            'ros2_interface = alba_manager.ros2_interface:main',
            'test_ros2_publisher = alba_manager.test_ros2_publisher:main',
        ],
    },
)
