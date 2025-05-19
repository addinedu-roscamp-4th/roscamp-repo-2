from setuptools import find_packages, setup

package_name = 'alba_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='skzosxm2@gmail.com',
    description='Alba manager package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_status_publisher = alba_manager.robot_status_publisher:main',
            'ros2_interface = alba_manager.ros2_interface:main',
        ],
    },
)
