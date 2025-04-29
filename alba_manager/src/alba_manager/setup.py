from setuptools import find_packages, setup
import os

package_name = 'alba_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [os.path.join(package_name, 'camera_calibration.pkl')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='shimky85@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = alba_manager.aruco_detector:main',
            'albabot_pos_publisher = alba_manager.albabot_pos_publisher:main',
        ],
    },
)
