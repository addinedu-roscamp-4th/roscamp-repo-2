from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'alba_manager'
domain_bridge_dir = os.path.join('domain_bridge')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [os.path.join(package_name, 'camera_calibration.pkl')]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 도메인 브리지 관련 파일 설치
        (os.path.join('share', package_name, 'domain_bridge'),
            glob(os.path.join(domain_bridge_dir, '*.yaml')) +
            glob(os.path.join(domain_bridge_dir, '*.xml'))),
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
