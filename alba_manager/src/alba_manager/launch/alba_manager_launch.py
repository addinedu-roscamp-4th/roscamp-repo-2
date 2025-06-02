from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # 도메인 브리지 launch 포함
    domain_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('alba_manager'),
                'domain_bridge',
                'bridge_albabot_domain_lauch.xml'  # XML 파일 경로
            )
        ])
    )
    ld.add_action(domain_bridge_launch)

    # aruco_detector.py 노드 실행
    aruco_detector_node = Node(
        package="alba_manager",
        executable="aruco_detector",
        name="aruco_detector"
    )

    # albabot_pos_publisher.py 노드 실행
    albabot_pos_publisher_node = Node(
        package="alba_manager",
        executable="albabot_pos_publisher",
        name="albabot_pos_publisher"
    )

    # 모든 노드와 launch 파일을 LaunchDescription에 추가
    ld.add_action(aruco_detector_node)
    ld.add_action(albabot_pos_publisher_node)

    return ld
