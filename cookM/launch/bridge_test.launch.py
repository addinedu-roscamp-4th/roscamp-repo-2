from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge',
            parameters=['/home/addinedu/pymy_ws/src/mycobot_ros2/mycobot_280/mycobot_280pi/config/bridge_config.yaml'],
            output='screen'
        )
    ])