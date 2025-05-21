# 📁 cookmanager/launch/pose_broadcaster.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cookmanager',
            executable='unit_test',
            name='unit_test',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='cookmanager',
            executable='tcp_interface',
            name='tcp_interface',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='cookmanager',
            executable='cookgpt',
            name='cookgpt_service_node',
            output='screen',
            emulate_tty=True
        ),
        # Node(
        #     package='cookmanager',
        #     executable='pose_broadcaster',
        #     name='pose_broadcaster',
        #     output='screen',
        #     emulate_tty=True
        # )
    ])
