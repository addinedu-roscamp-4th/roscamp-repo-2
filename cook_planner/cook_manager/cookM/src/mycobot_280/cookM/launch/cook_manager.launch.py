from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='cookM',
        #     executable='cook_comp_test',
        #     name='cook_comp_test'
        # ),
        Node(
            package='cookM',
            executable='cook_total_test',
            name='cook_total_test'
        ),
        Node(
            package='cookM',
            executable='tcp_interface',
            name='tcp_interface'
        ),
        Node(
            package='cookM',  
            executable='cookgpt',
            name='cookgpt_service_node'
        )
    ])
