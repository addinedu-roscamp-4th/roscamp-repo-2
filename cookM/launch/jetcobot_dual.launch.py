from launch.substitutions import TextSubstitution
from launch.substitutions import ThisLaunchFileDir

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def get_robot_description(prefix: str):
    xacro_path = PathJoinSubstitution([
        FindPackageShare('cookM'),
        'config',
        'gazebo_jetcobot.urdf.xacro'
    ])
    return ParameterValue(
        Command([
            TextSubstitution(text='xacro '),  # 강제 공백 포함
            xacro_path,
            TextSubstitution(text=f' prefix:={prefix}')
        ]),
        value_type=str
    )

def generate_launch_description():
    robot1_description = get_robot_description('robot1_')
    robot2_description = get_robot_description('robot2_')

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('cookM'),
        'rviz',
        'dual_robot_config.rviz'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot1',
            name='rsp1',
            output='screen',
            parameters=[{'robot_description': robot1_description}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='robot2',
            name='rsp2',
            output='screen',
            parameters=[{'robot_description': robot2_description}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot1_robot1_base_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0.5', '0', '0', '0', '0', 'world', 'robot2_robot2_base_link'],
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        ),
        Node(
            package='cookM',
            executable='gripper_distance_node',
            name='gripper_distance_node',
            output='screen',
            parameters=[
                {'robot1_gripper_frame': 'robot1_jiazhua_Link'},
                {'robot2_gripper_frame': 'robot2_jiazhua_Link'}
            ]
        )
    ])
