from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 도메인 및 네임스페이스 설정을 위한 런치 인자
    domain_arg = DeclareLaunchArgument(
        'domains',
        default_value='[10, 20]',
        description='도메인 ID 패턴 목록'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespaces',
        default_value='["jetcobot_1", "jetcobot_2", "pinky_1", "pinky_2", "pinky_3"]',
        description='네임스페이스 패턴 목록'
    )
    
    return LaunchDescription([
        domain_arg,
        namespace_arg,
        Node(
            package='robot_debugger_ui',
            executable='robot_debugger_ui_node',
            name='robot_debugger_ui',
            output='screen',
            parameters=[{
                'domains': LaunchConfiguration('domains'),
                'namespaces': LaunchConfiguration('namespaces'),
                'log_level': 'info',
                'qos_reliability': 'reliable',
                'history_depth': 10
            }]
        )
    ]) 