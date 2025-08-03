from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aro_manager',
            executable='manager_node',
            name='aro_manager',
            output='screen'
        )
    ])
