from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        Node(
            package='turtle_control',
            executable='figure8_driver',
            name='figure8_driver',
            output='screen'
        ),
        Node(
            package='turtle_control',
            executable='trace_toggle_service',
            name='trace_toggle_service',
            output='screen'
        )
    ])

