from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim_node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        # Launch figure8_driver node
        Node(
            package='turtle_control',
            executable='figure8_driver',
            name='figure8_driver',
            parameters=[{
                'pattern_speed': 2.0
            }]
        ),

        #  Launch trace_toggle service node
        Node(
            package='turtle_control',
            executable='trace_toggle',
            name='trace_toggle_service'
        ),
    ])
