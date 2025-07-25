from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sindeed_alam_autonomous',
            executable='planner_node',
            output='screen'
        ),
    ])
