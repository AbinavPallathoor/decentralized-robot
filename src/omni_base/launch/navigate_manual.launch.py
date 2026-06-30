from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omni_base',
            executable='astar_planner',
            name='astar_planner',
            output='screen'
        ),
        Node(
            package='omni_base',
            executable='holonomic_pid',
            name='holonomic_pid',
            output='screen'
        )
    ])
