import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('omni_base')

    # Include the entire mapping and hardware stack
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mapping.launch.py')
        )
    )

    return LaunchDescription([
        mapping_launch,
        Node(
            package='omni_base',
            executable='astar_planner',
            name='astar_planner',
            output='screen'
        ),
        # The new Unified APF/PID replaces the flawed DWA implementation
        Node(
            package='omni_base',
            executable='holonomic_navigator',
            name='holonomic_navigator',
            output='screen'
        ),
        Node(
            package='omni_base',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen'
        )
    ])
