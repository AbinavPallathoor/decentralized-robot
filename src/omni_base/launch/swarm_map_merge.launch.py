import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('omni_base')
    params_file = os.path.join(pkg_share, 'config', 'map_merge_params.yaml')

    return LaunchDescription([
        Node(
            package='multirobot_map_merge',
            executable='map_merge',
            name='map_merge',
            output='screen',
            parameters=[params_file]
        )
    ])
