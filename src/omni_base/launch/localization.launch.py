import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('omni_base')
    
    # This expects the map to be saved in the pi user's home directory
    map_path = '/home/pi/my_map.yaml' 

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hardware.launch.py')
        )
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_path}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[{
            'use_sim_time': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'scan_topic': '/scan'
        }]
    )

    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        hardware_launch,
        map_server_node,
        amcl_node,
        lifecycle_node
    ])
