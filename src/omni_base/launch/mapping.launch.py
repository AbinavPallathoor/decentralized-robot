import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('omni_base')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hardware.launch.py')
        )
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'map_frame': 'map'},
            {'scan_topic': '/scan'},
            {'resolution': 0.05}, 
            {'max_laser_range': 12.0} 
        ]
    )

    return LaunchDescription([
        hardware_launch,
        slam_node
    ])
