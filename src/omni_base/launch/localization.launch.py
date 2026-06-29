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
            'scan_topic': '/scan',
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.yaw': 0.0,
            
            # --- THE AMCL FIX FOR OMNI-WHEELS ---
            'robot_model_type': 'nav2_amcl::OmniMotionModel',
            
            # Alpha values represent expected odometry noise.
            # Higher = "My wheels slip a lot, trust the Lidar more."
            'alpha1': 0.2, # Rotation noise from rotation
            'alpha2': 0.2, # Rotation noise from translation
            'alpha3': 0.2, # Translation noise from translation
            'alpha4': 0.2, # Translation noise from rotation
            'alpha5': 0.2, # Translation noise from strafing (omni specific)
            
            # Force AMCL to update more frequently to catch slips faster
            'update_min_d': 0.1, # Update position every 10cm driven (default is 0.25m)
            'update_min_a': 0.2  # Update position every ~11 degrees turned (default is 0.2 rad)
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
