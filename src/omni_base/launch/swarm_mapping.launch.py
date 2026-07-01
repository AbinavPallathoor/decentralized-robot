import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('omni_base')
    robot_name = LaunchConfiguration('robot_name')

    # Boot up the hardware, passing the robot_name down the chain
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'swarm_hardware.launch.py')
        ),
        launch_arguments={'robot_name': robot_name}.items()
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_name,
        output='screen',
        parameters=[
            {'use_sim_time': False},
            
            # TF Fix: Bind SLAM to the namespaced frames
            {'odom_frame': [robot_name, '/odom']},
            {'base_frame': [robot_name, '/base_link']},
            {'map_frame': [robot_name, '/map']},
            
            # The node namespace automatically prefixes this to /rancho/scan
            {'scan_topic': 'scan'}, 
            
            {'resolution': 0.05}, 
            {'max_laser_range': 12.0} 
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='rancho', description='Name of the robot'),
        hardware_launch,
        slam_node
    ])
