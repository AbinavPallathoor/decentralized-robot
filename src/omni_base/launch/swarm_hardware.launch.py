from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('omni_base')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'omni_robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Capture the robot_name argument passed from the terminal
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='rancho', description='Name of the robot'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            parameters=[{
                'robot_description': robot_desc,
                # TF Fix: Prefixes all URDF links with the robot's name (e.g., 'rancho/base_link')
                'frame_prefix': [robot_name, '/'] 
            }]
        ),
        Node(
            package='omni_base',
            executable='serial_bridge',
            name='serial_bridge',
            namespace=robot_name
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=robot_name,
            parameters=[ekf_config, {
                # TF Fix: Force the EKF to use the namespaced frames
                'map_frame': [robot_name, '/map'],
                'odom_frame': [robot_name, '/odom'],
                'base_link_frame': [robot_name, '/base_link'],
                'world_frame': [robot_name, '/odom']
            }]
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            namespace=robot_name
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            namespace=robot_name,
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                # TF Fix: Assign the Lidar to the namespaced laser frame
                'frame_id': [robot_name, '/laser'],
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
        )
    ])
