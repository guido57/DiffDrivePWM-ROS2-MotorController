from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import Command  # Import Command
import os

def generate_launch_description():

    # Get the directory where the LiDAR launch file is located
    sllidar_launch_dir = os.path.join(get_package_share_directory('sllidar_ros2'), 'launch')

    # Get the directory where the URDF file is located
    urdf_file = os.path.join(get_package_share_directory('rover_description'), 'urdf', 'robot_urdf.xacro')

    # Get the configuration file path for slam_toolbox
    slam_params_path = os.path.join(get_package_share_directory('rover_description'), 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([

        # Launch the LiDAR node by including its launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([sllidar_launch_dir, '/sllidar_c1_launch.py']),
        ),


       # Launch the SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_path],
            remappings=[
                ('scan', '/scan'),  # Ensure this matches your LIDAR topic
                ('imu', '/bno055/imu'),  # Ensure this matches your IMU topic
            ],
        )

    ])
