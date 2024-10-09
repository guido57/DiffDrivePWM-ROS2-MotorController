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

        # launch the battery monitor
        Node(
            package='battery_monitor',
            executable='battery_monitor_node',
            name='battery_monitor_node',
            output='screen',
        ),

        # Launch the BNO055 IMU node with the params file
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            output='screen',
            parameters=['./src/bno055/bno055/params/bno055_params_i2c.yaml'],
        ),

        # Launch the motor controller node
        Node(
            package='motor_controller',
            executable='motor_controller_node',
            name='motor_controller',
            output='screen',
        ),

         # Launch the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
        )
    ])
