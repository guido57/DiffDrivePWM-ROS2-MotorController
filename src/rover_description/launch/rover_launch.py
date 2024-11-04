from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, RosTimer
from launch.substitutions import Command  # Import Command
import os

def generate_launch_description():

    # Get the directory where the URDF file is located
    urdf_file = os.path.join(get_package_share_directory('rover_description'), 'urdf', 'robot_urdf.xacro')

    # Get the directory for the f249 package launch file
    f249_launch_dir = os.path.join(get_package_share_directory('f249'), 'launch')
    
    sllidar_node = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyS0', 
                         'serial_baudrate': 460800, 
                         'scan_frequency' : 10.0,
                         'frame_id': 'laser_frame',
                         'inverted': False, 
                         'angle_compensate': True, 
                         'scan_mode': 'Standard'}],
            output='screen')

    battery_monitor_node = Node(
            package='battery_monitor',
            executable='battery_monitor_node',
            name='battery_monitor_node',
            output='screen',
        )

    bno085_node = Node(
            package='bno085',
            executable='bno085_publisher',
            name='bno085',
            output='screen',
            parameters=[{'frame_id': 'imu_link'}],  # Example parameters
        )
 
    # Launch the motor controller node
    motor_controller_node = Node(
            package='motor_controller',
            executable='motor_controller_node',
            name='motor_controller',
            output='screen',
        )

    #  # Launch the robot state publisher
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
        )

    # launch the wheel encoders 
    f249_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([f249_launch_dir, '/f249driver_launch.py']),
                )

    # Launch the odometry
    odometry_estimator_node = Node(
            package='odometry_estimator',
            executable='odometry_estimator',
            name='odometry_estimator',
            output='screen',
        )

    return LaunchDescription([
        
        sllidar_node,
        RosTimer(
            period=5.0, 
            actions=[
                battery_monitor_node,
                #bno085_node,
                motor_controller_node,
                robot_state_publisher_node,
                f249_launch,
                odometry_estimator_node        
            ])
    ])
