from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, RosTimer
from launch.substitutions import Command  # Import Command
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnExecutionComplete

import os

def generate_launch_description():
    
    # Get the directory where the LiDAR launch file is located
    sllidar_launch_dir = os.path.join(get_package_share_directory('sllidar_ros2'), 'launch')

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyS0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    scan_freq = LaunchConfiguration('scan_frequency', default='10.0')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    # Get the directory for the f249 package launch file
    f249_launch_dir = os.path.join(get_package_share_directory('f249'), 'launch')

 
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'channel_type':channel_type,
                     'serial_port': serial_port, 
                     'serial_baudrate': serial_baudrate, 
                     'scan_frequency' : scan_freq,
                     'frame_id': frame_id,
                     'inverted': inverted, 
                     'angle_compensate': angle_compensate, 
                     'scan_mode': scan_mode}],
        output='screen')
    


    #launch the battery monitor
    battery_monitor_node = Node(
            package='battery_monitor',
            executable='battery_monitor_node',
            name='battery_monitor_node',
            output='screen',
        )

    # # Event to launch other_node after sllidar_node starts
    # launch_other_nodes = RegisterEventHandler(
    #     #OnProcessStart(
    #     OnExecutionComplete(
    #         target_action=sllidar_node,
    #         on_completion=[LogInfo(msg="sllidar_node started! "),
    #                   battery_monitor_node]
    #     )
    # )
    
    DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

    DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

    DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
            
    DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

    DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

    DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

    DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),



    return LaunchDescription([

        
        #Launch the f249 driver node
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([f249_launch_dir, '/f249driver_launch.py'])
        # ),
        
        sllidar_node,
        RosTimer(period=5.0, actions=[battery_monitor_node])
        
        
    ])
