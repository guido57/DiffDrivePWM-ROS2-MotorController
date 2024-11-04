# cartographer_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ],
            remappings=[
                ('/scan', '/scan'),  # Adjust to your scan topic
                ('/imu', '/bno055/imu'),
            ],
            arguments=['-configuration_directory', 'install/rover_description/share/rover_description/config',
                       '-configuration_basename', 'cartographer_config.lua']
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
         # Launch Nav2 (Navigation Stack)
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2_bringup',
            output='screen',
            parameters=["install/rover_description/share/rover_description/config/nav2_params.yaml"],
        ),
    ])
