from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ina219',  # Name of your package
            executable='my_ina219',  # Python script node without the .py extension
            name='battery_monitor',  # Node name
            output='screen',  # Output to console
            parameters=[{
            }]
        )
    ])