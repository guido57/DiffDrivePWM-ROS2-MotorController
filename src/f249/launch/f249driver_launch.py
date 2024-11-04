from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('f249')
    parameter_right_file_path = os.path.join(share_dir, 'params', 'f249_right.yaml')
    parameter_left_file_path = os.path.join(share_dir, 'params', 'f249_left.yaml')
    
    
    f249driver_node_right = Node(
        package='f249',
        executable='f249_node',
        name='f249driver_node',
        output="screen",
        emulate_tty=True,
        parameters=[parameter_right_file_path],
         remappings=[
            ('/f249', '/right_wheel_rpm')]
    )

    f249driver_node_left = Node(
        package='f249',
        executable='f249_node',
        name='f249driver_node',
        output="screen",
        emulate_tty=True,
        parameters=[parameter_left_file_path],
         remappings=[
            ('/f249', '/left_wheel_rpm')]
    )

    ld.add_action(f249driver_node_right)
    ld.add_action(f249driver_node_left)

    return ld