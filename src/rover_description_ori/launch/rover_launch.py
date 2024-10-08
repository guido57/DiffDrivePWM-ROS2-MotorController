from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Launch the LiDAR node
        Node(
            package='sllidar_ros2',
            executable='sllidar_c1_launch.py',
            name='lidar',
            output='screen'
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
        )
    ])
