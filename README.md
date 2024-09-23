# DiffDrivePWM-ROS2-MotorController
A ROS2 package to drive 2 DC motors with PWM generated by Raspberry PI 4

# Prepare the Raspberry PI 4

## Install Ubuntu 22.04

ROS2 Humble requires Ubuntu 22.04

On your development computer, use Raspberry imager to create an SD card with 

![](docs/ubuntu.png)

## Install ROS2 Humble on the Raspberry PI 4

### Follow these instructions:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

When you arrive at "Install ROS 2 packages" install only:

`sudo apt install ros-humble-ros-base`

and stop there.
