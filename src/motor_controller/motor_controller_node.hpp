#ifndef MOTOR_CONTROLLER_NODE_HPP_
#define MOTOR_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <pigpio.h>

class MotorController : public rclcpp::Node
{
public:
    MotorController();
    ~MotorController();

private:
    int pi;
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void set_motor_speed(float left, float right);
    void stop_motors();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    // GPIO pins for the motors
    int motor1_1_pin_;
    int motor1_2_pin_;
    int motor2_1_pin_;
    int motor2_2_pin_;

    // Parameters
    double wheels_distance_;
    double max_speed_;
};

#endif  // MOTOR_CONTROLLER_NODE_HPP_
