#ifndef MOTOR_CONTROLLER_NODE_HPP
#define MOTOR_CONTROLLER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <pigpiod_if2.h>


class MotorController : public rclcpp::Node {
public:
    MotorController();
    ~MotorController();
    
    // Encoder callbacks
    static void left_encoder_callback(int pi, unsigned int gpio, unsigned int level, uint32_t tick);
    static void right_encoder_callback(int pi, unsigned int gpio, unsigned int level, uint32_t tick);

private:
    // Declare the static instance pointer here
    static MotorController* instance_;

    // Helper functions
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void calculate_odometry();
    void update_motor_speeds(int left_pwm, int right_pwm);
    void stop_motors();
    void timeout_callback();
    void pid_timer_callback();
    void apply_pid_control(double actual_speed, double target_speed, int &current_pwm, double &last_error, double &integral);
    void calc_left_wheel_speed();
    void calc_right_wheel_speed();

    // ROS2 elements
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr pid_timer;

    // Parameters
    double wheels_distance_;
    double max_lin_speed_;
    double max_ang_speed_;
    double timeout_duration_;
    double pid_timer_interval;
    double conv_ratio_;  // Conversion ratio for PWM
    double max_speed_;   // Maximum motor speed in PWM-equivalent scale
    double left_cmd_vel_;
    double right_cmd_vel_;
    double left_target_speed;
    double right_target_speed;
    double last_left_time_seconds;
    double last_right_time_seconds;

    // Pigpio
    int gpio_handle_;
    int motor1_1_pin_, motor1_2_pin_, motor2_1_pin_, motor2_2_pin_;

    // Encoder variables
    static int left_ticks_;
    static int right_ticks_;
    int last_left_ticks_;
    int last_right_ticks_;
    static double left_wheel_speed_;
    static double right_wheel_speed_;

    // State variables
    rclcpp::Time last_time_;
    double x_pos_, y_pos_, theta_;  // Robot pose

    int    left_pwm_, right_pwm_;
    double left_last_error_, right_last_error_;
    double left_integral_, right_integral_;
    double kp_, ki_, kd_;  // PID constants
};

#endif  // MOTOR_CONTROLLER_NODE_HPP
