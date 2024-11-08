#include "motor_controller_node.hpp"
#include <pigpiod_if2.h>  // Include the pigpio daemon interface

MotorController::MotorController() : Node("motor_controller_node")
{
    // Declare parameters with default values
    this->declare_parameter<double>("wheels_distance", 0.135); // Example: 20 cm
    this->declare_parameter<double>("max_lin_speed", 0.3); // Example: 1 m/s
    this->declare_parameter<double>("max_ang_speed", 1.0); // Example: 0.1 rad/s
    this->declare_parameter<double>("timeout_duration", 3.0); // 3 seconds timeout by default

    // Connect to the pigpio daemon
    pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpio daemon!");
        throw std::runtime_error("Failed to connect to pigpio daemon");
    }

    // Define the GPIO pins (adjust these to match your wiring)
    motor2_1_pin_ = 19;  // GPIO19
    motor2_2_pin_ = 13;  // GPIO13
    motor1_1_pin_ = 12;  // GPIO12
    motor1_2_pin_ = 16;  // GPIO16

    // Set GPIO modes
    set_mode(pi, motor1_1_pin_, PI_OUTPUT);
    set_mode(pi, motor1_2_pin_, PI_OUTPUT);
    set_mode(pi, motor2_1_pin_, PI_OUTPUT);
    set_mode(pi, motor2_2_pin_, PI_OUTPUT);

    // Set PWM frequency for both motor pins (e.g., 1000 Hz)
    set_PWM_frequency(pi, motor1_1_pin_, 100000);  // Set 100000 Hz frequency on motor1_pwm_pin_
    set_PWM_frequency(pi, motor1_2_pin_, 100000);  // Set 100000 Hz frequency on motor1_pwm_pin_
    set_PWM_frequency(pi, motor2_1_pin_, 100000);  // Set 100000 Hz frequency on motor1_pwm_pin_
    set_PWM_frequency(pi, motor2_2_pin_, 100000);  // Set 100000 Hz frequency on motor1_pwm_pin_
    

    // Stop motors initially
    stop_motors();

    // Test
    // set_motor_speed(0.8, 0.8); 

    // Subscribe to the /cmd_vel topic
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MotorController::cmd_vel_callback, this, std::placeholders::_1));

    // Create a timer to stop the motors if no cmd_vel is received within the timeout duration
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timeout_duration_),
        std::bind(&MotorController::timeout_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Motor Controller Node has been started.");
}

MotorController::~MotorController()
{
    stop_motors();
    RCLCPP_INFO(this->get_logger(), "Motors stopped.");
    pigpio_stop(pi);  // Disconnect from the daemon
}

void MotorController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Reset the timeout timer whenever a new command is received
    timer_->reset();
    
    RCLCPP_INFO(this->get_logger(), "cmd_vel  x: %.2f, z: %.2f", msg->linear.x, msg->angular.z);

    // Get parameter values
    this->get_parameter("wheels_distance", wheels_distance_);
    this->get_parameter("max_lin_speed", max_lin_speed_);
    this->get_parameter("max_ang_speed", max_ang_speed_);
    this->get_parameter("timeout_duration", timeout_duration_);

    // Extract linear and angular velocity and clamp it
    double linear_x  = std::max(std::min(msg->linear.x,  max_lin_speed_), -max_lin_speed_);
    double angular_z = std::max(std::min(msg->angular.z, max_ang_speed_), - max_ang_speed_);

    // -max_lin_speed_ <=  linear.x <= max_lin_speed_
    // -max_ang_speed_ <= angular.z <= max_ang_speed_

    // Calculate individual motor speeds for differential drive
    double left_speed  = linear_x + (angular_z * wheels_distance_  / 2.0);
    double right_speed = linear_x - (angular_z * wheels_distance_  / 2.0);

    // - (max_lin_speed_ + max_ang_speed * wheels_distance_ / 2.0) <= left_speed  <= max_lin_speed_ + max_ang_speed * wheels_distance_ / 2.0
    // - (max_lin_speed_ + max_ang_speed * wheels_distance_ / 2.0) <= right_speed <= max_lin_speed_ + max_ang_speed * wheels_distance_ / 2.0
    
    // e.g. wheels_distance_=0.135m max_lin_speed=0.3m/sec max_ang_speed=1rad/sec
    // -0.3675 <= left_speed   <= 0.3675 m/sec
    // -0.3675 <= right_speed  <= 0.3675 m/sec
    
    // e.g.2 wheels_distance_=0.135m max_lin_speed=0.0m/sec max_ang_speed=1rad/sec
    // -0.0675 <= left_speed   <= 0.0675 m/sec
    // -0.0675 <= right_speed  <= 0.0675 m/sec
    
    // Clamp speeds to [-max_speed, max_speed]
    //left_speed = std::max(std::min(left_speed, max_speed_), -max_speed_);
    //right_speed = std::max(std::min(right_speed, max_speed_), -max_speed_);

    double max_speed = max_lin_speed_ + max_ang_speed_ *  wheels_distance_ / 2.0;
    
    // convert to the real motors capabilities
    double conv_ratio = 1.39;
    left_speed = conv_ratio * left_speed; 
    right_speed = conv_ratio * right_speed; 
    
    // Convert [-max_speed, max_speed] speed to PWM duty cycle [0, 255]
    int left_pwm  = 255 - static_cast<int>(std::abs(left_speed)  * 255.0 / (max_speed) );
    int right_pwm = 255 - static_cast<int>(std::abs(right_speed) * 255.0 / (max_speed) );
    left_pwm = std::min(255,std::max(0,left_pwm));
    right_pwm = std::min(255,std::max(0,right_pwm));


    RCLCPP_INFO(this->get_logger(), "Set left PWM: %d, right PWM: %d", left_pwm, right_pwm);

    // Set direction and apply PWM for the left motor
    if (left_speed >= 0) {
        gpio_write(pi, motor1_2_pin_, 1);  // Forward direction
        set_PWM_dutycycle(pi, motor1_1_pin_, left_pwm);
    } else {
        gpio_write(pi, motor1_1_pin_, 1);  // Reverse direction
        set_PWM_dutycycle(pi, motor1_2_pin_, left_pwm);  
    }

    // Set direction and apply PWM for the right motor
    if (right_speed >= 0) {
        gpio_write(pi, motor2_2_pin_, 1);  // Forward direction
        set_PWM_dutycycle(pi, motor2_1_pin_, right_pwm);
    } else {
        gpio_write(pi, motor2_1_pin_, 1);  // Reverse direction
        set_PWM_dutycycle(pi, motor2_2_pin_, right_pwm);  
    }


    // Create a timer to stop the motors if no cmd_vel is received within the timeout duration
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timeout_duration_),
        std::bind(&MotorController::timeout_callback, this));

}


void MotorController::stop_motors()
{
    set_PWM_dutycycle(pi, motor1_1_pin_, 0);
    set_PWM_dutycycle(pi, motor1_2_pin_, 0);
    set_PWM_dutycycle(pi, motor2_1_pin_, 0);
    set_PWM_dutycycle(pi, motor2_2_pin_, 0);
    
    //RCLCPP_INFO(this->get_logger(), "Motors stopped.");
}

void MotorController::timeout_callback()
{
    //RCLCPP_WARN(this->get_logger(), "Timeout! No cmd_vel received. Stopping motors.");
    stop_motors();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
