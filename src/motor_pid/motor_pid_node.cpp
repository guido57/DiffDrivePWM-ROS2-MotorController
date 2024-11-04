#include "motor_pid_node.hpp"
#include <chrono>
#include <cmath>

// Define the static instance_ pointer
MotorController* MotorController::instance_ = nullptr;

int MotorController::left_ticks_ = 0;
int MotorController::right_ticks_ = 0;
double MotorController::left_wheel_speed_ = 0.0;
double MotorController::right_wheel_speed_ = 0.0;

MotorController::MotorController() : Node("motor_controller_node"), x_pos_(0.0), y_pos_(0.0), theta_(0.0)
{
 
     // Set instance pointer
    instance_ = this;
 
    // Declare and get parameters
    this->declare_parameter<double>("wheels_distance", 0.135);
    this->declare_parameter<double>("max_lin_speed", 0.3);
    this->declare_parameter<double>("max_ang_speed", 1.0);
    this->declare_parameter<double>("timeout_duration", 3.0);
    this->declare_parameter<double>("pid_timer_interval", 0.5);
    this->declare_parameter<double>("conv_ratio", 0.011);
    // Declare PID parameters
    this->declare_parameter<double>("kp", 0.4);
    this->declare_parameter<double>("ki", 0.2);
    this->declare_parameter<double>("kd", 0.05);

    // Retrieve parameters
    this->get_parameter("wheels_distance", wheels_distance_);
    this->get_parameter("max_lin_speed", max_lin_speed_);
    this->get_parameter("max_ang_speed", max_ang_speed_);
    this->get_parameter("timeout_duration", timeout_duration_);
    this->get_parameter("pid_timer_interval", pid_timer_interval);
    this->get_parameter("conv_ratio", conv_ratio_);

    // Retrieve PID parameters
    this->get_parameter("kp", kp_);
    this->get_parameter("ki", ki_);
    this->get_parameter("kd", kd_);

    // Set max_speed based on max_lin_speed to scale PWM appropriately
    max_speed_ = max_lin_speed_;

    // Pigpio initialization
    gpio_handle_ = pigpio_start(nullptr, nullptr);
    if (gpio_handle_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpio daemon!");
        throw std::runtime_error("Failed to connect to pigpio daemon");
    }

    // Motor pin configuration
    motor1_1_pin_ = 12;
    motor1_2_pin_ = 16;
    motor2_1_pin_ = 19;
    motor2_2_pin_ = 13;
    set_mode(gpio_handle_, motor1_1_pin_, PI_OUTPUT);
    set_mode(gpio_handle_, motor1_2_pin_, PI_OUTPUT);
    set_mode(gpio_handle_, motor2_1_pin_, PI_OUTPUT);
    set_mode(gpio_handle_, motor2_2_pin_, PI_OUTPUT);

    // Set encoder GPIO callbacks
    set_mode(gpio_handle_, 18, PI_INPUT);
    set_pull_up_down(gpio_handle_,18, PI_PUD_OFF);
    set_mode(gpio_handle_, 21, PI_INPUT);
    set_pull_up_down(gpio_handle_,21, PI_PUD_OFF);
    callback(gpio_handle_, 18, RISING_EDGE, MotorController::right_encoder_callback);
    callback(gpio_handle_, 21, RISING_EDGE, MotorController::left_encoder_callback);

    // set initial values
    last_left_time_seconds  = 0.0;
    last_right_time_seconds = 0.0;
    last_left_ticks_ = 0;
    last_right_ticks_ = 0;

    // Subscribe to ros2 /cmd_vel topic
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MotorController::cmd_vel_callback, this, std::placeholders::_1));

    // Create publisher to ros2 /odom topic
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Create the ros2 timer for motor timeout
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timeout_duration_),
        std::bind(&MotorController::timeout_callback, this)
    );

    // Create the ros2 timer for pid motor control
    pid_timer = this->create_wall_timer(
        std::chrono::duration<double>(pid_timer_interval),
        std::bind(&MotorController::pid_timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Motor Controller Node has been started.");
}

MotorController::~MotorController() {
    stop_motors();
    pigpio_stop(gpio_handle_);
}

void MotorController::calc_left_wheel_speed(){

    double current_time_seconds = instance_->now().seconds();
    double time_diff = current_time_seconds - instance_->last_left_time_seconds;

    // RCLCPP_INFO(this->get_logger(), "calc_left_wheel_speed: time_diff:%f", time_diff);
    // RCLCPP_INFO(this->get_logger(), "calc_left_wheel_speed: left_ticks:%d last_left_ticks:%d",
    //                         left_ticks_, last_left_ticks_);

    // calculate left_wheel_speed_ which will be used for speed error estimation
    if (time_diff > 0) {
        left_wheel_speed_ = left_ticks_ - last_left_ticks_;
        left_wheel_speed_ = left_wheel_speed_ * instance_->conv_ratio_ / time_diff;
    }
    instance_->last_left_time_seconds = current_time_seconds;
    last_left_ticks_ = left_ticks_;
}

void MotorController::calc_right_wheel_speed(){

    double current_time_seconds = instance_->now().seconds();
    double time_diff = current_time_seconds - instance_->last_right_time_seconds;

    // RCLCPP_INFO(this->get_logger(), "calc_right_wheel_speed: current_time_seconds:%f last_right_time_seconds:%f",
    //                         current_time_seconds, instance_->last_right_time_seconds);
    // RCLCPP_INFO(this->get_logger(), "calc_right_wheel_speed: right_ticks:%d right_left_ticks:%d",
    //                         right_ticks_, last_right_ticks_);


    // calculate left_wheel_speed_ which will be used for speed error estimation
    if (time_diff > 0) {
        right_wheel_speed_ = right_ticks_ - last_right_ticks_;
        right_wheel_speed_ = right_wheel_speed_ * instance_->conv_ratio_ / time_diff;
        // RCLCPP_INFO(this->get_logger(), "calc_right_wheel_speed: right_wheel_speed=%f",
        //                 right_wheel_speed_);
        // Adjust direction based on cmd_vel
        // if (instance_->right_cmd_vel_ < 0) {
        //     right_wheel_speed_ = -right_wheel_speed_;
        // }
    }
    instance_->last_right_time_seconds = current_time_seconds;
    last_right_ticks_ = right_ticks_;
}

void MotorController::left_encoder_callback(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
    
    left_ticks_++;
}

void MotorController::right_encoder_callback(int pi, unsigned int gpio, unsigned int level, uint32_t tick) {
        
    right_ticks_++;
}

void MotorController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    timer_->reset();
    
    double linear_x = std::clamp(msg->linear.x, -max_lin_speed_, max_lin_speed_);
    double angular_z = std::clamp(msg->angular.z, -max_ang_speed_, max_ang_speed_);

    left_target_speed = linear_x + angular_z * wheels_distance_ / 2.0;
    right_target_speed = linear_x - angular_z * wheels_distance_ / 2.0;
}

void MotorController::apply_pid_control(double actual_speed, double target_speed, int &current_pwm, double &last_error, double &integral) {
    
    // Retrieve PID parameters
    this->get_parameter("kp", kp_);
    this->get_parameter("ki", ki_);
    this->get_parameter("kd", kd_);
    this->get_parameter("conv_ratio", conv_ratio_);
    this->get_parameter("max_lin_speed", max_lin_speed_);
        
    double error;
    
    error = target_speed >= 0 ? target_speed - actual_speed : target_speed + actual_speed;

    integral += error;
    double derivative = error - last_error;

    double current_pwm_double = kp_ * error + ki_ * integral + kd_ * derivative;
    current_pwm = (255.0 * current_pwm_double) / max_lin_speed_;
    
    if(target_speed >=0 && current_pwm < 0)
        current_pwm = 0;
    if(target_speed <=0 && current_pwm > 0)
        current_pwm = 0;

    // introduce a deadband, linearize and clamp
    if(current_pwm > 255)    
        current_pwm = 255;
    else if(current_pwm > 5)
        current_pwm = 30 + ((255-30)*current_pwm)/255;
    else if(current_pwm >= -5)
         current_pwm = 0;
    else if(current_pwm >= -255)
        current_pwm = -30 + ((255-30)*current_pwm)/255;
    else
        current_pwm = -255;     
    
    last_error = error;

    // reset integral in case of saturation
    if (current_pwm == 255 || current_pwm == -255)
        integral = 0;
    // reset integral in case of target_speed is 0
    if (target_speed == 0)
        integral = 0;
}

void MotorController::update_motor_speeds(int left_pwm, int right_pwm) {

    // Set direction and apply PWM for the left motor
    if (left_pwm > 0) {
        gpio_write(gpio_handle_, motor1_2_pin_, 1);  // Forward direction
        set_PWM_dutycycle(gpio_handle_, motor1_1_pin_, 255 - left_pwm);
    } else {
        gpio_write(gpio_handle_, motor1_1_pin_, 1);  // Reverse direction
        set_PWM_dutycycle(gpio_handle_, motor1_2_pin_, 255 + left_pwm);  
    }

    // Set direction and apply PWM for the right motor
    if (right_pwm > 0) {
        gpio_write(gpio_handle_, motor2_2_pin_, 1);  // Forward direction
        set_PWM_dutycycle(gpio_handle_, motor2_1_pin_, 255 - right_pwm);
    } else {
        gpio_write(gpio_handle_, motor2_1_pin_, 1);  // Reverse direction
        set_PWM_dutycycle(gpio_handle_, motor2_2_pin_, 255 + right_pwm);  
    }
}

void MotorController::calculate_odometry() {
    double dist_per_tick = 0.011;  // Example distance per tick
    double delta_left = left_ticks_ * dist_per_tick;
    double delta_right = right_ticks_ * dist_per_tick;

    double delta_dist = (delta_right + delta_left) / 2.0;
    double delta_theta = (delta_right - delta_left) / wheels_distance_;

    x_pos_ += delta_dist * cos(theta_);
    y_pos_ += delta_dist * sin(theta_);
    theta_ += delta_theta;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    odom_msg.pose.pose.position.x = x_pos_;
    odom_msg.pose.pose.position.y = y_pos_;
    odom_publisher_->publish(odom_msg);

    left_ticks_ = 0;
    last_left_ticks_ = 0;
    right_ticks_ = 0;
    last_right_ticks_ = 0;
}

void MotorController::stop_motors() {
    set_PWM_dutycycle(gpio_handle_, motor1_1_pin_, 0);
    set_PWM_dutycycle(gpio_handle_, motor1_2_pin_, 0);
    set_PWM_dutycycle(gpio_handle_, motor2_1_pin_, 0);
    set_PWM_dutycycle(gpio_handle_, motor2_2_pin_, 0);
}

void MotorController::timeout_callback() {
    //RCLCPP_INFO(this->get_logger(), "timeout: stop motors!");
    left_target_speed = 0.0;
    right_target_speed = 0.0;
    stop_motors();
}

void MotorController::pid_timer_callback() {
    
    //RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------");
    
    calc_left_wheel_speed();
    calc_right_wheel_speed();
    
    apply_pid_control(left_wheel_speed_, left_target_speed , left_pwm_, left_last_error_, left_integral_);
    apply_pid_control(right_wheel_speed_, right_target_speed, right_pwm_, right_last_error_, right_integral_);

    // RCLCPP_INFO(this->get_logger(), "left_target_speed_=%f right_target_speed_=%f", left_target_speed,right_target_speed);
    // RCLCPP_INFO(this->get_logger(), "left_wheel_speed_=%f right_wheel_speed_=%f", left_wheel_speed_,right_wheel_speed_);
    // RCLCPP_INFO(this->get_logger(), "left_last_error_=%f right_last_error_=%f", left_last_error_,right_last_error_);
    // RCLCPP_INFO(this->get_logger(), "left_integral_=%f right_integral_=%f", left_integral_,right_integral_);
    // RCLCPP_INFO(this->get_logger(), "left_pwm=%d right_pwm=%d", left_pwm_,right_pwm_);

    update_motor_speeds(left_pwm_, right_pwm_);

    // calculate and publish odometry
    calculate_odometry();
    
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
