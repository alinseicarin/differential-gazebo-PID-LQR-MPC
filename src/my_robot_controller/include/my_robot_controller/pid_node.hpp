#ifndef PID_NODE_HPP
#define PID_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// NEW: C++ Standard Libraries for files and arrays
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

class PIDController {
private:
    double kp_, ki_, kd_;
    double max_i_; // NEW: The absolute limit for the integral memory
    double integral_;
    double prev_error_;

public:
    // NEW: Constructor now takes a 4th parameter (max_i)
    PIDController(double kp, double ki, double kd, double max_i) 
        : kp_(kp), ki_(ki), kd_(kd), max_i_(max_i), integral_(0.0), prev_error_(0.0) {}

    // The math engine (runs at 30Hz)
    double calculate(double error, double dt) {
        // 1. Proportional
        double p_out = kp_ * error;

        // 2. Integral (Accumulates over time)
        integral_ += error * dt;

        // --- NEW: ANTI-WINDUP CLAMP ---
        // Prevents the memory from exploding if the robot gets physically stuck
        if (integral_ > max_i_) {
            integral_ = max_i_;
        } else if (integral_ < -max_i_) {
            integral_ = -max_i_;
        }
        
        double i_out = ki_ * integral_;

        // 3. Derivative (Rate of change)
        double derivative = (error - prev_error_) / dt;
        double d_out = kd_ * derivative;

        // Save current error for the next loop
        prev_error_ = error;

        // Total Output
        return p_out + i_out + d_out;
    }

    // Reset memory (useful when switching to a new waypoint)
    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }
};

// 2. Your Actual ROS 2 Node "Menu"
class PidNode : public rclcpp::Node { // ii dam clasei PidNode inheritance de la clasa Node standard ros2 (deci e capabila de networking cu publisheri subscriberi si timere)
public:
    PidNode(); // The Constructor promise
    ~PidNode();
private:
    std::ofstream trajectory_csv_;
    void control_loop(); // The Math Loop promise
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // NEW: A function to read the CSV
    void load_waypoints(const std::string& file_path);

    // NEW: Memory storage for the trajectory
    // A vector (list) of X, Y pairs
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_wp_index_; // Which waypoint are we currently targeting?
    
    // Create the two PID objects
    PIDController linear_pid_;
    PIDController angular_pid_;
    
    // ROS 2 plumbing, data pipes
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables to hold current robot state
    double current_x_, current_y_, current_theta_;
};

#endif
