#include "my_robot_controller/pid_node.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>

// Forces any angle to stay strictly between -PI and +PI
double wrap_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}


// --- 1. THE FILE READER FUNCTION ---
void PidNode::load_waypoints(const std::string& file_path) {
    std::ifstream file(file_path);
    std::string line;
    
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "CRITICAL: Could not open CSV file!");
        return;
    }

    // Read file line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        
        // Split at the comma
        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
            // Convert string to double and save to our vector
            double x = std::stod(x_str);
            double y = std::stod(y_str);
            waypoints_.push_back({x, y});
        }
    }
    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints.", waypoints_.size());
}   

// --- 2. THE CONSTRUCTOR ---
PidNode::PidNode() : Node("pid_node"), 
    // linear_pid_: Kp=0.8, Ki=0.0, Kd=0.2, Max_I=0.0 (Unused)
    linear_pid_(1, 0.1, 0.2, 1),  
    
    // angular_pid_: Kp=0.5, Ki=0.0, Kd=0.1, Max_I=10.0 (The Robust Steering)
    angular_pid_(1.5, 0.0, 0.3, 1.0)
{
    // Declare a parameter so you can pass the file path from the terminal
    this->declare_parameter<std::string>("csv_path", "/home/ws/track_1.csv");
    std::string path = this->get_parameter("csv_path").as_string();
    
    // Load the file into memory before starting the timers
    load_waypoints(path);

    trajectory_csv_.open("/home/ws/robot_actual_trajectory.csv");
    
    if (trajectory_csv_.is_open()) {
        trajectory_csv_ << "X,Y\n"; // Write the column headers
        RCLCPP_INFO(this->get_logger(), "Successfully created robot_actual_trajectory.csv");
    } else {
        RCLCPP_ERROR(this->get_logger(), "FAILED to create trajectory CSV!");
    }

    // 1. Create the Publisher (Topic: /cmd_vel, Queue Size: 10)
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 2. Create the Subscriber (Topic: /odometry/filtered, Queue Size: 10)
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&PidNode::odom_callback, this, std::placeholders::_1)
    );

    // 3. Create the 30Hz Timer (1000ms / 30 = ~33ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&PidNode::control_loop, this)
    );
}

// --- THE DESTRUCTOR ---
PidNode::~PidNode() {
    if (trajectory_csv_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Saving and closing trajectory CSV...");
        trajectory_csv_.close();
    }
}

// Callback si conversie din quaternioni in unghi euler
void PidNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { 
    // 1. Update current X and Y from the EKF
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // 2. Extract Quaternions
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    // 3. Convert Quaternion to Yaw (Theta) in radians
    current_theta_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // ADD THIS TO THE BOTTOM OF THE CALLBACK:
    if (trajectory_csv_.is_open()) {
        trajectory_csv_ << current_x_ << "," << current_y_ << "\n";
    }
}

// --- 3. THE MATH LOOP ---
void PidNode::control_loop() {
    // 1. Safety check: Did we finish the track?
    if (current_wp_index_ >= waypoints_.size() - 1) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Track Complete! Stopping robot.");
        auto stop_msg = geometry_msgs::msg::Twist();
        velocity_publisher_->publish(stop_msg);
        return;
    }

    double dt = 1.0 / 30.0; 

    // 2. THE CLOSEST POINT SEARCH (No more missed point traps!)
    double min_dist = 9999.0;
    size_t closest_idx = current_wp_index_;

    // Search the next 20 points ahead to find the one physically closest
    size_t search_end = std::min(current_wp_index_ + 20, waypoints_.size());
    for (size_t i = current_wp_index_; i < search_end; i++) {
        double dx = waypoints_[i].first - current_x_;
        double dy = waypoints_[i].second - current_y_;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    // Update memory to the closest physical point
    current_wp_index_ = closest_idx;

    // 3. THE LOOKAHEAD (Target a dot 5 indices / 25cm ahead)
    size_t target_idx = std::min(current_wp_index_ + 5, waypoints_.size() - 1);
    
    double target_x = waypoints_[target_idx].first;
    double target_y = waypoints_[target_idx].second;

    // Calculate errors to the Lookahead point
    double x_err = target_x - current_x_;
    double y_err = target_y - current_y_;
    double distance_err = std::sqrt((x_err * x_err) + (y_err * y_err));

    // 4. CONTINUOUS PID MATH
    
    // 1. Calculate the Heading Error FIRST
    double target_angle = std::atan2(y_err, x_err);
    double heading_err = wrap_angle(target_angle - current_theta_);
    
    // If the heading error is greater than ~20 degrees (0.35 rads), we are in a sharp corner.
    // Wipe the linear "turbo boost" memory so it doesn't spool up while turning.
    if (std::abs(heading_err) > 0.35) {
        linear_pid_.reset(); 
    }

    // Then calculate the PID as normal
    double v_command = linear_pid_.calculate(distance_err, dt);
    double omega_command = angular_pid_.calculate(heading_err, dt);

    // 5. ADVANCED KINEMATIC COUPLING cosine filter
    // Slow down on sharp turns. If the point is behind us (>90 deg), drop v to 0.0 so we only spin!
    v_command = v_command * std::max(0.0, std::cos(heading_err));

    // 6. PHYSICAL CLAMPS
    v_command = std::clamp(v_command, -1.0, 1.0);
    omega_command = std::clamp(omega_command, -1.5, 1.5);

    // 7. PUBLISH
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = v_command;
    twist_msg.angular.z = omega_command;
    velocity_publisher_->publish(twist_msg);

}

// 4. The Main Executable
// (ROS 2 absolutely requires this at the bottom of the .cpp file to run)
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidNode>());
    rclcpp::shutdown();
    return 0;
}