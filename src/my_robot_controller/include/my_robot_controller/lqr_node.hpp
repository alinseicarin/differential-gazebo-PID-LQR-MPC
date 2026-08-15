#ifndef MY_ROBOT_CONTROLLER__LQR_NODE_HPP_
#define MY_ROBOT_CONTROLLER__LQR_NODE_HPP_

#include "my_robot_controller/motion_command_policy.hpp"
#include "my_robot_controller/time_varying_lqr_controller.hpp"
#include "my_robot_controller/trajectory_reference_manager.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <cstddef>
#include <fstream>

/// ROS adapter for finite-horizon time-varying LQR trajectory tracking.
///
/// It intentionally mirrors the timed PID node's transport synchronization,
/// EKF input, reference manager, completion rule, actuator policy, and logging
/// boundary. Only the feedback law differs between the compared controllers.
class LqrNode : public rclcpp::Node
{
public:
  LqrNode();
  ~LqrNode() override;

  void stop();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  void control_loop(double stamp_seconds);
  void watchdog_callback();
  void publish_stop();
  void build_gain_schedule();
  std::size_t model_index_at(double elapsed_time) const;
  void log_sample(
    double stamp_seconds,
    const my_robot_controller::TrajectoryReference & reference,
    const my_robot_controller::TimeVaryingLqrOutput & lqr_output,
    const my_robot_controller::MotionCommand & motion_command);

  std::ofstream trajectory_csv_;
  my_robot_controller::TrajectoryReferenceManager reference_manager_;
  my_robot_controller::TimeVaryingLqrController lqr_controller_;
  my_robot_controller::MotionCommandPolicy motion_command_policy_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr experiment_start_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  double current_x_{0.0};
  double current_y_{0.0};
  double current_theta_{0.0};
  double previous_stamp_seconds_{0.0};
  double first_stamp_seconds_{0.0};
  double command_connection_stamp_seconds_{0.0};

  double nominal_dt_{1.0 / 30.0};
  double max_control_dt_{0.2};
  double goal_tolerance_{0.08};
  double goal_heading_tolerance_{0.15};
  double odom_timeout_{2.0};
  double startup_settling_time_{1.0};

  bool odom_received_{false};
  bool command_transport_connected_{false};
  bool command_path_ready_{false};
  bool waiting_for_command_path_logged_{false};
  bool track_complete_{false};
  bool stop_sent_{false};
  bool post_horizon_warning_logged_{false};
  std::chrono::steady_clock::time_point last_odom_wall_time_;
};

#endif  // MY_ROBOT_CONTROLLER__LQR_NODE_HPP_
