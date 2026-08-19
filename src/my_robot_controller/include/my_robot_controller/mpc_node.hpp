#ifndef MY_ROBOT_CONTROLLER__MPC_NODE_HPP_
#define MY_ROBOT_CONTROLLER__MPC_NODE_HPP_

#include "my_robot_controller/linear_mpc_controller.hpp"
#include "my_robot_controller/motion_command_policy.hpp"
#include "my_robot_controller/trajectory_reference_manager.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <fstream>
#include <vector>

/// ROS adapter for constrained linear time-varying MPC trajectory tracking.
///
/// Transport, EKF feedback, reference generation, completion rules, final
/// safety policy, and evaluator interface mirror the PID and TVLQR nodes. The
/// only intentional difference is the online finite-horizon feedback law.
class MpcNode : public rclcpp::Node
{
public:
  MpcNode();
  ~MpcNode() override;

  void stop();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  void control_loop(double stamp_seconds);
  void watchdog_callback();
  void publish_stop();
  void build_prediction(
    double elapsed_time,
    std::vector<my_robot_controller::DiscreteErrorModel> & models,
    std::vector<my_robot_controller::MpcReferenceInput> & inputs) const;
  void log_sample(
    double stamp_seconds,
    const my_robot_controller::TrajectoryReference & reference,
    const my_robot_controller::LinearMpcOutput & mpc_output,
    const my_robot_controller::MotionCommand & motion_command);

  // Output stream plus the ROS-independent trajectory, QP, and command blocks.
  std::ofstream trajectory_csv_;
  my_robot_controller::TrajectoryReferenceManager reference_manager_;
  my_robot_controller::LinearMpcController mpc_controller_;
  my_robot_controller::MotionCommandPolicy motion_command_policy_;

  // ROS transport objects; odometry callbacks drive control synchronously.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr experiment_start_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Latest planar EKF state and timestamps expressed in simulation seconds.
  double current_x_{0.0};
  double current_y_{0.0};
  double current_theta_{0.0};
  double previous_stamp_seconds_{0.0};
  double first_stamp_seconds_{0.0};
  double command_connection_stamp_seconds_{0.0};

  // Common timing, completion, and watchdog configuration.
  double nominal_dt_{1.0 / 30.0};
  double max_control_dt_{0.2};
  double goal_tolerance_{0.08};
  double goal_heading_tolerance_{0.15};
  double odom_timeout_{2.0};
  double startup_settling_time_{1.0};

  // Lifecycle/readiness flags prevent premature timing or duplicate stop data.
  bool odom_received_{false};
  bool command_transport_connected_{false};
  bool command_path_ready_{false};
  bool waiting_for_command_path_logged_{false};
  bool track_complete_{false};
  bool stop_sent_{false};
  bool post_horizon_warning_logged_{false};
  std::chrono::steady_clock::time_point last_odom_wall_time_;
};

#endif  // MY_ROBOT_CONTROLLER__MPC_NODE_HPP_
