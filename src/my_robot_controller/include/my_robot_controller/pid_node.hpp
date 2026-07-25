#ifndef PID_NODE_HPP
#define PID_NODE_HPP

#include "my_robot_controller/cascaded_pid_controller.hpp"
#include "my_robot_controller/path_reference_manager.hpp"
#include "my_robot_controller/pid_controller.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstddef>
#include <fstream>
#include <string>

/// ROS 2 node that can run either the saved lookahead PID or cascaded PID.
///
/// Data flow:
///   /odometry/filtered -> PidNode -> /cmd_vel
///
/// Each filtered odometry message triggers one control update. This keeps the
/// control loop synchronized with Gazebo simulation time instead of host wall
/// time. A separate wall-clock watchdog only handles loss of odometry.
class PidNode : public rclcpp::Node
{
public:
  PidNode();
  ~PidNode() override;

  /// Publish a zero Twist command. Exposed so main() can request a final stop.
  void stop();

private:
  enum class ControllerMode
  {
    kLookahead,
    kCascade
  };

  /// Controller-specific values accompanying the common path reference.
  ///
  /// Keeping a fixed numeric schema for both modes makes the resulting files
  /// straightforward to import, filter, and compare in MATLAB.
  struct ControllerDiagnostics
  {
    double control_heading_error{0.0};
    double desired_heading{0.0};
    double heading_correction{0.0};
    double cross_track_pid_output{0.0};
    double heading_pid_output{0.0};
    double heading_speed_factor{1.0};
    double cross_track_speed_factor{1.0};
  };

  /// Store the latest pose, derive dt from its timestamp, and run control.
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// Select path references, calculate PID commands, publish, and log them.
  void control_loop(double stamp_seconds, double dt);

  /// Stop the robot if filtered odometry disappears for too long.
  void watchdog_callback();

  /// Publish an all-zero geometry_msgs/Twist and remember that it was sent.
  void publish_stop();

  /// Clear every PID state after an odometry discontinuity or timeout.
  void reset_controllers();

  /// Write one MATLAB-friendly experiment row with a consistent column order.
  void log_sample(
    double stamp_seconds, const my_robot_controller::PathReference & reference,
    const ControllerDiagnostics & diagnostics,
    double linear_command, double angular_command);

  // Experiment output and controller-independent interpretation of the input
  // path. The manager owns waypoints, projection, progress, and curvature.
  std::ofstream trajectory_csv_;
  my_robot_controller::PathReferenceManager reference_manager_;

  // The lookahead controllers remain intact as the saved preliminary baseline.
  my_robot_controller::PIDController linear_pid_{1.0, 0.1, 0.2, 1.0};
  my_robot_controller::PIDController angular_pid_{1.5, 0.0, 0.3, 1.0};

  // The thesis controller uses an outer cross-track loop and inner heading
  // loop, both contained in a ROS-independent, unit-tested component.
  my_robot_controller::CascadedPidController cascaded_pid_;
  ControllerMode controller_mode_{ControllerMode::kLookahead};

  // ROS communication objects. The watchdog timer is not the control timer.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Latest planar state estimated by robot_localization's EKF.
  double current_x_{0.0};
  double current_y_{0.0};
  double current_theta_{0.0};

  // ROS timestamps used for measured dt and experiment-relative time.
  double previous_stamp_seconds_{0.0};
  double first_stamp_seconds_{0.0};

  // Path search and controller safety parameters loaded from ROS parameters.
  std::size_t lookahead_points_{5};

  // nominal_dt_ is a safe fallback after the first sample or a large data gap.
  double nominal_dt_{1.0 / 30.0};
  double max_control_dt_{0.2};
  double goal_tolerance_{0.08};
  double max_linear_velocity_{1.0};
  double max_angular_velocity_{1.5};
  double odom_timeout_{2.0};

  // Lifecycle/safety flags and the wall-clock time of the latest odometry.
  bool odom_received_{false};
  bool track_complete_{false};
  bool stop_sent_{false};
  std::chrono::steady_clock::time_point last_odom_wall_time_;
};

#endif
