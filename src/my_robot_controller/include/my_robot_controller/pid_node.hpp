#ifndef PID_NODE_HPP
#define PID_NODE_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstddef>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

/// Generic single-input, single-output PID controller.
///
/// The controller evaluates
///   u = Kp * e + Ki * integral(e dt) + Kd * de/dt
/// and limits the stored integral state to avoid integral windup. It is used
/// twice by PidNode: once for distance/linear velocity and once for
/// heading/angular velocity.
class PIDController
{
public:
  PIDController(double kp, double ki, double kd, double max_i)
  : kp_(kp), ki_(ki), kd_(kd), max_i_(max_i) {}

  /// Replace all gains and clear the controller's accumulated state.
  void configure(double kp, double ki, double kd, double max_i);

  /// Calculate one control output from the current error and elapsed time.
  double calculate(double error, double dt);

  /// Clear integral and derivative memory after discontinuities or timeouts.
  void reset();

private:
  // Controller gains and the absolute integral-state limit.
  double kp_;
  double ki_;
  double kd_;
  double max_i_;

  // State retained between consecutive controller updates.
  double integral_{0.0};
  double prev_error_{0.0};

  // Suppresses derivative kick on the first calculation after a reset.
  bool has_previous_error_{false};
};

/// ROS 2 node that follows an x,y waypoint path with two PID controllers.
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
  /// Read and validate a headerless two-column x,y waypoint CSV.
  void load_waypoints(const std::string & file_path);

  /// Store the latest pose, derive dt from its timestamp, and run control.
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// Select path references, calculate PID commands, publish, and log them.
  void control_loop(double stamp_seconds, double dt);

  /// Stop the robot if filtered odometry disappears for too long.
  void watchdog_callback();

  /// Publish an all-zero geometry_msgs/Twist and remember that it was sent.
  void publish_stop();

  /// Write one MATLAB-friendly experiment row with a consistent column order.
  void log_sample(
    double stamp_seconds, double reference_x, double reference_y, double reference_yaw,
    double cross_track_error, double path_heading_error, double target_heading_error,
    double linear_command, double angular_command);

  // Experiment input/output. Waypoints are stored as ordered x,y pairs.
  std::ofstream trajectory_csv_;
  std::vector<std::pair<double, double>> waypoints_;

  // Path progress is monotonic: the closest-point search never moves backward.
  std::size_t current_wp_index_{0};

  // Independent controllers for translational and rotational commands.
  PIDController linear_pid_{1.0, 0.1, 0.2, 1.0};
  PIDController angular_pid_{1.5, 0.0, 0.3, 1.0};

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
  std::size_t search_window_{20};

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
