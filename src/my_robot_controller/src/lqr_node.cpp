#include "my_robot_controller/lqr_node.hpp"

#include "my_robot_controller/linearized_error_model.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

// Odometry stores orientation as a quaternion. Under planar motion this
// standard conversion extracts the chassis yaw used by the error model.
double yaw_from_odometry(const nav_msgs::msg::Odometry & message)
{
  const auto & q = message.pose.pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

LqrNode::LqrNode()
: Node("lqr_node")
{
  // Construction performs all experiment setup once: declare/read parameters,
  // configure pure control blocks, load the common reference, create the log,
  // and finally connect ROS publishers/subscribers.
  // -----------------------------------------------------------------------
  // Experiment files and quadratic cost
  // -----------------------------------------------------------------------
  declare_parameter<std::string>("csv_path", "");
  declare_parameter<std::string>("output_csv_path", "robot_lqr_trajectory.csv");
  declare_parameter<double>("lqr_longitudinal_error_weight", 100.0);
  declare_parameter<double>("lqr_lateral_error_weight", 400.0);
  declare_parameter<double>("lqr_heading_error_weight", 100.0);
  declare_parameter<double>("lqr_linear_correction_weight", 125.0);
  declare_parameter<double>("lqr_angular_correction_weight", 15.625);
  declare_parameter<double>("lqr_terminal_weight_multiplier", 10.0);

  // -----------------------------------------------------------------------
  // Common reference, timing, completion, and actuator policy parameters
  // -----------------------------------------------------------------------
  declare_parameter<int>("search_window", 20);
  declare_parameter<double>("reference_linear_velocity", 0.4);
  declare_parameter<double>("curvature_speed_gain", 0.5);
  declare_parameter<double>("endpoint_slowdown_distance", 0.0);
  declare_parameter<double>("maximum_reference_curvature", 5.0);
  declare_parameter<double>("trajectory_spatial_step", 0.01);
  declare_parameter<double>("maximum_reference_linear_acceleration", 0.5);
  declare_parameter<double>("maximum_reference_linear_deceleration", 0.1);
  declare_parameter<double>("maximum_reference_angular_velocity", 1.5);
  declare_parameter<double>("nominal_control_frequency", 30.0);
  declare_parameter<double>("max_control_dt", 0.2);
  declare_parameter<double>("goal_tolerance", 0.08);
  declare_parameter<double>("goal_heading_tolerance", 0.15);
  declare_parameter<double>("max_linear_velocity", 1.0);
  declare_parameter<double>("max_angular_velocity", 1.5);
  declare_parameter<double>("translation_stop_lateral_error", 0.75);
  declare_parameter<double>("translation_stop_heading_error", 1.2);
  declare_parameter<double>("odom_timeout", 2.0);
  declare_parameter<double>("startup_settling_time", 1.0);

  // Convert external ROS parameters to validated C++ configuration objects.
  // Failing early is safer than discovering an invalid limit during motion.
  const auto search_window = get_parameter("search_window").as_int();
  const double frequency = get_parameter("nominal_control_frequency").as_double();
  if (search_window < 1 || !std::isfinite(frequency) || frequency <= 0.0) {
    throw std::runtime_error("search_window and nominal_control_frequency must be positive");
  }
  nominal_dt_ = 1.0 / frequency;
  max_control_dt_ = get_parameter("max_control_dt").as_double();
  goal_tolerance_ = get_parameter("goal_tolerance").as_double();
  goal_heading_tolerance_ = get_parameter("goal_heading_tolerance").as_double();
  odom_timeout_ = get_parameter("odom_timeout").as_double();
  startup_settling_time_ = get_parameter("startup_settling_time").as_double();
  const double maximum_linear_velocity = get_parameter("max_linear_velocity").as_double();
  const double maximum_angular_velocity = get_parameter("max_angular_velocity").as_double();
  if (!std::isfinite(max_control_dt_) || max_control_dt_ <= 0.0 ||
    !std::isfinite(goal_tolerance_) || goal_tolerance_ <= 0.0 ||
    !std::isfinite(goal_heading_tolerance_) || goal_heading_tolerance_ <= 0.0 ||
    !std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0 ||
    !std::isfinite(startup_settling_time_) || startup_settling_time_ < 0.0)
  {
    throw std::runtime_error(
            "LQR timing, tolerances, and timeout must be positive and finite");
  }

  // LQR-specific part: Q, R, and terminal scaling.
  my_robot_controller::TimeVaryingLqrConfig lqr_config;
  lqr_config.longitudinal_error_weight =
    get_parameter("lqr_longitudinal_error_weight").as_double();
  lqr_config.lateral_error_weight =
    get_parameter("lqr_lateral_error_weight").as_double();
  lqr_config.heading_error_weight =
    get_parameter("lqr_heading_error_weight").as_double();
  lqr_config.linear_correction_weight =
    get_parameter("lqr_linear_correction_weight").as_double();
  lqr_config.angular_correction_weight =
    get_parameter("lqr_angular_correction_weight").as_double();
  lqr_config.terminal_weight_multiplier =
    get_parameter("lqr_terminal_weight_multiplier").as_double();
  lqr_controller_.configure(lqr_config);

  // Controller-independent actuator/safety policy used after feedback.
  my_robot_controller::MotionCommandPolicyConfig motion_config;
  motion_config.maximum_linear_velocity = maximum_linear_velocity;
  motion_config.maximum_angular_velocity = maximum_angular_velocity;
  motion_config.translation_stop_lateral_error =
    get_parameter("translation_stop_lateral_error").as_double();
  motion_config.translation_stop_heading_error =
    get_parameter("translation_stop_heading_error").as_double();
  motion_command_policy_.configure(motion_config);

  // Controller-independent geometric and temporal reference policy.
  my_robot_controller::TrajectoryReferenceConfig reference_config;
  reference_config.path.search_window = static_cast<std::size_t>(search_window);
  reference_config.path.nominal_linear_velocity =
    get_parameter("reference_linear_velocity").as_double();
  reference_config.path.curvature_speed_gain =
    get_parameter("curvature_speed_gain").as_double();
  reference_config.path.endpoint_slowdown_distance =
    get_parameter("endpoint_slowdown_distance").as_double();
  reference_config.path.maximum_abs_curvature =
    get_parameter("maximum_reference_curvature").as_double();
  reference_config.spatial_step = get_parameter("trajectory_spatial_step").as_double();
  reference_config.maximum_linear_acceleration =
    get_parameter("maximum_reference_linear_acceleration").as_double();
  reference_config.maximum_linear_deceleration =
    get_parameter("maximum_reference_linear_deceleration").as_double();
  reference_config.maximum_reference_angular_velocity =
    get_parameter("maximum_reference_angular_velocity").as_double();
  if (reference_config.maximum_reference_angular_velocity > maximum_angular_velocity) {
    throw std::runtime_error(
            "Reference yaw-rate limit must not exceed the actuator yaw-rate limit");
  }
  reference_manager_.configure(reference_config);

  const std::string path_file = get_parameter("csv_path").as_string();
  const std::string output_file = get_parameter("output_csv_path").as_string();
  if (path_file.empty() || output_file.empty()) {
    throw std::runtime_error("LQR input and output CSV paths must not be empty");
  }
  if (path_file == output_file) {
    throw std::runtime_error("LQR input and output CSV paths must be different");
  }
  reference_manager_.load_csv(path_file);
  build_gain_schedule();

  // Truncate stale data and write a fixed schema before any callback can log.
  trajectory_csv_.open(output_file, std::ios::out | std::ios::trunc);
  if (!trajectory_csv_.is_open()) {
    throw std::runtime_error("Could not create LQR trajectory CSV: " + output_file);
  }
  trajectory_csv_ << std::setprecision(10);
  trajectory_csv_ <<
    "time,actual_x,actual_y,actual_yaw,reference_x,reference_y,reference_yaw,"
    "projection_x,projection_y,projection_yaw,longitudinal_error,lateral_error,"
    "heading_error,position_error,cross_track_error,path_heading_error,"
    "reference_linear_velocity,reference_angular_velocity,linear_command,angular_command,"
    "linear_feedback_command,angular_feedback_command,linear_feedforward_command,"
    "angular_feedforward_command,lqr_gain_index,k_v_ex,k_v_ey,k_v_eheading,"
    "k_omega_ex,k_omega_ey,k_omega_eheading,instantaneous_state_cost,"
    "reference_progress,reference_remaining_length,reference_curvature,"
    "projection_progress,projection_remaining_length,waypoint_index,segment_index,"
    "segment_fraction,reference_time,trajectory_complete,translation_safety_stop\n";

  RCLCPP_INFO(
    get_logger(),
    "Loaded %zu waypoints (%.3f m, %.3f s) and precomputed %zu TVLQR gains",
    reference_manager_.waypoint_count(), reference_manager_.total_length(),
    reference_manager_.duration(), lqr_controller_.gain_count());
  RCLCPP_INFO(get_logger(), "Writing LQR experiment data to %s", output_file.c_str());

  // Transient-local experiment time is retained for late-starting injectors
  // and evaluators, giving the entire ROS graph one simulation-time epoch.
  velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  experiment_start_publisher_ = create_publisher<std_msgs::msg::Float64>(
    "experiment_start_time", rclcpp::QoS(1).reliable().transient_local());
  odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered", 10,
    std::bind(&LqrNode::odom_callback, this, std::placeholders::_1));

  last_odom_wall_time_ = std::chrono::steady_clock::now();
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&LqrNode::watchdog_callback, this));
}

LqrNode::~LqrNode()
{
  // RAII shutdown leaves the simulated actuator at zero and commits buffered
  // CSV data even when the executor exits through an exception.
  stop();
  if (trajectory_csv_.is_open()) {
    trajectory_csv_.flush();
    trajectory_csv_.close();
  }
}

void LqrNode::build_gain_schedule()
{
  // One gain is generated for every nominal controller interval covering the
  // complete timed trajectory. Runtime therefore requires no Riccati solve.
  const std::size_t model_count = std::max<std::size_t>(
    1u, static_cast<std::size_t>(std::ceil(reference_manager_.duration() / nominal_dt_)));
  std::vector<my_robot_controller::DiscreteErrorModel> models;
  models.reserve(model_count);

  for (std::size_t index = 0u; index < model_count; ++index) {
    // Midpoint sampling reduces the error made when the known LTV coefficients
    // vary within a control interval. Each resulting model still uses exact ZOH
    // for the coefficients frozen at that midpoint.
    const double midpoint_time = std::min(
      (static_cast<double>(index) + 0.5) * nominal_dt_, reference_manager_.duration());
    const auto sample = reference_manager_.sample_at_time(midpoint_time);
    models.push_back(
      my_robot_controller::LinearizedErrorModel::discretize_zero_order_hold(
        sample.reference_linear_velocity,
        sample.reference_angular_velocity,
        nominal_dt_));
  }
  lqr_controller_.build_gain_schedule(models);
}

std::size_t LqrNode::model_index_at(double elapsed_time) const
{
  // Floor maps absolute experiment time to the corresponding precomputed
  // interval; calculate() safely clamps late indices to the final gain.
  if (!std::isfinite(elapsed_time) || elapsed_time <= 0.0) {
    return 0u;
  }
  return static_cast<std::size_t>(std::floor(elapsed_time / nominal_dt_));
}

void LqrNode::publish_stop()
{
  // A default Twist is all zero. Remembering it avoids repeated watchdog work.
  if (!velocity_publisher_) {
    return;
  }
  velocity_publisher_->publish(geometry_msgs::msg::Twist());
  stop_sent_ = true;
}

void LqrNode::stop()
{
  publish_stop();
}

void LqrNode::watchdog_callback()
{
  // Simulation time may pause intentionally, so availability is monitored in
  // host steady-clock time. The watchdog never generates normal control updates.
  if (!odom_received_ || track_complete_ || stop_sent_) {
    return;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_odom_wall_time_).count();
  if (elapsed > odom_timeout_) {
    RCLCPP_WARN(get_logger(), "Odometry timed out; commanding zero velocity");
    publish_stop();
  }
}

void LqrNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  // Every fresh EKF sample is the trigger for exactly one possible control
  // update; there is no independent timer that could reuse stale state.
  current_x_ = message->pose.pose.position.x;
  current_y_ = message->pose.pose.position.y;
  current_theta_ = yaw_from_odometry(*message);
  if (!std::isfinite(current_x_) || !std::isfinite(current_y_) ||
    !std::isfinite(current_theta_))
  {
    RCLCPP_ERROR(get_logger(), "Received non-finite odometry; commanding zero velocity");
    publish_stop();
    return;
  }

  const double stamp_seconds = rclcpp::Time(message->header.stamp).seconds();
  if (!command_transport_connected_) {
    // DDS discovery is asynchronous. Do not start the reference clock until a
    // downstream command subscriber exists, otherwise the virtual robot could
    // advance while Gazebo receives no commands.
    last_odom_wall_time_ = std::chrono::steady_clock::now();
    if (velocity_publisher_->get_subscription_count() == 0u) {
      if (!waiting_for_command_path_logged_) {
        RCLCPP_INFO(get_logger(), "Waiting for the downstream command subscriber");
        waiting_for_command_path_logged_ = true;
      }
      return;
    }
    command_transport_connected_ = true;
    command_connection_stamp_seconds_ = stamp_seconds;
    waiting_for_command_path_logged_ = false;
    publish_stop();
    RCLCPP_INFO(
      get_logger(), "Command path connected; holding zero for %.3f s of simulation time",
      startup_settling_time_);
    return;
  }

  if (!command_path_ready_) {
    // Hold zero for a reproducible settling interval after discovery. This
    // removes spawn/estimator transients from measured controller performance.
    last_odom_wall_time_ = std::chrono::steady_clock::now();
    publish_stop();
    if (stamp_seconds - command_connection_stamp_seconds_ < startup_settling_time_) {
      return;
    }
    command_path_ready_ = true;
    odom_received_ = false;
    stop_sent_ = false;
    reference_manager_.reset_projection();
    RCLCPP_INFO(get_logger(), "Startup settling complete; starting the TVLQR experiment");
  }

  if (!odom_received_) {
    // The first post-settling sample defines t=0 for controller, injector, and
    // evaluator through the latched experiment_start_time topic.
    first_stamp_seconds_ = stamp_seconds;
    std_msgs::msg::Float64 start_message;
    start_message.data = first_stamp_seconds_;
    experiment_start_publisher_->publish(start_message);
  } else {
    const double dt = stamp_seconds - previous_stamp_seconds_;
    if (dt <= 0.0) {
      RCLCPP_WARN(get_logger(), "Ignoring odometry with a non-increasing timestamp");
      return;
    }
    if (dt > max_control_dt_) {
      // TVLQR has no integral or derivative memory to reset. The absolute
      // reference clock automatically selects the gain matching the new time.
      RCLCPP_WARN(
        get_logger(), "Odometry gap %.3f s exceeded max_control_dt; resynchronizing gain index",
        dt);
    }
  }

  previous_stamp_seconds_ = stamp_seconds;
  odom_received_ = true;
  stop_sent_ = false;
  last_odom_wall_time_ = std::chrono::steady_clock::now();
  if (track_complete_) {
    publish_stop();
    return;
  }
  control_loop(stamp_seconds);
}

void LqrNode::control_loop(double stamp_seconds)
{
  // Data path: timestamp -> common timed reference -> common body-frame error
  // -> scheduled LQR feedback -> common command policy -> Twist and CSV.
  const double elapsed_time = stamp_seconds - first_stamp_seconds_;
  const auto reference = reference_manager_.update(
    elapsed_time, current_x_, current_y_, current_theta_);
  const my_robot_controller::ErrorState error(
    reference.longitudinal_error,
    reference.lateral_error,
    reference.heading_error);
  const auto lqr_output = lqr_controller_.calculate(
    model_index_at(elapsed_time), error);

  // Time alone is insufficient: after the reference ends, allow the robot to
  // settle until both terminal position and heading tolerances are satisfied.
  const bool experiment_complete =
    reference.trajectory_complete && reference.position_error <= goal_tolerance_ &&
    std::abs(reference.heading_error) <= goal_heading_tolerance_;
  if (experiment_complete) {
    track_complete_ = true;
    publish_stop();
    my_robot_controller::MotionCommand stopped_command;
    log_sample(stamp_seconds, reference, lqr_output, stopped_command);
    trajectory_csv_.flush();
    RCLCPP_INFO(
      get_logger(),
      "Trajectory complete at simulation time %.6f s at (%.3f, %.3f); "
      "position error %.3f m, heading error %.3f rad",
      stamp_seconds, current_x_, current_y_, reference.position_error,
      reference.heading_error);
    return;
  }

  if (reference.trajectory_complete && !post_horizon_warning_logged_) {
    RCLCPP_WARN(
      get_logger(),
      "Reference horizon ended before pose tolerance; holding the final TVLQR gain");
    post_horizon_warning_logged_ = true;
  }

  const auto motion_command = motion_command_policy_.calculate(
    reference.trajectory.reference_linear_velocity,
    reference.trajectory.reference_angular_velocity,
    reference.lateral_error,
    reference.heading_error,
    lqr_output.correction(0),
    lqr_output.correction(1));

  geometry_msgs::msg::Twist command;
  command.linear.x = motion_command.linear_command;
  command.angular.z = motion_command.angular_command;
  velocity_publisher_->publish(command);
  log_sample(stamp_seconds, reference, lqr_output, motion_command);
}

void LqrNode::log_sample(
  double stamp_seconds,
  const my_robot_controller::TrajectoryReference & reference,
  const my_robot_controller::TimeVaryingLqrOutput & lqr_output,
  const my_robot_controller::MotionCommand & motion_command)
{
  // Store inputs, reference/projection states, feedback gains, final commands,
  // and completion flags in one row for MATLAB/Python analysis and traceability.
  const auto & gain = lqr_output.gain;
  trajectory_csv_ <<
    stamp_seconds - first_stamp_seconds_ << ',' <<
    current_x_ << ',' << current_y_ << ',' << current_theta_ << ',' <<
    reference.trajectory.position.x << ',' << reference.trajectory.position.y << ',' <<
    reference.trajectory.heading << ',' <<
    reference.projection.path.position.x << ',' <<
    reference.projection.path.position.y << ',' << reference.projection.path.heading << ',' <<
    reference.longitudinal_error << ',' << reference.lateral_error << ',' <<
    reference.heading_error << ',' << reference.position_error << ',' <<
    reference.projection.cross_track_error << ',' << reference.projection.heading_error << ',' <<
    reference.trajectory.reference_linear_velocity << ',' <<
    reference.trajectory.reference_angular_velocity << ',' <<
    motion_command.linear_command << ',' << motion_command.angular_command << ',' <<
    lqr_output.correction(0) << ',' << lqr_output.correction(1) << ',' <<
    motion_command.linear_feedforward_command << ',' <<
    motion_command.angular_feedforward_command << ',' <<
    lqr_output.gain_index << ',' <<
    gain(0, 0) << ',' << gain(0, 1) << ',' << gain(0, 2) << ',' <<
    gain(1, 0) << ',' << gain(1, 1) << ',' << gain(1, 2) << ',' <<
    lqr_output.instantaneous_state_cost << ',' <<
    reference.trajectory.progress << ',' << reference.trajectory.remaining_length << ',' <<
    reference.trajectory.curvature << ',' << reference.projection.path.progress << ',' <<
    reference.projection.path.remaining_length << ',' <<
    reference.projection.closest_waypoint_index << ',' <<
    reference.trajectory.segment_index << ',' << reference.trajectory.segment_fraction << ',' <<
    reference.reference_time << ',' << (reference.trajectory_complete ? 1 : 0) << ',' <<
    (motion_command.translation_safety_stop ? 1 : 0) << '\n';
}

int main(int argc, char ** argv)
{
  // Convert configuration/runtime exceptions into a nonzero process exit while
  // still performing orderly ROS shutdown and a final stop command.
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<LqrNode>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lqr_node"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
