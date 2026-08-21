#include "my_robot_controller/mpc_node.hpp"

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

// Extract planar yaw from the EKF odometry quaternion.
double yaw_from_odometry(const nav_msgs::msg::Odometry & message)
{
  const auto & q = message.pose.pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

MpcNode::MpcNode()
: Node("mpc_node")
{
  // Setup mirrors LQR/PID as closely as possible; only the feedback component
  // and its solver parameters are MPC-specific.
  // -----------------------------------------------------------------------
  // Experiment files, horizon cost, and numerical solver
  // -----------------------------------------------------------------------
  declare_parameter<std::string>("csv_path", "");
  declare_parameter<std::string>("output_csv_path", "robot_mpc_trajectory.csv");
  declare_parameter<int>("mpc_prediction_horizon_steps", 45);
  declare_parameter<double>("mpc_longitudinal_error_weight", 100.0);
  declare_parameter<double>("mpc_lateral_error_weight", 400.0);
  declare_parameter<double>("mpc_heading_error_weight", 100.0);
  declare_parameter<double>("mpc_linear_correction_weight", 125.0);
  declare_parameter<double>("mpc_angular_correction_weight", 15.625);
  declare_parameter<double>("mpc_terminal_weight_multiplier", 10.0);
  declare_parameter<int>("mpc_maximum_solver_iterations", 4000);
  declare_parameter<double>("mpc_absolute_solver_tolerance", 1.0e-5);
  declare_parameter<double>("mpc_relative_solver_tolerance", 1.0e-5);
  declare_parameter<double>("mpc_solver_time_limit", 0.02);
  declare_parameter<bool>("mpc_polish_solution", false);

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

  // Read basic dimensions/timing first because they determine every later
  // configuration and the real-time budget of each QP.
  const auto search_window = get_parameter("search_window").as_int();
  const auto horizon_steps = get_parameter("mpc_prediction_horizon_steps").as_int();
  const double frequency = get_parameter("nominal_control_frequency").as_double();
  if (search_window < 1 || horizon_steps < 1 ||
    !std::isfinite(frequency) || frequency <= 0.0)
  {
    throw std::runtime_error(
            "search_window, MPC horizon, and nominal_control_frequency must be positive");
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
    throw std::runtime_error("MPC timing, tolerances, and timeout must be positive and finite");
  }

  // MPC-specific horizon cost, physical input limits, and OSQP settings.
  my_robot_controller::LinearMpcConfig mpc_config;
  mpc_config.prediction_horizon_steps = static_cast<std::size_t>(horizon_steps);
  mpc_config.longitudinal_error_weight =
    get_parameter("mpc_longitudinal_error_weight").as_double();
  mpc_config.lateral_error_weight =
    get_parameter("mpc_lateral_error_weight").as_double();
  mpc_config.heading_error_weight = get_parameter("mpc_heading_error_weight").as_double();
  mpc_config.linear_correction_weight =
    get_parameter("mpc_linear_correction_weight").as_double();
  mpc_config.angular_correction_weight =
    get_parameter("mpc_angular_correction_weight").as_double();
  mpc_config.terminal_weight_multiplier =
    get_parameter("mpc_terminal_weight_multiplier").as_double();
  mpc_config.minimum_linear_velocity = 0.0;
  mpc_config.maximum_linear_velocity = maximum_linear_velocity;
  mpc_config.maximum_absolute_angular_velocity = maximum_angular_velocity;
  mpc_config.maximum_solver_iterations = static_cast<int>(
    get_parameter("mpc_maximum_solver_iterations").as_int());
  mpc_config.absolute_solver_tolerance =
    get_parameter("mpc_absolute_solver_tolerance").as_double();
  mpc_config.relative_solver_tolerance =
    get_parameter("mpc_relative_solver_tolerance").as_double();
  mpc_config.solver_time_limit = get_parameter("mpc_solver_time_limit").as_double();
  mpc_config.polish_solution = get_parameter("mpc_polish_solution").as_bool();
  mpc_controller_.configure(mpc_config);

  // Shared post-controller limits and gross-error guard.
  my_robot_controller::MotionCommandPolicyConfig motion_config;
  motion_config.maximum_linear_velocity = maximum_linear_velocity;
  motion_config.maximum_angular_velocity = maximum_angular_velocity;
  motion_config.translation_stop_lateral_error =
    get_parameter("translation_stop_lateral_error").as_double();
  motion_config.translation_stop_heading_error =
    get_parameter("translation_stop_heading_error").as_double();
  motion_command_policy_.configure(motion_config);

  // Shared geometry/time law; future samples are also used inside prediction.
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
    throw std::runtime_error("MPC input and output CSV paths must not be empty");
  }
  if (path_file == output_file) {
    throw std::runtime_error("MPC input and output CSV paths must be different");
  }
  reference_manager_.load_csv(path_file);

  // Create a fresh machine-readable experiment log before ROS callbacks start.
  trajectory_csv_.open(output_file, std::ios::out | std::ios::trunc);
  if (!trajectory_csv_.is_open()) {
    throw std::runtime_error("Could not create MPC trajectory CSV: " + output_file);
  }
  trajectory_csv_ << std::setprecision(10);
  trajectory_csv_ <<
    "time,actual_x,actual_y,actual_yaw,reference_x,reference_y,reference_yaw,"
    "projection_x,projection_y,projection_yaw,longitudinal_error,lateral_error,"
    "heading_error,position_error,cross_track_error,path_heading_error,"
    "reference_linear_velocity,reference_angular_velocity,linear_command,angular_command,"
    "linear_feedback_command,angular_feedback_command,linear_feedforward_command,"
    "angular_feedforward_command,mpc_solved,mpc_solver_status,mpc_iterations,"
    "mpc_objective,mpc_solve_time_seconds,mpc_horizon_steps,reference_progress,"
    "reference_remaining_length,reference_curvature,projection_progress,"
    "projection_remaining_length,waypoint_index,segment_index,segment_fraction,"
    "reference_time,trajectory_complete,translation_safety_stop\n";

  RCLCPP_INFO(
    get_logger(),
    "Loaded %zu waypoints (%.3f m, %.3f s); LTV-MPC horizon %zu steps (%.3f s)",
    reference_manager_.waypoint_count(), reference_manager_.total_length(),
    reference_manager_.duration(), mpc_controller_.config().prediction_horizon_steps,
    static_cast<double>(mpc_controller_.config().prediction_horizon_steps) * nominal_dt_);
  RCLCPP_INFO(get_logger(), "Writing MPC experiment data to %s", output_file.c_str());

  // The retained start-time message synchronizes any injector/evaluator that
  // discovers its subscription after the MPC node is already ready.
  velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  experiment_start_publisher_ = create_publisher<std_msgs::msg::Float64>(
    "experiment_start_time", rclcpp::QoS(1).reliable().transient_local());
  odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered", 10,
    std::bind(&MpcNode::odom_callback, this, std::placeholders::_1));

  last_odom_wall_time_ = std::chrono::steady_clock::now();
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&MpcNode::watchdog_callback, this));
}

MpcNode::~MpcNode()
{
  // Ensure zero actuation and durable CSV output on normal or exceptional exit.
  stop();
  if (trajectory_csv_.is_open()) {
    trajectory_csv_.flush();
    trajectory_csv_.close();
  }
}

void MpcNode::build_prediction(
  double elapsed_time,
  std::vector<my_robot_controller::DiscreteErrorModel> & models,
  std::vector<my_robot_controller::MpcReferenceInput> & inputs) const
{
  // Rebuild the LTV sequence at every sample because the horizon moves forward
  // in absolute trajectory time. Vectors are cleared but capacity is reserved
  // to avoid repeated reallocations within this call.
  const std::size_t horizon = mpc_controller_.config().prediction_horizon_steps;
  models.clear();
  inputs.clear();
  models.reserve(horizon);
  inputs.reserve(horizon);

  for (std::size_t stage = 0u; stage < horizon; ++stage) {
    // Input bounds use the stage-start feedforward command. Model matrices use
    // midpoint coefficients, reducing frozen-model error within the interval.
    const double stage_time = elapsed_time + static_cast<double>(stage) * nominal_dt_;
    const double midpoint_time = stage_time + 0.5 * nominal_dt_;
    const auto input_reference = reference_manager_.sample_at_time(stage_time);
    const auto model_reference = reference_manager_.sample_at_time(midpoint_time);

    inputs.push_back(
      {input_reference.reference_linear_velocity,
        input_reference.reference_angular_velocity});
    models.push_back(
      my_robot_controller::LinearizedErrorModel::discretize_zero_order_hold(
        model_reference.reference_linear_velocity,
        model_reference.reference_angular_velocity,
        nominal_dt_));
  }
}

void MpcNode::publish_stop()
{
  // Default-constructed Twist means v=0 and omega=0.
  if (!velocity_publisher_) {
    return;
  }
  velocity_publisher_->publish(geometry_msgs::msg::Twist());
  stop_sent_ = true;
}

void MpcNode::stop()
{
  publish_stop();
}

void MpcNode::watchdog_callback()
{
  // This wall-clock timer only detects stale EKF input; odometry messages
  // themselves pace the 30 Hz optimization/control loop in simulation time.
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

void MpcNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  // Cache one finite planar EKF state before performing lifecycle checks.
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
    // Wait for DDS discovery so the reference clock cannot advance before the
    // downstream command injector/plugin is actually able to receive Twist.
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
    // Reproducible zero-command settling separates spawn/EKF startup from the
    // recorded control experiment and clears any prior QP warm start.
    last_odom_wall_time_ = std::chrono::steady_clock::now();
    publish_stop();
    if (stamp_seconds - command_connection_stamp_seconds_ < startup_settling_time_) {
      return;
    }
    command_path_ready_ = true;
    odom_received_ = false;
    stop_sent_ = false;
    reference_manager_.reset_projection();
    mpc_controller_.reset_warm_start();
    RCLCPP_INFO(get_logger(), "Startup settling complete; starting the LTV-MPC experiment");
  }

  if (!odom_received_) {
    // Publish the common simulation-time epoch exactly once per run.
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
      // A stale primal solution is a poor initial guess after a long gap.
      mpc_controller_.reset_warm_start();
      RCLCPP_WARN(
        get_logger(), "Odometry gap %.3f s exceeded max_control_dt; reset MPC warm start", dt);
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

void MpcNode::control_loop(double stamp_seconds)
{
  // Data path: timed reference and EKF pose -> current error plus future LTV
  // sequence -> OSQP -> first correction -> common saturation -> Twist/log.
  const double elapsed_time = stamp_seconds - first_stamp_seconds_;
  const auto reference = reference_manager_.update(
    elapsed_time, current_x_, current_y_, current_theta_);
  const my_robot_controller::ErrorState error(
    reference.longitudinal_error,
    reference.lateral_error,
    reference.heading_error);

  std::vector<my_robot_controller::DiscreteErrorModel> models;
  std::vector<my_robot_controller::MpcReferenceInput> inputs;
  build_prediction(elapsed_time, models, inputs);
  const auto mpc_output = mpc_controller_.calculate(error, models, inputs);

  // Completion requires both the end of the prescribed time law and terminal
  // pose tolerances, allowing recovery after a late disturbance.
  const bool experiment_complete =
    reference.trajectory_complete && reference.position_error <= goal_tolerance_ &&
    std::abs(reference.heading_error) <= goal_heading_tolerance_;
  if (experiment_complete) {
    track_complete_ = true;
    publish_stop();
    my_robot_controller::MotionCommand stopped_command;
    log_sample(stamp_seconds, reference, mpc_output, stopped_command);
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
      "Reference horizon ended before pose tolerance; predicting from final reference");
    post_horizon_warning_logged_ = true;
  }

  if (!mpc_output.solved) {
    // A failed/unfinished QP must never leak a stale correction to the robot.
    publish_stop();
    my_robot_controller::MotionCommand stopped_command;
    log_sample(stamp_seconds, reference, mpc_output, stopped_command);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "MPC solver failed (%s, status %d); commanding zero velocity",
      mpc_output.status_message.c_str(), mpc_output.solver_status);
    return;
  }

  const auto motion_command = motion_command_policy_.calculate(
    reference.trajectory.reference_linear_velocity,
    reference.trajectory.reference_angular_velocity,
    reference.lateral_error,
    reference.heading_error,
    mpc_output.correction(0),
    mpc_output.correction(1));

  geometry_msgs::msg::Twist command;
  command.linear.x = motion_command.linear_command;
  command.angular.z = motion_command.angular_command;
  velocity_publisher_->publish(command);
  log_sample(stamp_seconds, reference, mpc_output, motion_command);
}

void MpcNode::log_sample(
  double stamp_seconds,
  const my_robot_controller::TrajectoryReference & reference,
  const my_robot_controller::LinearMpcOutput & mpc_output,
  const my_robot_controller::MotionCommand & motion_command)
{
  // Include solver diagnostics alongside the same state/reference/command
  // fields used by PID and LQR, so timing failures are visible in analysis.
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
    mpc_output.correction(0) << ',' << mpc_output.correction(1) << ',' <<
    motion_command.linear_feedforward_command << ',' <<
    motion_command.angular_feedforward_command << ',' <<
    (mpc_output.solved ? 1 : 0) << ',' << mpc_output.solver_status << ',' <<
    mpc_output.iterations << ',' << mpc_output.objective << ',' <<
    mpc_output.solve_time_seconds << ',' <<
    mpc_controller_.config().prediction_horizon_steps << ',' <<
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
  // Standard ROS lifecycle with exception-to-exit-code conversion.
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<MpcNode>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mpc_node"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
