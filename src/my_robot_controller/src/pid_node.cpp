#include "my_robot_controller/pid_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <stdexcept>

PidNode::PidNode()
: Node("pid_node")
{
  // -----------------------------------------------------------------------
  // Experiment files
  // -----------------------------------------------------------------------
  // The launch file provides a package-installed default track. Keeping both
  // paths as parameters allows each run to select its input and output without
  // recompiling or depending on the container's directory layout.
  declare_parameter<std::string>("csv_path", "");
  declare_parameter<std::string>("output_csv_path", "robot_actual_trajectory.csv");
  declare_parameter<std::string>("controller_mode", "cascade");

  // -----------------------------------------------------------------------
  // PID gains
  // -----------------------------------------------------------------------
  // Separate gain sets are needed because distance is measured in metres and
  // heading is measured in radians; their dynamics and output units differ.
  declare_parameter<double>("linear_kp", 1.0);
  declare_parameter<double>("linear_ki", 0.1);
  declare_parameter<double>("linear_kd", 0.2);
  declare_parameter<double>("linear_integral_limit", 1.0);
  declare_parameter<double>("angular_kp", 1.5);
  declare_parameter<double>("angular_ki", 0.0);
  declare_parameter<double>("angular_kd", 0.3);
  declare_parameter<double>("angular_integral_limit", 1.0);

  // -----------------------------------------------------------------------
  // Cascaded PID gains
  // -----------------------------------------------------------------------
  // The outer loop maps signed cross-track error to a heading correction. The
  // inner loop maps corrected heading error to angular-velocity feedback.
  declare_parameter<double>("cascade_cross_track_kp", 1.2);
  declare_parameter<double>("cascade_cross_track_ki", 0.0);
  declare_parameter<double>("cascade_cross_track_kd", 0.15);
  declare_parameter<double>("cascade_cross_track_integral_limit", 0.5);
  declare_parameter<double>("cascade_heading_kp", 2.5);
  declare_parameter<double>("cascade_heading_ki", 0.0);
  declare_parameter<double>("cascade_heading_kd", 0.15);
  declare_parameter<double>("cascade_heading_integral_limit", 0.5);
  declare_parameter<double>("cascade_max_heading_correction", 0.7);
  declare_parameter<double>("cascade_cross_track_speed_gain", 1.5);

  // -----------------------------------------------------------------------
  // Path-following, timing, and actuator constraints
  // -----------------------------------------------------------------------
  // lookahead_points chooses the steering target. search_window limits the
  // closest-point scan and prevents path progress from jumping across distant
  // sections that happen to overlap geometrically.
  declare_parameter<int>("lookahead_points", 5);
  declare_parameter<int>("search_window", 20);
  declare_parameter<double>("reference_linear_velocity", 0.4);
  declare_parameter<double>("curvature_speed_gain", 0.5);
  declare_parameter<double>("endpoint_slowdown_distance", 0.5);
  declare_parameter<double>("maximum_reference_curvature", 5.0);
  declare_parameter<double>("nominal_control_frequency", 30.0);
  declare_parameter<double>("max_control_dt", 0.2);
  declare_parameter<double>("goal_tolerance", 0.08);
  declare_parameter<double>("max_linear_velocity", 1.0);
  declare_parameter<double>("max_angular_velocity", 1.5);
  declare_parameter<double>("odom_timeout", 2.0);
  declare_parameter<double>("startup_settling_time", 1.0);

  // Validate integer/rate settings before converting signed parameter values
  // to unsigned indices.
  const auto lookahead = get_parameter("lookahead_points").as_int();
  const auto search_window = get_parameter("search_window").as_int();
  const double frequency = get_parameter("nominal_control_frequency").as_double();
  if (lookahead < 1 || search_window < 1 || frequency <= 0.0) {
    throw std::runtime_error(
            "lookahead_points, search_window, and nominal_control_frequency must be positive");
  }

  lookahead_points_ = static_cast<std::size_t>(lookahead);
  nominal_dt_ = 1.0 / frequency;
  max_control_dt_ = get_parameter("max_control_dt").as_double();
  goal_tolerance_ = get_parameter("goal_tolerance").as_double();
  max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
  odom_timeout_ = get_parameter("odom_timeout").as_double();
  startup_settling_time_ = get_parameter("startup_settling_time").as_double();

  const std::string controller_mode = get_parameter("controller_mode").as_string();
  if (controller_mode == "lookahead") {
    controller_mode_ = ControllerMode::kLookahead;
  } else if (controller_mode == "cascade") {
    controller_mode_ = ControllerMode::kCascade;
  } else {
    throw std::runtime_error("controller_mode must be either 'lookahead' or 'cascade'");
  }

  // Zero or negative values would make timing, completion, or clamping
  // undefined, so reject the entire configuration before ROS I/O starts.
  if (max_control_dt_ <= 0.0 || goal_tolerance_ <= 0.0 ||
    max_linear_velocity_ <= 0.0 || max_angular_velocity_ <= 0.0 ||
    odom_timeout_ <= 0.0 || startup_settling_time_ < 0.0)
  {
    throw std::runtime_error(
            "Timing, tolerance, timeout, and velocity limits must be positive; "
            "startup settling time must be non-negative");
  }

  // Apply parameter values and start both controllers with empty memory.
  linear_pid_.configure(
    get_parameter("linear_kp").as_double(),
    get_parameter("linear_ki").as_double(),
    get_parameter("linear_kd").as_double(),
    get_parameter("linear_integral_limit").as_double());
  angular_pid_.configure(
    get_parameter("angular_kp").as_double(),
    get_parameter("angular_ki").as_double(),
    get_parameter("angular_kd").as_double(),
    get_parameter("angular_integral_limit").as_double());

  // Configure the cascaded controller even in lookahead mode. This validates a
  // complete YAML profile at startup and makes mode changes reproducible rather
  // than dependent on hidden constructor defaults.
  my_robot_controller::CascadedPidConfig cascade_config;
  cascade_config.cross_track_kp = get_parameter("cascade_cross_track_kp").as_double();
  cascade_config.cross_track_ki = get_parameter("cascade_cross_track_ki").as_double();
  cascade_config.cross_track_kd = get_parameter("cascade_cross_track_kd").as_double();
  cascade_config.cross_track_integral_limit =
    get_parameter("cascade_cross_track_integral_limit").as_double();
  cascade_config.heading_kp = get_parameter("cascade_heading_kp").as_double();
  cascade_config.heading_ki = get_parameter("cascade_heading_ki").as_double();
  cascade_config.heading_kd = get_parameter("cascade_heading_kd").as_double();
  cascade_config.heading_integral_limit =
    get_parameter("cascade_heading_integral_limit").as_double();
  cascade_config.maximum_heading_correction =
    get_parameter("cascade_max_heading_correction").as_double();
  cascade_config.cross_track_speed_gain =
    get_parameter("cascade_cross_track_speed_gain").as_double();
  cascade_config.maximum_linear_velocity = max_linear_velocity_;
  cascade_config.maximum_angular_velocity = max_angular_velocity_;
  cascaded_pid_.configure(cascade_config);

  // Configure the common geometric reference independently of PID gains. The
  // checkpointed lookahead controller does not use reference speed or
  // curvature for commands yet, but logging them now freezes the interface
  // that cascade PID, LQR, and MPC will share.
  my_robot_controller::PathReferenceConfig reference_config;
  reference_config.search_window = static_cast<std::size_t>(search_window);
  reference_config.nominal_linear_velocity =
    get_parameter("reference_linear_velocity").as_double();
  reference_config.curvature_speed_gain =
    get_parameter("curvature_speed_gain").as_double();
  reference_config.endpoint_slowdown_distance =
    get_parameter("endpoint_slowdown_distance").as_double();
  reference_config.maximum_abs_curvature =
    get_parameter("maximum_reference_curvature").as_double();
  reference_manager_.configure(reference_config);

  const std::string csv_path = get_parameter("csv_path").as_string();
  const std::string output_path = get_parameter("output_csv_path").as_string();
  if (output_path.empty()) {
    throw std::runtime_error("Parameter 'output_csv_path' must not be empty");
  }
  if (csv_path == output_path) {
    throw std::runtime_error("Input and output CSV paths must be different");
  }

  reference_manager_.load_csv(csv_path);
  RCLCPP_INFO(
    get_logger(), "Loaded %zu waypoints (%.3f m) from %s",
    reference_manager_.waypoint_count(), reference_manager_.total_length(), csv_path.c_str());
  RCLCPP_INFO(get_logger(), "Controller mode: %s", controller_mode.c_str());

  // Truncate by design: every launch represents one experiment. Users should
  // provide a unique output_csv_path when retaining multiple benchmark runs.
  trajectory_csv_.open(output_path, std::ios::out | std::ios::trunc);
  if (!trajectory_csv_.is_open()) {
    throw std::runtime_error("Could not create trajectory CSV: " + output_path);
  }
  trajectory_csv_ << std::setprecision(10);
  // Log actual state, the closest projected path reference, both definitions
  // of heading error, and controller outputs. The distinction is important:
  // path_heading_error is the common evaluation error. control_heading_error
  // records either the lookahead bearing error or the cascade inner-loop error.
  trajectory_csv_ <<
    "time,actual_x,actual_y,actual_yaw,reference_x,reference_y,reference_yaw,"
    "cross_track_error,path_heading_error,control_heading_error,linear_command,"
    "angular_command,waypoint_index,segment_index,segment_fraction,path_progress,"
    "remaining_path_length,reference_curvature,reference_linear_velocity,"
    "reference_angular_velocity,desired_heading,heading_correction,"
    "cross_track_pid_output,heading_pid_output,heading_speed_factor,"
    "cross_track_speed_factor\n";
  RCLCPP_INFO(get_logger(), "Writing experiment data to %s", output_path.c_str());

  // Relative topic names remain namespace/remapping friendly while resolving
  // to /cmd_vel and /odometry/filtered in the default root namespace.
  velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered", 10,
    std::bind(&PidNode::odom_callback, this, std::placeholders::_1));

  // Control is odometry-driven; this wall timer is only an independent safety
  // watchdog for a failed EKF or disconnected sensor stream.
  last_odom_wall_time_ = std::chrono::steady_clock::now();
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&PidNode::watchdog_callback, this));
}

PidNode::~PidNode()
{
  // Request a final zero command and close the stream so buffered experiment
  // rows are written even during an orderly shutdown.
  stop();
  if (trajectory_csv_.is_open()) {
    trajectory_csv_.close();
  }
}

void PidNode::publish_stop()
{
  if (!velocity_publisher_) {
    return;
  }

  // A default-constructed Twist has all six velocity components set to zero.
  velocity_publisher_->publish(geometry_msgs::msg::Twist());
  stop_sent_ = true;
}

void PidNode::stop()
{
  publish_stop();
}

void PidNode::reset_controllers()
{
  // Reset both selectable implementations. This keeps a recovered experiment
  // independent of which mode is active and avoids stale memory if runtime
  // mode selection is added in a later milestone.
  linear_pid_.reset();
  angular_pid_.reset();
  cascaded_pid_.reset();
}

void PidNode::watchdog_callback()
{
  // Before the first odometry sample there is no active command to cancel.
  // After completion or a previous timeout, repeatedly publishing stop adds no
  // safety benefit and would only create unnecessary traffic.
  if (!odom_received_ || track_complete_ || stop_sent_) {
    return;
  }

  // steady_clock is monotonic and independent of /clock, so this protection
  // still works if Gazebo pauses or the ROS simulation clock disappears.
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_odom_wall_time_).count();
  if (elapsed > odom_timeout_) {
    RCLCPP_WARN(get_logger(), "Odometry timed out; commanding zero velocity");
    publish_stop();

    // Old PID memory is invalid after an unknown-duration data interruption.
    reset_controllers();
  }
}

void PidNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // The EKF publishes the planar pose in the odom frame. two_d_mode in the EKF
  // configuration constrains the state to x, y, and yaw for this controller.
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  // ROS orientations are quaternions. Extract yaw directly because roll and
  // pitch do not participate in differential-drive planar control.
  const double qx = msg->pose.pose.orientation.x;
  const double qy = msg->pose.pose.orientation.y;
  const double qz = msg->pose.pose.orientation.z;
  const double qw = msg->pose.pose.orientation.w;
  current_theta_ = std::atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz));

  // Reject corrupted estimator output before it enters geometric or PID math.
  if (!std::isfinite(current_x_) || !std::isfinite(current_y_) ||
    !std::isfinite(current_theta_))
  {
    RCLCPP_ERROR(get_logger(), "Received non-finite odometry; commanding zero velocity");
    publish_stop();
    return;
  }

  const double stamp_seconds = rclcpp::Time(msg->header.stamp).seconds();

  // DDS endpoint discovery is asynchronous. During earlier experiments the
  // controller sometimes logged nonzero commands for almost two seconds before
  // Gazebo's subscriber was connected. Do not start experiment time, path
  // progress, or PID memory until the complete downstream command path exists.
  if (!command_transport_connected_) {
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
      get_logger(), "Command path connected; holding zero command for %.3f s of simulation time",
      startup_settling_time_);
    return;
  }

  // Allow contacts, the wheel-velocity servo, and the estimator to reach a
  // repeatable stationary state after transport discovery. This interval is
  // simulation-time based and is excluded from every logged experiment metric.
  if (!command_path_ready_) {
    last_odom_wall_time_ = std::chrono::steady_clock::now();
    publish_stop();
    if (stamp_seconds - command_connection_stamp_seconds_ < startup_settling_time_) {
      return;
    }

    command_path_ready_ = true;
    odom_received_ = false;
    stop_sent_ = false;
    reset_controllers();
    reference_manager_.reset_progress();
    RCLCPP_INFO(get_logger(), "Startup settling complete; starting the path-tracking experiment");
  }

  // Compute dt from ROS timestamps rather than the host clock. Therefore the
  // controller follows simulation time when Gazebo runs slower/faster than
  // real time. The nominal period is used for the first available sample.
  double dt = nominal_dt_;
  if (!odom_received_) {
    first_stamp_seconds_ = stamp_seconds;
  } else {
    dt = stamp_seconds - previous_stamp_seconds_;
    if (dt <= 0.0) {
      // Repeated or backwards timestamps can occur during a simulation reset.
      // Ignoring them prevents division by zero and reversed integration.
      RCLCPP_WARN(get_logger(), "Ignoring odometry with a non-increasing timestamp");
      return;
    }
    if (dt > max_control_dt_) {
      // A large gap would produce misleading integral and derivative terms.
      // Reset their memory and use one nominal interval for safe recovery.
      RCLCPP_WARN(
        get_logger(), "Odometry gap %.3f s exceeded max_control_dt; resetting PID memory", dt);
      reset_controllers();
      dt = nominal_dt_;
    }
  }

  // Mark the stream healthy before control so the watchdog will measure from
  // this sample. A recovered stream is allowed to publish commands again.
  previous_stamp_seconds_ = stamp_seconds;
  odom_received_ = true;
  stop_sent_ = false;
  last_odom_wall_time_ = std::chrono::steady_clock::now();

  if (track_complete_) {
    // Continue reinforcing zero velocity if odometry arrives after completion.
    publish_stop();
    return;
  }

  control_loop(stamp_seconds, dt);
}

void PidNode::control_loop(double stamp_seconds, double dt)
{
  // -----------------------------------------------------------------------
  // 1. Obtain the shared path-relative reference
  // -----------------------------------------------------------------------
  // Projection, signed error, arc-length progress, curvature, and common speed
  // policy are centralized here so later controllers cannot define them
  // differently. The manager also protects progress at path intersections.
  const my_robot_controller::PathReference reference =
    reference_manager_.update(current_x_, current_y_, current_theta_);

  // -----------------------------------------------------------------------
  // 2. Prepare the mode-specific target and completion condition
  // -----------------------------------------------------------------------
  // The legacy controller still needs a waypoint target. Calculating it here
  // also provides meaningful terminal diagnostics for its last CSV row.
  std::size_t target_index = reference.closest_waypoint_index;
  double distance_error = reference.distance_to_goal;
  double target_angle = reference.path.heading;
  double target_heading_error = reference.heading_error;
  bool has_final_progress = reference.path.remaining_length <= goal_tolerance_;

  if (controller_mode_ == ControllerMode::kLookahead) {
    target_index = reference_manager_.lookahead_waypoint_index(lookahead_points_);
    const my_robot_controller::Point2D & target = reference_manager_.waypoint(target_index);
    const double x_error = target.x - current_x_;
    const double y_error = target.y - current_y_;
    distance_error = std::hypot(x_error, y_error);
    target_angle = std::atan2(y_error, x_error);
    target_heading_error =
      my_robot_controller::wrap_angle(target_angle - current_theta_);
    has_final_progress = target_index + 1 == reference_manager_.waypoint_count();
  }

  // Distance alone is insufficient for a circle or figure eight because its
  // endpoint is also close to the start. Requiring path progress prevents an
  // immediate false completion. Cascade uses continuous remaining arc length;
  // the baseline retains its checkpointed final-lookahead condition.
  if (has_final_progress && reference.distance_to_goal <= goal_tolerance_) {
    track_complete_ = true;
    publish_stop();

    // The terminal pose and zero commands are part of the dataset. Flush here
    // so MATLAB can read a completed experiment even while the node remains up.
    ControllerDiagnostics diagnostics;
    diagnostics.control_heading_error =
      controller_mode_ == ControllerMode::kCascade ?
      reference.heading_error : target_heading_error;
    diagnostics.desired_heading =
      controller_mode_ == ControllerMode::kCascade ?
      reference.path.heading : target_angle;
    log_sample(stamp_seconds, reference, diagnostics, 0.0, 0.0);
    trajectory_csv_.flush();
    RCLCPP_INFO(
      get_logger(),
      "Track complete at simulation time %.6f s at (%.3f, %.3f); final error %.3f m",
      stamp_seconds, current_x_, current_y_, reference.distance_to_goal);
    return;
  }

  // -----------------------------------------------------------------------
  // 3. Calculate commands using the selected PID architecture
  // -----------------------------------------------------------------------
  ControllerDiagnostics diagnostics;
  double linear_command = 0.0;
  double angular_command = 0.0;

  if (controller_mode_ == ControllerMode::kCascade) {
    // Outer loop: cross-track error -> bounded heading correction.
    // Inner loop: corrected heading error -> yaw-rate feedback.
    // The common curvature yaw rate is added as feedforward inside calculate().
    const my_robot_controller::CascadedPidOutput output = cascaded_pid_.calculate(
      reference.cross_track_error,
      reference.path.heading,
      current_theta_,
      reference.path.reference_linear_velocity,
      reference.path.reference_angular_velocity,
      dt);

    linear_command = output.linear_command;
    angular_command = output.angular_command;
    diagnostics.control_heading_error = output.heading_error;
    diagnostics.desired_heading = output.desired_heading;
    diagnostics.heading_correction = output.heading_correction;
    diagnostics.cross_track_pid_output = output.cross_track_pid_output;
    diagnostics.heading_pid_output = output.heading_pid_output;
    diagnostics.heading_speed_factor = output.heading_speed_factor;
    diagnostics.cross_track_speed_factor = output.cross_track_speed_factor;
  } else {
    // Checkpointed lookahead baseline: distance error controls translation and
    // bearing-to-waypoint error controls rotation. Its implementation remains
    // available so cascade improvements can be measured against it.
    if (std::abs(target_heading_error) > 0.35) {
      linear_pid_.reset();
    }

    linear_command = linear_pid_.calculate(distance_error, dt);
    const double angular_pid_output = angular_pid_.calculate(target_heading_error, dt);

    // At 90 degrees or more, stop translating and turn in place.
    const double heading_speed_factor =
      std::max(0.0, std::cos(target_heading_error));
    linear_command *= heading_speed_factor;
    linear_command = std::clamp(linear_command, 0.0, max_linear_velocity_);
    angular_command = std::clamp(
      angular_pid_output, -max_angular_velocity_, max_angular_velocity_);

    diagnostics.control_heading_error = target_heading_error;
    diagnostics.desired_heading = target_angle;
    diagnostics.heading_pid_output = angular_pid_output;
    diagnostics.heading_speed_factor = heading_speed_factor;
  }

  // Differential-drive motion uses forward velocity (x) and yaw rate (z).
  geometry_msgs::msg::Twist command;
  command.linear.x = linear_command;
  command.angular.z = angular_command;
  velocity_publisher_->publish(command);

  // Log the exact state, reference, errors, and commands from this update.
  log_sample(stamp_seconds, reference, diagnostics, linear_command, angular_command);
}

void PidNode::log_sample(
  double stamp_seconds, const my_robot_controller::PathReference & reference,
  const ControllerDiagnostics & diagnostics,
  double linear_command, double angular_command)
{
  // Time is relative to the first odometry sample, making separate runs easy
  // to overlay in MATLAB. setprecision(10), configured at file creation, keeps
  // enough numerical precision for error and derivative calculations.
  trajectory_csv_ <<
    stamp_seconds - first_stamp_seconds_ << ',' <<
    current_x_ << ',' << current_y_ << ',' << current_theta_ << ',' <<
    reference.path.position.x << ',' << reference.path.position.y << ',' <<
    reference.path.heading << ',' << reference.cross_track_error << ',' <<
    reference.heading_error << ',' << diagnostics.control_heading_error << ',' <<
    linear_command << ',' << angular_command << ',' <<
    reference.closest_waypoint_index << ',' << reference.path.segment_index << ',' <<
    reference.path.segment_fraction << ',' << reference.path.progress << ',' <<
    reference.path.remaining_length << ',' << reference.path.curvature << ',' <<
    reference.path.reference_linear_velocity << ',' <<
    reference.path.reference_angular_velocity << ',' << diagnostics.desired_heading << ',' <<
    diagnostics.heading_correction << ',' << diagnostics.cross_track_pid_output << ',' <<
    diagnostics.heading_pid_output << ',' << diagnostics.heading_speed_factor << ',' <<
    diagnostics.cross_track_speed_factor << '\n';
}

int main(int argc, char ** argv)
{
  // rclcpp owns signal handling and ROS middleware initialization.
  rclcpp::init(argc, argv);
  int result = 0;

  try {
    // Constructor exceptions indicate invalid parameters/files and are treated
    // as startup failures rather than allowing a partially configured robot.
    auto node = std::make_shared<PidNode>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pid_node"), "%s", error.what());
    result = 1;
  }

  // shutdown() is only necessary if an exception occurred before ROS began its
  // normal signal-driven shutdown sequence.
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
