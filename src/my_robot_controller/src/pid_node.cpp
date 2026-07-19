#include "my_robot_controller/pid_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace
{
// Use a local constant instead of the non-standard M_PI macro so this source
// behaves consistently across compilers.
constexpr double kPi = 3.14159265358979323846;

// Convert an arbitrary angular difference to the shortest signed rotation.
// Without wrapping, crossing from +pi to -pi would look like a nearly 2*pi
// error and could make the robot turn the long way around.
double wrap_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

// Parse one CSV field and reject malformed, trailing, NaN, or infinite data.
// Failing during startup is safer than commanding the robot from a partially
// loaded path. The line number is included to make track files easy to debug.
double parse_finite_double(const std::string & value, std::size_t line_number)
{
  std::size_t consumed = 0;
  double parsed = 0.0;

  try {
    parsed = std::stod(value, &consumed);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "Invalid number in waypoint CSV at line " + std::to_string(line_number));
  }

  while (consumed < value.size() &&
    std::isspace(static_cast<unsigned char>(value[consumed])))
  {
    ++consumed;
  }

  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error(
            "Invalid number in waypoint CSV at line " + std::to_string(line_number));
  }

  return parsed;
}
}  // namespace

void PIDController::configure(double kp, double ki, double kd, double max_i)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  // Treat a negative limit as its magnitude so clamp bounds remain ordered.
  max_i_ = std::abs(max_i);
  reset();
}

double PIDController::calculate(double error, double dt)
{
  // Invalid data must never be propagated into a velocity command.
  if (!std::isfinite(error) || !std::isfinite(dt) || dt <= 0.0) {
    return 0.0;
  }

  // Integral term with anti-windup. This bounds the stored state, not merely
  // the final output, so a temporarily stuck robot can recover promptly.
  integral_ = std::clamp(integral_ + error * dt, -max_i_, max_i_);

  // On the first update after reset there is no previous measurement. Using
  // zero derivative avoids a large artificial derivative kick.
  double derivative = 0.0;
  if (has_previous_error_) {
    derivative = (error - prev_error_) / dt;
  }

  prev_error_ = error;
  has_previous_error_ = true;

  return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void PIDController::reset()
{
  // A reset is used after discontinuous timing and during sharp turns, where
  // old accumulated error is no longer representative of the current motion.
  integral_ = 0.0;
  prev_error_ = 0.0;
  has_previous_error_ = false;
}

void PidNode::load_waypoints(const std::string & file_path)
{
  // csv_path is deliberately required. Silently choosing a missing or stale
  // benchmark would invalidate comparisons between different controllers.
  if (file_path.empty()) {
    throw std::runtime_error("Parameter 'csv_path' must not be empty");
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open waypoint CSV: " + file_path);
  }

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }

    // Benchmark tracks are headerless and must contain exactly "x,y". Extra
    // columns are rejected so a trajectory CSV cannot be used accidentally.
    std::stringstream stream(line);
    std::string x_value;
    std::string y_value;
    std::string extra_value;
    if (!std::getline(stream, x_value, ',') ||
      !std::getline(stream, y_value, ',') ||
      std::getline(stream, extra_value, ','))
    {
      throw std::runtime_error(
              "Expected exactly two columns in waypoint CSV at line " +
              std::to_string(line_number));
    }

    waypoints_.emplace_back(
      parse_finite_double(x_value, line_number),
      parse_finite_double(y_value, line_number));
  }

  if (waypoints_.size() < 2) {
    throw std::runtime_error("Waypoint CSV must contain at least two valid points");
  }

  RCLCPP_INFO(
    get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(), file_path.c_str());
}

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
  // Path-following, timing, and actuator constraints
  // -----------------------------------------------------------------------
  // lookahead_points chooses the steering target. search_window limits the
  // closest-point scan and prevents path progress from jumping across distant
  // sections that happen to overlap geometrically.
  declare_parameter<int>("lookahead_points", 5);
  declare_parameter<int>("search_window", 20);
  declare_parameter<double>("nominal_control_frequency", 30.0);
  declare_parameter<double>("max_control_dt", 0.2);
  declare_parameter<double>("goal_tolerance", 0.08);
  declare_parameter<double>("max_linear_velocity", 1.0);
  declare_parameter<double>("max_angular_velocity", 1.5);
  declare_parameter<double>("odom_timeout", 2.0);

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
  search_window_ = static_cast<std::size_t>(search_window);
  nominal_dt_ = 1.0 / frequency;
  max_control_dt_ = get_parameter("max_control_dt").as_double();
  goal_tolerance_ = get_parameter("goal_tolerance").as_double();
  max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
  odom_timeout_ = get_parameter("odom_timeout").as_double();

  // Zero or negative values would make timing, completion, or clamping
  // undefined, so reject the entire configuration before ROS I/O starts.
  if (max_control_dt_ <= 0.0 || goal_tolerance_ <= 0.0 ||
    max_linear_velocity_ <= 0.0 || max_angular_velocity_ <= 0.0 ||
    odom_timeout_ <= 0.0)
  {
    throw std::runtime_error("Timing, tolerance, timeout, and velocity limits must be positive");
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

  const std::string csv_path = get_parameter("csv_path").as_string();
  const std::string output_path = get_parameter("output_csv_path").as_string();
  if (output_path.empty()) {
    throw std::runtime_error("Parameter 'output_csv_path' must not be empty");
  }
  if (csv_path == output_path) {
    throw std::runtime_error("Input and output CSV paths must be different");
  }

  load_waypoints(csv_path);

  // Truncate by design: every launch represents one experiment. Users should
  // provide a unique output_csv_path when retaining multiple benchmark runs.
  trajectory_csv_.open(output_path, std::ios::out | std::ios::trunc);
  if (!trajectory_csv_.is_open()) {
    throw std::runtime_error("Could not create trajectory CSV: " + output_path);
  }
  trajectory_csv_ << std::setprecision(10);
  // Log actual state, the closest projected path reference, both definitions
  // of heading error, and controller outputs. The distinction is important:
  // path_heading_error is useful for evaluation, while target_heading_error
  // is the lookahead error actually used by the angular PID.
  trajectory_csv_ <<
    "time,actual_x,actual_y,actual_yaw,reference_x,reference_y,reference_yaw,"
    "cross_track_error,path_heading_error,target_heading_error,linear_command,"
    "angular_command,waypoint_index\n";
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
    linear_pid_.reset();
    angular_pid_.reset();
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

  // Compute dt from ROS timestamps rather than the host clock. Therefore the
  // controller follows simulation time when Gazebo runs slower/faster than
  // real time. The nominal period is used for the first available sample.
  const double stamp_seconds = rclcpp::Time(msg->header.stamp).seconds();
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
      linear_pid_.reset();
      angular_pid_.reset();
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
  // 1. Monotonic closest-waypoint search
  // -----------------------------------------------------------------------
  // Scan only forward from the remembered path index. This avoids getting
  // trapped by a waypoint that was missed between samples and prevents a
  // figure-eight crossing from making progress jump to an unrelated branch.
  double min_distance_squared = std::numeric_limits<double>::infinity();
  std::size_t closest_index = current_wp_index_;
  const std::size_t search_end = std::min(
    current_wp_index_ + search_window_ + 1, waypoints_.size());

  for (std::size_t i = current_wp_index_; i < search_end; ++i) {
    const double dx = waypoints_[i].first - current_x_;
    const double dy = waypoints_[i].second - current_y_;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < min_distance_squared) {
      min_distance_squared = distance_squared;
      closest_index = i;
    }
  }
  current_wp_index_ = closest_index;

  // -----------------------------------------------------------------------
  // 2. Project the robot onto the local path segment
  // -----------------------------------------------------------------------
  // Evaluation requires geometric path error, not comparison with an equally
  // numbered CSV row. Projection finds the closest point on the local segment;
  // clamping keeps that point between the segment endpoints.
  const std::size_t segment_start = std::min(current_wp_index_, waypoints_.size() - 2);
  const auto & segment_a = waypoints_[segment_start];
  const auto & segment_b = waypoints_[segment_start + 1];
  const double segment_x = segment_b.first - segment_a.first;
  const double segment_y = segment_b.second - segment_a.second;
  const double segment_length_squared = segment_x * segment_x + segment_y * segment_y;
  double projection = 0.0;
  if (segment_length_squared > 0.0) {
    projection = std::clamp(
      ((current_x_ - segment_a.first) * segment_x +
      (current_y_ - segment_a.second) * segment_y) / segment_length_squared,
      0.0, 1.0);
  }
  const double reference_x = segment_a.first + projection * segment_x;
  const double reference_y = segment_a.second + projection * segment_y;
  const double reference_yaw = std::atan2(segment_y, segment_x);
  const double path_heading_error = wrap_angle(reference_yaw - current_theta_);
  const double segment_length = std::sqrt(segment_length_squared);

  // Signed cross-track error is positive when the robot is to the left of the
  // path tangent and negative when it is to the right.
  const double cross_track_error = segment_length > 0.0 ?
    (segment_x * (current_y_ - reference_y) -
    segment_y * (current_x_ - reference_x)) / segment_length : 0.0;

  // -----------------------------------------------------------------------
  // 3. Select a forward lookahead target
  // -----------------------------------------------------------------------
  // Steering toward a point ahead of the closest point is smoother and less
  // sensitive to waypoint spacing than steering back toward the closest point.
  const std::size_t target_index = std::min(
    current_wp_index_ + lookahead_points_, waypoints_.size() - 1);
  const double target_x = waypoints_[target_index].first;
  const double target_y = waypoints_[target_index].second;
  const double x_error = target_x - current_x_;
  const double y_error = target_y - current_y_;
  const double distance_error = std::hypot(x_error, y_error);
  const double target_angle = std::atan2(y_error, x_error);
  const double target_heading_error = wrap_angle(target_angle - current_theta_);

  // -----------------------------------------------------------------------
  // 4. Detect completion near the end of the path
  // -----------------------------------------------------------------------
  // Distance alone is insufficient for closed paths such as a figure eight,
  // whose endpoint may be close to its start. Requiring final-path progress
  // prevents immediate false completion at such intersections.
  const auto & goal = waypoints_.back();
  const double goal_dx = goal.first - current_x_;
  const double goal_dy = goal.second - current_y_;
  const double goal_distance = std::hypot(goal_dx, goal_dy);
  const std::size_t final_approach_index =
    waypoints_.size() > lookahead_points_ ?
    waypoints_.size() - lookahead_points_ - 1 : 0;

  if (current_wp_index_ >= final_approach_index && goal_distance <= goal_tolerance_) {
    track_complete_ = true;
    publish_stop();

    // The terminal pose and zero commands are part of the dataset. Flush here
    // so MATLAB can read a completed experiment even while the node remains up.
    log_sample(
      stamp_seconds, reference_x, reference_y, reference_yaw, cross_track_error,
      path_heading_error, target_heading_error, 0.0, 0.0);
    trajectory_csv_.flush();
    RCLCPP_INFO(
      get_logger(), "Track complete at (%.3f, %.3f); final error %.3f m",
      current_x_, current_y_, goal_distance);
    return;
  }

  // -----------------------------------------------------------------------
  // 5. Calculate coupled linear and angular PID commands
  // -----------------------------------------------------------------------
  // During a sharp turn, discard accumulated longitudinal error so it cannot
  // create a surge when the robot finishes rotating.
  if (std::abs(target_heading_error) > 0.35) {
    linear_pid_.reset();
  }

  double linear_command = linear_pid_.calculate(distance_error, dt);
  double angular_command = angular_pid_.calculate(target_heading_error, dt);

  // The cosine coupling progressively slows translation as steering error
  // grows. At 90 degrees or more it becomes zero, making the robot turn in
  // place rather than drive away from the path.
  linear_command *= std::max(0.0, std::cos(target_heading_error));

  // This path follower never intentionally reverses. Both commands are also
  // clamped to the actuator limits shared by future controller comparisons.
  linear_command = std::clamp(linear_command, 0.0, max_linear_velocity_);
  angular_command = std::clamp(
    angular_command, -max_angular_velocity_, max_angular_velocity_);

  // Differential-drive motion uses forward velocity (x) and yaw rate (z).
  geometry_msgs::msg::Twist command;
  command.linear.x = linear_command;
  command.angular.z = angular_command;
  velocity_publisher_->publish(command);

  // Log the exact state, reference, errors, and commands from this update.
  log_sample(
    stamp_seconds, reference_x, reference_y, reference_yaw, cross_track_error,
    path_heading_error, target_heading_error, linear_command, angular_command);
}

void PidNode::log_sample(
  double stamp_seconds, double reference_x, double reference_y, double reference_yaw,
  double cross_track_error, double path_heading_error, double target_heading_error,
  double linear_command, double angular_command)
{
  // Time is relative to the first odometry sample, making separate runs easy
  // to overlay in MATLAB. setprecision(10), configured at file creation, keeps
  // enough numerical precision for error and derivative calculations.
  trajectory_csv_ <<
    stamp_seconds - first_stamp_seconds_ << ',' <<
    current_x_ << ',' << current_y_ << ',' << current_theta_ << ',' <<
    reference_x << ',' << reference_y << ',' << reference_yaw << ',' <<
    cross_track_error << ',' << path_heading_error << ',' << target_heading_error << ',' <<
    linear_command << ',' << angular_command << ',' << current_wp_index_ << '\n';
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
