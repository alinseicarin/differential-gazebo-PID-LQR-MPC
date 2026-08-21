#include "my_robot_controller/trajectory_reference_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace
{
constexpr double kMinimumPositiveVelocity = 1.0e-9;
}

namespace my_robot_controller
{

TrajectoryReferenceManager::TrajectoryReferenceManager(
  const TrajectoryReferenceConfig & config)
: path_manager_(config.path)
{
  configure(config);
}

void TrajectoryReferenceManager::validate_config(
  const TrajectoryReferenceConfig & config) const
{
  if (!std::isfinite(config.spatial_step) || config.spatial_step <= 0.0 ||
    !std::isfinite(config.maximum_linear_acceleration) ||
    config.maximum_linear_acceleration <= 0.0 ||
    !std::isfinite(config.maximum_linear_deceleration) ||
    config.maximum_linear_deceleration <= 0.0 ||
    !std::isfinite(config.maximum_reference_angular_velocity) ||
    config.maximum_reference_angular_velocity <= 0.0)
  {
    throw std::invalid_argument(
            "Trajectory spacing, acceleration, deceleration, and yaw-rate limits "
            "must be finite and positive");
  }
}

void TrajectoryReferenceManager::configure(const TrajectoryReferenceConfig & config)
{
  validate_config(config);
  config_ = config;
  path_manager_.configure(config.path);
  if (path_manager_.waypoint_count() >= 2) {
    build_time_profile();
  }
}

void TrajectoryReferenceManager::load_csv(const std::string & file_path)
{
  path_manager_.load_csv(file_path);
  build_time_profile();
}

void TrajectoryReferenceManager::set_path(const std::vector<Point2D> & waypoints)
{
  path_manager_.set_path(waypoints);
  build_time_profile();
}

void TrajectoryReferenceManager::reset_projection()
{
  path_manager_.reset_progress();
}

void TrajectoryReferenceManager::build_time_profile()
{
  // Stage 1: resample the geometric path on an approximately uniform arc-length
  // grid. The original CSV remains untouched; these internal knots make speed
  // constraints independent of the original waypoint density.
  const double length = path_manager_.total_length();
  if (!std::isfinite(length) || length <= 0.0) {
    throw std::logic_error("Cannot time-parameterize an empty or zero-length path");
  }

  const std::size_t interval_count = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(length / config_.spatial_step)));
  const double interval_length = length / static_cast<double>(interval_count);

  time_knots_.assign(interval_count + 1, {});
  for (std::size_t index = 0; index <= interval_count; ++index) {
    TimeKnot & knot = time_knots_[index];
    knot.progress = index == interval_count ?
      length : static_cast<double>(index) * interval_length;

    // Start from the curvature-aware geometric limit, then enforce
    // |omega_ref|=|kappa*v_ref| <= maximum_reference_angular_velocity.
    const PathGeometrySample geometry = path_manager_.sample_at_progress(knot.progress);
    double speed_limit = geometry.reference_linear_velocity;
    if (std::abs(geometry.curvature) > kMinimumPositiveVelocity) {
      speed_limit = std::min(
        speed_limit,
        config_.maximum_reference_angular_velocity / std::abs(geometry.curvature));
    }
    knot.linear_velocity = std::max(0.0, speed_limit);
  }

  // The virtual robot starts and ends at rest. Forward/backward passes impose
  // v_next^2 <= v_now^2 + 2*a*ds without requiring an actuator model.
  time_knots_.front().linear_velocity = 0.0;
  time_knots_.back().linear_velocity = 0.0;

  // Stage 2a, forward pass: a knot cannot be faster than a robot accelerating
  // from the already feasible speed of the preceding knot.
  for (std::size_t index = 1; index < time_knots_.size(); ++index) {
    const double ds = time_knots_[index].progress - time_knots_[index - 1].progress;
    const double reachable_speed = std::sqrt(
      time_knots_[index - 1].linear_velocity *
      time_knots_[index - 1].linear_velocity +
      2.0 * config_.maximum_linear_acceleration * ds);
    time_knots_[index].linear_velocity = std::min(
      time_knots_[index].linear_velocity, reachable_speed);
  }

  // Stage 2b, backward pass: a knot cannot be faster than a robot that must
  // still brake to the feasible speed at the following knot (and finally zero).
  for (std::size_t index = time_knots_.size() - 1; index > 0; --index) {
    const double ds = time_knots_[index].progress - time_knots_[index - 1].progress;
    const double braking_speed = std::sqrt(
      time_knots_[index].linear_velocity * time_knots_[index].linear_velocity +
      2.0 * config_.maximum_linear_deceleration * ds);
    time_knots_[index - 1].linear_velocity = std::min(
      time_knots_[index - 1].linear_velocity, braking_speed);
  }

  // Stage 3: integrate travel time along the now-feasible spatial speed
  // profile. The resulting monotonically increasing times form the time law.
  double accumulated_time = 0.0;
  time_knots_.front().time = 0.0;
  for (std::size_t index = 1; index < time_knots_.size(); ++index) {
    const double ds = time_knots_[index].progress - time_knots_[index - 1].progress;
    const double velocity_sum =
      time_knots_[index - 1].linear_velocity + time_knots_[index].linear_velocity;
    if (velocity_sum <= kMinimumPositiveVelocity) {
      throw std::runtime_error(
              "Time parameterization contains an interval with zero reachable speed");
    }

    // Constant acceleration over the interval gives ds = 0.5*(v0+v1)*dt.
    accumulated_time += 2.0 * ds / velocity_sum;
    time_knots_[index].time = accumulated_time;
  }
  duration_ = accumulated_time;
}

PathGeometrySample TrajectoryReferenceManager::sample_at_time(double elapsed_time) const
{
  if (time_knots_.empty()) {
    throw std::logic_error("Cannot sample a trajectory before loading a path");
  }
  if (!std::isfinite(elapsed_time)) {
    throw std::invalid_argument("Trajectory time must be finite");
  }

  const double time = std::clamp(elapsed_time, 0.0, duration_);
  if (time >= duration_) {
    PathGeometrySample result = path_manager_.sample_at_progress(path_manager_.total_length());
    result.reference_linear_velocity = 0.0;
    result.reference_angular_velocity = 0.0;
    return result;
  }

  // Binary search selects the two time knots bracketing the query. The
  // interval is then reconstructed under the same constant-acceleration
  // assumption used to obtain its duration.
  const auto upper = std::upper_bound(
    time_knots_.begin(), time_knots_.end(), time,
    [](double value, const TimeKnot & knot) {return value < knot.time;});
  const std::size_t upper_index = static_cast<std::size_t>(
    std::distance(time_knots_.begin(), upper));
  const std::size_t lower_index = upper_index - 1;
  const TimeKnot & lower = time_knots_[lower_index];
  const TimeKnot & next = time_knots_[upper_index];

  const double interval_time = next.time - lower.time;
  const double local_time = time - lower.time;
  const double acceleration =
    (next.linear_velocity - lower.linear_velocity) / interval_time;
  const double progress = std::clamp(
    lower.progress + lower.linear_velocity * local_time +
    0.5 * acceleration * local_time * local_time,
    lower.progress, next.progress);
  const double linear_velocity = std::max(
    0.0, lower.linear_velocity + acceleration * local_time);

  PathGeometrySample result = path_manager_.sample_at_progress(progress);
  result.reference_linear_velocity = linear_velocity;
  result.reference_angular_velocity = linear_velocity * result.curvature;
  return result;
}

TrajectoryReference TrajectoryReferenceManager::update(
  double elapsed_time, double robot_x, double robot_y, double robot_heading)
{
  if (!std::isfinite(robot_x) || !std::isfinite(robot_y) ||
    !std::isfinite(robot_heading))
  {
    throw std::invalid_argument("Robot pose supplied to trajectory reference is not finite");
  }

  TrajectoryReference reference;
  reference.reference_time = std::clamp(elapsed_time, 0.0, duration_);
  reference.trajectory_complete = elapsed_time >= duration_;
  // These are deliberately different references: trajectory is the prescribed
  // time-indexed virtual robot used for control, while projection is the
  // closest causal path point used for geometric evaluation and completion.
  reference.trajectory = sample_at_time(elapsed_time);
  reference.projection = path_manager_.update(robot_x, robot_y, robot_heading);

  const double global_x_error = reference.trajectory.position.x - robot_x;
  const double global_y_error = reference.trajectory.position.y - robot_y;
  const double cosine = std::cos(robot_heading);
  const double sine = std::sin(robot_heading);

  // Rotate reference-minus-actual position into the actual robot body frame.
  reference.longitudinal_error = cosine * global_x_error + sine * global_y_error;
  reference.lateral_error = -sine * global_x_error + cosine * global_y_error;
  reference.heading_error = wrap_angle(reference.trajectory.heading - robot_heading);
  reference.position_error = std::hypot(global_x_error, global_y_error);
  return reference;
}

std::size_t TrajectoryReferenceManager::lookahead_waypoint_index(
  std::size_t lookahead_points) const
{
  return path_manager_.lookahead_waypoint_index(lookahead_points);
}

const Point2D & TrajectoryReferenceManager::waypoint(std::size_t index) const
{
  return path_manager_.waypoint(index);
}

std::size_t TrajectoryReferenceManager::waypoint_count() const
{
  return path_manager_.waypoint_count();
}

double TrajectoryReferenceManager::total_length() const
{
  return path_manager_.total_length();
}

double TrajectoryReferenceManager::duration() const
{
  return duration_;
}

}  // namespace my_robot_controller
