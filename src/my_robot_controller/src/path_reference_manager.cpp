#include "my_robot_controller/path_reference_manager.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumSegmentLength = 1.0e-9;
constexpr double kProjectionTieTolerance = 1.0e-12;

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

namespace my_robot_controller
{

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

PathReferenceManager::PathReferenceManager(const PathReferenceConfig & config)
{
  configure(config);
}

void PathReferenceManager::validate_config(const PathReferenceConfig & config) const
{
  if (config.search_window < 1) {
    throw std::invalid_argument("Path-reference search_window must be positive");
  }
  if (!std::isfinite(config.nominal_linear_velocity) ||
    config.nominal_linear_velocity < 0.0)
  {
    throw std::invalid_argument("nominal_linear_velocity must be finite and non-negative");
  }
  if (!std::isfinite(config.curvature_speed_gain) || config.curvature_speed_gain < 0.0) {
    throw std::invalid_argument("curvature_speed_gain must be finite and non-negative");
  }
  if (!std::isfinite(config.endpoint_slowdown_distance) ||
    config.endpoint_slowdown_distance < 0.0)
  {
    throw std::invalid_argument("endpoint_slowdown_distance must be finite and non-negative");
  }
  if (!std::isfinite(config.maximum_abs_curvature) || config.maximum_abs_curvature <= 0.0) {
    throw std::invalid_argument("maximum_abs_curvature must be finite and positive");
  }
}

void PathReferenceManager::configure(const PathReferenceConfig & config)
{
  validate_config(config);
  config_ = config;
  if (!waypoints_.empty()) {
    preprocess_path();
    reset_progress();
  }
}

void PathReferenceManager::load_csv(const std::string & file_path)
{
  // The benchmark format is intentionally minimal: every non-empty row must
  // contain exactly x,y. Strict parsing catches accidental headers, missing
  // fields, and malformed tracks before an experiment starts moving the robot.
  if (file_path.empty()) {
    throw std::runtime_error("Path CSV file name must not be empty");
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open waypoint CSV: " + file_path);
  }

  std::vector<Point2D> loaded_waypoints;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }

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

    loaded_waypoints.push_back(
      {parse_finite_double(x_value, line_number),
        parse_finite_double(y_value, line_number)});
  }

  set_path(loaded_waypoints);
}

void PathReferenceManager::set_path(const std::vector<Point2D> & waypoints)
{
  // Programmatic paths pass through the same checks and preprocessing as CSV
  // paths, which keeps unit tests representative of real experiment loading.
  if (waypoints.size() < 2) {
    throw std::invalid_argument("Path must contain at least two waypoints");
  }

  for (const auto & waypoint_value : waypoints) {
    if (!std::isfinite(waypoint_value.x) || !std::isfinite(waypoint_value.y)) {
      throw std::invalid_argument("Path contains a non-finite waypoint");
    }
  }

  waypoints_ = waypoints;
  preprocess_path();
  reset_progress();
}

void PathReferenceManager::preprocess_path()
{
  // Convert raw waypoints into reusable segment geometry once. The 30 Hz
  // control loop can then project and sample without recalculating lengths,
  // headings, or cumulative curvilinear coordinates on every callback.
  segments_.clear();
  cumulative_lengths_.assign(waypoints_.size(), 0.0);
  segments_.reserve(waypoints_.size() - 1);

  double cumulative_length = 0.0;
  for (std::size_t index = 0; index + 1 < waypoints_.size(); ++index) {
    const Point2D & start = waypoints_[index];
    const Point2D & end = waypoints_[index + 1];
    const double delta_x = end.x - start.x;
    const double delta_y = end.y - start.y;
    const double length_squared = delta_x * delta_x + delta_y * delta_y;
    const double length = std::sqrt(length_squared);

    if (length <= kMinimumSegmentLength) {
      throw std::invalid_argument(
              "Path contains a duplicate or zero-length segment at waypoint " +
              std::to_string(index));
    }

    cumulative_lengths_[index] = cumulative_length;
    // Each segment stores p(s)=start+fraction*delta and the curvilinear
    // coordinate at its start. Curvature is filled in by the following pass.
    segments_.push_back(
      {start, {delta_x, delta_y}, length, length_squared,
        std::atan2(delta_y, delta_x), 0.0, cumulative_length});
    cumulative_length += length;
  }

  cumulative_lengths_.back() = cumulative_length;
  total_length_ = cumulative_length;

  // Estimate curvature as change in tangent angle divided by distance between
  // segment centers. The first and final segments use one-sided differences;
  // interior segments use centered differences. Clamping converts an ideal
  // polygon corner into a finite, explicitly bounded reference quantity.
  for (std::size_t index = 0; index < segments_.size(); ++index) {
    double heading_change = 0.0;
    double center_distance = 0.0;

    if (segments_.size() == 1) {
      segments_[index].curvature = 0.0;
      continue;
    }

    if (index == 0) {
      heading_change = wrap_angle(segments_[1].heading - segments_[0].heading);
      center_distance = 0.5 * (segments_[0].length + segments_[1].length);
    } else if (index + 1 == segments_.size()) {
      heading_change = wrap_angle(
        segments_[index].heading - segments_[index - 1].heading);
      center_distance = 0.5 * (segments_[index - 1].length + segments_[index].length);
    } else {
      heading_change = wrap_angle(
        segments_[index + 1].heading - segments_[index - 1].heading);
      center_distance =
        0.5 * segments_[index - 1].length + segments_[index].length +
        0.5 * segments_[index + 1].length;
    }

    const double raw_curvature = heading_change / center_distance;
    segments_[index].curvature = std::clamp(
      raw_curvature, -config_.maximum_abs_curvature, config_.maximum_abs_curvature);
  }
}

void PathReferenceManager::reset_progress()
{
  // Projection is causal within a run. Resetting permits a new run to search
  // again from the first segment instead of inheriting the previous endpoint.
  current_segment_index_ = 0;
  current_progress_ = 0.0;
  has_progress_ = false;
}

PathGeometrySample PathReferenceManager::make_sample(
  std::size_t segment_index, double fraction) const
{
  if (segments_.empty()) {
    throw std::logic_error("Cannot sample a path before loading waypoints");
  }

  segment_index = std::min(segment_index, segments_.size() - 1);
  fraction = std::clamp(fraction, 0.0, 1.0);
  const Segment & segment = segments_[segment_index];

  PathGeometrySample sample;
  sample.position.x = segment.start.x + fraction * segment.delta.x;
  sample.position.y = segment.start.y + fraction * segment.delta.y;
  sample.heading = segment.heading;
  sample.curvature = segment.curvature;
  sample.progress = segment.cumulative_start + fraction * segment.length;
  sample.remaining_length = std::max(0.0, total_length_ - sample.progress);
  sample.segment_index = segment_index;
  sample.segment_fraction = fraction;

  const double curvature_factor =
    1.0 / (1.0 + config_.curvature_speed_gain * std::abs(sample.curvature));
  double endpoint_factor = 1.0;
  if (config_.endpoint_slowdown_distance > 0.0) {
    endpoint_factor = std::clamp(
      sample.remaining_length / config_.endpoint_slowdown_distance, 0.0, 1.0);
  }

  // Geometry supplies a preliminary curvature-aware speed and omega=kappa*v.
  // The trajectory manager later adds time, acceleration, and braking limits.
  sample.reference_linear_velocity =
    config_.nominal_linear_velocity * curvature_factor * endpoint_factor;
  sample.reference_angular_velocity =
    sample.reference_linear_velocity * sample.curvature;
  return sample;
}

PathGeometrySample PathReferenceManager::sample_at_progress(double progress) const
{
  if (segments_.empty()) {
    throw std::logic_error("Cannot sample a path before loading waypoints");
  }
  if (!std::isfinite(progress)) {
    throw std::invalid_argument("Requested path progress must be finite");
  }

  const double clamped_progress = std::clamp(progress, 0.0, total_length_);
  // Cumulative lengths are sorted, so binary search locates the segment that
  // contains the requested arc length without scanning the entire path.
  const auto upper = std::upper_bound(
    cumulative_lengths_.begin(), cumulative_lengths_.end(), clamped_progress);
  std::size_t segment_index = 0;
  if (upper != cumulative_lengths_.begin()) {
    segment_index = static_cast<std::size_t>(
      std::distance(cumulative_lengths_.begin(), upper) - 1);
  }
  segment_index = std::min(segment_index, segments_.size() - 1);

  const Segment & segment = segments_[segment_index];
  const double fraction =
    (clamped_progress - segment.cumulative_start) / segment.length;
  return make_sample(segment_index, fraction);
}

PathReference PathReferenceManager::update(
  double robot_x, double robot_y, double robot_heading)
{
  if (segments_.empty()) {
    throw std::logic_error("Cannot update a path reference before loading waypoints");
  }
  if (!std::isfinite(robot_x) || !std::isfinite(robot_y) ||
    !std::isfinite(robot_heading))
  {
    throw std::invalid_argument("Robot state supplied to path reference is not finite");
  }

  // Search only forward from the last accepted segment. This reduces work and
  // prevents a closed path or figure-eight crossing from jumping to an earlier
  // branch with nearly the same Cartesian distance.
  const std::size_t search_begin = current_segment_index_;
  const std::size_t search_end = std::min(
    search_begin + config_.search_window + 1, segments_.size());

  double best_distance_squared = std::numeric_limits<double>::infinity();
  std::size_t best_segment_index = search_begin;
  double best_fraction = 0.0;

  for (std::size_t index = search_begin; index < search_end; ++index) {
    const Segment & segment = segments_[index];
    const double from_start_x = robot_x - segment.start.x;
    const double from_start_y = robot_y - segment.start.y;
    // Orthogonal projection parameter t = ((robot-start).delta)/|delta|^2.
    // Clamping t to [0,1] turns projection onto the infinite supporting line
    // into projection onto this finite path segment.
    const double raw_fraction =
      (from_start_x * segment.delta.x + from_start_y * segment.delta.y) /
      segment.length_squared;
    const double fraction = std::clamp(raw_fraction, 0.0, 1.0);
    const double projected_x = segment.start.x + fraction * segment.delta.x;
    const double projected_y = segment.start.y + fraction * segment.delta.y;
    const double error_x = robot_x - projected_x;
    const double error_y = robot_y - projected_y;
    const double distance_squared = error_x * error_x + error_y * error_y;

    // At a waypoint, two adjacent segments can have exactly equal distance.
    // Advance only across that immediate boundary. Preferring an arbitrary
    // later equal-distance segment would jump branches at a closed-path or
    // figure-eight intersection.
    const bool strictly_closer =
      distance_squared < best_distance_squared - kProjectionTieTolerance;
    const bool adjacent_boundary_tie =
      std::abs(distance_squared - best_distance_squared) <= kProjectionTieTolerance &&
      index == best_segment_index + 1 && best_fraction >= 1.0 - kProjectionTieTolerance &&
      fraction <= kProjectionTieTolerance;
    if (strictly_closer || adjacent_boundary_tie) {
      best_distance_squared = distance_squared;
      best_segment_index = index;
      best_fraction = fraction;
    }
  }

  const Segment & best_segment = segments_[best_segment_index];
  double candidate_progress =
    best_segment.cumulative_start + best_fraction * best_segment.length;
  // Geometric noise must not move the accepted path coordinate backwards.
  if (has_progress_ && candidate_progress < current_progress_) {
    candidate_progress = current_progress_;
  }

  PathGeometrySample sample = sample_at_progress(candidate_progress);
  current_segment_index_ = sample.segment_index;
  current_progress_ = sample.progress;
  has_progress_ = true;

  const double error_x = robot_x - sample.position.x;
  const double error_y = robot_y - sample.position.y;
  const double tangent_x = std::cos(sample.heading);
  const double tangent_y = std::sin(sample.heading);

  PathReference reference;
  reference.path = sample;
  // The 2-D signed cross product tangent x (robot-reference) defines the
  // cross-track sign consistently on straight and curved segments.
  reference.cross_track_error = tangent_x * error_y - tangent_y * error_x;
  reference.heading_error = wrap_angle(sample.heading - robot_heading);

  const Point2D & goal = waypoints_.back();
  reference.distance_to_goal = std::hypot(goal.x - robot_x, goal.y - robot_y);
  reference.closest_waypoint_index = std::min(
    sample.segment_index + (sample.segment_fraction >= 0.5 ? 1U : 0U),
    waypoints_.size() - 1);
  return reference;
}

std::size_t PathReferenceManager::lookahead_waypoint_index(
  std::size_t lookahead_points) const
{
  if (waypoints_.empty()) {
    throw std::logic_error("Cannot select lookahead before loading waypoints");
  }
  return std::min(
    current_segment_index_ + lookahead_points, waypoints_.size() - 1);
}

const Point2D & PathReferenceManager::waypoint(std::size_t index) const
{
  if (index >= waypoints_.size()) {
    throw std::out_of_range("Waypoint index is outside the loaded path");
  }
  return waypoints_[index];
}

std::size_t PathReferenceManager::waypoint_count() const
{
  return waypoints_.size();
}

std::size_t PathReferenceManager::segment_count() const
{
  return segments_.size();
}

double PathReferenceManager::total_length() const
{
  return total_length_;
}

}  // namespace my_robot_controller
