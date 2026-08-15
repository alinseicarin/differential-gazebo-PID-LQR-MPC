#ifndef MY_ROBOT_CONTROLLER__PATH_REFERENCE_MANAGER_HPP_
#define MY_ROBOT_CONTROLLER__PATH_REFERENCE_MANAGER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace my_robot_controller
{

/// Minimal two-dimensional point used by the path geometry code.
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/// Configuration shared by every controller that consumes the reference path.
struct PathReferenceConfig
{
  /// Maximum number of segments examined ahead of remembered path progress.
  std::size_t search_window{20};

  /// Desired forward speed before curvature and endpoint slowdown are applied.
  double nominal_linear_velocity{0.4};

  /// Strength of the common 1/(1 + gain*abs(curvature)) speed reduction.
  double curvature_speed_gain{0.5};

  /// Remaining distance over which the common endpoint speed ramps to zero.
  double endpoint_slowdown_distance{0.0};

  /// Safety bound for curvature estimated at discontinuous polygon corners.
  double maximum_abs_curvature{5.0};
};

/// Path-only quantities at one continuous arc-length coordinate.
///
/// This structure does not depend on the robot state. MPC can request future
/// samples while PID and LQR normally use the sample embedded in PathReference.
struct PathGeometrySample
{
  Point2D position;
  double heading{0.0};
  double curvature{0.0};
  double progress{0.0};
  double remaining_length{0.0};
  double reference_linear_velocity{0.0};
  double reference_angular_velocity{0.0};
  std::size_t segment_index{0};
  double segment_fraction{0.0};
};

/// Common controller input produced by comparing the robot with the path.
struct PathReference
{
  PathGeometrySample path;
  double cross_track_error{0.0};
  double heading_error{0.0};
  double distance_to_goal{0.0};
  std::size_t closest_waypoint_index{0};
};

/// Normalize an angle to the shortest signed representation in [-pi, pi].
double wrap_angle(double angle);

/// Convert a headerless x,y waypoint CSV into a continuous path reference.
///
/// The manager deliberately contains no ROS interfaces and no controller
/// gains. It is deterministic geometry code that can be shared and unit-tested
/// independently of Gazebo, PID, LQR, and MPC.
class PathReferenceManager
{
public:
  explicit PathReferenceManager(const PathReferenceConfig & config = {});

  /// Replace the common reference and search settings after validating them.
  void configure(const PathReferenceConfig & config);

  /// Load, strictly validate, and preprocess a headerless two-column CSV.
  void load_csv(const std::string & file_path);

  /// Install an in-memory path. Primarily useful for tests and generated paths.
  void set_path(const std::vector<Point2D> & waypoints);

  /// Reset remembered monotonic progress to the first path segment.
  void reset_progress();

  /// Project the robot onto a nearby forward segment and calculate all errors.
  PathReference update(double robot_x, double robot_y, double robot_heading);

  /// Sample path geometry at an arbitrary clamped arc-length coordinate.
  PathGeometrySample sample_at_progress(double progress) const;

  /// Select the legacy lookahead target without exposing path storage mutation.
  std::size_t lookahead_waypoint_index(std::size_t lookahead_points) const;

  /// Read one immutable waypoint for the checkpointed lookahead controller.
  const Point2D & waypoint(std::size_t index) const;

  std::size_t waypoint_count() const;
  std::size_t segment_count() const;
  double total_length() const;

private:
  struct Segment
  {
    Point2D start;
    Point2D delta;
    double length{0.0};
    double length_squared{0.0};
    double heading{0.0};
    double curvature{0.0};
    double cumulative_start{0.0};
  };

  void validate_config(const PathReferenceConfig & config) const;
  void preprocess_path();
  PathGeometrySample make_sample(std::size_t segment_index, double fraction) const;

  PathReferenceConfig config_;
  std::vector<Point2D> waypoints_;
  std::vector<Segment> segments_;
  std::vector<double> cumulative_lengths_;
  double total_length_{0.0};

  std::size_t current_segment_index_{0};
  double current_progress_{0.0};
  bool has_progress_{false};
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__PATH_REFERENCE_MANAGER_HPP_
