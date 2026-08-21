#ifndef MY_ROBOT_CONTROLLER__TRAJECTORY_REFERENCE_MANAGER_HPP_
#define MY_ROBOT_CONTROLLER__TRAJECTORY_REFERENCE_MANAGER_HPP_

#include "my_robot_controller/path_reference_manager.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace my_robot_controller
{

/// Settings used to convert the geometric CSV path into a timed trajectory.
struct TrajectoryReferenceConfig
{
  PathReferenceConfig path;

  /// Maximum spacing between time-profile knots measured along the path [m].
  double spatial_step{0.01};

  /// Common tangential acceleration and braking bounds [m/s^2].
  double maximum_linear_acceleration{0.5};
  double maximum_linear_deceleration{0.1};

  /// Bound used while constructing a feasible v_ref/omega_ref pair [rad/s].
  double maximum_reference_angular_velocity{1.5};
};

/// Complete common reference seen by every trajectory-tracking controller.
struct TrajectoryReference
{
  /// Time-indexed pose and velocities of the ideal virtual robot.
  PathGeometrySample trajectory;

  /// Closest geometric path point, retained for spatial benchmark metrics.
  PathReference projection;

  /// Reference-pose error expressed in the actual robot body frame.
  double longitudinal_error{0.0};
  double lateral_error{0.0};
  double heading_error{0.0};
  double position_error{0.0};

  double reference_time{0.0};
  bool trajectory_complete{false};
};

/// Build and sample a deterministic trajectory from a geometric waypoint CSV.
///
/// The reference clock is supplied by the caller and never depends on robot
/// progress. Consequently, a disturbed robot develops a measurable along-track
/// error instead of silently moving the reference point back to its projection.
class TrajectoryReferenceManager
{
public:
  explicit TrajectoryReferenceManager(const TrajectoryReferenceConfig & config = {});

  void configure(const TrajectoryReferenceConfig & config);
  void load_csv(const std::string & file_path);
  void set_path(const std::vector<Point2D> & waypoints);
  void reset_projection();

  /// Return the ideal reference pose and velocity at elapsed experiment time.
  PathGeometrySample sample_at_time(double elapsed_time) const;

  /// Combine the timed reference with geometric projection and body-frame error.
  TrajectoryReference update(
    double elapsed_time, double robot_x, double robot_y, double robot_heading);

  std::size_t lookahead_waypoint_index(std::size_t lookahead_points) const;
  const Point2D & waypoint(std::size_t index) const;
  std::size_t waypoint_count() const;
  double total_length() const;
  double duration() const;

private:
  struct TimeKnot
  {
    double time{0.0};
    double progress{0.0};
    double linear_velocity{0.0};
  };

  void validate_config(const TrajectoryReferenceConfig & config) const;
  void build_time_profile();

  TrajectoryReferenceConfig config_;
  PathReferenceManager path_manager_;
  std::vector<TimeKnot> time_knots_;
  double duration_{0.0};
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__TRAJECTORY_REFERENCE_MANAGER_HPP_
