#include "my_robot_controller/path_reference_manager.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

using my_robot_controller::PathReferenceConfig;
using my_robot_controller::PathReferenceManager;
using my_robot_controller::Point2D;

// Construct a reusable x-axis path whose projection and tangent are known.
PathReferenceManager make_straight_manager()
{
  PathReferenceConfig config;
  config.search_window = 10;
  config.nominal_linear_velocity = 0.4;
  config.curvature_speed_gain = 0.5;
  config.endpoint_slowdown_distance = 0.5;

  PathReferenceManager manager(config);
  manager.set_path({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  return manager;
}

// The following tests cover continuous projection, error signs, curvature,
// closed-path branch selection, input validation, and endpoint slowdown.
TEST(PathReferenceManagerTest, ProjectsOntoStraightSegment)
{
  auto manager = make_straight_manager();
  const auto reference = manager.update(0.5, 0.2, 0.0);

  EXPECT_NEAR(reference.path.position.x, 0.5, 1.0e-12);
  EXPECT_NEAR(reference.path.position.y, 0.0, 1.0e-12);
  EXPECT_NEAR(reference.path.segment_fraction, 0.5, 1.0e-12);
  EXPECT_NEAR(reference.path.progress, 0.5, 1.0e-12);
  EXPECT_NEAR(reference.path.remaining_length, 1.5, 1.0e-12);
  EXPECT_NEAR(reference.cross_track_error, 0.2, 1.0e-12);
  EXPECT_NEAR(reference.heading_error, 0.0, 1.0e-12);
  EXPECT_NEAR(reference.path.curvature, 0.0, 1.0e-12);
}

TEST(PathReferenceManagerTest, CrossTrackSignChangesAcrossPath)
{
  auto above_manager = make_straight_manager();
  auto below_manager = make_straight_manager();

  EXPECT_GT(above_manager.update(0.5, 0.2, 0.0).cross_track_error, 0.0);
  EXPECT_LT(below_manager.update(0.5, -0.2, 0.0).cross_track_error, 0.0);
}

TEST(PathReferenceManagerTest, ClampsProjectionToFinitePath)
{
  auto before_manager = make_straight_manager();
  auto after_manager = make_straight_manager();

  const auto before = before_manager.update(-0.5, 0.1, 0.0);
  EXPECT_NEAR(before.path.progress, 0.0, 1.0e-12);
  EXPECT_NEAR(before.path.position.x, 0.0, 1.0e-12);

  const auto after = after_manager.update(3.0, 0.1, 0.0);
  EXPECT_NEAR(after.path.progress, 2.0, 1.0e-12);
  EXPECT_NEAR(after.path.position.x, 2.0, 1.0e-12);
}

TEST(PathReferenceManagerTest, HeadingErrorIsIndependentOfCrossTrackError)
{
  auto manager = make_straight_manager();
  const auto reference = manager.update(0.5, 0.0, kPi / 6.0);

  EXPECT_NEAR(reference.cross_track_error, 0.0, 1.0e-12);
  EXPECT_NEAR(reference.heading_error, -kPi / 6.0, 1.0e-12);
}

TEST(PathReferenceManagerTest, EstimatesUnitCircleCurvature)
{
  PathReferenceConfig config;
  config.search_window = 200;
  config.nominal_linear_velocity = 0.4;
  config.curvature_speed_gain = 0.5;
  config.endpoint_slowdown_distance = 0.0;

  std::vector<Point2D> circle;
  constexpr std::size_t segment_count = 128;
  for (std::size_t index = 0; index <= segment_count; ++index) {
    const double angle = 2.0 * kPi * static_cast<double>(index) /
      static_cast<double>(segment_count);
    circle.push_back({std::sin(angle), 1.0 - std::cos(angle)});
  }

  PathReferenceManager manager(config);
  manager.set_path(circle);
  const auto sample = manager.sample_at_progress(manager.total_length() * 0.25);

  EXPECT_NEAR(sample.curvature, 1.0, 0.02);
  EXPECT_NEAR(sample.reference_linear_velocity, 0.4 / 1.5, 1.0e-3);
  EXPECT_NEAR(
    sample.reference_angular_velocity,
    sample.reference_linear_velocity * sample.curvature, 1.0e-12);
}

TEST(PathReferenceManagerTest, ClosedPathDoesNotImplyInitialProgress)
{
  PathReferenceConfig config;
  config.search_window = 3;
  PathReferenceManager manager(config);
  manager.set_path(
    {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {1.0, -1.0}, {0.0, 0.0},
      {-1.0, 1.0}, {-2.0, 0.0}, {-1.0, -1.0}, {0.0, 0.0}});

  const auto initial = manager.update(0.0, 0.0, kPi / 4.0);
  EXPECT_NEAR(initial.distance_to_goal, 0.0, 1.0e-12);
  EXPECT_NEAR(initial.path.progress, 0.0, 1.0e-12);
  EXPECT_GT(initial.path.remaining_length, 10.0);

  // Move through both lobes in order. Monotonic local search must reach the
  // final occurrence of the shared origin instead of jumping there initially.
  for (std::size_t index = 1; index < manager.waypoint_count(); ++index) {
    const Point2D & point = manager.waypoint(index);
    manager.update(point.x, point.y, 0.0);
  }
  const auto final_sample = manager.sample_at_progress(manager.total_length());
  EXPECT_NEAR(final_sample.remaining_length, 0.0, 1.0e-12);
}

TEST(PathReferenceManagerTest, RejectsDuplicateConsecutiveWaypoints)
{
  PathReferenceManager manager;
  EXPECT_THROW(
    manager.set_path({{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}),
    std::invalid_argument);
}

TEST(PathReferenceManagerTest, EndpointSpeedRampsToZero)
{
  auto manager = make_straight_manager();
  const auto far_sample = manager.sample_at_progress(1.0);
  const auto near_sample = manager.sample_at_progress(1.9);
  const auto goal_sample = manager.sample_at_progress(2.0);

  EXPECT_NEAR(far_sample.reference_linear_velocity, 0.4, 1.0e-12);
  EXPECT_NEAR(near_sample.reference_linear_velocity, 0.08, 1.0e-12);
  EXPECT_NEAR(goal_sample.reference_linear_velocity, 0.0, 1.0e-12);
}
}  // namespace
