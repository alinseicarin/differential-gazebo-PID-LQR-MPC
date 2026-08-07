#include "my_robot_controller/trajectory_reference_manager.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
my_robot_controller::TrajectoryReferenceManager make_manager()
{
  my_robot_controller::TrajectoryReferenceConfig config;
  config.path.nominal_linear_velocity = 1.0;
  config.path.curvature_speed_gain = 0.0;
  config.path.endpoint_slowdown_distance = 0.0;
  config.spatial_step = 0.01;
  config.maximum_linear_acceleration = 0.5;
  config.maximum_linear_deceleration = 0.5;

  my_robot_controller::TrajectoryReferenceManager manager(config);
  manager.set_path({{0.0, 0.0}, {2.0, 0.0}});
  return manager;
}

TEST(TrajectoryReferenceManager, StartsAndEndsAtRest)
{
  auto manager = make_manager();
  const auto start = manager.sample_at_time(0.0);
  const auto finish = manager.sample_at_time(manager.duration());

  EXPECT_NEAR(start.progress, 0.0, 1.0e-12);
  EXPECT_NEAR(start.reference_linear_velocity, 0.0, 1.0e-12);
  EXPECT_NEAR(finish.progress, 2.0, 1.0e-12);
  EXPECT_NEAR(finish.reference_linear_velocity, 0.0, 1.0e-12);
  EXPECT_GT(manager.duration(), 2.0);
}

TEST(TrajectoryReferenceManager, ReferenceDependsOnTimeNotRobotProgress)
{
  auto first_manager = make_manager();
  auto second_manager = make_manager();
  const double query_time = 0.75;

  const auto robot_behind = first_manager.update(query_time, 0.0, 0.0, 0.0);
  const auto robot_ahead = second_manager.update(query_time, 1.5, 0.0, 0.0);

  EXPECT_NEAR(
    robot_behind.trajectory.progress, robot_ahead.trajectory.progress, 1.0e-12);
  EXPECT_GT(robot_behind.longitudinal_error, 0.0);
  EXPECT_LT(robot_ahead.longitudinal_error, 0.0);
}

TEST(TrajectoryReferenceManager, CalculatesBodyFramePoseError)
{
  auto manager = make_manager();
  const auto reference = manager.update(1.0, 0.0, 0.2, 0.0);

  EXPECT_GT(reference.longitudinal_error, 0.0);
  EXPECT_LT(reference.lateral_error, 0.0);
  EXPECT_NEAR(reference.heading_error, 0.0, 1.0e-12);
  EXPECT_NEAR(
    reference.position_error,
    std::hypot(reference.longitudinal_error, reference.lateral_error), 1.0e-12);
}

TEST(TrajectoryReferenceManager, MarksCompletionOnlyAfterReferenceDuration)
{
  auto manager = make_manager();
  EXPECT_FALSE(
    manager.update(manager.duration() - 1.0e-6, 0.0, 0.0, 0.0).
    trajectory_complete);
  EXPECT_TRUE(manager.update(manager.duration(), 0.0, 0.0, 0.0).trajectory_complete);
}

TEST(TrajectoryReferenceManager, RejectsInvalidTimeProfileConfiguration)
{
  my_robot_controller::TrajectoryReferenceConfig config;
  config.maximum_linear_acceleration = 0.0;
  EXPECT_THROW(
    my_robot_controller::TrajectoryReferenceManager manager(config),
    std::invalid_argument);

  auto manager = make_manager();
  EXPECT_THROW(
    manager.sample_at_time(std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}
}  // namespace
