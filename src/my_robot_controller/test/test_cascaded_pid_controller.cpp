#include "my_robot_controller/cascaded_pid_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;

TEST(CascadedPidController, AlignedRobotUsesCommonReferenceSpeed)
{
  my_robot_controller::CascadedPidController controller;

  const auto output = controller.calculate(0.0, 0.0, 0.0, 0.4, 0.0, 0.1);

  EXPECT_NEAR(output.linear_command, 0.4, 1.0e-12);
  EXPECT_NEAR(output.angular_command, 0.0, 1.0e-12);
  EXPECT_NEAR(output.heading_speed_factor, 1.0, 1.0e-12);
  EXPECT_NEAR(output.cross_track_speed_factor, 1.0, 1.0e-12);
}

TEST(CascadedPidController, RobotLeftOfPathIsCommandedRight)
{
  my_robot_controller::CascadedPidController controller;

  // Positive cross-track error is the left side of a path pointing along +x.
  const auto output = controller.calculate(0.2, 0.0, 0.0, 0.4, 0.0, 0.1);

  EXPECT_LT(output.heading_correction, 0.0);
  EXPECT_LT(output.desired_heading, 0.0);
  EXPECT_LT(output.heading_error, 0.0);
  EXPECT_LT(output.angular_command, 0.0);
}

TEST(CascadedPidController, SuddenRotationTriggersInnerLoopImmediately)
{
  my_robot_controller::CascadedPidController controller;

  // Cross-track error is exactly zero, but the robot was rotated 90 degrees.
  // The inner loop must react now instead of waiting for lateral drift.
  const auto output = controller.calculate(0.0, 0.0, 0.5 * kPi, 0.4, 0.0, 0.1);

  EXPECT_NEAR(output.heading_error, -0.5 * kPi, 1.0e-12);
  EXPECT_LT(output.angular_command, 0.0);
  EXPECT_NEAR(output.linear_command, 0.0, 1.0e-12);
}

TEST(CascadedPidController, AddsCurvatureFeedforwardWhenErrorsAreZero)
{
  my_robot_controller::CascadedPidController controller;

  const auto output = controller.calculate(0.0, 0.4, 0.4, 0.3, 0.25, 0.1);

  EXPECT_NEAR(output.heading_pid_output, 0.0, 1.0e-12);
  EXPECT_NEAR(output.angular_command, 0.25, 1.0e-12);
}

TEST(CascadedPidController, AppliesHeadingAndActuatorLimits)
{
  my_robot_controller::CascadedPidConfig config;
  config.cross_track_kp = 10.0;
  config.cross_track_kd = 0.0;
  config.heading_kp = 10.0;
  config.heading_kd = 0.0;
  config.maximum_heading_correction = 0.3;
  config.maximum_linear_velocity = 0.2;
  config.maximum_angular_velocity = 0.4;
  my_robot_controller::CascadedPidController controller(config);

  const auto output = controller.calculate(1.0, 0.0, 0.0, 2.0, 2.0, 0.1);

  EXPECT_NEAR(output.heading_correction, -0.3, 1.0e-12);
  EXPECT_LE(output.linear_command, 0.2);
  EXPECT_NEAR(output.angular_command, -0.4, 1.0e-12);
}

TEST(CascadedPidController, RejectsInvalidConfiguration)
{
  my_robot_controller::CascadedPidConfig config;
  config.maximum_heading_correction = 0.0;

  EXPECT_THROW(
    my_robot_controller::CascadedPidController controller(config),
    std::invalid_argument);
}
}  // namespace
