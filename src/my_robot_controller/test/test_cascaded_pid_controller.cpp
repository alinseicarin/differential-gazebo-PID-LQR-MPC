#include "my_robot_controller/cascaded_pid_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;

// The suite supplies simple errors whose correct response direction is known
// analytically. Each test creates fresh PID memory and isolates one contract.

// Perfect alignment should leave both command corrections at zero.
TEST(CascadedPidController, AlignedRobotNeedsNoAngularFeedback)
{
  my_robot_controller::CascadedPidController controller;

  const auto output = controller.calculate(0.0, 0.0, 0.0, 0.0, 0.1);

  EXPECT_NEAR(output.longitudinal_pid_output, 0.0, 1.0e-12);
  EXPECT_NEAR(output.heading_pid_output, 0.0, 1.0e-12);
}

// A timed reference ahead of the robot requires positive speed correction.
TEST(CascadedPidController, ReferenceAheadRequestsPositiveLinearCorrection)
{
  my_robot_controller::CascadedPidController controller;

  const auto output = controller.calculate(0.2, 0.0, 0.0, 0.0, 0.1);

  EXPECT_GT(output.longitudinal_pid_output, 0.0);
}

// Check propagation of lateral-error sign through outer and inner loops.
TEST(CascadedPidController, ReferenceRightOfRobotCommandsRightTurn)
{
  my_robot_controller::CascadedPidController controller;

  // Negative body-frame e_y means that the virtual reference is to the right.
  const auto output = controller.calculate(0.0, -0.2, 0.0, 0.0, 0.1);

  EXPECT_LT(output.heading_correction, 0.0);
  EXPECT_LT(output.desired_heading, 0.0);
  EXPECT_LT(output.heading_error, 0.0);
  EXPECT_LT(output.heading_pid_output, 0.0);
}

// Rotation must excite the inner loop before lateral error has time to grow.
TEST(CascadedPidController, SuddenRotationTriggersInnerLoopImmediately)
{
  my_robot_controller::CascadedPidController controller;

  // Cross-track error is exactly zero, but the robot was rotated 90 degrees.
  // The inner loop must react now instead of waiting for lateral drift.
  const auto output = controller.calculate(0.0, 0.0, 0.0, 0.5 * kPi, 0.1);

  EXPECT_NEAR(output.heading_error, -0.5 * kPi, 1.0e-12);
  EXPECT_LT(output.heading_pid_output, 0.0);
}

// The outer heading request must respect its configured saturation.
TEST(CascadedPidController, AppliesOuterHeadingLimit)
{
  my_robot_controller::CascadedPidConfig config;
  config.cross_track_kp = 10.0;
  config.cross_track_kd = 0.0;
  config.heading_kp = 10.0;
  config.heading_kd = 0.0;
  config.maximum_heading_correction = 0.3;
  my_robot_controller::CascadedPidController controller(config);

  const auto output = controller.calculate(0.0, 1.0, 0.0, 0.0, 0.1);

  EXPECT_NEAR(output.heading_correction, 0.3, 1.0e-12);
  EXPECT_GT(output.heading_pid_output, 0.0);
}

// Non-finite or nonpositive safety configuration must fail immediately.
TEST(CascadedPidController, RejectsInvalidConfiguration)
{
  my_robot_controller::CascadedPidConfig config;
  config.maximum_heading_correction = 0.0;

  EXPECT_THROW(
    my_robot_controller::CascadedPidController controller(config),
    std::invalid_argument);
}
}  // namespace
