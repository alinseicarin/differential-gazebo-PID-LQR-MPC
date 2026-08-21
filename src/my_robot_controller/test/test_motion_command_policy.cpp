#include "my_robot_controller/motion_command_policy.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace
{
// The shared post-controller policy is tested independently so PID, LQR, and
// MPC inherit the same verified feedforward, bounds, and recovery behavior.

// Normal operation adds feedback to its corresponding reference channel.
TEST(MotionCommandPolicy, CombinesBothFeedforwardAndFeedbackChannels)
{
  my_robot_controller::MotionCommandPolicy policy;

  const auto output = policy.calculate(0.4, 0.2, 0.1, 0.2, 0.1, -0.05);

  EXPECT_NEAR(output.linear_command, 0.5, 1.0e-12);
  EXPECT_NEAR(output.angular_feedforward_command, 0.2, 1.0e-12);
  EXPECT_NEAR(output.angular_feedback_command, -0.05, 1.0e-12);
  EXPECT_NEAR(output.angular_command, 0.15, 1.0e-12);
  EXPECT_FALSE(output.translation_safety_stop);
}

// Linear feedback must genuinely alter the feedforward speed.
TEST(MotionCommandPolicy, ControllerCanChangeLinearVelocity)
{
  my_robot_controller::MotionCommandPolicy policy;

  const auto centered = policy.calculate(0.4, 0.0, 0.0, 0.0, 0.0, 0.0);
  const auto behind = policy.calculate(0.4, 0.0, 0.0, 0.0, 0.2, 0.0);

  EXPECT_NEAR(centered.linear_command, 0.4, 1.0e-12);
  EXPECT_NEAR(behind.linear_command, 0.6, 1.0e-12);
  EXPECT_FALSE(behind.translation_safety_stop);
}

// Common bounds forbid reverse motion after longitudinal overshoot.
TEST(MotionCommandPolicy, PreventsReverseCorrectionAfterOvershoot)
{
  my_robot_controller::MotionCommandPolicy policy;

  const auto output = policy.calculate(0.0, 0.0, 0.0, 0.0, -0.2, 0.0);

  EXPECT_NEAR(output.linear_command, 0.0, 1.0e-12);
}

// Gross deviation stops translation but preserves bounded reorientation.
TEST(MotionCommandPolicy, StopsOnlyTranslationAfterGrossDeviation)
{
  my_robot_controller::MotionCommandPolicy policy;

  const auto lateral_stop = policy.calculate(0.4, 0.2, 0.75, 0.0, 0.2, -0.3);
  const auto heading_stop = policy.calculate(0.4, 0.2, 0.0, -1.2, 0.2, 0.3);

  EXPECT_TRUE(lateral_stop.translation_safety_stop);
  EXPECT_NEAR(lateral_stop.linear_command, 0.0, 1.0e-12);
  EXPECT_NEAR(lateral_stop.angular_feedforward_command, 0.0, 1.0e-12);
  EXPECT_NEAR(lateral_stop.angular_command, -0.3, 1.0e-12);
  EXPECT_TRUE(heading_stop.translation_safety_stop);
  EXPECT_NEAR(heading_stop.linear_command, 0.0, 1.0e-12);
  EXPECT_NEAR(heading_stop.angular_command, 0.3, 1.0e-12);
}

// Final commands must obey both common actuator limits.
TEST(MotionCommandPolicy, AppliesSharedLinearAndAngularLimits)
{
  my_robot_controller::MotionCommandPolicyConfig config;
  config.maximum_linear_velocity = 0.3;
  config.maximum_angular_velocity = 0.5;
  my_robot_controller::MotionCommandPolicy policy(config);

  const auto output = policy.calculate(0.8, 0.6, 0.0, 0.0, 1.0, 1.0);

  EXPECT_NEAR(output.linear_command, 0.3, 1.0e-12);
  EXPECT_NEAR(output.angular_feedforward_command, 0.6, 1.0e-12);
  EXPECT_NEAR(output.angular_command, 0.5, 1.0e-12);
}

// Configuration and runtime validity checks prevent unsafe propagation.
TEST(MotionCommandPolicy, RejectsInvalidConfigurationAndInput)
{
  my_robot_controller::MotionCommandPolicyConfig config;
  config.translation_stop_heading_error = 0.0;
  EXPECT_THROW(my_robot_controller::MotionCommandPolicy policy(config), std::invalid_argument);

  my_robot_controller::MotionCommandPolicy policy;
  EXPECT_THROW(
    policy.calculate(
      0.4, 0.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}
}  // namespace
