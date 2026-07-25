#include "my_robot_controller/command_disturbance.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{
TEST(CommandDisturbance, PassesNominalCommandBeforeFault)
{
  my_robot_controller::CommandDisturbance disturbance;

  const auto output = disturbance.apply(0.4, -0.1, 4.9);

  EXPECT_FALSE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_linear_velocity, 0.4);
  EXPECT_DOUBLE_EQ(output.applied_angular_velocity, -0.1);
}

TEST(CommandDisturbance, AddsAngularBiasAtStartBoundary)
{
  my_robot_controller::CommandDisturbance disturbance;

  const auto output = disturbance.apply(0.4, -0.1, 5.0);

  EXPECT_TRUE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_linear_velocity, 0.4);
  EXPECT_DOUBLE_EQ(output.applied_angular_velocity, 0.4);
}

TEST(CommandDisturbance, KeepsFaultActiveInsideWindow)
{
  my_robot_controller::CommandDisturbance disturbance;

  const auto output = disturbance.apply(0.3, 0.2, 5.25);

  EXPECT_TRUE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_angular_velocity, 0.7);
}

TEST(CommandDisturbance, EndsFaultAtEndBoundary)
{
  my_robot_controller::CommandDisturbance disturbance;

  const auto output = disturbance.apply(0.4, -0.1, 5.5);

  EXPECT_FALSE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_angular_velocity, -0.1);
}

TEST(CommandDisturbance, DoesNotExtendFaultAtRoundedEndTimestamp)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.duration = 1.0;
  my_robot_controller::CommandDisturbance disturbance(config);

  // A value microscopically below 6.0 can result from subtracting two ROS
  // timestamps that individually have exact integer-nanosecond storage.
  const auto output = disturbance.apply(0.4, -0.1, 5.999999999);

  EXPECT_FALSE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_angular_velocity, -0.1);
}

TEST(CommandDisturbance, SupportsAControllerIndependentLinearBias)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.linear_velocity_bias = -0.2;
  config.angular_velocity_bias = 0.0;
  my_robot_controller::CommandDisturbance disturbance(config);

  const auto output = disturbance.apply(0.4, 0.0, 5.1);

  EXPECT_TRUE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_linear_velocity, 0.2);
}

TEST(CommandDisturbance, ClampsAppliedCommandsToSafetyLimits)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.linear_velocity_bias = 1.0;
  config.angular_velocity_bias = 2.0;
  config.maximum_abs_linear_velocity = 0.8;
  config.maximum_abs_angular_velocity = 1.2;
  my_robot_controller::CommandDisturbance disturbance(config);

  const auto output = disturbance.apply(0.5, 0.5, 5.1);

  EXPECT_DOUBLE_EQ(output.applied_linear_velocity, 0.8);
  EXPECT_DOUBLE_EQ(output.applied_angular_velocity, 1.2);
}

TEST(CommandDisturbance, RejectsInvalidDuration)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.duration = 0.0;

  EXPECT_THROW(
    my_robot_controller::CommandDisturbance disturbance(config),
    std::invalid_argument);
}
}  // namespace
