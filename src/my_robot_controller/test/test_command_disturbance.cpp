#include "my_robot_controller/command_disturbance.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{
// Boundary cases use explicit times because a one-sample scheduling error would
// undermine fairness. Later tests isolate repeated/persistent windows, wheel
// loss, safety clamping, invalid configurations, and command-delay history.
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

TEST(CommandDisturbance, AppliesEachRepeatedFaultWindowAndReportsItsIndex)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.start_delays = {1.0, 3.0, 5.0};
  config.duration = 0.5;
  config.angular_velocity_bias = 0.8;
  my_robot_controller::CommandDisturbance disturbance(config);

  const auto first = disturbance.apply(0.4, 0.1, 1.1);
  const auto gap = disturbance.apply(0.4, 0.1, 2.0);
  const auto second = disturbance.apply(0.4, 0.1, 3.1);
  const auto third = disturbance.apply(0.4, 0.1, 5.1);

  EXPECT_TRUE(first.fault_active);
  EXPECT_EQ(first.active_window_index, 0);
  EXPECT_DOUBLE_EQ(first.applied_angular_velocity, 0.9);
  EXPECT_FALSE(gap.fault_active);
  EXPECT_EQ(gap.active_window_index, -1);
  EXPECT_DOUBLE_EQ(gap.applied_angular_velocity, 0.1);
  EXPECT_EQ(second.active_window_index, 1);
  EXPECT_EQ(third.active_window_index, 2);
}

TEST(CommandDisturbance, KeepsPersistentFaultActiveAfterItsNominalDuration)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.start_delay = 1.0;
  config.duration = 0.5;
  config.persistent = true;
  config.left_wheel_effectiveness = 0.7;
  config.angular_velocity_bias = 0.0;
  my_robot_controller::CommandDisturbance disturbance(config);

  const auto before = disturbance.apply(0.4, 0.0, 0.9);
  const auto long_after = disturbance.apply(0.4, 0.0, 100.0);

  EXPECT_FALSE(before.fault_active);
  EXPECT_TRUE(long_after.fault_active);
  EXPECT_EQ(long_after.active_window_index, 0);
  EXPECT_LT(long_after.applied_linear_velocity, 0.4);
  EXPECT_GT(long_after.applied_angular_velocity, 0.0);
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

TEST(CommandDisturbance, ReducesOneWheelEffectivenessInWheelSpace)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.left_wheel_effectiveness = 0.5;
  config.wheel_separation = 0.4;
  config.angular_velocity_bias = 0.0;
  my_robot_controller::CommandDisturbance disturbance(config);

  const auto output = disturbance.apply(0.4, 0.0, 5.1);

  EXPECT_TRUE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.nominal_left_wheel_velocity, 0.4);
  EXPECT_DOUBLE_EQ(output.effective_left_wheel_velocity, 0.2);
  EXPECT_DOUBLE_EQ(output.applied_linear_velocity, 0.3);
  EXPECT_NEAR(output.applied_angular_velocity, 0.5, 1.0e-12);
}

TEST(CommandDisturbance, WheelEffectivenessDoesNotApplyOutsideFaultWindow)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.left_wheel_effectiveness = 0.5;
  config.angular_velocity_bias = 0.0;
  my_robot_controller::CommandDisturbance disturbance(config);

  const auto output = disturbance.apply(0.4, 0.2, 4.0);

  EXPECT_FALSE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_linear_velocity, 0.4);
  EXPECT_NEAR(output.applied_angular_velocity, 0.2, 1.0e-12);
}

TEST(CommandDisturbance, DisabledScheduleAlwaysPassesThrough)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.enabled = false;
  config.angular_velocity_bias = 1.0;
  config.left_wheel_effectiveness = 0.0;
  my_robot_controller::CommandDisturbance disturbance(config);

  const auto output = disturbance.apply(0.4, -0.1, 5.1);

  EXPECT_FALSE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_linear_velocity, 0.4);
  EXPECT_NEAR(output.applied_angular_velocity, -0.1, 1.0e-12);
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

TEST(CommandDisturbance, RejectsOverlappingRepeatedWindows)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.start_delays = {1.0, 1.5};
  config.duration = 1.0;

  EXPECT_THROW(
    my_robot_controller::CommandDisturbance disturbance(config),
    std::invalid_argument);
}

TEST(CommandDisturbance, RejectsMultipleStartsForPersistentFault)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.start_delays = {1.0, 3.0};
  config.persistent = true;

  EXPECT_THROW(
    my_robot_controller::CommandDisturbance disturbance(config),
    std::invalid_argument);
}

TEST(CommandDisturbance, RejectsInvalidWheelEffectiveness)
{
  my_robot_controller::CommandDisturbanceConfig config;
  config.left_wheel_effectiveness = 1.1;

  EXPECT_THROW(
    my_robot_controller::CommandDisturbance disturbance(config),
    std::invalid_argument);
}

TEST(CommandDelay, PassesCurrentCommandOutsideFault)
{
  my_robot_controller::CommandDelay delay(0.1);

  const auto output = delay.apply(0.4, 0.2, 1.0, false);

  EXPECT_FALSE(output.delay_active);
  EXPECT_DOUBLE_EQ(output.source_linear_velocity, 0.4);
  EXPECT_DOUBLE_EQ(output.source_angular_velocity, 0.2);
}

TEST(CommandDelay, SelectsNewestSampleOlderThanRequestedDelay)
{
  my_robot_controller::CommandDelay delay(0.1);
  delay.apply(0.1, 1.0, 0.00, false);
  delay.apply(0.2, 2.0, 0.05, false);
  delay.apply(0.3, 3.0, 0.10, false);

  const auto output = delay.apply(0.4, 4.0, 0.15, true);

  EXPECT_TRUE(output.delay_active);
  EXPECT_DOUBLE_EQ(output.source_linear_velocity, 0.2);
  EXPECT_DOUBLE_EQ(output.source_angular_velocity, 2.0);
  EXPECT_DOUBLE_EQ(output.source_time, 0.05);
}

TEST(CommandDelay, ResetsHistoryWhenSimulationClockMovesBackwards)
{
  my_robot_controller::CommandDelay delay(0.1);
  delay.apply(0.4, 1.0, 5.0, false);

  const auto output = delay.apply(0.2, 2.0, 0.0, true);

  EXPECT_TRUE(output.delay_active);
  EXPECT_DOUBLE_EQ(output.source_linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(output.source_angular_velocity, 0.0);
}
}  // namespace
