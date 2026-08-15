#include "my_robot_controller/odometry_disturbance.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{
TEST(OdometryDisturbance, PassesPoseBeforeFault)
{
  my_robot_controller::OdometryDisturbanceConfig config;
  config.enabled = true;
  config.x_bias = 1.0;
  my_robot_controller::OdometryDisturbance disturbance(config);

  const auto output = disturbance.apply(0.2, -0.3, 0.4, 4.0);

  EXPECT_FALSE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_x, 0.2);
  EXPECT_DOUBLE_EQ(output.applied_y, -0.3);
  EXPECT_DOUBLE_EQ(output.applied_yaw, 0.4);
}

TEST(OdometryDisturbance, AppliesDeterministicBiasInsideWindow)
{
  my_robot_controller::OdometryDisturbanceConfig config;
  config.enabled = true;
  config.x_bias = 0.1;
  config.y_bias = -0.2;
  config.yaw_bias = 0.3;
  my_robot_controller::OdometryDisturbance disturbance(config);

  const auto output = disturbance.apply(1.0, 2.0, 0.4, 5.0);

  EXPECT_TRUE(output.fault_active);
  EXPECT_DOUBLE_EQ(output.applied_x, 1.1);
  EXPECT_DOUBLE_EQ(output.applied_y, 1.8);
  EXPECT_NEAR(output.applied_yaw, 0.7, 1.0e-12);
}

TEST(OdometryDisturbance, SameSeedReproducesSameNoiseTrace)
{
  my_robot_controller::OdometryDisturbanceConfig config;
  config.enabled = true;
  config.start_delay = 0.0;
  config.position_noise_standard_deviation = 0.01;
  config.yaw_noise_standard_deviation = 0.02;
  config.random_seed = 42u;
  my_robot_controller::OdometryDisturbance first(config);
  my_robot_controller::OdometryDisturbance second(config);

  for (int index = 0; index < 10; ++index) {
    const auto first_output = first.apply(0.0, 0.0, 0.0, 0.1 * index);
    const auto second_output = second.apply(0.0, 0.0, 0.0, 0.1 * index);
    EXPECT_DOUBLE_EQ(first_output.x_perturbation, second_output.x_perturbation);
    EXPECT_DOUBLE_EQ(first_output.y_perturbation, second_output.y_perturbation);
    EXPECT_DOUBLE_EQ(first_output.yaw_perturbation, second_output.yaw_perturbation);
  }
}

TEST(OdometryDisturbance, ResetReplaysNoiseTrace)
{
  my_robot_controller::OdometryDisturbanceConfig config;
  config.enabled = true;
  config.start_delay = 0.0;
  config.position_noise_standard_deviation = 0.01;
  my_robot_controller::OdometryDisturbance disturbance(config);

  const auto first = disturbance.apply(0.0, 0.0, 0.0, 0.0);
  disturbance.reset();
  const auto replay = disturbance.apply(0.0, 0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(first.x_perturbation, replay.x_perturbation);
  EXPECT_DOUBLE_EQ(first.y_perturbation, replay.y_perturbation);
}

TEST(OdometryDisturbance, RejectsNegativeNoiseDeviation)
{
  my_robot_controller::OdometryDisturbanceConfig config;
  config.position_noise_standard_deviation = -0.1;

  EXPECT_THROW(
    my_robot_controller::OdometryDisturbance disturbance(config),
    std::invalid_argument);
}
}  // namespace
