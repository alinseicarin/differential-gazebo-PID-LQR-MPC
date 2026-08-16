#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "my_robot_controller/wheel_slip_metrics.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(WheelSlipMetrics, PureRollingStraightHasZeroDiscrepancy)
{
  const auto result = my_robot_controller::calculate_wheel_slip_metrics(
    {4.0, 4.0, 0.4, 0.0, 0.0, 0.0}, {0.1, 0.35, 0.05});

  EXPECT_NEAR(result.wheel_kinematic_linear_velocity, 0.4, 1.0e-12);
  EXPECT_NEAR(result.truth_body_longitudinal_velocity, 0.4, 1.0e-12);
  EXPECT_NEAR(result.left_longitudinal_slip_ratio, 0.0, 1.0e-12);
  EXPECT_NEAR(result.right_longitudinal_slip_ratio, 0.0, 1.0e-12);
  EXPECT_NEAR(result.center_longitudinal_slip_ratio, 0.0, 1.0e-12);
  EXPECT_NEAR(result.yaw_velocity_discrepancy_ratio, 0.0, 1.0e-12);
  EXPECT_NEAR(result.sideslip_angle, 0.0, 1.0e-12);
}

TEST(WheelSlipMetrics, PureRollingTurnUsesLocalWheelCentreSpeeds)
{
  // Body travel is along world +y because yaw is pi/2. At v=0.4 m/s and
  // omega=0.5 rad/s, ideal tangential wheel speeds are 0.3125 and 0.4875 m/s.
  const auto result = my_robot_controller::calculate_wheel_slip_metrics(
    {3.125, 4.875, 0.0, 0.4, 0.5 * kPi, 0.5}, {0.1, 0.35, 0.05});

  EXPECT_NEAR(result.wheel_kinematic_linear_velocity, 0.4, 1.0e-12);
  EXPECT_NEAR(result.wheel_kinematic_angular_velocity, 0.5, 1.0e-12);
  EXPECT_NEAR(result.left_longitudinal_slip_ratio, 0.0, 1.0e-12);
  EXPECT_NEAR(result.right_longitudinal_slip_ratio, 0.0, 1.0e-12);
  EXPECT_NEAR(result.yaw_velocity_discrepancy_ratio, 0.0, 1.0e-12);
}

TEST(WheelSlipMetrics, ReportsTractionSlipAndLateralSideslip)
{
  const auto result = my_robot_controller::calculate_wheel_slip_metrics(
    {5.0, 5.0, 0.4, 0.1, 0.0, 0.0}, {0.1, 0.35, 0.05});

  EXPECT_NEAR(result.center_longitudinal_slip_ratio, 0.2, 1.0e-12);
  EXPECT_NEAR(result.left_longitudinal_slip_ratio, 0.2, 1.0e-12);
  EXPECT_NEAR(result.right_longitudinal_slip_ratio, 0.2, 1.0e-12);
  EXPECT_NEAR(result.truth_body_lateral_velocity, 0.1, 1.0e-12);
  EXPECT_NEAR(result.sideslip_angle, std::atan2(0.1, 0.4), 1.0e-12);
}

TEST(WheelSlipMetrics, RejectsInvalidConfigurationAndInput)
{
  EXPECT_THROW(
    my_robot_controller::calculate_wheel_slip_metrics(
      {4.0, 4.0, 0.4, 0.0, 0.0, 0.0}, {0.0, 0.35, 0.05}),
    std::invalid_argument);
  EXPECT_THROW(
    my_robot_controller::calculate_wheel_slip_metrics(
      {std::numeric_limits<double>::quiet_NaN(), 4.0, 0.4, 0.0, 0.0, 0.0},
      {0.1, 0.35, 0.05}),
    std::invalid_argument);
}

}  // namespace
