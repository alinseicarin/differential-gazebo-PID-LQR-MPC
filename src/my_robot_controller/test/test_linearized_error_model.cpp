#include "my_robot_controller/linearized_error_model.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace
{

TEST(LinearizedErrorModel, BuildsExpectedContinuousMatrices)
{
  const auto model = my_robot_controller::LinearizedErrorModel::continuous(0.4, -0.2);

  EXPECT_NEAR(model.state_matrix(0, 1), -0.2, 1.0e-12);
  EXPECT_NEAR(model.state_matrix(1, 0), 0.2, 1.0e-12);
  EXPECT_NEAR(model.state_matrix(1, 2), 0.4, 1.0e-12);
  EXPECT_NEAR(model.input_matrix(0, 0), -1.0, 1.0e-12);
  EXPECT_NEAR(model.input_matrix(2, 1), -1.0, 1.0e-12);
  EXPECT_NEAR(model.input_matrix(1, 0), 0.0, 1.0e-12);
}

TEST(LinearizedErrorModel, ZeroOrderHoldIsExactAtRest)
{
  const double sample_period = 0.1;
  const auto model =
    my_robot_controller::LinearizedErrorModel::discretize_zero_order_hold(
    0.0, 0.0, sample_period);

  EXPECT_TRUE(model.state_matrix.isApprox(Eigen::Matrix3d::Identity(), 1.0e-12));
  EXPECT_NEAR(model.input_matrix(0, 0), -sample_period, 1.0e-12);
  EXPECT_NEAR(model.input_matrix(2, 1), -sample_period, 1.0e-12);
  EXPECT_NEAR(model.input_matrix(1, 1), 0.0, 1.0e-12);
}

TEST(LinearizedErrorModel, ZeroOrderHoldCapturesLateralHeadingCoupling)
{
  const double sample_period = 0.1;
  const double reference_speed = 0.4;
  const auto model =
    my_robot_controller::LinearizedErrorModel::discretize_zero_order_hold(
    reference_speed, 0.0, sample_period);

  EXPECT_NEAR(model.state_matrix(1, 2), reference_speed * sample_period, 1.0e-12);
  EXPECT_NEAR(
    model.input_matrix(1, 1),
    -0.5 * reference_speed * sample_period * sample_period, 1.0e-12);
}

TEST(LinearizedErrorModel, RejectsInvalidInputs)
{
  EXPECT_THROW(
    my_robot_controller::LinearizedErrorModel::continuous(-0.1, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    my_robot_controller::LinearizedErrorModel::continuous(
      0.1, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    my_robot_controller::LinearizedErrorModel::discretize_zero_order_hold(0.1, 0.0, 0.0),
    std::invalid_argument);
}

}  // namespace
