#include "my_robot_controller/linear_mpc_controller.hpp"

#include "my_robot_controller/linearized_error_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{

constexpr std::size_t kHorizon = 12u;
constexpr double kSamplePeriod = 1.0 / 30.0;

// Helpers create one repeatable straight-path quadratic program and keep the
// individual tests focused on an optimization property rather than setup.
std::vector<my_robot_controller::DiscreteErrorModel> straight_models()
{
  return std::vector<my_robot_controller::DiscreteErrorModel>(
    kHorizon,
    my_robot_controller::LinearizedErrorModel::discretize_zero_order_hold(
      0.4, 0.0, kSamplePeriod));
}

std::vector<my_robot_controller::MpcReferenceInput> straight_references(
  double speed = 0.4, double angular_velocity = 0.0)
{
  return std::vector<my_robot_controller::MpcReferenceInput>(
    kHorizon, {speed, angular_velocity});
}

my_robot_controller::LinearMpcController make_controller()
{
  my_robot_controller::LinearMpcConfig config;
  config.prediction_horizon_steps = kHorizon;
  config.solver_time_limit = 0.0;
  return my_robot_controller::LinearMpcController(config);
}

}  // namespace

// The suite checks configuration, equilibrium, correction signs, absolute
// command constraints, and prediction-horizon dimension validation.
TEST(LinearMpcController, RejectsInvalidConfiguration)
{
  my_robot_controller::LinearMpcConfig config;
  config.prediction_horizon_steps = 0u;
  EXPECT_THROW(my_robot_controller::LinearMpcController controller(config), std::invalid_argument);

  config.prediction_horizon_steps = kHorizon;
  config.maximum_linear_velocity = -1.0;
  EXPECT_THROW(my_robot_controller::LinearMpcController controller(config), std::invalid_argument);
}

TEST(LinearMpcController, ZeroErrorProducesZeroCorrection)
{
  auto controller = make_controller();
  const auto output = controller.calculate(
    my_robot_controller::ErrorState::Zero(), straight_models(), straight_references());

  ASSERT_TRUE(output.solved) << output.status_message;
  EXPECT_NEAR(output.correction(0), 0.0, 1.0e-7);
  EXPECT_NEAR(output.correction(1), 0.0, 1.0e-7);
  EXPECT_NEAR(output.objective, 0.0, 1.0e-7);
}

TEST(LinearMpcController, LongitudinalErrorIncreasesForwardSpeed)
{
  auto controller = make_controller();
  const auto output = controller.calculate(
    my_robot_controller::ErrorState(0.5, 0.0, 0.0),
    straight_models(), straight_references());

  ASSERT_TRUE(output.solved) << output.status_message;
  EXPECT_GT(output.correction(0), 0.0);
  EXPECT_NEAR(output.correction(1), 0.0, 1.0e-7);
}

TEST(LinearMpcController, HeadingErrorCommandsCorrectAngularDirection)
{
  auto controller = make_controller();
  const auto output = controller.calculate(
    my_robot_controller::ErrorState(0.0, 0.0, 0.4),
    straight_models(), straight_references());

  ASSERT_TRUE(output.solved) << output.status_message;
  EXPECT_GT(output.correction(1), 0.0);
}

TEST(LinearMpcController, RespectsAbsoluteLinearVelocityBounds)
{
  auto controller = make_controller();
  const auto output_at_maximum = controller.calculate(
    my_robot_controller::ErrorState(10.0, 0.0, 0.0),
    straight_models(), straight_references(1.0));
  ASSERT_TRUE(output_at_maximum.solved) << output_at_maximum.status_message;
  EXPECT_LE(1.0 + output_at_maximum.correction(0), 1.0 + 2.0e-5);
  EXPECT_GE(1.0 + output_at_maximum.correction(0), -2.0e-5);

  controller.reset_warm_start();
  const auto output_at_minimum = controller.calculate(
    my_robot_controller::ErrorState(-10.0, 0.0, 0.0),
    straight_models(), straight_references(0.2));
  ASSERT_TRUE(output_at_minimum.solved) << output_at_minimum.status_message;
  EXPECT_GE(0.2 + output_at_minimum.correction(0), -2.0e-5);
  EXPECT_LE(0.2 + output_at_minimum.correction(0), 1.0 + 2.0e-5);
}

TEST(LinearMpcController, RespectsAbsoluteAngularVelocityBounds)
{
  auto controller = make_controller();
  const auto output_at_maximum = controller.calculate(
    my_robot_controller::ErrorState(0.0, 0.0, 10.0),
    straight_models(), straight_references(0.4, 1.5));
  ASSERT_TRUE(output_at_maximum.solved) << output_at_maximum.status_message;
  EXPECT_LE(1.5 + output_at_maximum.correction(1), 1.5 + 2.0e-5);
  EXPECT_GE(1.5 + output_at_maximum.correction(1), -1.5 - 2.0e-5);

  controller.reset_warm_start();
  const auto output_at_minimum = controller.calculate(
    my_robot_controller::ErrorState(0.0, 0.0, -10.0),
    straight_models(), straight_references(0.4, -1.5));
  ASSERT_TRUE(output_at_minimum.solved) << output_at_minimum.status_message;
  EXPECT_GE(-1.5 + output_at_minimum.correction(1), -1.5 - 2.0e-5);
  EXPECT_LE(-1.5 + output_at_minimum.correction(1), 1.5 + 2.0e-5);
}

TEST(LinearMpcController, RejectsPredictionSequenceWithWrongLength)
{
  auto controller = make_controller();
  auto models = straight_models();
  models.pop_back();
  EXPECT_THROW(
    controller.calculate(
      my_robot_controller::ErrorState::Zero(), models, straight_references()),
    std::invalid_argument);
}
