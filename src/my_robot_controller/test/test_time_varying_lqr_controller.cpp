#include "my_robot_controller/linearized_error_model.hpp"
#include "my_robot_controller/time_varying_lqr_controller.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

// Generate a constant straight-reference model sequence for controlled tests.
std::vector<my_robot_controller::DiscreteErrorModel> straight_models(
  std::size_t count = 150u)
{
  std::vector<my_robot_controller::DiscreteErrorModel> models;
  models.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    models.push_back(
      my_robot_controller::LinearizedErrorModel::discretize_zero_order_hold(
        0.4, 0.0, 1.0 / 30.0));
  }
  return models;
}

// The suite verifies Riccati construction, equilibrium feedback, correction
// signs, post-horizon behavior, and rejection of invalid data.
TEST(TimeVaryingLqrController, BuildsFiniteGainForEveryModel)
{
  my_robot_controller::TimeVaryingLqrController controller;
  const auto models = straight_models();
  controller.build_gain_schedule(models);

  ASSERT_EQ(controller.gain_count(), models.size());
  EXPECT_TRUE(controller.gain(0u).allFinite());
  EXPECT_TRUE(controller.gain(models.size() / 2u).allFinite());
  EXPECT_TRUE(controller.gain(models.size() - 1u).allFinite());
  EXPECT_TRUE(
    controller.cost_to_go(0u).isApprox(
      controller.cost_to_go(0u).transpose(), 1.0e-10));
}

TEST(TimeVaryingLqrController, ZeroErrorProducesOnlyZeroFeedback)
{
  my_robot_controller::TimeVaryingLqrController controller;
  controller.build_gain_schedule(straight_models());

  const auto output = controller.calculate(10u, my_robot_controller::ErrorState::Zero());

  EXPECT_TRUE(output.correction.isZero(1.0e-14));
  EXPECT_NEAR(output.instantaneous_state_cost, 0.0, 1.0e-14);
}

TEST(TimeVaryingLqrController, FeedbackSignsMoveTowardStraightReference)
{
  my_robot_controller::TimeVaryingLqrController controller;
  controller.build_gain_schedule(straight_models());

  // Positive ex means the virtual robot is ahead, so the actual robot must
  // speed up. Positive ey means the reference lies left, so it must turn left.
  const auto behind = controller.calculate(
    20u, my_robot_controller::ErrorState(0.1, 0.0, 0.0));
  const auto reference_left = controller.calculate(
    20u, my_robot_controller::ErrorState(0.0, 0.1, 0.0));

  EXPECT_GT(behind.correction(0), 0.0);
  EXPECT_GT(reference_left.correction(1), 0.0);
}

TEST(TimeVaryingLqrController, HoldsLastGainAfterFiniteHorizon)
{
  my_robot_controller::TimeVaryingLqrController controller;
  controller.build_gain_schedule(straight_models(5u));
  const my_robot_controller::ErrorState error(0.1, -0.1, 0.05);

  const auto at_end = controller.calculate(4u, error);
  const auto after_end = controller.calculate(500u, error);

  EXPECT_EQ(after_end.gain_index, 4u);
  EXPECT_TRUE(after_end.gain.isApprox(at_end.gain, 1.0e-14));
  EXPECT_TRUE(after_end.correction.isApprox(at_end.correction, 1.0e-14));
}

TEST(TimeVaryingLqrController, RejectsInvalidConfigurationAndModels)
{
  my_robot_controller::TimeVaryingLqrConfig config;
  config.lateral_error_weight = 0.0;
  EXPECT_THROW(
    my_robot_controller::TimeVaryingLqrController controller(config),
    std::invalid_argument);

  my_robot_controller::TimeVaryingLqrController controller;
  EXPECT_THROW(
    controller.build_gain_schedule({}),
    std::invalid_argument);

  controller.build_gain_schedule(straight_models(2u));
  my_robot_controller::ErrorState invalid = my_robot_controller::ErrorState::Zero();
  invalid(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(controller.calculate(0u, invalid), std::invalid_argument);
}

}  // namespace
