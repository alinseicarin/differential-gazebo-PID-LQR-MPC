#include "my_robot_controller/time_varying_lqr_controller.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace my_robot_controller
{

TimeVaryingLqrController::TimeVaryingLqrController(
  const TimeVaryingLqrConfig & config)
{
  configure(config);
}

void TimeVaryingLqrController::validate_config(
  const TimeVaryingLqrConfig & config) const
{
  const std::array<double, 6> values{
    config.longitudinal_error_weight,
    config.lateral_error_weight,
    config.heading_error_weight,
    config.linear_correction_weight,
    config.angular_correction_weight,
    config.terminal_weight_multiplier};
  if (!std::all_of(
      values.begin(), values.end(),
      [](double value) {return std::isfinite(value) && value > 0.0;}))
  {
    throw std::invalid_argument("Every TVLQR weight must be finite and strictly positive");
  }
}

void TimeVaryingLqrController::update_weight_matrices()
{
  state_weight_ = StateMatrix::Zero();
  state_weight_.diagonal() <<
    config_.longitudinal_error_weight,
    config_.lateral_error_weight,
    config_.heading_error_weight;

  input_weight_ = Eigen::Matrix2d::Zero();
  input_weight_.diagonal() <<
    config_.linear_correction_weight,
    config_.angular_correction_weight;
  terminal_state_weight_ = config_.terminal_weight_multiplier * state_weight_;
}

void TimeVaryingLqrController::configure(const TimeVaryingLqrConfig & config)
{
  validate_config(config);
  config_ = config;
  update_weight_matrices();
  gains_.clear();
  costs_to_go_.clear();
}

void TimeVaryingLqrController::build_gain_schedule(
  const std::vector<DiscreteErrorModel> & models)
{
  if (models.empty()) {
    throw std::invalid_argument("TVLQR requires at least one discrete model");
  }

  gains_.assign(models.size(), FeedbackGain::Zero());
  costs_to_go_.assign(models.size() + 1u, StateMatrix::Zero());
  costs_to_go_.back() = terminal_state_weight_;

  for (std::size_t reverse = models.size(); reverse > 0u; --reverse) {
    const std::size_t index = reverse - 1u;
    const StateMatrix & a = models[index].state_matrix;
    const InputMatrix & b = models[index].input_matrix;
    const StateMatrix & next_cost = costs_to_go_[index + 1u];
    if (!a.allFinite() || !b.allFinite() ||
      !std::isfinite(models[index].sample_period) || models[index].sample_period <= 0.0)
    {
      throw std::invalid_argument("TVLQR model sequence contains a non-finite value");
    }

    const Eigen::Matrix2d control_hessian = input_weight_ + b.transpose() * next_cost * b;
    const Eigen::LDLT<Eigen::Matrix2d> decomposition(control_hessian);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
      throw std::runtime_error("TVLQR control Hessian is not positive definite");
    }

    const FeedbackGain gain = decomposition.solve(b.transpose() * next_cost * a);
    if (decomposition.info() != Eigen::Success || !gain.allFinite()) {
      throw std::runtime_error("TVLQR Riccati gain solve failed");
    }
    gains_[index] = gain;

    StateMatrix current_cost =
      state_weight_ + a.transpose() * next_cost * a -
      a.transpose() * next_cost * b * gain;
    // Roundoff can introduce a tiny antisymmetric component even though the
    // Riccati solution is symmetric. Remove it before the next recursion step.
    current_cost = 0.5 * (current_cost + current_cost.transpose());
    if (!current_cost.allFinite()) {
      throw std::runtime_error("TVLQR Riccati recursion produced a non-finite cost");
    }
    costs_to_go_[index] = current_cost;
  }
}

TimeVaryingLqrOutput TimeVaryingLqrController::calculate(
  std::size_t model_index,
  const ErrorState & error) const
{
  if (gains_.empty()) {
    throw std::logic_error("TVLQR gain schedule has not been built");
  }
  if (!error.allFinite()) {
    throw std::invalid_argument("TVLQR error state must be finite");
  }

  TimeVaryingLqrOutput output;
  output.gain_index = std::min(model_index, gains_.size() - 1u);
  output.gain = gains_[output.gain_index];
  output.correction = -output.gain * error;
  output.instantaneous_state_cost = (error.transpose() * state_weight_ * error).value();
  return output;
}

std::size_t TimeVaryingLqrController::gain_count() const
{
  return gains_.size();
}

const FeedbackGain & TimeVaryingLqrController::gain(std::size_t index) const
{
  return gains_.at(index);
}

const StateMatrix & TimeVaryingLqrController::cost_to_go(std::size_t index) const
{
  return costs_to_go_.at(index);
}

}  // namespace my_robot_controller
