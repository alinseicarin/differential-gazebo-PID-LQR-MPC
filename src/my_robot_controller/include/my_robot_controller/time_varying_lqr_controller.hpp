#ifndef MY_ROBOT_CONTROLLER__TIME_VARYING_LQR_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLER__TIME_VARYING_LQR_CONTROLLER_HPP_

#include "my_robot_controller/linearized_error_model.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace my_robot_controller
{

using FeedbackGain = Eigen::Matrix<double, 2, 3>;

/// Dimensionless quadratic weights after the user selects meaningful error and
/// correction scales. All values must be finite and strictly positive.
struct TimeVaryingLqrConfig
{
  double longitudinal_error_weight{100.0};
  double lateral_error_weight{400.0};
  double heading_error_weight{100.0};
  double linear_correction_weight{125.0};
  double angular_correction_weight{15.625};
  double terminal_weight_multiplier{10.0};
};

/// Values retained for one controller update and written to the experiment CSV.
struct TimeVaryingLqrOutput
{
  VelocityCorrection correction{VelocityCorrection::Zero()};
  FeedbackGain gain{FeedbackGain::Zero()};
  std::size_t gain_index{0u};
  double instantaneous_state_cost{0.0};
};

/// Finite-horizon discrete TVLQR solved by a backward Riccati recursion.
///
/// The complete gain schedule is calculated once after the timed trajectory is
/// loaded. Runtime work is only a 2x3 matrix-vector multiplication, keeping the
/// ROS callback deterministic while MPC performs online optimization separately.
class TimeVaryingLqrController
{
public:
  explicit TimeVaryingLqrController(const TimeVaryingLqrConfig & config = {});

  void configure(const TimeVaryingLqrConfig & config);

  /// Replace the complete nominal model sequence and solve Riccati backwards.
  void build_gain_schedule(const std::vector<DiscreteErrorModel> & models);

  /// Apply the gain associated with model_index. Indices beyond the finite
  /// horizon deliberately hold the last gain so a late robot can still settle.
  TimeVaryingLqrOutput calculate(
    std::size_t model_index,
    const ErrorState & error) const;

  std::size_t gain_count() const;
  const FeedbackGain & gain(std::size_t index) const;
  const StateMatrix & cost_to_go(std::size_t index) const;

private:
  void validate_config(const TimeVaryingLqrConfig & config) const;
  void update_weight_matrices();

  TimeVaryingLqrConfig config_;
  StateMatrix state_weight_{StateMatrix::Identity()};
  Eigen::Matrix2d input_weight_{Eigen::Matrix2d::Identity()};
  StateMatrix terminal_state_weight_{StateMatrix::Identity()};
  std::vector<FeedbackGain> gains_;
  std::vector<StateMatrix> costs_to_go_;
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__TIME_VARYING_LQR_CONTROLLER_HPP_
