#ifndef MY_ROBOT_CONTROLLER__LINEARIZED_ERROR_MODEL_HPP_
#define MY_ROBOT_CONTROLLER__LINEARIZED_ERROR_MODEL_HPP_

#include <Eigen/Core>

namespace my_robot_controller
{

using ErrorState = Eigen::Vector3d;
using VelocityCorrection = Eigen::Vector2d;
using StateMatrix = Eigen::Matrix3d;
using InputMatrix = Eigen::Matrix<double, 3, 2>;

/// Continuous local model of the body-frame trajectory-tracking errors.
///
/// State: [longitudinal error, lateral error, heading error].
/// Input: [linear-velocity correction, angular-velocity correction].
struct ContinuousErrorModel
{
  StateMatrix state_matrix{StateMatrix::Zero()};
  InputMatrix input_matrix{InputMatrix::Zero()};
};

/// Zero-order-hold discretization used by both TVLQR and MPC.
struct DiscreteErrorModel
{
  StateMatrix state_matrix{StateMatrix::Identity()};
  InputMatrix input_matrix{InputMatrix::Zero()};
  double sample_period{0.0};
};

/// Construct the same linearized kinematic error model for every model-based
/// controller in the comparison.
class LinearizedErrorModel
{
public:
  /// Linearize around perfect tracking at the supplied reference velocities.
  static ContinuousErrorModel continuous(
    double reference_linear_velocity,
    double reference_angular_velocity);

  /// Freeze the continuous matrices over one sample and discretize exactly
  /// under a zero-order-hold assumption using an augmented matrix exponential.
  static DiscreteErrorModel discretize_zero_order_hold(
    double reference_linear_velocity,
    double reference_angular_velocity,
    double sample_period);
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__LINEARIZED_ERROR_MODEL_HPP_
