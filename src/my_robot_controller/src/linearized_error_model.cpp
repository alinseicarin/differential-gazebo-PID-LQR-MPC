#include "my_robot_controller/linearized_error_model.hpp"

#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <stdexcept>

namespace my_robot_controller
{

// Build the frozen continuous-time model used at one point of the prescribed
// trajectory. Calling this function at successive reference samples produces
// the LTV model sequence consumed by LQR and MPC.
ContinuousErrorModel LinearizedErrorModel::continuous(
  double reference_linear_velocity,
  double reference_angular_velocity)
{
  if (!std::isfinite(reference_linear_velocity) || reference_linear_velocity < 0.0 ||
    !std::isfinite(reference_angular_velocity))
  {
    throw std::invalid_argument(
            "Reference velocities must be finite and linear velocity non-negative");
  }

  ContinuousErrorModel model;

  // The error convention is reference minus actual, expressed in the actual
  // robot body frame. It gives the local dynamics
  //   ex_dot     =  omega_ref * ey - delta_v
  //   ey_dot     = -omega_ref * ex + v_ref * e_heading
  //   ehead_dot  = -delta_omega.
  model.state_matrix <<
    0.0, reference_angular_velocity, 0.0,
    -reference_angular_velocity, 0.0, reference_linear_velocity,
    0.0, 0.0, 0.0;
  model.input_matrix <<
    -1.0, 0.0,
    0.0, 0.0,
    0.0, -1.0;
  return model;
}

// Convert one frozen continuous model to the exact discrete equivalent under
// a zero-order hold: the correction is assumed constant during one sample.
DiscreteErrorModel LinearizedErrorModel::discretize_zero_order_hold(
  double reference_linear_velocity,
  double reference_angular_velocity,
  double sample_period)
{
  if (!std::isfinite(sample_period) || sample_period <= 0.0) {
    throw std::invalid_argument("Model sample period must be finite and positive");
  }

  const ContinuousErrorModel continuous_model = continuous(
    reference_linear_velocity, reference_angular_velocity);

  // For x_dot=A*x+B*u, exp([[A,B],[0,0]]*Ts) contains Ad in its upper-left
  // block and Bd in its upper-right block. This avoids an Euler approximation
  // while retaining the standard assumption that A and B are frozen for Ts.
  Eigen::Matrix<double, 5, 5> augmented = Eigen::Matrix<double, 5, 5>::Zero();
  augmented.topLeftCorner<3, 3>() = continuous_model.state_matrix;
  augmented.topRightCorner<3, 2>() = continuous_model.input_matrix;
  const Eigen::Matrix<double, 5, 5> transition = (augmented * sample_period).exp();

  DiscreteErrorModel discrete_model;
  // Extract Ad and Bd from the augmented exponential and retain Ts alongside
  // them so later code can audit that every prediction stage uses valid timing.
  discrete_model.state_matrix = transition.topLeftCorner<3, 3>();
  discrete_model.input_matrix = transition.topRightCorner<3, 2>();
  discrete_model.sample_period = sample_period;
  return discrete_model;
}

}  // namespace my_robot_controller
