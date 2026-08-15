#ifndef MY_ROBOT_CONTROLLER__LINEAR_MPC_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLER__LINEAR_MPC_CONTROLLER_HPP_

#include "my_robot_controller/linearized_error_model.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace my_robot_controller
{

/// Finite-horizon quadratic weights, command bounds, and OSQP tolerances.
struct LinearMpcConfig
{
  std::size_t prediction_horizon_steps{45u};

  double longitudinal_error_weight{100.0};
  double lateral_error_weight{400.0};
  double heading_error_weight{100.0};
  double linear_correction_weight{125.0};
  double angular_correction_weight{15.625};
  double terminal_weight_multiplier{10.0};

  double minimum_linear_velocity{0.0};
  double maximum_linear_velocity{1.0};
  double maximum_absolute_angular_velocity{1.5};

  int maximum_solver_iterations{4000};
  double absolute_solver_tolerance{1.0e-5};
  double relative_solver_tolerance{1.0e-5};
  double solver_time_limit{0.02};
  bool polish_solution{false};
};

/// Reference body velocities about which one prediction stage is linearized.
struct MpcReferenceInput
{
  double linear_velocity{0.0};
  double angular_velocity{0.0};
};

/// First receding-horizon action plus diagnostics retained for experiment logs.
struct LinearMpcOutput
{
  VelocityCorrection correction{VelocityCorrection::Zero()};
  bool solved{false};
  int solver_status{0};
  int iterations{0};
  double objective{0.0};
  double solve_time_seconds{0.0};
  std::string status_message;
};

/// Constrained finite-horizon LTV-MPC solved as a sparse convex QP.
///
/// Decision vector:
///   z = [e_0, ..., e_N, delta_u_0, ..., delta_u_(N-1)].
/// Dynamics are equality constraints. Absolute actuator limits are converted
/// to bounds on delta_u using the supplied v_ref and omega_ref sequence. No
/// hard error-state constraints are imposed, so actuator-feasible problems do
/// not become infeasible after a large disturbance.
class LinearMpcController
{
public:
  explicit LinearMpcController(const LinearMpcConfig & config = {});

  void configure(const LinearMpcConfig & config);
  const LinearMpcConfig & config() const;

  LinearMpcOutput calculate(
    const ErrorState & initial_error,
    const std::vector<DiscreteErrorModel> & models,
    const std::vector<MpcReferenceInput> & reference_inputs);

  void reset_warm_start();

private:
  void validate_config(const LinearMpcConfig & config) const;

  LinearMpcConfig config_;
  std::vector<double> previous_primal_solution_;
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__LINEAR_MPC_CONTROLLER_HPP_
