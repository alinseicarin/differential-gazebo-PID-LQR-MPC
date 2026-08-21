#include "my_robot_controller/linear_mpc_controller.hpp"

#include <osqp.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{

constexpr std::size_t kStateDimension = 3u;
constexpr std::size_t kInputDimension = 2u;

/// Owning storage for an OSQP compressed-sparse-column matrix.
///
/// OSQP 0.6 keeps pointers to the arrays during setup. This wrapper therefore
/// owns the arrays until osqp_cleanup has completed in calculate().
struct CscMatrixStorage
{
  std::vector<c_float> values;
  std::vector<c_int> row_indices;
  std::vector<c_int> column_pointers;
  csc matrix{};

  void bind(std::size_t rows, std::size_t columns)
  {
    matrix.nzmax = static_cast<c_int>(values.size());
    matrix.m = static_cast<c_int>(rows);
    matrix.n = static_cast<c_int>(columns);
    matrix.p = column_pointers.data();
    matrix.i = row_indices.data();
    matrix.x = values.data();
    matrix.nz = -1;
  }
};

/// Convert per-column entries to the exact CSC layout required by OSQP.
CscMatrixStorage make_csc(
  std::size_t rows,
  const std::vector<std::vector<std::pair<std::size_t, double>>> & columns)
{
  CscMatrixStorage result;
  result.column_pointers.reserve(columns.size() + 1u);
  result.column_pointers.push_back(0);

  for (auto entries : columns) {
    std::sort(
      entries.begin(), entries.end(),
      [](const auto & left, const auto & right) {return left.first < right.first;});
    for (const auto & entry : entries) {
      if (entry.first >= rows || !std::isfinite(entry.second)) {
        throw std::logic_error("MPC sparse matrix contains an invalid entry");
      }
      if (entry.second == 0.0) {
        continue;
      }
      result.row_indices.push_back(static_cast<c_int>(entry.first));
      result.values.push_back(static_cast<c_float>(entry.second));
    }
    result.column_pointers.push_back(static_cast<c_int>(result.values.size()));
  }
  result.bind(rows, columns.size());
  return result;
}

bool is_solved_status(c_int status)
{
  // OSQP's "solved inaccurate" status still represents a usable solution
  // within relaxed tolerances; all other statuses trigger a safe zero action.
  return status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE;
}

}  // namespace

namespace my_robot_controller
{

LinearMpcController::LinearMpcController(const LinearMpcConfig & config)
{
  configure(config);
}

void LinearMpcController::validate_config(const LinearMpcConfig & config) const
{
  // Positive weights keep the quadratic cost convex and meaningful. Actuator
  // and solver checks prevent malformed online optimization problems.
  if (config.prediction_horizon_steps == 0u) {
    throw std::invalid_argument("MPC prediction horizon must contain at least one step");
  }

  const std::array<double, 6> weights{
    config.longitudinal_error_weight,
    config.lateral_error_weight,
    config.heading_error_weight,
    config.linear_correction_weight,
    config.angular_correction_weight,
    config.terminal_weight_multiplier};
  if (!std::all_of(
      weights.begin(), weights.end(),
      [](double value) {return std::isfinite(value) && value > 0.0;}))
  {
    throw std::invalid_argument("Every MPC quadratic weight must be finite and positive");
  }

  if (!std::isfinite(config.minimum_linear_velocity) ||
    !std::isfinite(config.maximum_linear_velocity) ||
    config.minimum_linear_velocity > config.maximum_linear_velocity ||
    !std::isfinite(config.maximum_absolute_angular_velocity) ||
    config.maximum_absolute_angular_velocity <= 0.0)
  {
    throw std::invalid_argument("MPC actuator limits are invalid");
  }

  if (config.maximum_solver_iterations <= 0 ||
    !std::isfinite(config.absolute_solver_tolerance) ||
    config.absolute_solver_tolerance <= 0.0 ||
    !std::isfinite(config.relative_solver_tolerance) ||
    config.relative_solver_tolerance <= 0.0 ||
    !std::isfinite(config.solver_time_limit) || config.solver_time_limit < 0.0)
  {
    throw std::invalid_argument("MPC solver settings are invalid");
  }
}

void LinearMpcController::configure(const LinearMpcConfig & config)
{
  validate_config(config);
  config_ = config;
  reset_warm_start();
}

const LinearMpcConfig & LinearMpcController::config() const
{
  return config_;
}

void LinearMpcController::reset_warm_start()
{
  previous_primal_solution_.clear();
}

LinearMpcOutput LinearMpcController::calculate(
  const ErrorState & initial_error,
  const std::vector<DiscreteErrorModel> & models,
  const std::vector<MpcReferenceInput> & reference_inputs)
{
  const std::size_t horizon = config_.prediction_horizon_steps;
  if (!initial_error.allFinite()) {
    throw std::invalid_argument("MPC initial error must be finite");
  }
  if (models.size() != horizon || reference_inputs.size() != horizon) {
    throw std::invalid_argument("MPC model and reference sequences must match its horizon");
  }

  for (std::size_t stage = 0u; stage < horizon; ++stage) {
    if (!models[stage].state_matrix.allFinite() ||
      !models[stage].input_matrix.allFinite() ||
      !std::isfinite(models[stage].sample_period) || models[stage].sample_period <= 0.0 ||
      !std::isfinite(reference_inputs[stage].linear_velocity) ||
      !std::isfinite(reference_inputs[stage].angular_velocity))
    {
      throw std::invalid_argument("MPC prediction sequence contains an invalid value");
    }
  }

  // Decision-vector layout is [e_0 ... e_N, delta_u_0 ... delta_u_(N-1)].
  // Equality rows cover the known initial state and N dynamics transitions;
  // the remaining rows impose component-wise input bounds.
  const std::size_t state_variable_count = kStateDimension * (horizon + 1u);
  const std::size_t input_variable_count = kInputDimension * horizon;
  const std::size_t variable_count = state_variable_count + input_variable_count;
  const std::size_t dynamics_constraint_count = state_variable_count;
  const std::size_t constraint_count = dynamics_constraint_count + input_variable_count;

  // OSQP uses 0.5*z'*P*z + q'*z, hence every diagonal below is twice the
  // desired quadratic weight. Only the upper triangle of P is supplied.
  std::vector<std::vector<std::pair<std::size_t, double>>> p_columns(variable_count);
  const std::array<double, kStateDimension> state_weights{
    config_.longitudinal_error_weight,
    config_.lateral_error_weight,
    config_.heading_error_weight};
  const std::array<double, kInputDimension> input_weights{
    config_.linear_correction_weight,
    config_.angular_correction_weight};

  for (std::size_t stage = 0u; stage <= horizon; ++stage) {
    const double multiplier = stage == horizon ? config_.terminal_weight_multiplier : 1.0;
    for (std::size_t state = 0u; state < kStateDimension; ++state) {
      const std::size_t column = stage * kStateDimension + state;
      p_columns[column].push_back({column, 2.0 * multiplier * state_weights[state]});
    }
  }
  for (std::size_t stage = 0u; stage < horizon; ++stage) {
    for (std::size_t input = 0u; input < kInputDimension; ++input) {
      const std::size_t column = state_variable_count + stage * kInputDimension + input;
      p_columns[column].push_back({column, 2.0 * input_weights[input]});
    }
  }
  CscMatrixStorage p_matrix = make_csc(variable_count, p_columns);

  // Equality rows encode -e_0=-e_measured and
  // A_k*e_k + B_k*delta_u_k - e_(k+1)=0. The final rows select each
  // delta_u component so its absolute-command bounds can be imposed.
  std::vector<std::vector<std::pair<std::size_t, double>>> a_columns(variable_count);
  for (std::size_t state = 0u; state < kStateDimension; ++state) {
    a_columns[state].push_back({state, -1.0});
  }

  for (std::size_t stage = 0u; stage < horizon; ++stage) {
    const std::size_t row_offset = (stage + 1u) * kStateDimension;
    const std::size_t current_state_offset = stage * kStateDimension;
    const std::size_t next_state_offset = (stage + 1u) * kStateDimension;
    const std::size_t input_offset = state_variable_count + stage * kInputDimension;

    for (std::size_t row = 0u; row < kStateDimension; ++row) {
      for (std::size_t column = 0u; column < kStateDimension; ++column) {
        a_columns[current_state_offset + column].push_back(
          {row_offset + row, models[stage].state_matrix(row, column)});
      }
      a_columns[next_state_offset + row].push_back({row_offset + row, -1.0});
      for (std::size_t input = 0u; input < kInputDimension; ++input) {
        a_columns[input_offset + input].push_back(
          {row_offset + row, models[stage].input_matrix(row, input)});
      }
    }

    for (std::size_t input = 0u; input < kInputDimension; ++input) {
      a_columns[input_offset + input].push_back(
        {dynamics_constraint_count + stage * kInputDimension + input, 1.0});
    }
  }
  CscMatrixStorage a_matrix = make_csc(constraint_count, a_columns);

  // There is no linear term in the regulation cost, hence q=0. Equality
  // constraints use identical lower/upper bounds; input rows use intervals.
  std::vector<c_float> linear_cost(variable_count, 0.0);
  std::vector<c_float> lower_bounds(constraint_count, 0.0);
  std::vector<c_float> upper_bounds(constraint_count, 0.0);
  for (std::size_t state = 0u; state < kStateDimension; ++state) {
    lower_bounds[state] = static_cast<c_float>(-initial_error(state));
    upper_bounds[state] = lower_bounds[state];
  }

  for (std::size_t stage = 0u; stage < horizon; ++stage) {
    const std::size_t bound_offset = dynamics_constraint_count + stage * kInputDimension;
    const double reference_linear = reference_inputs[stage].linear_velocity;
    const double reference_angular = reference_inputs[stage].angular_velocity;
    // Optimized variables are corrections, but physical limits constrain the
    // absolute command: u_min-u_ref <= delta_u <= u_max-u_ref.
    lower_bounds[bound_offset] = static_cast<c_float>(
      config_.minimum_linear_velocity - reference_linear);
    upper_bounds[bound_offset] = static_cast<c_float>(
      config_.maximum_linear_velocity - reference_linear);
    lower_bounds[bound_offset + 1u] = static_cast<c_float>(
      -config_.maximum_absolute_angular_velocity - reference_angular);
    upper_bounds[bound_offset + 1u] = static_cast<c_float>(
      config_.maximum_absolute_angular_velocity - reference_angular);
  }

  // OSQPData holds non-owning pointers into the vectors and CSC wrappers above;
  // all storage therefore remains in scope until osqp_cleanup below.
  OSQPData problem{};
  problem.n = static_cast<c_int>(variable_count);
  problem.m = static_cast<c_int>(constraint_count);
  problem.P = &p_matrix.matrix;
  problem.q = linear_cost.data();
  problem.A = &a_matrix.matrix;
  problem.l = lower_bounds.data();
  problem.u = upper_bounds.data();

  // Start from library defaults and override only reproducibility/performance
  // settings exposed by the thesis configuration.
  OSQPSettings settings{};
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.warm_start = true;
  settings.polish = config_.polish_solution;
  settings.max_iter = config_.maximum_solver_iterations;
  settings.eps_abs = config_.absolute_solver_tolerance;
  settings.eps_rel = config_.relative_solver_tolerance;
  settings.time_limit = config_.solver_time_limit;

  LinearMpcOutput output;
  OSQPWorkspace * workspace = nullptr;
  const auto solve_start = std::chrono::steady_clock::now();
  const c_int setup_result = osqp_setup(&workspace, &problem, &settings);
  if (setup_result != 0 || workspace == nullptr) {
    throw std::runtime_error("OSQP failed to set up the MPC problem");
  }

  // Receding-horizon problems change only slightly between control samples.
  // Reusing the previous primal vector usually reduces solver iterations.
  if (previous_primal_solution_.size() == variable_count) {
    std::vector<c_float> warm_start(previous_primal_solution_.begin(),
      previous_primal_solution_.end());
    osqp_warm_start_x(workspace, warm_start.data());
  }

  const c_int solve_result = osqp_solve(workspace);
  const auto solve_end = std::chrono::steady_clock::now();
  output.solve_time_seconds = std::chrono::duration<double>(solve_end - solve_start).count();

  if (solve_result == 0 && workspace->info != nullptr) {
    output.solver_status = workspace->info->status_val;
    output.iterations = workspace->info->iter;
    output.objective = workspace->info->obj_val;
    output.status_message = workspace->info->status;
    output.solved = is_solved_status(workspace->info->status_val);
  } else {
    output.status_message = "osqp_solve API error";
  }

  if (output.solved && workspace->solution != nullptr && workspace->solution->x != nullptr) {
    // Receding-horizon policy publishes only delta_u_0, then rebuilds and solves
    // the problem after the next state measurement. The rest is a prediction.
    const c_float * solution = workspace->solution->x;
    const std::size_t first_input_offset = state_variable_count;
    output.correction(0) = solution[first_input_offset];
    output.correction(1) = solution[first_input_offset + 1u];
    previous_primal_solution_.assign(solution, solution + variable_count);
  } else {
    // Never reuse a stale action after a failed solve; clear both output and
    // warm start so the next QP begins from a known safe state.
    output.correction.setZero();
    previous_primal_solution_.clear();
  }

  osqp_cleanup(workspace);
  return output;
}

}  // namespace my_robot_controller
