#include "my_robot_controller/odometry_disturbance.hpp"

#include "my_robot_controller/path_reference_manager.hpp"

#include <cmath>
#include <stdexcept>

namespace my_robot_controller
{
namespace
{
// Accommodate sub-microsecond error introduced when ROS integer timestamps are
// converted to floating-point elapsed seconds.
constexpr double kScheduleTimeTolerance = 1.0e-6;
}

OdometryDisturbance::OdometryDisturbance(const OdometryDisturbanceConfig & config)
{
  configure(config);
}

void OdometryDisturbance::validate_config(const OdometryDisturbanceConfig & config) const
{
  if (!std::isfinite(config.start_delay) || config.start_delay < 0.0) {
    throw std::invalid_argument(
            "Odometry-fault start delay must be finite and non-negative");
  }
  if (!std::isfinite(config.duration) || config.duration <= 0.0) {
    throw std::invalid_argument("Odometry-fault duration must be finite and positive");
  }
  if (!std::isfinite(config.x_bias) || !std::isfinite(config.y_bias) ||
    !std::isfinite(config.yaw_bias))
  {
    throw std::invalid_argument("Odometry-fault biases must be finite");
  }
  if (!std::isfinite(config.position_noise_standard_deviation) ||
    config.position_noise_standard_deviation < 0.0 ||
    !std::isfinite(config.yaw_noise_standard_deviation) ||
    config.yaw_noise_standard_deviation < 0.0)
  {
    throw std::invalid_argument(
            "Odometry-fault standard deviations must be finite and non-negative");
  }
}

void OdometryDisturbance::configure(const OdometryDisturbanceConfig & config)
{
  validate_config(config);
  config_ = config;
  reset();
}

void OdometryDisturbance::reset()
{
  // Re-seeding makes nominally identical repetitions reproduce the same noise
  // realization; varying random_seed creates controlled independent trials.
  random_engine_.seed(config_.random_seed);
  standard_normal_.reset();
}

bool OdometryDisturbance::is_active(double elapsed_time) const
{
  if (!std::isfinite(elapsed_time) || elapsed_time < 0.0) {
    throw std::invalid_argument(
            "Odometry-fault elapsed time must be finite and non-negative");
  }
  return config_.enabled &&
         elapsed_time + kScheduleTimeTolerance >= config_.start_delay &&
         elapsed_time + kScheduleTimeTolerance < config_.start_delay + config_.duration;
}

OdometryDisturbanceOutput OdometryDisturbance::apply(
  double nominal_x,
  double nominal_y,
  double nominal_yaw,
  double elapsed_time)
{
  if (!std::isfinite(nominal_x) || !std::isfinite(nominal_y) ||
    !std::isfinite(nominal_yaw))
  {
    throw std::invalid_argument("Nominal odometry pose must be finite");
  }

  OdometryDisturbanceOutput output;
  // Preserve clean and disturbed poses together for transparent CSV logging.
  output.nominal_x = nominal_x;
  output.nominal_y = nominal_y;
  output.nominal_yaw = wrap_angle(nominal_yaw);
  output.applied_x = output.nominal_x;
  output.applied_y = output.nominal_y;
  output.applied_yaw = output.nominal_yaw;
  output.fault_active = is_active(elapsed_time);

  if (output.fault_active) {
    // Independent standard-normal draws are scaled by configured standard
    // deviations and combined with deterministic sensor biases.
    output.x_perturbation = config_.x_bias +
      config_.position_noise_standard_deviation * standard_normal_(random_engine_);
    output.y_perturbation = config_.y_bias +
      config_.position_noise_standard_deviation * standard_normal_(random_engine_);
    output.yaw_perturbation = config_.yaw_bias +
      config_.yaw_noise_standard_deviation * standard_normal_(random_engine_);
    output.applied_x += output.x_perturbation;
    output.applied_y += output.y_perturbation;
    output.applied_yaw = wrap_angle(output.applied_yaw + output.yaw_perturbation);
  }
  return output;
}

const OdometryDisturbanceConfig & OdometryDisturbance::config() const
{
  return config_;
}

}  // namespace my_robot_controller
