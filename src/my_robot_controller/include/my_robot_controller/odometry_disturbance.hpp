#ifndef MY_ROBOT_CONTROLLER__ODOMETRY_DISTURBANCE_HPP_
#define MY_ROBOT_CONTROLLER__ODOMETRY_DISTURBANCE_HPP_

#include <random>

namespace my_robot_controller
{

/// Reproducible corruption applied to the planar pose seen by a controller.
struct OdometryDisturbanceConfig
{
  /// Master switch leaves the injector in the graph as an exact pass-through.
  bool enabled{false};
  /// Fault window relative to the common experiment start [s].
  double start_delay{5.0};
  double duration{5.0};
  /// Constant world-frame position biases [m] and yaw bias [rad].
  double x_bias{0.0};
  double y_bias{0.0};
  double yaw_bias{0.0};
  /// Standard deviations of independent zero-mean Gaussian samples.
  double position_noise_standard_deviation{0.0};
  double yaw_noise_standard_deviation{0.0};
  unsigned int random_seed{2026u};
};

/// Both clean and corrupted values from one update, retained for audit logs.
struct OdometryDisturbanceOutput
{
  double nominal_x{0.0};
  double nominal_y{0.0};
  double nominal_yaw{0.0};
  double applied_x{0.0};
  double applied_y{0.0};
  double applied_yaw{0.0};
  double x_perturbation{0.0};
  double y_perturbation{0.0};
  double yaw_perturbation{0.0};
  bool fault_active{false};
};

/// Stateful Gaussian-noise and bias policy independent of ROS messages.
class OdometryDisturbance
{
public:
  explicit OdometryDisturbance(const OdometryDisturbanceConfig & config = {});

  /// Validate the schedule and restart the deterministic random sequence.
  void configure(const OdometryDisturbanceConfig & config);
  /// Return the pseudo-random generator to the configured seed.
  void reset();

  OdometryDisturbanceOutput apply(
    double nominal_x,
    double nominal_y,
    double nominal_yaw,
    double elapsed_time);

  bool is_active(double elapsed_time) const;
  const OdometryDisturbanceConfig & config() const;

private:
  void validate_config(const OdometryDisturbanceConfig & config) const;

  OdometryDisturbanceConfig config_;
  // Mersenne Twister plus a unit normal distribution generate reproducible
  // samples later scaled independently for position and yaw.
  std::mt19937 random_engine_;
  std::normal_distribution<double> standard_normal_{0.0, 1.0};
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__ODOMETRY_DISTURBANCE_HPP_
