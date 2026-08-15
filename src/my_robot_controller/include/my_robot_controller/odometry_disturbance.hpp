#ifndef MY_ROBOT_CONTROLLER__ODOMETRY_DISTURBANCE_HPP_
#define MY_ROBOT_CONTROLLER__ODOMETRY_DISTURBANCE_HPP_

#include <random>

namespace my_robot_controller
{

/// Reproducible corruption applied to the planar pose seen by a controller.
struct OdometryDisturbanceConfig
{
  bool enabled{false};
  double start_delay{5.0};
  double duration{5.0};
  double x_bias{0.0};
  double y_bias{0.0};
  double yaw_bias{0.0};
  double position_noise_standard_deviation{0.0};
  double yaw_noise_standard_deviation{0.0};
  unsigned int random_seed{2026u};
};

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

  void configure(const OdometryDisturbanceConfig & config);
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
  std::mt19937 random_engine_;
  std::normal_distribution<double> standard_normal_{0.0, 1.0};
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__ODOMETRY_DISTURBANCE_HPP_
