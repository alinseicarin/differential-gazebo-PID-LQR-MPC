#ifndef MY_ROBOT_CONTROLLER__WHEEL_SLIP_METRICS_HPP_
#define MY_ROBOT_CONTROLLER__WHEEL_SLIP_METRICS_HPP_

namespace my_robot_controller
{

/// Inputs needed to compare differential-drive wheel motion with body truth.
struct WheelSlipInput
{
  double left_wheel_angular_velocity{0.0};
  double right_wheel_angular_velocity{0.0};
  double truth_world_linear_x{0.0};
  double truth_world_linear_y{0.0};
  double truth_yaw{0.0};
  double truth_angular_velocity{0.0};
};

/// Geometric and numerical settings of the differential-drive model.
struct WheelSlipConfig
{
  double wheel_radius{0.1};
  double wheel_separation{0.35};
  double minimum_speed_denominator{0.05};
};

/// Dimensioned velocities and dimensionless discrepancies under pure rolling.
struct WheelSlipResult
{
  double left_wheel_tangential_velocity{0.0};
  double right_wheel_tangential_velocity{0.0};
  double wheel_kinematic_linear_velocity{0.0};
  double wheel_kinematic_angular_velocity{0.0};
  double truth_body_longitudinal_velocity{0.0};
  double truth_body_lateral_velocity{0.0};
  double left_longitudinal_slip_ratio{0.0};
  double right_longitudinal_slip_ratio{0.0};
  double center_longitudinal_slip_ratio{0.0};
  double yaw_velocity_discrepancy_ratio{0.0};
  double sideslip_angle{0.0};
};

/// Compare measured wheel rotation with the planar Gazebo body twist.
WheelSlipResult calculate_wheel_slip_metrics(
  const WheelSlipInput & input, const WheelSlipConfig & config);

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__WHEEL_SLIP_METRICS_HPP_
