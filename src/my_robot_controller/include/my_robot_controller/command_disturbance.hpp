#ifndef MY_ROBOT_CONTROLLER__COMMAND_DISTURBANCE_HPP_
#define MY_ROBOT_CONTROLLER__COMMAND_DISTURBANCE_HPP_

namespace my_robot_controller
{

/// Reproducible actuator-command fault applied downstream of any controller.
struct CommandDisturbanceConfig
{
  /// Seconds from the first nominal command until the fault starts.
  double start_delay{5.0};

  /// Duration of the fault window in simulation seconds.
  double duration{0.5};

  /// Additive forward-velocity fault. Kept at zero for the swerve benchmark.
  double linear_velocity_bias{0.0};

  /// Additive yaw-rate fault used to force a temporary erroneous swerve.
  double angular_velocity_bias{0.5};

  /// Symmetric safety limits applied to the command sent to Gazebo.
  double maximum_abs_linear_velocity{1.0};
  double maximum_abs_angular_velocity{1.5};
};

/// Nominal and applied values from one command-path update.
struct CommandDisturbanceOutput
{
  double nominal_linear_velocity{0.0};
  double nominal_angular_velocity{0.0};
  double applied_linear_velocity{0.0};
  double applied_angular_velocity{0.0};
  bool fault_active{false};
};

/// Stateless command-bias schedule independent of ROS, Gazebo, and controller.
///
/// Keeping this policy separate means PID, LQR, and MPC can be tested with the
/// exact same downstream fault. The controller never receives advance warning;
/// it observes only the resulting pose and heading errors through odometry.
class CommandDisturbance
{
public:
  explicit CommandDisturbance(const CommandDisturbanceConfig & config = {});

  /// Validate and replace the complete fault schedule and safety limits.
  void configure(const CommandDisturbanceConfig & config);

  /// Apply the scheduled bias at a time relative to the first nominal command.
  CommandDisturbanceOutput apply(
    double nominal_linear_velocity,
    double nominal_angular_velocity,
    double elapsed_time) const;

  const CommandDisturbanceConfig & config() const;

private:
  void validate_config(const CommandDisturbanceConfig & config) const;

  CommandDisturbanceConfig config_;
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__COMMAND_DISTURBANCE_HPP_
