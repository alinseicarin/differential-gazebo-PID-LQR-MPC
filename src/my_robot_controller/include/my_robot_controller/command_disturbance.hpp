#ifndef MY_ROBOT_CONTROLLER__COMMAND_DISTURBANCE_HPP_
#define MY_ROBOT_CONTROLLER__COMMAND_DISTURBANCE_HPP_

#include <cstddef>
#include <deque>

namespace my_robot_controller
{

/// Reproducible actuator-command fault applied downstream of any controller.
struct CommandDisturbanceConfig
{
  /// Disable every command modification while retaining the same ROS graph.
  bool enabled{true};

  /// Seconds from the controller-published experiment start until the fault starts.
  double start_delay{5.0};

  /// Duration of the fault window in simulation seconds.
  double duration{0.5};

  /// Additive forward-velocity fault. Kept at zero for the swerve benchmark.
  double linear_velocity_bias{0.0};

  /// Additive yaw-rate fault used to force a temporary erroneous swerve.
  double angular_velocity_bias{0.5};

  /// Multiplicative actuator-effectiveness factors applied in wheel space.
  /// A value of 1 means healthy; 0 means that wheel produces no commanded
  /// longitudinal velocity during the fault window.
  double left_wheel_effectiveness{1.0};
  double right_wheel_effectiveness{1.0};

  /// Distance between the two drive-wheel contact planes.
  double wheel_separation{0.35};

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
  double nominal_left_wheel_velocity{0.0};
  double nominal_right_wheel_velocity{0.0};
  double effective_left_wheel_velocity{0.0};
  double effective_right_wheel_velocity{0.0};
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

  /// Apply the scheduled bias at a time relative to the common experiment start.
  CommandDisturbanceOutput apply(
    double nominal_linear_velocity,
    double nominal_angular_velocity,
    double elapsed_time) const;

  /// Report the common schedule state without modifying a command.
  bool is_active(double elapsed_time) const;

  const CommandDisturbanceConfig & config() const;

private:
  void validate_config(const CommandDisturbanceConfig & config) const;

  CommandDisturbanceConfig config_;
};

/// Result of selecting the command available through a delayed transport.
struct DelayedCommandOutput
{
  double current_linear_velocity{0.0};
  double current_angular_velocity{0.0};
  double source_linear_velocity{0.0};
  double source_angular_velocity{0.0};
  double source_time{0.0};
  bool delay_active{false};
};

/// Simulation-time command delay with deterministic zero-order hold.
///
/// Every current command is retained with its elapsed experiment time. During
/// the scheduled fault, the newest sample not newer than `time-delay` is sent
/// downstream. Outside the window the current command passes through.
class CommandDelay
{
public:
  explicit CommandDelay(double delay = 0.0);

  void configure(double delay);
  void reset();

  DelayedCommandOutput apply(
    double current_linear_velocity,
    double current_angular_velocity,
    double elapsed_time,
    bool fault_active);

  double delay() const;

private:
  struct TimedCommand
  {
    double time;
    double linear_velocity;
    double angular_velocity;
  };

  double delay_{0.0};
  std::deque<TimedCommand> history_;
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__COMMAND_DISTURBANCE_HPP_
