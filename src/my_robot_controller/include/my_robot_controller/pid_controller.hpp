#ifndef MY_ROBOT_CONTROLLER__PID_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLER__PID_CONTROLLER_HPP_

namespace my_robot_controller
{

/// Generic single-input, single-output PID controller.
///
/// The controller evaluates
///   u = Kp * e + Ki * integral(e dt) + Kd * de/dt
/// and clamps the stored integral state to limit windup. Keeping this small
/// controller independent of ROS allows both path-following implementations to
/// use identical PID mathematics and makes the cascade independently testable.
class PIDController
{
public:
  PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0, double max_i = 0.0);

  /// Replace all gains and clear the accumulated integral/derivative state.
  void configure(double kp, double ki, double kd, double max_i);

  /// Calculate one control output from the current error and elapsed time.
  double calculate(double error, double dt);

  /// Clear integral and derivative memory after discontinuities or timeouts.
  void reset();

private:
  // Controller gains and the absolute integral-state limit.
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};
  double max_i_{0.0};

  // State retained between consecutive controller updates.
  double integral_{0.0};
  double previous_error_{0.0};
  bool has_previous_error_{false};
};

}  // namespace my_robot_controller

#endif  // MY_ROBOT_CONTROLLER__PID_CONTROLLER_HPP_
