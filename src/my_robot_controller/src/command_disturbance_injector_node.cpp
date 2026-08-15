#include "my_robot_controller/command_disturbance.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

/// ROS adapter placed between a controller and the differential-drive plugin.
///
/// Input:  /cmd_vel_nominal -- untouched output of PID, LQR, or MPC
/// Output: /cmd_vel         -- command actually received by Gazebo
///
/// A scheduled additive bias emulates a temporary actuator or command-path
/// fault without applying forces that change wheel loading or ground contact.
class CommandDisturbanceInjectorNode : public rclcpp::Node
{
public:
  CommandDisturbanceInjectorNode()
  : Node("command_disturbance_injector")
  {
    declare_parameter<double>("fault_start_delay", 5.0);
    declare_parameter<double>("fault_duration", 0.5);
    declare_parameter<bool>("fault_enabled", true);
    declare_parameter<double>("linear_velocity_bias", 0.0);
    declare_parameter<double>("angular_velocity_bias", 0.5);
    declare_parameter<double>("left_wheel_effectiveness", 1.0);
    declare_parameter<double>("right_wheel_effectiveness", 1.0);
    declare_parameter<double>("wheel_separation", 0.35);
    declare_parameter<double>("command_delay", 0.0);
    declare_parameter<double>("maximum_abs_linear_velocity", 1.0);
    declare_parameter<double>("maximum_abs_angular_velocity", 1.5);
    declare_parameter<double>("input_timeout", 2.0);
    declare_parameter<std::string>(
      "output_csv_path", "command_disturbance_actual_commands.csv");

    my_robot_controller::CommandDisturbanceConfig config;
    config.enabled = get_parameter("fault_enabled").as_bool();
    config.start_delay = get_parameter("fault_start_delay").as_double();
    config.duration = get_parameter("fault_duration").as_double();
    config.linear_velocity_bias = get_parameter("linear_velocity_bias").as_double();
    config.angular_velocity_bias = get_parameter("angular_velocity_bias").as_double();
    config.left_wheel_effectiveness =
      get_parameter("left_wheel_effectiveness").as_double();
    config.right_wheel_effectiveness =
      get_parameter("right_wheel_effectiveness").as_double();
    config.wheel_separation = get_parameter("wheel_separation").as_double();
    config.maximum_abs_linear_velocity =
      get_parameter("maximum_abs_linear_velocity").as_double();
    config.maximum_abs_angular_velocity =
      get_parameter("maximum_abs_angular_velocity").as_double();
    disturbance_.configure(config);
    command_delay_.configure(get_parameter("command_delay").as_double());

    input_timeout_ = get_parameter("input_timeout").as_double();
    if (!std::isfinite(input_timeout_) || input_timeout_ <= 0.0) {
      throw std::runtime_error("input_timeout must be finite and positive");
    }

    const std::string output_path = get_parameter("output_csv_path").as_string();
    if (output_path.empty()) {
      throw std::runtime_error("Command-disturbance output_csv_path must not be empty");
    }
    command_csv_.open(output_path, std::ios::out | std::ios::trunc);
    if (!command_csv_.is_open()) {
      throw std::runtime_error("Could not create command-disturbance CSV: " + output_path);
    }
    command_csv_ << std::setprecision(10);
    command_csv_ <<
      "time,nominal_linear_command,nominal_angular_command,applied_linear_command,"
      "applied_angular_command,fault_active,stamp,delay_source_linear_command,"
      "delay_source_angular_command,delay_source_time,configured_command_delay,"
      "nominal_left_wheel_velocity,nominal_right_wheel_velocity,"
      "effective_left_wheel_velocity,effective_right_wheel_velocity\n";

    applied_command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    experiment_start_subscriber_ = create_subscription<std_msgs::msg::Float64>(
      "experiment_start_time", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &CommandDisturbanceInjectorNode::experiment_start_callback, this,
        std::placeholders::_1));

    // Do not expose the upstream subscription until this output publisher is
    // matched to Gazebo. The PID waits for its own subscriber, so delaying this
    // subscription creates a transitive readiness handshake across the entire
    // PID -> injector -> Gazebo command path.
    readiness_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CommandDisturbanceInjectorNode::readiness_callback, this));

    last_input_wall_time_ = std::chrono::steady_clock::now();
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CommandDisturbanceInjectorNode::watchdog_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Command fault enabled=%s, start=%.3f s, duration=%.3f s, "
      "bias=(%.3f m/s, %.3f rad/s), wheel effectiveness=(%.3f, %.3f), delay=%.3f s",
      config.enabled ? "true" : "false", config.start_delay, config.duration,
      config.linear_velocity_bias, config.angular_velocity_bias,
      config.left_wheel_effectiveness, config.right_wheel_effectiveness,
      command_delay_.delay());
    RCLCPP_INFO(get_logger(), "Writing applied commands to %s", output_path.c_str());
  }

  ~CommandDisturbanceInjectorNode() override
  {
    publish_stop();
    if (command_csv_.is_open()) {
      command_csv_.close();
    }
  }

  void stop()
  {
    publish_stop();
  }

private:
  void experiment_start_callback(const std_msgs::msg::Float64::SharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid experiment start time");
      return;
    }

    experiment_start_stamp_seconds_ = message->data;
    experiment_started_ = true;
    previous_fault_active_ = false;
    command_delay_.reset();
    RCLCPP_INFO(
      get_logger(), "Command-fault schedule synchronized at %.6f s",
      experiment_start_stamp_seconds_);
  }

  void readiness_callback()
  {
    if (nominal_command_subscriber_ ||
      applied_command_publisher_->get_subscription_count() == 0u)
    {
      return;
    }

    nominal_command_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nominal", 10,
      std::bind(
        &CommandDisturbanceInjectorNode::command_callback, this,
        std::placeholders::_1));
    readiness_timer_->cancel();
    RCLCPP_INFO(
      get_logger(), "Gazebo command subscriber connected; accepting nominal commands");
  }

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const double stamp_seconds = now().seconds();
    if (!std::isfinite(stamp_seconds) || !std::isfinite(message->linear.x) ||
      !std::isfinite(message->angular.z))
    {
      RCLCPP_ERROR(get_logger(), "Ignoring non-finite nominal command or simulation time");
      publish_stop();
      return;
    }

    if (input_received_ && stamp_seconds < previous_stamp_seconds_) {
      RCLCPP_WARN(
        get_logger(),
        "Simulation clock moved backwards; waiting for a new experiment-start message");
      experiment_started_ = false;
      previous_fault_active_ = false;
      command_delay_.reset();
    }

    // Startup zero commands are required by the readiness handshake, but they
    // are outside the measured experiment and must not advance the fault
    // schedule. Forward them transparently and start both scheduling and CSV
    // time only from the controller's common experiment-start announcement.
    if (!experiment_started_ || stamp_seconds < experiment_start_stamp_seconds_) {
      const auto & config = disturbance_.config();
      const double inactive_time = config.start_delay + config.duration + 1.0;
      try {
        const auto output = disturbance_.apply(
          message->linear.x, message->angular.z, inactive_time);
        geometry_msgs::msg::Twist applied_message = *message;
        applied_message.linear.x = output.applied_linear_velocity;
        applied_message.angular.z = output.applied_angular_velocity;
        applied_command_publisher_->publish(applied_message);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Invalid nominal command: %s", error.what());
        publish_stop();
        return;
      }

      input_received_ = true;
      stop_sent_ = false;
      previous_stamp_seconds_ = stamp_seconds;
      last_input_wall_time_ = std::chrono::steady_clock::now();
      return;
    }

    const double elapsed_time = stamp_seconds - experiment_start_stamp_seconds_;
    my_robot_controller::CommandDisturbanceOutput output;
    my_robot_controller::DelayedCommandOutput delayed_output;
    try {
      const bool fault_active = disturbance_.is_active(elapsed_time);
      delayed_output = command_delay_.apply(
        message->linear.x, message->angular.z, elapsed_time, fault_active);
      output = disturbance_.apply(
        delayed_output.source_linear_velocity,
        delayed_output.source_angular_velocity,
        elapsed_time);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Invalid nominal command: %s", error.what());
      publish_stop();
      return;
    }

    geometry_msgs::msg::Twist applied_message = *message;
    applied_message.linear.x = output.applied_linear_velocity;
    applied_message.angular.z = output.applied_angular_velocity;
    applied_command_publisher_->publish(applied_message);

    command_csv_ <<
      elapsed_time << ',' << message->linear.x << ',' << message->angular.z << ',' <<
      output.applied_linear_velocity << ',' <<
      output.applied_angular_velocity << ',' << (output.fault_active ? 1 : 0) << ',' <<
      stamp_seconds << ',' << delayed_output.source_linear_velocity << ',' <<
      delayed_output.source_angular_velocity << ',' << delayed_output.source_time << ',' <<
      command_delay_.delay() << ',' << output.nominal_left_wheel_velocity << ',' <<
      output.nominal_right_wheel_velocity << ',' <<
      output.effective_left_wheel_velocity << ',' <<
      output.effective_right_wheel_velocity << '\n';

    if (output.fault_active != previous_fault_active_) {
      if (output.fault_active) {
        RCLCPP_WARN(
          get_logger(), "Command fault started at %.3f s", elapsed_time);
      } else {
        RCLCPP_INFO(get_logger(), "Command fault ended at %.3f s", elapsed_time);
      }
      command_csv_.flush();
    }

    input_received_ = true;
    stop_sent_ = false;
    previous_fault_active_ = output.fault_active;
    previous_stamp_seconds_ = stamp_seconds;
    last_input_wall_time_ = std::chrono::steady_clock::now();
  }

  void watchdog_callback()
  {
    if (!input_received_ || stop_sent_) {
      return;
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_input_wall_time_).count();
    if (elapsed > input_timeout_) {
      RCLCPP_WARN(get_logger(), "Nominal command timed out; publishing zero velocity");
      publish_stop();
    }
  }

  void publish_stop()
  {
    if (!applied_command_publisher_) {
      return;
    }
    applied_command_publisher_->publish(geometry_msgs::msg::Twist());
    stop_sent_ = true;
  }

  my_robot_controller::CommandDisturbance disturbance_;
  my_robot_controller::CommandDelay command_delay_;
  std::ofstream command_csv_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr applied_command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nominal_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr experiment_start_subscriber_;
  rclcpp::TimerBase::SharedPtr readiness_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  double input_timeout_{2.0};
  double experiment_start_stamp_seconds_{0.0};
  double previous_stamp_seconds_{0.0};
  bool input_received_{false};
  bool experiment_started_{false};
  bool previous_fault_active_{false};
  bool stop_sent_{false};
  std::chrono::steady_clock::time_point last_input_wall_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;

  try {
    auto node = std::make_shared<CommandDisturbanceInjectorNode>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("command_disturbance_injector"), "%s", error.what());
    result = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
