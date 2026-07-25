#include "my_robot_controller/command_disturbance.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

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
    declare_parameter<double>("linear_velocity_bias", 0.0);
    declare_parameter<double>("angular_velocity_bias", 0.5);
    declare_parameter<double>("maximum_abs_linear_velocity", 1.0);
    declare_parameter<double>("maximum_abs_angular_velocity", 1.5);
    declare_parameter<double>("input_timeout", 2.0);
    declare_parameter<std::string>(
      "output_csv_path", "command_disturbance_actual_commands.csv");

    my_robot_controller::CommandDisturbanceConfig config;
    config.start_delay = get_parameter("fault_start_delay").as_double();
    config.duration = get_parameter("fault_duration").as_double();
    config.linear_velocity_bias = get_parameter("linear_velocity_bias").as_double();
    config.angular_velocity_bias = get_parameter("angular_velocity_bias").as_double();
    config.maximum_abs_linear_velocity =
      get_parameter("maximum_abs_linear_velocity").as_double();
    config.maximum_abs_angular_velocity =
      get_parameter("maximum_abs_angular_velocity").as_double();
    disturbance_.configure(config);

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
      "applied_angular_command,fault_active\n";

    applied_command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    nominal_command_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nominal", 10,
      std::bind(
        &CommandDisturbanceInjectorNode::command_callback, this,
        std::placeholders::_1));

    last_input_wall_time_ = std::chrono::steady_clock::now();
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CommandDisturbanceInjectorNode::watchdog_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Command fault scheduled at %.3f s for %.3f s: linear bias %.3f m/s, "
      "angular bias %.3f rad/s",
      config.start_delay, config.duration, config.linear_velocity_bias,
      config.angular_velocity_bias);
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
  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    const double stamp_seconds = now().seconds();
    if (!input_received_ || stamp_seconds < previous_stamp_seconds_) {
      if (input_received_ && stamp_seconds < previous_stamp_seconds_) {
        RCLCPP_WARN(get_logger(), "Simulation clock moved backwards; restarting fault schedule");
      }
      first_stamp_seconds_ = stamp_seconds;
      previous_fault_active_ = false;
    }

    const double elapsed_time = stamp_seconds - first_stamp_seconds_;
    my_robot_controller::CommandDisturbanceOutput output;
    try {
      output = disturbance_.apply(message->linear.x, message->angular.z, elapsed_time);
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
      elapsed_time << ',' << output.nominal_linear_velocity << ',' <<
      output.nominal_angular_velocity << ',' << output.applied_linear_velocity << ',' <<
      output.applied_angular_velocity << ',' << (output.fault_active ? 1 : 0) << '\n';

    if (output.fault_active != previous_fault_active_) {
      if (output.fault_active) {
        RCLCPP_WARN(
          get_logger(), "Command fault started at %.3f s: angular bias %.3f rad/s",
          elapsed_time, disturbance_.config().angular_velocity_bias);
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
  std::ofstream command_csv_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr applied_command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nominal_command_subscriber_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  double input_timeout_{2.0};
  double first_stamp_seconds_{0.0};
  double previous_stamp_seconds_{0.0};
  bool input_received_{false};
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
