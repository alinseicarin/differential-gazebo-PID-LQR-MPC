#include "my_robot_controller/odometry_disturbance.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
double yaw_from_odometry(const nav_msgs::msg::Odometry & message)
{
  const auto & q = message.pose.pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Inserts a known error after the EKF and before the controller only.
///
/// The evaluator remains subscribed to the clean /odometry/filtered topic, so
/// physical ground-truth metrics never mistake injected information error for
/// actual robot displacement.
class OdometryDisturbanceInjectorNode : public rclcpp::Node
{
public:
  OdometryDisturbanceInjectorNode()
  : Node("odometry_disturbance_injector")
  {
    declare_parameter<bool>("fault_enabled", false);
    declare_parameter<double>("fault_start_delay", 5.0);
    declare_parameter<double>("fault_duration", 5.0);
    declare_parameter<double>("x_bias", 0.0);
    declare_parameter<double>("y_bias", 0.0);
    declare_parameter<double>("yaw_bias", 0.0);
    declare_parameter<double>("position_noise_standard_deviation", 0.0);
    declare_parameter<double>("yaw_noise_standard_deviation", 0.0);
    declare_parameter<int>("random_seed", 2026);
    declare_parameter<std::string>(
      "output_csv_path", "odometry_disturbance_samples.csv");

    my_robot_controller::OdometryDisturbanceConfig config;
    config.enabled = get_parameter("fault_enabled").as_bool();
    config.start_delay = get_parameter("fault_start_delay").as_double();
    config.duration = get_parameter("fault_duration").as_double();
    config.x_bias = get_parameter("x_bias").as_double();
    config.y_bias = get_parameter("y_bias").as_double();
    config.yaw_bias = get_parameter("yaw_bias").as_double();
    config.position_noise_standard_deviation =
      get_parameter("position_noise_standard_deviation").as_double();
    config.yaw_noise_standard_deviation =
      get_parameter("yaw_noise_standard_deviation").as_double();
    const int seed = get_parameter("random_seed").as_int();
    if (seed < 0) {
      throw std::runtime_error("Odometry disturbance random_seed must be non-negative");
    }
    config.random_seed = static_cast<unsigned int>(seed);
    disturbance_.configure(config);

    const std::string output_path = get_parameter("output_csv_path").as_string();
    if (output_path.empty()) {
      throw std::runtime_error("Odometry-disturbance output path must not be empty");
    }
    output_csv_.open(output_path, std::ios::out | std::ios::trunc);
    if (!output_csv_.is_open()) {
      throw std::runtime_error("Could not create odometry-disturbance CSV: " + output_path);
    }
    output_csv_ << std::setprecision(12);
    output_csv_ <<
      "time,stamp,nominal_x,nominal_y,nominal_yaw,applied_x,applied_y,applied_yaw,"
      "x_perturbation,y_perturbation,yaw_perturbation,fault_active\n";

    disturbed_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "odometry/filtered_disturbed", 10);
    clean_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10,
      std::bind(
        &OdometryDisturbanceInjectorNode::odometry_callback, this,
        std::placeholders::_1));
    experiment_start_subscriber_ = create_subscription<std_msgs::msg::Float64>(
      "experiment_start_time", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &OdometryDisturbanceInjectorNode::experiment_start_callback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Odometry fault enabled=%s, start=%.3f s, duration=%.3f s, "
      "position sigma=%.4f m, yaw sigma=%.4f rad, seed=%u",
      config.enabled ? "true" : "false", config.start_delay, config.duration,
      config.position_noise_standard_deviation,
      config.yaw_noise_standard_deviation, config.random_seed);
    RCLCPP_INFO(get_logger(), "Writing disturbed odometry to %s", output_path.c_str());
  }

  ~OdometryDisturbanceInjectorNode() override
  {
    if (output_csv_.is_open()) {
      output_csv_.flush();
      output_csv_.close();
    }
  }

private:
  void experiment_start_callback(const std_msgs::msg::Float64::SharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid experiment start time");
      return;
    }
    experiment_start_stamp_ = message->data;
    experiment_started_ = true;
    previous_fault_active_ = false;
    disturbance_.reset();
    RCLCPP_INFO(
      get_logger(), "Odometry-fault schedule synchronized at %.6f s",
      experiment_start_stamp_);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    const double nominal_yaw = yaw_from_odometry(*message);
    if (!std::isfinite(stamp) || !std::isfinite(message->pose.pose.position.x) ||
      !std::isfinite(message->pose.pose.position.y) || !std::isfinite(nominal_yaw))
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid clean odometry sample");
      return;
    }

    nav_msgs::msg::Odometry disturbed_message = *message;
    my_robot_controller::OdometryDisturbanceOutput output;
    if (experiment_started_ && stamp >= experiment_start_stamp_) {
      const double elapsed_time = stamp - experiment_start_stamp_;
      output = disturbance_.apply(
        message->pose.pose.position.x,
        message->pose.pose.position.y,
        nominal_yaw,
        elapsed_time);
      disturbed_message.pose.pose.position.x = output.applied_x;
      disturbed_message.pose.pose.position.y = output.applied_y;
      // The controller is planar; replace orientation by the perturbed yaw and
      // leave pose/twist covariance unchanged so the injected error itself is
      // the sole controlled variable in this benchmark.
      disturbed_message.pose.pose.orientation.x = 0.0;
      disturbed_message.pose.pose.orientation.y = 0.0;
      disturbed_message.pose.pose.orientation.z = std::sin(0.5 * output.applied_yaw);
      disturbed_message.pose.pose.orientation.w = std::cos(0.5 * output.applied_yaw);

      output_csv_ << elapsed_time << ',' << stamp << ',' << output.nominal_x << ',' <<
        output.nominal_y << ',' << output.nominal_yaw << ',' << output.applied_x << ',' <<
        output.applied_y << ',' << output.applied_yaw << ',' << output.x_perturbation << ',' <<
        output.y_perturbation << ',' << output.yaw_perturbation << ',' <<
        (output.fault_active ? 1 : 0) << '\n';

      if (output.fault_active != previous_fault_active_) {
        if (output.fault_active) {
          RCLCPP_WARN(get_logger(), "Odometry fault started at %.3f s", elapsed_time);
        } else {
          RCLCPP_INFO(get_logger(), "Odometry fault ended at %.3f s", elapsed_time);
        }
        output_csv_.flush();
      }
      previous_fault_active_ = output.fault_active;
    }

    disturbed_publisher_->publish(disturbed_message);
  }

  my_robot_controller::OdometryDisturbance disturbance_;
  std::ofstream output_csv_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr disturbed_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr clean_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr experiment_start_subscriber_;
  double experiment_start_stamp_{0.0};
  bool experiment_started_{false};
  bool previous_fault_active_{false};
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<OdometryDisturbanceInjectorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("odometry_disturbance_injector"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
