#include <cmath>
#include <cstddef>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "my_robot_controller/trajectory_reference_manager.hpp"

namespace
{

double yaw_from_odometry(const nav_msgs::msg::Odometry & message)
{
  const auto & q = message.pose.pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool finite_planar_pose(const nav_msgs::msg::Odometry & message)
{
  return std::isfinite(message.pose.pose.position.x) &&
         std::isfinite(message.pose.pose.position.y) &&
         std::isfinite(yaw_from_odometry(message));
}

struct PlanarPoseSample
{
  double stamp;
  double x;
  double y;
  double yaw;
};

/// Controller-independent comparison of EKF state and Gazebo world truth.
class TrajectoryEvaluatorNode : public rclcpp::Node
{
public:
  TrajectoryEvaluatorNode()
  : Node("trajectory_evaluator_node")
  {
    declare_parameter<std::string>("csv_path", "");
    declare_parameter<std::string>("output_csv_path", "ground_truth_trajectory.csv");
    declare_parameter<int>("search_window", 20);
    declare_parameter<double>("reference_linear_velocity", 0.4);
    declare_parameter<double>("curvature_speed_gain", 0.5);
    declare_parameter<double>("endpoint_slowdown_distance", 0.0);
    declare_parameter<double>("maximum_reference_curvature", 5.0);
    declare_parameter<double>("trajectory_spatial_step", 0.01);
    declare_parameter<double>("maximum_reference_linear_acceleration", 0.5);
    declare_parameter<double>("maximum_reference_linear_deceleration", 0.5);
    declare_parameter<double>("maximum_reference_angular_velocity", 1.5);

    const std::string path_file = get_parameter("csv_path").as_string();
    const std::string output_file = get_parameter("output_csv_path").as_string();
    const int search_window = get_parameter("search_window").as_int();
    if (path_file.empty() || output_file.empty()) {
      throw std::runtime_error("Evaluator path and output CSV parameters cannot be empty");
    }
    if (path_file == output_file) {
      throw std::runtime_error("Evaluator input and output CSV paths must be different");
    }
    if (search_window < 1) {
      throw std::runtime_error("Evaluator search_window must be positive");
    }

    my_robot_controller::TrajectoryReferenceConfig reference_config;
    reference_config.path.search_window = static_cast<std::size_t>(search_window);
    reference_config.path.nominal_linear_velocity =
      get_parameter("reference_linear_velocity").as_double();
    reference_config.path.curvature_speed_gain =
      get_parameter("curvature_speed_gain").as_double();
    reference_config.path.endpoint_slowdown_distance =
      get_parameter("endpoint_slowdown_distance").as_double();
    reference_config.path.maximum_abs_curvature =
      get_parameter("maximum_reference_curvature").as_double();
    reference_config.spatial_step = get_parameter("trajectory_spatial_step").as_double();
    reference_config.maximum_linear_acceleration =
      get_parameter("maximum_reference_linear_acceleration").as_double();
    reference_config.maximum_linear_deceleration =
      get_parameter("maximum_reference_linear_deceleration").as_double();
    reference_config.maximum_reference_angular_velocity =
      get_parameter("maximum_reference_angular_velocity").as_double();
    reference_manager_.configure(reference_config);
    reference_manager_.load_csv(path_file);

    output_csv_.open(output_file, std::ios::out | std::ios::trunc);
    if (!output_csv_.is_open()) {
      throw std::runtime_error("Could not create ground-truth evaluation CSV: " + output_file);
    }
    output_csv_ << std::setprecision(12);
    output_csv_ <<
      "time,truth_stamp,truth_x,truth_y,truth_yaw,reference_x,reference_y,"
      "reference_yaw,reference_linear_velocity,reference_angular_velocity,"
      "true_longitudinal_error,true_lateral_error,true_trajectory_heading_error,"
      "true_position_error,projection_x,projection_y,projection_yaw,"
      "true_cross_track_error,true_path_heading_error,reference_progress,"
      "projection_progress,remaining_path_length,filtered_stamp,filtered_age,"
      "estimated_x,estimated_y,"
      "estimated_yaw,aligned_truth_x,aligned_truth_y,aligned_truth_yaw,"
      "localization_position_error,localization_heading_error,"
      "truth_linear_velocity,truth_angular_velocity\n";

    filtered_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10,
      std::bind(&TrajectoryEvaluatorNode::filtered_callback, this, std::placeholders::_1));
    truth_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      "ground_truth/odom", 10,
      std::bind(&TrajectoryEvaluatorNode::truth_callback, this, std::placeholders::_1));
    experiment_start_subscriber_ = create_subscription<std_msgs::msg::Float64>(
      "experiment_start_time", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &TrajectoryEvaluatorNode::experiment_start_callback, this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Ground-truth evaluator loaded %zu waypoints (%.3f s reference) and writes to %s",
      reference_manager_.waypoint_count(), reference_manager_.duration(), output_file.c_str());
  }

  ~TrajectoryEvaluatorNode() override
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
      RCLCPP_WARN(get_logger(), "Ignoring non-finite experiment start timestamp");
      return;
    }
    experiment_start_stamp_ = message->data;
    experiment_started_ = true;
    reference_manager_.reset_projection();
    RCLCPP_INFO(
      get_logger(), "Ground-truth trajectory evaluation starts at %.6f s",
      experiment_start_stamp_);
  }

  bool interpolate_truth_at(double stamp, PlanarPoseSample & result) const
  {
    if (truth_history_.size() < 2u || stamp < truth_history_.front().stamp ||
      stamp > truth_history_.back().stamp)
    {
      return false;
    }

    for (std::size_t index = 1u; index < truth_history_.size(); ++index) {
      const auto & before = truth_history_[index - 1u];
      const auto & after = truth_history_[index];
      if (stamp > after.stamp) {
        continue;
      }

      const double interval = after.stamp - before.stamp;
      if (interval <= 0.0) {
        return false;
      }
      const double fraction = (stamp - before.stamp) / interval;
      result.stamp = stamp;
      result.x = before.x + fraction * (after.x - before.x);
      result.y = before.y + fraction * (after.y - before.y);
      result.yaw = my_robot_controller::wrap_angle(
        before.yaw + fraction * my_robot_controller::wrap_angle(after.yaw - before.yaw));
      return true;
    }
    return false;
  }

  void filtered_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!finite_planar_pose(*message)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite filtered pose in evaluator");
      return;
    }
    filtered_stamp_ = rclcpp::Time(message->header.stamp).seconds();
    estimated_x_ = message->pose.pose.position.x;
    estimated_y_ = message->pose.pose.position.y;
    estimated_yaw_ = yaw_from_odometry(*message);
    filtered_received_ = true;
  }

  void truth_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!finite_planar_pose(*message)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite Gazebo truth pose");
      return;
    }
    const double stamp = rclcpp::Time(message->header.stamp).seconds();
    if (truth_received_ && stamp <= previous_truth_stamp_) {
      RCLCPP_WARN(get_logger(), "Ignoring non-increasing ground-truth timestamp");
      return;
    }
    previous_truth_stamp_ = stamp;
    truth_received_ = true;

    const double truth_x = message->pose.pose.position.x;
    const double truth_y = message->pose.pose.position.y;
    const double truth_yaw = yaw_from_odometry(*message);
    // P3D reports linear velocity in the world frame. Rotate its horizontal
    // components onto the robot's forward axis so terminal settling can use the
    // same longitudinal-velocity convention as the controller and encoder.
    const double truth_linear_velocity =
      std::cos(truth_yaw) * message->twist.twist.linear.x +
      std::sin(truth_yaw) * message->twist.twist.linear.y;
    const double truth_angular_velocity = message->twist.twist.angular.z;
    truth_history_.push_back({stamp, truth_x, truth_y, truth_yaw});
    // Twenty seconds is far longer than the expected estimator latency while
    // still placing a fixed upper bound on memory use during long experiments.
    while (truth_history_.size() > 600u) {
      truth_history_.pop_front();
    }

    // Before the controller announces its synchronized start, truth samples are
    // retained only for timestamp-aligned localization evaluation.
    if (!experiment_started_ || stamp < experiment_start_stamp_) {
      return;
    }
    const double elapsed_time = stamp - experiment_start_stamp_;
    const auto reference = reference_manager_.update(
      elapsed_time, truth_x, truth_y, truth_yaw);

    const double missing = std::numeric_limits<double>::quiet_NaN();
    double filtered_age = missing;
    PlanarPoseSample aligned_truth{missing, missing, missing, missing};
    double localization_position_error = missing;
    double localization_heading_error = missing;
    if (filtered_received_) {
      filtered_age = stamp - filtered_stamp_;
      // Compare poses at one simulation timestamp. Comparing the newest two
      // messages directly would turn ordinary publisher latency into a false
      // position error approximately equal to speed times message age.
      if (interpolate_truth_at(filtered_stamp_, aligned_truth)) {
        localization_position_error = std::hypot(
          estimated_x_ - aligned_truth.x, estimated_y_ - aligned_truth.y);
        localization_heading_error = my_robot_controller::wrap_angle(
          estimated_yaw_ - aligned_truth.yaw);
      }
    }

    output_csv_ << elapsed_time << ',' << stamp << ',' <<
      truth_x << ',' << truth_y << ',' << truth_yaw << ',' <<
      reference.trajectory.position.x << ',' << reference.trajectory.position.y << ',' <<
      reference.trajectory.heading << ',' <<
      reference.trajectory.reference_linear_velocity << ',' <<
      reference.trajectory.reference_angular_velocity << ',' <<
      reference.longitudinal_error << ',' << reference.lateral_error << ',' <<
      reference.heading_error << ',' << reference.position_error << ',' <<
      reference.projection.path.position.x << ',' <<
      reference.projection.path.position.y << ',' <<
      reference.projection.path.heading << ',' <<
      reference.projection.cross_track_error << ',' <<
      reference.projection.heading_error << ',' <<
      reference.trajectory.progress << ',' << reference.projection.path.progress << ',' <<
      reference.projection.path.remaining_length << ',' <<
      (filtered_received_ ? filtered_stamp_ : missing) << ',' << filtered_age << ',' <<
      (filtered_received_ ? estimated_x_ : missing) << ',' <<
      (filtered_received_ ? estimated_y_ : missing) << ',' <<
      (filtered_received_ ? estimated_yaw_ : missing) << ',' <<
      aligned_truth.x << ',' << aligned_truth.y << ',' << aligned_truth.yaw << ',' <<
      localization_position_error << ',' << localization_heading_error << ',' <<
      truth_linear_velocity << ',' << truth_angular_velocity << '\n';

    ++sample_count_;
    if (sample_count_ % 30u == 0u) {
      output_csv_.flush();
    }
  }

  my_robot_controller::TrajectoryReferenceManager reference_manager_;
  std::ofstream output_csv_;
  std::deque<PlanarPoseSample> truth_history_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr truth_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr experiment_start_subscriber_;

  double previous_truth_stamp_{0.0};
  double experiment_start_stamp_{0.0};
  double filtered_stamp_{0.0};
  double estimated_x_{0.0};
  double estimated_y_{0.0};
  double estimated_yaw_{0.0};
  bool truth_received_{false};
  bool filtered_received_{false};
  bool experiment_started_{false};
  std::size_t sample_count_{0u};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<TrajectoryEvaluatorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_evaluator_node"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
