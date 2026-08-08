#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rosmaster_a1/pure_pursuit.hpp"

namespace rosmaster_a1
{

class PurePursuitControllerNode : public rclcpp::Node
{
public:
  PurePursuitControllerNode()
  : Node("pure_pursuit_controller_node")
  {
    control_period_s_ = declare_parameter<double>("control_period_s", 0.1);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.8);
    minimum_speed_ = declare_parameter<double>("minimum_speed", 0.1);
    max_speed_ = declare_parameter<double>("max_speed", 1.0);
    max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.6);
    odometry_timeout_s_ = declare_parameter<double>("odometry_timeout_s", 0.5);
    steering_gain_ = declare_parameter<double>("steering_gain", 1.5);
    wheelbase_ = declare_parameter<double>("wheelbase", 0.3);
    validate_parameters();

    command_publisher_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd_drive", 10);
    metrics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/metrics/controller", 10);
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](nav_msgs::msg::Path::SharedPtr message) { path_ = std::move(message); });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr message) { receive_odometry(*message); });
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(control_period_s_), [this]() { update_control(); });

    RCLCPP_INFO(get_logger(), "Pure Pursuit controller started");
  }

private:
  struct ControlResult
  {
    ackermann_msgs::msg::AckermannDriveStamped command;
    double heading_error;
    double curvature;
    geometry_msgs::msg::Point target;
  };

  void validate_parameters() const
  {
    if (control_period_s_ <= 0.0 || lookahead_distance_ <= 0.0 || minimum_speed_ < 0.0 ||
      max_speed_ < minimum_speed_ ||
      max_steering_angle_ <= 0.0 || odometry_timeout_s_ <= 0.0 || wheelbase_ <= 0.0)
    {
      throw std::invalid_argument("Controller periods, geometry, and steering limits must be positive; speed limits must satisfy 0 <= minimum_speed <= max_speed.");
    }
  }

  void receive_odometry(const nav_msgs::msg::Odometry & message)
  {
    pose_ = VehiclePose{
      message.pose.pose.position.x,
      message.pose.pose.position.y,
      tf2::getYaw(message.pose.pose.orientation),
    };
    last_odometry_time_ = now();
  }

  void update_control()
  {
    const auto update_start = std::chrono::steady_clock::now();
    const auto target = find_lookahead_point();
    if (!target.has_value()) {
      publish_stop_command();
      return;
    }

    const auto result = compute_control(*target);
    const double cross_track_error = compute_cross_track_error();
    const double steering_oscillation = compute_steering_oscillation(result.command.drive.steering_angle);
    const auto update_end = std::chrono::steady_clock::now();
    const double control_latency_ms = std::chrono::duration<double, std::milli>(update_end - update_start).count();

    command_publisher_->publish(result.command);
    publish_metrics(result, cross_track_error, steering_oscillation, control_latency_ms);
  }

  std::optional<geometry_msgs::msg::Point> find_lookahead_point() const
  {
    if (!pose_.has_value() || !path_ || path_->poses.empty() || odometry_is_stale()) {
      return std::nullopt;
    }

    std::size_t closest_index = 0U;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < path_->poses.size(); ++index) {
      const auto & pose_stamped = path_->poses[index];
      const double distance = std::hypot(
        pose_stamped.pose.position.x - pose_->x,
        pose_stamped.pose.position.y - pose_->y);
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = index;
      }
    }

    for (std::size_t index = closest_index + 1U; index < path_->poses.size(); ++index) {
      const auto & pose_stamped = path_->poses[index];
      const double delta_x = pose_stamped.pose.position.x - pose_->x;
      const double delta_y = pose_stamped.pose.position.y - pose_->y;
      if (std::hypot(delta_x, delta_y) >= lookahead_distance_) {
        return pose_stamped.pose.position;
      }
    }
    return std::nullopt;
  }

  ControlResult compute_control(const geometry_msgs::msg::Point & target) const
  {
    const PurePursuitCommand control = compute_pure_pursuit(
      *pose_, VehiclePoint{target.x, target.y},
      PurePursuitParameters{
        wheelbase_, steering_gain_, minimum_speed_, max_speed_, max_steering_angle_});

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = now();
    command.drive.speed = static_cast<float>(control.speed);
    command.drive.steering_angle = static_cast<float>(control.steering_angle);

    return ControlResult{std::move(command), control.heading_error, control.curvature, target};
  }

  double compute_cross_track_error() const
  {
    if (!pose_.has_value() || !path_ || path_->poses.empty()) {
      return 0.0;
    }

    double minimum_distance = std::numeric_limits<double>::infinity();
    for (const auto & pose_stamped : path_->poses) {
      minimum_distance = std::min(
        minimum_distance,
        std::hypot(
          pose_stamped.pose.position.x - pose_->x,
          pose_stamped.pose.position.y - pose_->y));
    }
    return minimum_distance;
  }

  double compute_steering_oscillation(const double steering_command)
  {
    const double oscillation = previous_steering_command_.has_value() ?
      std::abs(steering_command - *previous_steering_command_) : 0.0;
    previous_steering_command_ = steering_command;
    return oscillation;
  }

  bool odometry_is_stale() const
  {
    return !last_odometry_time_.has_value() ||
           (now() - *last_odometry_time_).seconds() > odometry_timeout_s_;
  }

  void publish_stop_command()
  {
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = now();
    command_publisher_->publish(command);
  }

  void publish_metrics(
    const ControlResult & result, const double cross_track_error,
    const double steering_oscillation, const double control_latency_ms)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "controller_metrics";
    status.message = "controller telemetry";
    status.values.push_back(make_key_value("cross_track_error", std::to_string(cross_track_error)));
    status.values.push_back(make_key_value("heading_error", std::to_string(result.heading_error)));
    status.values.push_back(make_key_value("steering_command", std::to_string(result.command.drive.steering_angle)));
    status.values.push_back(make_key_value("steering_oscillation", std::to_string(steering_oscillation)));
    status.values.push_back(make_key_value("commanded_speed", std::to_string(result.command.drive.speed)));
    status.values.push_back(make_key_value("curvature", std::to_string(result.curvature)));
    status.values.push_back(make_key_value("control_latency_ms", std::to_string(control_latency_ms)));
    status.values.push_back(make_key_value("lookahead_distance", std::to_string(lookahead_distance_)));
    status.values.push_back(make_key_value("target_x", std::to_string(result.target.x)));
    status.values.push_back(make_key_value("target_y", std::to_string(result.target.y)));
    status.values.push_back(make_key_value("path_pose_count", std::to_string(path_->poses.size())));

    diagnostic_msgs::msg::DiagnosticArray metrics;
    metrics.header.stamp = now();
    metrics.status.push_back(std::move(status));
    metrics_publisher_->publish(metrics);
  }

  static diagnostic_msgs::msg::KeyValue make_key_value(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = key;
    key_value.value = value;
    return key_value;
  }

  double control_period_s_;
  double lookahead_distance_;
  double minimum_speed_;
  double max_speed_;
  double max_steering_angle_;
  double odometry_timeout_s_;
  double steering_gain_;
  double wheelbase_;
  nav_msgs::msg::Path::SharedPtr path_;
  std::optional<VehiclePose> pose_;
  std::optional<rclcpp::Time> last_odometry_time_;
  std::optional<double> previous_steering_command_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr metrics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace rosmaster_a1

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosmaster_a1::PurePursuitControllerNode>());
  rclcpp::shutdown();
  return 0;
}