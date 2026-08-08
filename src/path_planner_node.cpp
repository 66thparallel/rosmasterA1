#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rosmaster_a1
{

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode()
  : Node("path_planner_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const auto flat_waypoints = declare_parameter<std::vector<double>>(
      "waypoints", {0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 0.0, 0.0});
    waypoints_ = load_waypoints(flat_waypoints);

    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/path", 10);
    metrics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/metrics/planner", 10);
    publish_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_path(); });

    RCLCPP_INFO(get_logger(), "Path planner started with %zu waypoints", waypoints_.size());
  }

private:
  static std::vector<std::pair<double, double>> load_waypoints(
    const std::vector<double> & flat_waypoints)
  {
    if (flat_waypoints.size() < 4U) {
      throw std::invalid_argument("The 'waypoints' parameter must contain at least two x/y pairs.");
    }

    if (flat_waypoints.size() % 2U != 0U) {
      throw std::invalid_argument("The 'waypoints' parameter must contain an even number of values.");
    }

    std::vector<std::pair<double, double>> waypoints;
    waypoints.reserve(flat_waypoints.size() / 2U);
    for (std::size_t index = 0U; index < flat_waypoints.size(); index += 2U) {
      waypoints.emplace_back(flat_waypoints[index], flat_waypoints[index + 1U]);
    }
    return waypoints;
  }

  void publish_path()
  {
    const auto publish_time = now();
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = publish_time;

    path.poses.reserve(waypoints_.size());
    for (const auto & [x, y] : waypoints_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(std::move(pose));
    }

    path_publisher_->publish(path);
    publish_metrics(publish_time);
  }

  void publish_metrics(const rclcpp::Time & publish_time)
  {
    double publish_interval_ms = 0.0;
    double loop_rate_hz = 0.0;
    if (previous_publish_time_.nanoseconds() != 0) {
      publish_interval_ms = static_cast<double>((publish_time - previous_publish_time_).nanoseconds()) * 1e-6;
      if (publish_interval_ms > 0.0) {
        loop_rate_hz = 1000.0 / publish_interval_ms;
      }
    }
    previous_publish_time_ = publish_time;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "planner_metrics";
    status.message = "planner telemetry";
    status.values.push_back(make_key_value("frame_id", frame_id_));
    status.values.push_back(make_key_value("waypoint_count", std::to_string(waypoints_.size())));
    status.values.push_back(make_key_value("publish_interval_ms", std::to_string(publish_interval_ms)));
    status.values.push_back(make_key_value("loop_rate_hz", std::to_string(loop_rate_hz)));

    diagnostic_msgs::msg::DiagnosticArray metrics;
    metrics.header.stamp = publish_time;
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

  std::string frame_id_;
  std::vector<std::pair<double, double>> waypoints_;
  rclcpp::Time previous_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr metrics_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace rosmaster_a1

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosmaster_a1::PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}