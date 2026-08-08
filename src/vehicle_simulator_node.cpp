#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rosmaster_a1/vehicle_kinematics.hpp"

namespace rosmaster_a1
{

class VehicleSimulatorNode : public rclcpp::Node
{
public:
  VehicleSimulatorNode()
  : Node("vehicle_simulator_node")
  {
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);
    max_speed_ = declare_parameter<double>("max_speed", 1.0);
    max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.6);
    simulation_period_s_ = declare_parameter<double>("simulation_period_s", 0.02);
    wheelbase_ = declare_parameter<double>("wheelbase", 0.3);
    validate_parameters();

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_transform_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    command_subscription_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd_drive", 10,
      [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr message) {
        receive_command(*message);
      });
    simulation_timer_ = create_wall_timer(
      std::chrono::duration<double>(simulation_period_s_), [this]() { update_simulation(); });

    last_update_time_ = now();
    publish_static_map_transform();
    RCLCPP_INFO(get_logger(), "Vehicle simulator started");
  }

private:
  void validate_parameters() const
  {
    if (command_timeout_s_ <= 0.0 || max_speed_ < 0.0 || max_steering_angle_ <= 0.0 ||
      simulation_period_s_ <= 0.0 || wheelbase_ <= 0.0)
    {
      throw std::invalid_argument("Simulator periods, geometry, and steering limits must be positive; max_speed cannot be negative.");
    }
  }

  void receive_command(const ackermann_msgs::msg::AckermannDriveStamped & message)
  {
    commanded_speed_ = std::clamp(
      static_cast<double>(message.drive.speed), -max_speed_, max_speed_);
    commanded_steering_angle_ = std::clamp(
      static_cast<double>(message.drive.steering_angle), -max_steering_angle_, max_steering_angle_);
    last_command_time_ = now();
  }

  void update_simulation()
  {
    const auto update_time = now();
    const double delta_time_s = (update_time - last_update_time_).seconds();
    if (delta_time_s <= 0.0) {
      return;
    }
    last_update_time_ = update_time;

    const bool command_is_current = last_command_time_.has_value() &&
      (update_time - *last_command_time_).seconds() <= command_timeout_s_;
    const double speed = command_is_current ? commanded_speed_ : 0.0;
    const double steering_angle = command_is_current ? commanded_steering_angle_ : 0.0;
    const double yaw_rate = integrate_bicycle(
      state_, speed, steering_angle, wheelbase_, delta_time_s);

    publish_odometry_and_transform(update_time, yaw_rate);
  }

  void publish_static_map_transform()
  {
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.rotation.w = 1.0;
    static_transform_broadcaster_->sendTransform(map_to_odom);
  }

  void publish_odometry_and_transform(const rclcpp::Time & stamp, const double yaw_rate)
  {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, state_.yaw);

    geometry_msgs::msg::TransformStamped odom_to_base_link;
    odom_to_base_link.header.stamp = stamp;
    odom_to_base_link.header.frame_id = "odom";
    odom_to_base_link.child_frame_id = "base_link";
    odom_to_base_link.transform.translation.x = state_.x;
    odom_to_base_link.transform.translation.y = state_.y;
    odom_to_base_link.transform.rotation = tf2::toMsg(orientation);
    transform_broadcaster_->sendTransform(odom_to_base_link);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = state_.x;
    odometry.pose.pose.position.y = state_.y;
    odometry.pose.pose.orientation = tf2::toMsg(orientation);
    odometry.twist.twist.linear.x = state_.velocity;
    odometry.twist.twist.angular.z = yaw_rate;
    odometry_publisher_->publish(odometry);
  }

  double command_timeout_s_;
  double max_speed_;
  double max_steering_angle_;
  double simulation_period_s_;
  double wheelbase_;
  double commanded_speed_{0.0};
  double commanded_steering_angle_{0.0};
  VehicleState state_;
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> last_command_time_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
};

}  // namespace rosmaster_a1

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosmaster_a1::VehicleSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}