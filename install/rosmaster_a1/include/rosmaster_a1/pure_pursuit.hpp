#ifndef ROSMASTER_A1__PURE_PURSUIT_HPP_
#define ROSMASTER_A1__PURE_PURSUIT_HPP_

#include <algorithm>
#include <cmath>

namespace rosmaster_a1
{

struct VehiclePose
{
  double x;
  double y;
  double yaw;
};

struct VehiclePoint
{
  double x;
  double y;
};

struct PurePursuitParameters
{
  double wheelbase;
  double steering_gain;
  double minimum_speed;
  double max_speed;
  double max_steering_angle;
};

struct PurePursuitCommand
{
  double speed;
  double steering_angle;
  double heading_error;
  double curvature;
};

inline double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline PurePursuitCommand compute_pure_pursuit(
  const VehiclePose & pose, const VehiclePoint & target,
  const PurePursuitParameters & parameters)
{
  const double delta_x = target.x - pose.x;
  const double delta_y = target.y - pose.y;
  const double target_distance = std::max(std::hypot(delta_x, delta_y), 0.01);
  const double target_heading = std::atan2(delta_y, delta_x);
  const double heading_error = normalize_angle(target_heading - pose.yaw);
  const double curvature = 2.0 * std::sin(heading_error) / target_distance;
  const double steering_angle = std::clamp(
    parameters.steering_gain * std::atan(parameters.wheelbase * curvature),
    -parameters.max_steering_angle, parameters.max_steering_angle);
  const double speed = std::max(
    parameters.minimum_speed,
    parameters.max_speed * (1.0 - std::min(std::abs(heading_error), 1.0)));

  return PurePursuitCommand{speed, steering_angle, heading_error, curvature};
}

}  // namespace rosmaster_a1

#endif  // ROSMASTER_A1__PURE_PURSUIT_HPP_