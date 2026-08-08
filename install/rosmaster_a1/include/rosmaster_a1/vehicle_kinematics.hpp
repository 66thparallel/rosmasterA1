#ifndef ROSMASTER_A1__VEHICLE_KINEMATICS_HPP_
#define ROSMASTER_A1__VEHICLE_KINEMATICS_HPP_

#include <cmath>

#include "rosmaster_a1/pure_pursuit.hpp"

namespace rosmaster_a1
{

struct VehicleState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double velocity{0.0};
};

inline double integrate_bicycle(
  VehicleState & state, const double speed, const double steering_angle,
  const double wheelbase, const double delta_time_s)
{
  const double yaw_rate = speed / wheelbase * std::tan(steering_angle);
  state.x += speed * std::cos(state.yaw) * delta_time_s;
  state.y += speed * std::sin(state.yaw) * delta_time_s;
  state.yaw = normalize_angle(state.yaw + yaw_rate * delta_time_s);
  state.velocity = speed;
  return yaw_rate;
}

}  // namespace rosmaster_a1

#endif  // ROSMASTER_A1__VEHICLE_KINEMATICS_HPP_