#include <cmath>

#include <gtest/gtest.h>

#include "rosmaster_a1/vehicle_kinematics.hpp"

namespace rosmaster_a1
{
namespace
{

constexpr double kTolerance = 1e-9;

TEST(VehicleKinematics, IntegratesStraightMotion)
{
  VehicleState state;
  const double yaw_rate = integrate_bicycle(state, 1.0, 0.0, 0.3, 0.5);

  EXPECT_NEAR(state.x, 0.5, kTolerance);
  EXPECT_NEAR(state.y, 0.0, kTolerance);
  EXPECT_NEAR(state.yaw, 0.0, kTolerance);
  EXPECT_NEAR(state.velocity, 1.0, kTolerance);
  EXPECT_NEAR(yaw_rate, 0.0, kTolerance);
}

TEST(VehicleKinematics, IntegratesHeadingUsingBicycleYawRate)
{
  VehicleState state;
  const double yaw_rate = integrate_bicycle(state, 1.0, std::atan(0.3), 0.3, 0.1);

  EXPECT_NEAR(yaw_rate, 1.0, kTolerance);
  EXPECT_NEAR(state.x, 0.1, kTolerance);
  EXPECT_NEAR(state.y, 0.0, kTolerance);
  EXPECT_NEAR(state.yaw, 0.1, kTolerance);
}

}  // namespace
}  // namespace rosmaster_a1