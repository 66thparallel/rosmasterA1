#include <cmath>

#include <gtest/gtest.h>

#include "rosmaster_a1/pure_pursuit.hpp"

namespace rosmaster_a1
{
namespace
{

constexpr double kTolerance = 1e-9;
const PurePursuitParameters kParameters{0.3, 1.5, 0.1, 1.0, 0.6};

TEST(PurePursuit, CommandsMaximumSpeedForStraightAheadTarget)
{
  const auto command = compute_pure_pursuit(
    VehiclePose{0.0, 0.0, 0.0}, VehiclePoint{2.0, 0.0}, kParameters);

  EXPECT_NEAR(command.heading_error, 0.0, kTolerance);
  EXPECT_NEAR(command.curvature, 0.0, kTolerance);
  EXPECT_NEAR(command.steering_angle, 0.0, kTolerance);
  EXPECT_NEAR(command.speed, 1.0, kTolerance);
}

TEST(PurePursuit, LimitsSteeringAndReducesSpeedForLargeHeadingError)
{
  const auto command = compute_pure_pursuit(
    VehiclePose{0.0, 0.0, 0.0}, VehiclePoint{0.0, 1.0}, kParameters);

  EXPECT_NEAR(command.heading_error, M_PI_2, kTolerance);
  EXPECT_NEAR(command.curvature, 2.0, kTolerance);
  EXPECT_NEAR(command.steering_angle, 0.6, kTolerance);
  EXPECT_NEAR(command.speed, 0.1, kTolerance);
}

TEST(PurePursuit, WrapsHeadingErrorAcrossPiBoundary)
{
  const auto command = compute_pure_pursuit(
    VehiclePose{0.0, 0.0, -M_PI + 0.1}, VehiclePoint{-1.0, 0.01}, kParameters);

  EXPECT_LT(command.heading_error, 0.0);
  EXPECT_GT(command.heading_error, -0.2);
}

}  // namespace
}  // namespace rosmaster_a1