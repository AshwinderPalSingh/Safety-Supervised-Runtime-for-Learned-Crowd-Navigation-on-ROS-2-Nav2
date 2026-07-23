// Hand-computed cases matching multi_human_rl.py's compute_reward() exactly, read directly
// from the reference source, not memory (IMPLEMENTATION_PLAN.md S4.7).
#include "gtest/gtest.h"

#include "crowd_nav_policy_adapters/sarl_reward.hpp"

using crowd_nav_observation::RobotSelfState;
using crowd_nav_policy_adapters::computeImmediateReward;
using crowd_nav_policy_adapters::PropagatedHuman;

namespace
{
RobotSelfState makeSelf(double px, double py, double radius, double gx, double gy)
{
  RobotSelfState s;
  s.px = px;
  s.py = py;
  s.radius = radius;
  s.gx = gx;
  s.gy = gy;
  return s;
}
}  // namespace

TEST(ComputeImmediateReward, CollisionYieldsFixedPenalty)
{
  // dist = |0.3-0| - 0.3 - 0.3 = -0.3 < 0 -> collision, regardless of goal/discomfort.
  const auto self = makeSelf(0.0, 0.0, 0.3, 5.0, 5.0);
  const std::vector<PropagatedHuman> humans = {{0.3, 0.0, 0.3}};
  EXPECT_DOUBLE_EQ(computeImmediateReward(self, humans, 0.25), -0.25);
}

TEST(ComputeImmediateReward, ReachingGoalWithNoNearbyHumanYieldsSuccessReward)
{
  const auto self = makeSelf(5.0, 5.0, 0.3, 5.0, 5.0);  // dist to goal = 0 < radius
  const std::vector<PropagatedHuman> humans = {{-5.0, -5.0, 0.3}};  // far away
  EXPECT_DOUBLE_EQ(computeImmediateReward(self, humans, 0.25), 1.0);
}

TEST(ComputeImmediateReward, DiscomfortZoneYieldsProximityPenalty)
{
  // dist = 0.65 - 0.3 - 0.3 = 0.05, in [0, 0.2) -> (dmin - 0.2) * 0.5 * time_step.
  const auto self = makeSelf(0.0, 0.0, 0.3, 10.0, 10.0);
  const std::vector<PropagatedHuman> humans = {{0.65, 0.0, 0.3}};
  const double expected = (0.05 - 0.2) * 0.5 * 0.25;
  EXPECT_NEAR(computeImmediateReward(self, humans, 0.25), expected, 1e-9);
}

TEST(ComputeImmediateReward, ClearOfEverythingYieldsZero)
{
  const auto self = makeSelf(0.0, 0.0, 0.3, 10.0, 10.0);
  const std::vector<PropagatedHuman> humans = {{5.0, 5.0, 0.3}};
  EXPECT_DOUBLE_EQ(computeImmediateReward(self, humans, 0.25), 0.0);
}

TEST(ComputeImmediateReward, NoHumansAndNotAtGoalYieldsZero)
{
  const auto self = makeSelf(0.0, 0.0, 0.3, 10.0, 10.0);
  EXPECT_DOUBLE_EQ(computeImmediateReward(self, {}, 0.25), 0.0);
}
