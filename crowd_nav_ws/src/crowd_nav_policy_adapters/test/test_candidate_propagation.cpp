#include <cmath>

#include "gtest/gtest.h"

#include "crowd_nav_policy_adapters/candidate_propagation.hpp"

using crowd_nav_observation::WorldState;
using crowd_nav_perception::HumanObservation;
using crowd_nav_policy_adapters::CandidateAction;
using crowd_nav_policy_adapters::propagateCandidate;

namespace
{
rclcpp::Time t(double seconds)
{
  return rclcpp::Time(
    static_cast<int32_t>(seconds),
    static_cast<uint32_t>((seconds - std::floor(seconds)) * 1e9),
    RCL_ROS_TIME);
}
}  // namespace

TEST(PropagateCandidate, SelfAdvancesUnderCandidateActionHolonomic)
{
  WorldState state;
  state.robot = {1.0, 2.0, 0.0, 0.0, 0.14, 5.0, 5.0, 1.0, 0.0};

  const CandidateAction action{0.5, -0.3};
  const WorldState next = propagateCandidate(state, action, /*time_step_s=*/0.25);

  EXPECT_DOUBLE_EQ(next.robot.px, 1.0 + 0.5 * 0.25);
  EXPECT_DOUBLE_EQ(next.robot.py, 2.0 + (-0.3) * 0.25);
  EXPECT_DOUBLE_EQ(next.robot.vx, 0.5);
  EXPECT_DOUBLE_EQ(next.robot.vy, -0.3);
  // radius/gx/gy/v_pref/theta unchanged (propagate() doesn't touch them, holonomic branch).
  EXPECT_DOUBLE_EQ(next.robot.radius, 0.14);
  EXPECT_DOUBLE_EQ(next.robot.gx, 5.0);
  EXPECT_DOUBLE_EQ(next.robot.gy, 5.0);
  EXPECT_DOUBLE_EQ(next.robot.v_pref, 1.0);
  EXPECT_DOUBLE_EQ(next.robot.theta, 0.0);
}

TEST(PropagateCandidate, HumansAdvanceUnderTheirOwnConstantVelocityNotTheCandidateAction)
{
  WorldState state;
  state.robot = {0.0, 0.0, 0.0, 0.0, 0.14, 1.0, 1.0, 1.0, 0.0};

  HumanObservation h;
  h.id = 3;
  h.x = 3.0;
  h.y = 4.0;
  h.vx = 1.0;
  h.vy = -2.0;
  h.stamp = t(10.0);
  state.humans.push_back(h);

  // A candidate action wildly different from the human's own velocity - if the propagation
  // accidentally applied the candidate action to the human instead of its own velocity, this
  // test would catch it.
  const CandidateAction action{-5.0, 5.0};
  const WorldState next = propagateCandidate(state, action, /*time_step_s=*/0.25);

  ASSERT_EQ(next.humans.size(), 1u);
  EXPECT_DOUBLE_EQ(next.humans[0].x, 3.0 + 1.0 * 0.25);
  EXPECT_DOUBLE_EQ(next.humans[0].y, 4.0 + (-2.0) * 0.25);
  EXPECT_DOUBLE_EQ(next.humans[0].vx, 1.0);
  EXPECT_DOUBLE_EQ(next.humans[0].vy, -2.0);
  EXPECT_EQ(next.humans[0].id, 3u);
  EXPECT_EQ(next.humans[0].stamp, t(10.25));
}

TEST(PropagateCandidate, StopActionLeavesSelfPositionUnchanged)
{
  WorldState state;
  state.robot = {2.0, -1.0, 0.5, 0.5, 0.14, 0.0, 0.0, 1.0, 0.0};

  const WorldState next = propagateCandidate(state, CandidateAction{0.0, 0.0}, 0.25);
  EXPECT_DOUBLE_EQ(next.robot.px, 2.0);
  EXPECT_DOUBLE_EQ(next.robot.py, -1.0);
  EXPECT_DOUBLE_EQ(next.robot.vx, 0.0);
  EXPECT_DOUBLE_EQ(next.robot.vy, 0.0);
}
