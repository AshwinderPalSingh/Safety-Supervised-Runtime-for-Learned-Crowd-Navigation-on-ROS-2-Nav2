// Phase 9 done-bar: "supervisor geometry and OOD-threshold
// unit tests green," including a real (not hypothetical) test that LOW_PERCEPTION_CONFIDENCE
// actually fires on a dropout and does not fire on a legitimately empty/clear scene.
#include <cmath>

#include "gtest/gtest.h"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

#include "crowd_nav_safety_supervisor/safety_supervisor.hpp"

using crowd_nav_observation::WorldState;
using crowd_nav_perception::HumanObservation;
using crowd_nav_policy_adapters::Velocity2D;
using crowd_nav_safety_supervisor::InterventionCause;
using crowd_nav_safety_supervisor::SafetySupervisor;
using crowd_nav_safety_supervisor::SafetySupervisorConfig;

namespace
{
WorldState baseState()
{
  WorldState state;
  state.robot = {0.0, 0.0, 0.0, 0.0, 0.3, 5.0, 0.0, 1.0, 0.0};
  return state;
}

HumanObservation humanAt(double x, double y, double vx = 0.0, double vy = 0.0)
{
  HumanObservation h;
  h.x = x;
  h.y = y;
  h.vx = vx;
  h.vy = vy;
  return h;
}
}  // namespace

TEST(SafetySupervisorOod, AllClearWhenNothingTriggers)
{
  SafetySupervisorConfig config;
  SafetySupervisor supervisor(config);
  WorldState state = baseState();
  state.humans.push_back(humanAt(3.0, 3.0));  // far, slow, well within crowd-size limit

  const auto result = supervisor.checkOodCriteria(state, {0.3, 0.0}, 0);
  EXPECT_TRUE(result.safe);
  EXPECT_FALSE(result.cause.has_value());
}

TEST(SafetySupervisorOod, CrowdSizeTriggersWhenTooManyHumans)
{
  SafetySupervisorConfig config;
  config.max_train_humans = 2;
  SafetySupervisor supervisor(config);
  WorldState state = baseState();
  state.humans.push_back(humanAt(3.0, 3.0));
  state.humans.push_back(humanAt(3.0, -3.0));
  state.humans.push_back(humanAt(-3.0, 3.0));  // 3rd human - over the configured limit

  const auto result = supervisor.checkOodCriteria(state, {0.0, 0.0}, 0);
  ASSERT_FALSE(result.safe);
  EXPECT_EQ(result.cause, InterventionCause::kCrowdSize);
}

TEST(SafetySupervisorOod, ProximityTriggersWhenHumanTooClose)
{
  SafetySupervisorConfig config;
  config.min_train_distance_m = 0.8;
  SafetySupervisor supervisor(config);
  WorldState state = baseState();
  state.humans.push_back(humanAt(0.5, 0.0));  // 0.5m from robot at origin - inside 0.8m

  const auto result = supervisor.checkOodCriteria(state, {0.0, 0.0}, 0);
  ASSERT_FALSE(result.safe);
  EXPECT_EQ(result.cause, InterventionCause::kProximity);
}

TEST(SafetySupervisorOod, RelativeSpeedTriggersWhenHumanTooFast)
{
  SafetySupervisorConfig config;
  config.max_train_speed_mps = 1.5;
  SafetySupervisor supervisor(config);
  WorldState state = baseState();
  state.humans.push_back(humanAt(3.0, 3.0, 2.0, 0.0));  // 2.0 m/s - beyond the training range

  const auto result = supervisor.checkOodCriteria(state, {0.0, 0.0}, 0);
  ASSERT_FALSE(result.safe);
  EXPECT_EQ(result.cause, InterventionCause::kRelativeSpeed);
}

TEST(SafetySupervisorOod, CommandLimitTriggersWhenCandidateTooFast)
{
  SafetySupervisorConfig config;
  config.max_commanded_speed_mps = 1.0;
  SafetySupervisor supervisor(config);
  WorldState state = baseState();

  const auto result = supervisor.checkOodCriteria(state, {2.0, 0.0}, 0);
  ASSERT_FALSE(result.safe);
  EXPECT_EQ(result.cause, InterventionCause::kCommandLimit);
}

TEST(SafetySupervisorOod, LowPerceptionConfidenceTriggersOnlyWhenDropoutOccurred)
{
  SafetySupervisorConfig config;
  SafetySupervisor supervisor(config);
  WorldState state = baseState();  // no humans at all - a legitimately empty, clear scene

  // Legitimately empty scene, zero dropouts - must NOT trigger (S4.8.1's correction: empty
  // isn't inherently degraded).
  EXPECT_TRUE(supervisor.checkOodCriteria(state, {0.0, 0.0}, 0).safe);

  // Same scene, but the perception layer's own dropout model discarded something this tick.
  const auto result = supervisor.checkOodCriteria(state, {0.0, 0.0}, 1);
  ASSERT_FALSE(result.safe);
  EXPECT_EQ(result.cause, InterventionCause::kLowPerceptionConfidence);
}

namespace
{
// 4m x 4m grid, 0.1m resolution. Origin deliberately offset by half a cell (-2.05, not -2.0):
// with a "round" origin, a "round" test world-point like (0.5, 0.0) sits EXACTLY on a cell
// boundary (mathematically integer (w-o)/resolution), which floating-point noise can push to
// either neighboring cell depending on the order of operations - found the hard way when a
// keepout-mask cross-check (built from an OccupancyGrid whose resolution field is float32, not
// double) disagreed with this costmap by exactly one cell at such a point. Offsetting the
// origin keeps every test world-point safely mid-cell instead.
nav2_costmap_2d::Costmap2D makeCostmap(unsigned char default_value = nav2_costmap_2d::FREE_SPACE)
{
  return nav2_costmap_2d::Costmap2D(40, 40, 0.1, -2.05, -2.05, default_value);
}
}  // namespace

TEST(SafetySupervisorForwardSim, SafeWhenPathClear)
{
  SafetySupervisorConfig config;
  SafetySupervisor supervisor(config);
  auto costmap = makeCostmap();
  WorldState state = baseState();

  const auto result = supervisor.checkForwardSim(state, {1.0, 0.0}, &costmap, nullptr);
  EXPECT_TRUE(result.safe);
}

TEST(SafetySupervisorForwardSim, CostmapCollisionWhenPathHitsLethalCell)
{
  SafetySupervisorConfig config;
  config.forward_sim_steps = 4;
  config.forward_sim_dt_s = 0.25;
  SafetySupervisor supervisor(config);
  auto costmap = makeCostmap();

  // Candidate {1.0, 0.0} reaches world (0.5, 0.0) at step 2 (t=0.5s) - mark that cell lethal.
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(0.5, 0.0, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  WorldState state = baseState();
  const auto result = supervisor.checkForwardSim(state, {1.0, 0.0}, &costmap, nullptr);
  ASSERT_FALSE(result.safe);
  EXPECT_EQ(result.cause, InterventionCause::kCostmapCollision);
}

TEST(SafetySupervisorForwardSim, NoInformationCellsAreNotTreatedAsUnsafe)
{
  SafetySupervisorConfig config;
  SafetySupervisor supervisor(config);
  // Every cell defaults to NO_INFORMATION (255), which numerically satisfies
  // ">= INSCRIBED_INFLATED_OBSTACLE" (253) - the exclusion this test targets directly.
  auto costmap = makeCostmap(nav2_costmap_2d::NO_INFORMATION);

  WorldState state = baseState();
  const auto result = supervisor.checkForwardSim(state, {1.0, 0.0}, &costmap, nullptr);
  EXPECT_TRUE(result.safe);
}

TEST(SafetySupervisorForwardSim, OutOfBoundsStepIsNotTreatedAsUnsafe)
{
  SafetySupervisorConfig config;
  SafetySupervisor supervisor(config);
  auto costmap = makeCostmap();

  // At 5 m/s for up to 1.0s, every forward-sim step lands well outside this 4x4m grid.
  WorldState state = baseState();
  const auto result = supervisor.checkForwardSim(state, {5.0, 0.0}, &costmap, nullptr);
  EXPECT_TRUE(result.safe);
}

TEST(SafetySupervisorForwardSim, KeepoutViolationLabelWhenMaskFlagsTheSameCell)
{
  SafetySupervisorConfig config;
  SafetySupervisor supervisor(config);
  auto costmap = makeCostmap();

  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(0.5, 0.0, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Secondary mask, same georeferencing as the costmap, flagging the same world point.
  nav_msgs::msg::OccupancyGrid mask;
  mask.info.resolution = 0.1;
  mask.info.width = 40;
  mask.info.height = 40;
  mask.info.origin.position.x = -2.05;
  mask.info.origin.position.y = -2.05;
  mask.data.assign(static_cast<size_t>(mask.info.width) * mask.info.height, 0);
  mask.data[static_cast<size_t>(my) * mask.info.width + mx] = 100;

  WorldState state = baseState();
  const auto result = supervisor.checkForwardSim(state, {1.0, 0.0}, &costmap, &mask);
  ASSERT_FALSE(result.safe);
  EXPECT_EQ(result.cause, InterventionCause::kKeepoutViolation);
}

TEST(SafetySupervisorForwardSim, CostmapCollisionWhenMaskProvidedButDoesNotFlagTheCell)
{
  SafetySupervisorConfig config;
  SafetySupervisor supervisor(config);
  auto costmap = makeCostmap();

  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(0.5, 0.0, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  // Mask present but all-clear (e.g. stale/unavailable data) - must degrade to the generic
  // label, never silently change the safety decision itself (S4.8.3).
  nav_msgs::msg::OccupancyGrid mask;
  mask.info.resolution = 0.1;
  mask.info.width = 40;
  mask.info.height = 40;
  mask.info.origin.position.x = -2.05;
  mask.info.origin.position.y = -2.05;
  mask.data.assign(static_cast<size_t>(mask.info.width) * mask.info.height, 0);

  WorldState state = baseState();
  const auto result = supervisor.checkForwardSim(state, {1.0, 0.0}, &costmap, &mask);
  ASSERT_FALSE(result.safe);
  EXPECT_EQ(result.cause, InterventionCause::kCostmapCollision);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
