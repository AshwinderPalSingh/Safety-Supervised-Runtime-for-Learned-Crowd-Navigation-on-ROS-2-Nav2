// Tests drive GroundTruthHumanSource's pure-logic surface (ingestPedestrian/setRobotPose)
// directly - no real ROS publishers/subscriptions needed, see the header's comment on why
// that's deliberately separated out. All test timestamps use an explicit RCL_ROS_TIME clock
// type consistently, so Time arithmetic/comparison never hits a clock-type mismatch.
#include <cmath>
#include <set>

#include "crowd_nav_perception/ground_truth_human_source.hpp"
#include "gtest/gtest.h"

using crowd_nav_perception::DegradationParams;
using crowd_nav_perception::GroundTruthHumanSource;
using crowd_nav_perception::HumanObservation;

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

TEST(GroundTruthHumanSource, OraclePassthroughWithAllDefaultsIsExact)
{
  DegradationParams params;  // all zero/off
  GroundTruthHumanSource source(params);

  source.ingestPedestrian(7, 1.5, -2.5, 0.3, -0.1, t(0.0));
  auto humans = source.getHumans(t(0.0));

  ASSERT_EQ(humans.size(), 1u);
  EXPECT_EQ(humans[0].id, 7u);
  EXPECT_DOUBLE_EQ(humans[0].x, 1.5);
  EXPECT_DOUBLE_EQ(humans[0].y, -2.5);
  EXPECT_DOUBLE_EQ(humans[0].vx, 0.3);
  EXPECT_DOUBLE_EQ(humans[0].vy, -0.1);
  EXPECT_DOUBLE_EQ(humans[0].cov_xx, 0.0);
  EXPECT_DOUBLE_EQ(humans[0].cov_yy, 0.0);
}

TEST(GroundTruthHumanSource, NoiseAppliesAndPopulatesCovariance)
{
  DegradationParams params;
  params.sigma_pos_m = 0.2;
  params.degradation_seed = 12345;
  GroundTruthHumanSource source(params);

  source.ingestPedestrian(1, 0.0, 0.0, 0.0, 0.0, t(0.0));
  auto humans = source.getHumans(t(0.0));

  ASSERT_EQ(humans.size(), 1u);
  // With sigma=0.2 and a nonzero seed, the noised value should not land exactly on 0,0 -
  // astronomically unlikely by chance, and confirms noise is actually being applied, not a
  // no-op.
  EXPECT_NE(humans[0].x, 0.0);
  EXPECT_NE(humans[0].y, 0.0);
  EXPECT_DOUBLE_EQ(humans[0].cov_xx, 0.04);
  EXPECT_DOUBLE_EQ(humans[0].cov_yy, 0.04);
  EXPECT_DOUBLE_EQ(humans[0].cov_xy, 0.0);
}

TEST(GroundTruthHumanSource, DropoutSweepMatchesExpectedRateWithinSamplingTolerance)
{
  DegradationParams params;
  params.dropout_prob = 0.3;
  params.degradation_seed = 999;
  GroundTruthHumanSource source(params);

  constexpr int kN = 1000;
  for (int i = 0; i < kN; ++i) {
    source.ingestPedestrian(
      static_cast<uint32_t>(i), 0.0, 0.0, 0.0, 0.0, t(0.0));
  }
  auto humans = source.getHumans(t(0.0));

  // Expected present count ~700 (dropout 0.3 of 1000). Binomial stddev = sqrt(1000*0.3*0.7)
  // ~= 14.5; allow a generous 5-sigma band (~72) to avoid test flakiness while still catching
  // a badly wrong dropout rate (e.g. if dropout were applied inverted, or not at all).
  const double expected = kN * (1.0 - params.dropout_prob);
  const double tolerance = 5.0 * std::sqrt(kN * params.dropout_prob * (1.0 - params.dropout_prob));
  EXPECT_NEAR(static_cast<double>(humans.size()), expected, tolerance);
}

TEST(GroundTruthHumanSource, LatencyIsKeyedToSimTimeNotTickCount)
{
  DegradationParams params;
  params.latency_s = 1.0;
  GroundTruthHumanSource source(params);

  // Deliberately UNEVEN sim-time spacing between samples - if latency were implemented as
  // "look back N ticks" rather than "look back by sim time," this test would fail, because
  // tick-index-3-back and time-1.0s-back disagree given this spacing.
  source.ingestPedestrian(1, 0.0, 0.0, 0.0, 0.0, t(0.0));   // tick 0
  source.ingestPedestrian(1, 1.0, 0.0, 0.0, 0.0, t(0.3));   // tick 1
  source.ingestPedestrian(1, 2.0, 0.0, 0.0, 0.0, t(1.7));   // tick 2
  source.ingestPedestrian(1, 3.0, 0.0, 0.0, 0.0, t(2.0));   // tick 3 (latest)

  // Query at t=2.0 with latency 1.0s -> target_time=1.0s -> freshest sample AT OR BEFORE 1.0s
  // is the t=0.3 one (x=1.0), NOT "3 ticks back" (which would be the t=0.0 one, x=0.0) and NOT
  // the latest (x=3.0).
  auto humans = source.getHumans(t(2.0));
  ASSERT_EQ(humans.size(), 1u);
  EXPECT_DOUBLE_EQ(humans[0].x, 1.0);
}

TEST(GroundTruthHumanSource, DerivedSubstreamSeedsAreIndependentAndDeterministic)
{
  const uint64_t scenario_seed = 42;
  const uint64_t seed_pedestrian_tag =
    GroundTruthHumanSource::deriveSubstreamSeed(scenario_seed, 0);
  const uint64_t seed_degradation_tag =
    GroundTruthHumanSource::deriveSubstreamSeed(scenario_seed, 1);

  // Different tags from the same scenario seed must not collide.
  EXPECT_NE(seed_pedestrian_tag, seed_degradation_tag);

  // Same (scenario_seed, tag) must always reproduce the same derived seed.
  EXPECT_EQ(
    seed_degradation_tag,
    GroundTruthHumanSource::deriveSubstreamSeed(scenario_seed, 1));

  // Two sources built from the SAME derived seed must produce identical degraded sequences.
  DegradationParams params_a;
  params_a.sigma_pos_m = 0.3;
  params_a.dropout_prob = 0.2;
  params_a.degradation_seed = seed_degradation_tag;
  GroundTruthHumanSource source_a(params_a);

  DegradationParams params_b = params_a;  // same seed
  GroundTruthHumanSource source_b(params_b);

  for (int i = 0; i < 50; ++i) {
    source_a.ingestPedestrian(static_cast<uint32_t>(i), 1.0, 2.0, 0.1, 0.2, t(0.0));
    source_b.ingestPedestrian(static_cast<uint32_t>(i), 1.0, 2.0, 0.1, 0.2, t(0.0));
  }
  auto humans_a = source_a.getHumans(t(0.0));
  auto humans_b = source_b.getHumans(t(0.0));
  ASSERT_EQ(humans_a.size(), humans_b.size());
  for (size_t i = 0; i < humans_a.size(); ++i) {
    EXPECT_EQ(humans_a[i].id, humans_b[i].id);
    EXPECT_DOUBLE_EQ(humans_a[i].x, humans_b[i].x);
    EXPECT_DOUBLE_EQ(humans_a[i].y, humans_b[i].y);
  }

  // A source built from the OTHER tag's seed must diverge (different RNG stream entirely) -
  // this is the actual substream-independence guarantee, not just "different seed value used."
  DegradationParams params_c = params_a;
  params_c.degradation_seed = seed_pedestrian_tag;
  GroundTruthHumanSource source_c(params_c);
  for (int i = 0; i < 50; ++i) {
    source_c.ingestPedestrian(static_cast<uint32_t>(i), 1.0, 2.0, 0.1, 0.2, t(0.0));
  }
  auto humans_c = source_c.getHumans(t(0.0));
  // With 50 humans, dropout_prob=0.2 and independent noise draws, an exact match across all
  // present humans' x values would be a coincidence on the order of 1e-15 - a real, practical
  // check that the two streams are not correlated, not a theoretical nicety.
  bool any_difference = (humans_a.size() != humans_c.size());
  if (!any_difference) {
    for (size_t i = 0; i < humans_a.size(); ++i) {
      if (humans_a[i].x != humans_c[i].x) {
        any_difference = true;
        break;
      }
    }
  }
  EXPECT_TRUE(any_difference);
}

TEST(GroundTruthHumanSource, MaxRangeExcludesDistantHumans)
{
  DegradationParams params;
  params.max_range_m = 5.0;
  GroundTruthHumanSource source(params);

  source.setRobotPose(0.0, 0.0, 0.0);
  source.ingestPedestrian(1, 3.0, 0.0, 0.0, 0.0, t(0.0));   // within range
  source.ingestPedestrian(2, 10.0, 0.0, 0.0, 0.0, t(0.0));  // out of range

  auto humans = source.getHumans(t(0.0));
  ASSERT_EQ(humans.size(), 1u);
  EXPECT_EQ(humans[0].id, 1u);
}

// Matches this robot's real ~180 degree sensor (half-angle pi/2).
TEST(GroundTruthHumanSource, FovHalfAngleExcludesHumansBehindRobot)
{
  DegradationParams params;
  params.fov_half_angle_rad = M_PI_2;
  GroundTruthHumanSource source(params);

  // Robot at origin, facing +x (theta=0).
  source.setRobotPose(0.0, 0.0, 0.0);
  source.ingestPedestrian(1, 3.0, 0.0, 0.0, 0.0, t(0.0));    // dead ahead - in FOV
  source.ingestPedestrian(2, 0.0, 3.0, 0.0, 0.0, t(0.0));    // directly to the left - at the
                                                              // +/-90 deg edge, in FOV
  source.ingestPedestrian(3, -3.0, 0.0, 0.0, 0.0, t(0.0));   // directly behind - out of FOV
  source.ingestPedestrian(4, -3.0, 3.0, 0.0, 0.0, t(0.0));   // rear-left quadrant - out of FOV

  auto humans = source.getHumans(t(0.0));
  std::set<uint32_t> ids;
  for (const auto & h : humans) {ids.insert(h.id);}
  EXPECT_EQ(ids, (std::set<uint32_t>{1u, 2u}));
}

// The FOV restriction must track the robot's actual current heading, not assume facing +x -
// otherwise this would pass by accident for the theta=0 case above alone.
TEST(GroundTruthHumanSource, FovHalfAngleRotatesWithRobotHeading)
{
  DegradationParams params;
  params.fov_half_angle_rad = M_PI_2;
  GroundTruthHumanSource source(params);

  // Robot at origin, now facing +y (theta = pi/2).
  source.setRobotPose(0.0, 0.0, M_PI_2);
  source.ingestPedestrian(1, 3.0, 0.0, 0.0, 0.0, t(0.0));   // was "ahead" when facing +x, now
                                                              // at the robot's right flank - at
                                                              // the FOV edge, still in FOV
  source.ingestPedestrian(2, 0.0, 3.0, 0.0, 0.0, t(0.0));   // now dead ahead - in FOV
  source.ingestPedestrian(3, 0.0, -3.0, 0.0, 0.0, t(0.0));  // now directly behind - out of FOV

  auto humans = source.getHumans(t(0.0));
  std::set<uint32_t> ids;
  for (const auto & h : humans) {ids.insert(h.id);}
  EXPECT_EQ(ids, (std::set<uint32_t>{1u, 2u}));
}

// The dropout model's own count, read-and-reset each
// getHumans() call, is what makes LOW_PERCEPTION_CONFIDENCE a real, checkable signal.
TEST(GroundTruthHumanSource, NumDegradedLastCallCountsOnlyDropoutNotFovExclusion)
{
  DegradationParams params;
  params.dropout_prob = 1.0;  // every ingestion this tick is dropped, deterministically
  params.degradation_seed = 1;
  params.fov_half_angle_rad = M_PI_2;
  GroundTruthHumanSource source(params);

  source.setRobotPose(0.0, 0.0, 0.0);
  source.ingestPedestrian(1, 3.0, 0.0, 0.0, 0.0, t(0.0));   // in FOV, dropped by the model
  source.ingestPedestrian(2, -3.0, 0.0, 0.0, 0.0, t(0.0));  // out of FOV - excluded before the
                                                              // dropout draw even happens

  source.getHumans(t(0.0));
  // Only human 1 reached the dropout coin-flip; human 2 was excluded by FOV first and must not
  // inflate this count (S4.8.5's explicit distinction).
  EXPECT_EQ(source.numDegradedLastCall(), 1u);

  // A second call with nothing new ingested must report zero, not the stale prior count -
  // proves this is "since last call," not a running total.
  source.getHumans(t(0.0));
  EXPECT_EQ(source.numDegradedLastCall(), 0u);
}

// docs/audit.md S1.3: GroundTruthHumanSource reads a privileged Gazebo WORLD-frame ground
// truth, but WorldState.robot (and everything the policy/OOD checks compare humans against) is
// MAP frame - confirmed by live measurement to differ by a real, non-negligible offset for this
// project's scenarios. setRobotMapPose() is the fix: getHumans() must translate every returned
// human position by (map pose - world pose) so it's directly comparable to state.robot.
TEST(GroundTruthHumanSource, SetRobotMapPoseCorrectsForWorldMapFrameOffset)
{
  DegradationParams params;  // all zero/off - isolate the frame correction from noise/dropout
  GroundTruthHumanSource source(params);

  // A human at world (5.0, 2.0). Robot's world pose is (3.0, 0.0) - matching this project's
  // real, confirmed offset direction and magnitude (docs/audit.md S1.3's live measurement:
  // world - map ~= (-2.9, -0.07), i.e. map = world + ~3 in x).
  source.setRobotPose(3.0, 0.0, 0.0);
  source.ingestPedestrian(1, 5.0, 2.0, 0.0, 0.0, t(0.0));

  // Robot's MAP pose is (0.0, 0.0) - the offset (map - world) is (-3.0, 0.0), so the human
  // should be reported at map (5.0 - 3.0, 2.0 - 0.0) = (2.0, 2.0), NOT the raw world (5.0, 2.0).
  source.setRobotMapPose(0.0, 0.0);
  auto humans = source.getHumans(t(0.0));

  ASSERT_EQ(humans.size(), 1u);
  EXPECT_DOUBLE_EQ(humans[0].x, 2.0);
  EXPECT_DOUBLE_EQ(humans[0].y, 2.0);
  // Sanity check on the test itself: if this ever regressed to a no-op, the raw world value
  // (5.0) would show up instead - fail loudly, not silently, if that happens.
  EXPECT_NE(humans[0].x, 5.0);
}

TEST(GroundTruthHumanSource, SetRobotMapPoseIsIdentityWhenWorldAndMapPosesMatch)
{
  DegradationParams params;
  GroundTruthHumanSource source(params);

  // If world and map poses genuinely coincide (offset zero), the correction must be a no-op -
  // this is the case every existing test above already implicitly relies on never having been
  // broken by this fix, made explicit here.
  source.setRobotPose(1.0, -1.0, 0.0);
  source.setRobotMapPose(1.0, -1.0);
  source.ingestPedestrian(1, 4.0, 4.0, 0.0, 0.0, t(0.0));

  auto humans = source.getHumans(t(0.0));
  ASSERT_EQ(humans.size(), 1u);
  EXPECT_DOUBLE_EQ(humans[0].x, 4.0);
  EXPECT_DOUBLE_EQ(humans[0].y, 4.0);
}

TEST(GroundTruthHumanSource, PassesThroughUnchangedWhenMapPoseNeverSet)
{
  // Backward-compatibility/safety case: a caller that never calls setRobotMapPose() at all
  // (every test above this point, and any HumanStateSource consumer that doesn't know about
  // this method) must see exactly the old behavior - raw ingested positions, unmodified. This
  // is also what stops the correction from inventing a wrong offset before the controller has
  // ever supplied one.
  DegradationParams params;
  GroundTruthHumanSource source(params);

  source.setRobotPose(3.0, 0.0, 0.0);
  source.ingestPedestrian(1, 5.0, 2.0, 0.0, 0.0, t(0.0));
  auto humans = source.getHumans(t(0.0));

  ASSERT_EQ(humans.size(), 1u);
  EXPECT_DOUBLE_EQ(humans[0].x, 5.0);
  EXPECT_DOUBLE_EQ(humans[0].y, 2.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
