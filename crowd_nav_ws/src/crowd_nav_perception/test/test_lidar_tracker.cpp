// All test timestamps use an explicit RCL_ROS_TIME clock type consistently (same convention as
// test_ground_truth_human_source.cpp), so rclcpp::Time arithmetic never hits a clock-type
// mismatch.
#include <cmath>

#include "crowd_nav_perception/lidar_tracker.hpp"
#include "gtest/gtest.h"

using crowd_nav_perception::Detection;
using crowd_nav_perception::LidarTracker;

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

TEST(LidarTracker, FirstDetectionCreatesTrackWithZeroVelocity)
{
  // min_displacement_m=0.0 here and below: these tests exercise tracking/matching/lifecycle
  // mechanics unrelated to the displacement gate (see the dedicated MinDisplacement* tests
  // further down) - decoupled explicitly rather than left to rely on small incidental motion
  // happening to clear whatever the gate's default is.
  LidarTracker tracker(0.6, 5, 0.5, /*min_displacement_m=*/0.0);
  auto obs = tracker.update({Detection{1.0, 2.0}}, t(0.0));
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_EQ(obs[0].id, 1u);
  EXPECT_DOUBLE_EQ(obs[0].x, 1.0);
  EXPECT_DOUBLE_EQ(obs[0].y, 2.0);
  EXPECT_DOUBLE_EQ(obs[0].vx, 0.0);
  EXPECT_DOUBLE_EQ(obs[0].vy, 0.0);
}

TEST(LidarTracker, TrackIdStaysStableAcrossUpdatesForTheSamePhysicalTrack)
{
  LidarTracker tracker(0.6, 5, 0.5, /*min_displacement_m=*/0.0);
  auto obs1 = tracker.update({Detection{0.0, 0.0}}, t(0.0));
  auto obs2 = tracker.update({Detection{0.1, 0.0}}, t(0.1));
  ASSERT_EQ(obs1.size(), 1u);
  ASSERT_EQ(obs2.size(), 1u);
  EXPECT_EQ(obs1[0].id, obs2[0].id);
}

TEST(LidarTracker, ConsistentMotionProducesVelocityInTheRightDirection)
{
  // velocity_smoothing=1.0 -> no smoothing lag, so after enough consistent-direction updates
  // the estimate converges to the true velocity, not just "some positive x component."
  LidarTracker tracker(
    /*gate_distance_m=*/0.6, /*max_misses=*/5, /*velocity_smoothing=*/1.0,
    /*min_displacement_m=*/0.0);
  tracker.update({Detection{0.0, 0.0}}, t(0.0));
  tracker.update({Detection{0.1, 0.0}}, t(0.1));
  auto obs = tracker.update({Detection{0.2, 0.0}}, t(0.2));
  ASSERT_EQ(obs.size(), 1u);
  EXPECT_NEAR(obs[0].vx, 1.0, 1e-6);  // moving +x at 1 m/s (0.1m per 0.1s)
  EXPECT_NEAR(obs[0].vy, 0.0, 1e-6);
}

TEST(LidarTracker, TrackCoastsThroughAShortGapWithoutBeingDropped)
{
  LidarTracker tracker(
    /*gate_distance_m=*/0.6, /*max_misses=*/3, /*velocity_smoothing=*/0.5,
    /*min_displacement_m=*/0.0);
  tracker.update({Detection{0.0, 0.0}}, t(0.0));
  // Two misses in a row - under max_misses=3, so still alive both times.
  auto obs_miss1 = tracker.update({}, t(0.1));
  auto obs_miss2 = tracker.update({}, t(0.2));
  EXPECT_EQ(obs_miss1.size(), 1u);
  EXPECT_EQ(obs_miss2.size(), 1u);
  EXPECT_EQ(tracker.numLostLastCall(), 0u);
}

TEST(LidarTracker, TrackIsDroppedAndReportedLostAfterTooManyConsecutiveMisses)
{
  LidarTracker tracker(/*gate_distance_m=*/0.6, /*max_misses=*/2, /*velocity_smoothing=*/0.5);
  tracker.update({Detection{0.0, 0.0}}, t(0.0));
  tracker.update({}, t(0.1));  // miss 1
  tracker.update({}, t(0.2));  // miss 2 - still <= max_misses
  auto obs = tracker.update({}, t(0.3));  // miss 3 - exceeds max_misses=2, dropped now
  EXPECT_TRUE(obs.empty());
  EXPECT_EQ(tracker.numLostLastCall(), 1u);
}

TEST(LidarTracker, ReappearingDetectionAfterCoastingGetsANewIdNotTheOldOneOnceDropped)
{
  LidarTracker tracker(
    /*gate_distance_m=*/0.6, /*max_misses=*/1, /*velocity_smoothing=*/0.5,
    /*min_displacement_m=*/0.0);
  auto obs1 = tracker.update({Detection{0.0, 0.0}}, t(0.0));
  tracker.update({}, t(0.1));  // miss 1
  tracker.update({}, t(0.2));  // miss 2 - exceeds max_misses=1, track dropped
  auto obs2 = tracker.update({Detection{0.0, 0.0}}, t(0.3));  // "reappears" at the same spot
  ASSERT_EQ(obs1.size(), 1u);
  ASSERT_EQ(obs2.size(), 1u);
  EXPECT_NE(obs1[0].id, obs2[0].id);
}

TEST(LidarTracker, DetectionFarOutsideGateCreatesANewTrackNotAFalseMatch)
{
  LidarTracker tracker(
    /*gate_distance_m=*/0.3, /*max_misses=*/5, /*velocity_smoothing=*/0.5,
    /*min_displacement_m=*/0.0);
  tracker.update({Detection{0.0, 0.0}}, t(0.0));
  // 5m away, far outside the 0.3m gate.
  auto obs = tracker.update({Detection{5.0, 0.0}}, t(0.1));
  // Both tracks should be alive: the original one coasting (missed this update), and a new one
  // for the far detection.
  ASSERT_EQ(obs.size(), 2u);
  bool found_original = false;
  bool found_new = false;
  for (const auto & o : obs) {
    if (std::abs(o.x - 0.0) < 1e-6 && std::abs(o.y - 0.0) < 1e-6) {found_original = true;}
    if (std::abs(o.x - 5.0) < 1e-6 && std::abs(o.y - 0.0) < 1e-6) {found_new = true;}
  }
  EXPECT_TRUE(found_original);
  EXPECT_TRUE(found_new);
}

TEST(LidarTracker, TwoTracksAssociateToTheirNearestDetectionNotSwapped)
{
  // Two existing tracks at x=0 and x=1. Two new detections close to each, respectively, but
  // each detection is technically within gate distance of BOTH tracks if the gate is generous -
  // greedy nearest-neighbor must still pick the closer pairing for each, not an arbitrary or
  // swapped one.
  LidarTracker tracker(
    /*gate_distance_m=*/0.6, /*max_misses=*/5, /*velocity_smoothing=*/1.0,
    /*min_displacement_m=*/0.0);
  tracker.update({Detection{0.0, 0.0}, Detection{1.0, 0.0}}, t(0.0));
  auto obs = tracker.update({Detection{0.05, 0.0}, Detection{1.05, 0.0}}, t(0.1));
  ASSERT_EQ(obs.size(), 2u);
  // Each observation's velocity should point toward +x by a small amount, not a large jump -
  // confirming each track matched its OWN nearby detection, not the other track's.
  for (const auto & o : obs) {
    EXPECT_NEAR(o.vx, 0.5, 1e-6);  // 0.05m in 0.1s
    EXPECT_NEAR(o.vy, 0.0, 1e-6);
  }
}

// The fix for docs/lidar_perception-findings.md S6's live-observed finding: a static pillar
// sized inside the width filter's human-plausible band is otherwise indistinguishable from a
// person from a single scan. These three tests exercise the gate directly, at its default.

TEST(LidarTracker, StationaryDetectionIsNeverReportedNoMatterHowManyUpdates)
{
  LidarTracker tracker;  // default min_displacement_m=0.15 - a static pillar's own scenario.
  // Same exact position every update, 20 times - a pillar sitting still under a real,
  // continuously-scanning LiDAR, not a synthetic one-shot check.
  std::vector<crowd_nav_perception::HumanObservation> obs;
  for (int i = 0; i < 20; ++i) {
    obs = tracker.update({Detection{1.0, 1.0}}, t(static_cast<double>(i) * 0.1));
  }
  EXPECT_TRUE(obs.empty());
}

TEST(LidarTracker, DetectionIsReportedOnceItClearsTheDisplacementThreshold)
{
  LidarTracker tracker(
    /*gate_distance_m=*/0.6, /*max_misses=*/5, /*velocity_smoothing=*/0.5,
    /*min_displacement_m=*/0.15);
  auto before = tracker.update({Detection{0.0, 0.0}}, t(0.0));
  EXPECT_TRUE(before.empty());  // hasn't moved from its own origin yet
  // 0.1m of real, continuous motion in a straight line - not yet past the 0.15m gate.
  before = tracker.update({Detection{0.1, 0.0}}, t(0.1));
  EXPECT_TRUE(before.empty());
  // Now 0.2m from origin - clears the gate.
  auto after = tracker.update({Detection{0.2, 0.0}}, t(0.2));
  ASSERT_EQ(after.size(), 1u);
  EXPECT_NEAR(after[0].x, 0.2, 1e-9);
}

TEST(LidarTracker, OnceClearedATrackStaysReportedEvenIfItLaterStopsMoving)
{
  // Checked against the track's own fixed origin, not a sliding window: a person who proved
  // they're not a pillar once shouldn't disappear again just because they later stand still.
  LidarTracker tracker(
    /*gate_distance_m=*/0.6, /*max_misses=*/5, /*velocity_smoothing=*/0.5,
    /*min_displacement_m=*/0.15);
  tracker.update({Detection{0.0, 0.0}}, t(0.0));
  auto cleared = tracker.update({Detection{0.2, 0.0}}, t(0.1));
  ASSERT_EQ(cleared.size(), 1u);
  // Stands still at (0.2, 0.0) for several more updates.
  auto still1 = tracker.update({Detection{0.2, 0.0}}, t(0.2));
  auto still2 = tracker.update({Detection{0.2, 0.0}}, t(0.3));
  ASSERT_EQ(still1.size(), 1u);
  ASSERT_EQ(still2.size(), 1u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
