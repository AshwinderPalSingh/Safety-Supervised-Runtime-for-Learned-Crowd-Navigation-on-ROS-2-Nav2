#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "crowd_nav_perception/lidar_clustering.hpp"
#include "gtest/gtest.h"

using crowd_nav_perception::clusterScan;
using crowd_nav_perception::ScanCluster;

namespace
{
constexpr double kInf = std::numeric_limits<float>::infinity();
}

TEST(LidarClustering, EmptyScanProducesNoClusters)
{
  std::vector<float> ranges;
  auto clusters = clusterScan(ranges, 0.0, 0.05, 0.1, 10.0);
  EXPECT_TRUE(clusters.empty());
}

TEST(LidarClustering, AllOutOfRangeProducesNoClusters)
{
  std::vector<float> ranges(20, static_cast<float>(kInf));
  auto clusters = clusterScan(ranges, 0.0, 0.05, 0.1, 10.0);
  EXPECT_TRUE(clusters.empty());
}

TEST(LidarClustering, SinglePointClusterHasExactCentroidAndZeroWidth)
{
  // The valid return is at index 1; angle = angle_min + 1*angle_increment, so angle_min must be
  // -angle_increment for that return to sit at angle 0 exactly: (range, 0).
  std::vector<float> ranges = {static_cast<float>(kInf), 2.0f, static_cast<float>(kInf)};
  auto clusters = clusterScan(ranges, -0.1, 0.1, 0.1, 10.0);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].num_points, 1);
  EXPECT_NEAR(clusters[0].x, 2.0, 1e-9);
  EXPECT_NEAR(clusters[0].y, 0.0, 1e-9);
  EXPECT_NEAR(clusters[0].width_m, 0.0, 1e-9);
}

TEST(LidarClustering, ClosePointsAtSameRangeMergeIntoOneCluster)
{
  // 5 consecutive returns at the same range with a small angle step - chord spacing between
  // neighbors is 2*r*sin(d_theta/2) ~= r*d_theta for small angles = 0.5*0.02 = 0.01m, well under
  // the default 0.15m break distance, so this must stay one cluster, not five.
  std::vector<float> ranges(5, 0.5f);
  auto clusters = clusterScan(ranges, 0.0, 0.02, 0.1, 10.0);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].num_points, 5);
  EXPECT_GT(clusters[0].width_m, 0.0);
}

TEST(LidarClustering, InvalidRangeGapSplitsOneGroupIntoTwoClusters)
{
  std::vector<float> ranges = {
    0.5f, 0.5f, 0.5f,
    static_cast<float>(kInf), static_cast<float>(kInf),
    0.5f, 0.5f, 0.5f,
  };
  auto clusters = clusterScan(ranges, 0.0, 0.02, 0.1, 10.0);
  ASSERT_EQ(clusters.size(), 2u);
  EXPECT_EQ(clusters[0].num_points, 3);
  EXPECT_EQ(clusters[1].num_points, 3);
}

TEST(LidarClustering, LargeRangeJumpAtAdjacentAngleSplitsIntoTwoClusters)
{
  // No invalid gap between the two groups - the break has to come from the adaptive-breakpoint
  // distance check itself (a real range discontinuity: something close, then something far,
  // at directly adjacent beams), not from a run of no-return samples.
  std::vector<float> ranges = {0.5f, 0.5f, 0.5f, 5.0f, 5.0f, 5.0f};
  auto clusters = clusterScan(ranges, 0.0, 0.02, 0.1, 10.0, /*break_distance_m=*/0.15);
  ASSERT_EQ(clusters.size(), 2u);
  EXPECT_EQ(clusters[0].num_points, 3);
  EXPECT_EQ(clusters[1].num_points, 3);
}

TEST(LidarClustering, RangeExactlyAtMinAndMaxBoundsIsValid)
{
  std::vector<float> ranges = {0.1f, 10.0f};
  auto clusters = clusterScan(ranges, 0.0, 1.0, 0.1, 10.0);
  // Both points are individually valid but far apart at a 1 rad step and very different ranges
  // - expect two single-point clusters, not zero (confirms the inclusive boundary check, not
  // just that valid points survive at all).
  ASSERT_EQ(clusters.size(), 2u);
  EXPECT_EQ(clusters[0].num_points, 1);
  EXPECT_EQ(clusters[1].num_points, 1);
}

TEST(LidarClustering, JustOutsideMinMaxBoundsIsInvalid)
{
  std::vector<float> ranges = {0.099f, 10.001f};
  auto clusters = clusterScan(ranges, 0.0, 1.0, 0.1, 10.0);
  EXPECT_TRUE(clusters.empty());
}

TEST(LidarClustering, NanRangeIsTreatedAsNoReturn)
{
  std::vector<float> ranges = {
    0.5f, std::numeric_limits<float>::quiet_NaN(), 0.5f,
  };
  auto clusters = clusterScan(ranges, 0.0, 0.02, 0.1, 10.0);
  ASSERT_EQ(clusters.size(), 2u);
  EXPECT_EQ(clusters[0].num_points, 1);
  EXPECT_EQ(clusters[1].num_points, 1);
}

// A real 360-degree LiDAR (this project's actual physical sensor - unlike the sim's own LiDAR,
// masked to ~180 degrees at the source for an unrelated gz-sim rendering bug, README's Known
// Limitations) has no "back of the robot" gap for the array's angular seam to hide in - a human
// standing near angle zero produces returns at both the very start AND the very end of the
// ranges array, which the main forward pass (each sample compared only to the one immediately
// before it in array order) can never see as adjacent on its own.
TEST(LidarClustering, HumanStraddlingTheAngularSeamOnAFullCircleScanIsOneCluster)
{
  // 8 samples spanning exactly 2*pi (45 degrees apart): indices 0,1 sit at angle -pi/-3pi/4,
  // indices 6,7 at pi/2/3pi/4 - i.e. indices 7 and 0 are the two samples immediately across the
  // seam (135 degrees and -180 degrees, one increment apart). A generous break_distance_m (1.0,
  // not this sensor's realistic ~0.15) keeps the geometry easy to hand-verify; the point of this
  // test is the wraparound logic, not resolution-realistic chord lengths (already covered by
  // ClosePointsAtSameRangeMergeIntoOneCluster).
  const double angle_min = -M_PI;
  const double angle_increment = M_PI / 4.0;
  std::vector<float> ranges(8, static_cast<float>(kInf));
  ranges[0] = 1.0f;
  ranges[1] = 1.0f;
  ranges[6] = 1.0f;
  ranges[7] = 1.0f;

  auto clusters = clusterScan(
      ranges, angle_min, angle_increment, 0.1, 10.0, /*break_distance_m=*/1.0);

  ASSERT_EQ(clusters.size(), 1u) << "seam-straddling returns must merge into one cluster, not two";
  EXPECT_EQ(clusters[0].num_points, 4);

  // Independently computed expected centroid/width, same style as the other tests in this file -
  // direct trig calls, not hand-rounded decimals, so EXPECT_NEAR can use a tight tolerance.
  auto pt = [&](int i) {
      const double a = angle_min + i * angle_increment;
      return std::make_pair(std::cos(a), std::sin(a));
    };
  const auto p0 = pt(0);
  const auto p1 = pt(1);
  const auto p6 = pt(6);
  const auto p7 = pt(7);
  const double expected_x = (p0.first + p1.first + p6.first + p7.first) / 4.0;
  const double expected_y = (p0.second + p1.second + p6.second + p7.second) / 4.0;
  const double expected_width = std::hypot(p6.first - p1.first, p6.second - p1.second);
  EXPECT_NEAR(clusters[0].x, expected_x, 1e-9);
  EXPECT_NEAR(clusters[0].y, expected_y, 1e-9);
  EXPECT_NEAR(clusters[0].width_m, expected_width, 1e-9);
}

// The same seam-adjacency the test above merges must NOT be applied to a scan that doesn't
// actually cover a full circle - the sim's own ~180-degree-masked LiDAR is exactly this case,
// and its array's first/last samples are not physically adjacent at all (there's a real,
// unscanned ~180-degree gap between them, not just an array-index gap). Constructed
// adversarially: the two endpoint returns ARE well within break_distance_m of each other in
// Cartesian space, so if the full-circle gate were missing (or buggy), this would incorrectly
// merge anyway - proving it's the gate itself doing the work, not incidental distance.
TEST(LidarClustering, EndpointsCloseTogetherOnAPartialScanAreNotMerged)
{
  const double angle_min = 0.0;
  const double angle_increment = 0.05;  // 4 samples * 0.05 rad = 0.2 rad total span, far from 2*pi
  std::vector<float> ranges = {1.0f, static_cast<float>(kInf), static_cast<float>(kInf), 1.0f};

  auto clusters = clusterScan(
      ranges, angle_min, angle_increment, 0.1, 10.0, /*break_distance_m=*/1.0);

  ASSERT_EQ(clusters.size(), 2u) << "a partial-FOV scan's endpoints must never wraparound-merge";
  EXPECT_EQ(clusters[0].num_points, 1);
  EXPECT_EQ(clusters[1].num_points, 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
