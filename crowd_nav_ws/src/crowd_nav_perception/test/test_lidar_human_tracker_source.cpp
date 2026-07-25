// Integration tests for the actual thing LidarHumanTrackerSource exists to do: cluster a real
// scan, width-filter it, transform surviving centroids through a REAL tf2_ros::Buffer (not a
// mock), and feed the result into the tracker - end to end, in one test, the same way
// buildWorldState() combining two independently-sourced poses should have been tested from the
// start (docs/audit.md S3.5's finding, not repeated here). The tf2_ros::Buffer used below is a
// standalone one with no live listener, pre-populated via setTransform() - the standard,
// well-established way to unit-test TF-consuming code without a real broadcaster node.
#include <cmath>
#include <memory>

#include "crowd_nav_perception/lidar_human_tracker_source.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "tf2_ros/buffer.h"

using crowd_nav_perception::LidarHumanTrackerSource;
using crowd_nav_perception::LidarPerceptionParams;

namespace
{
rclcpp::Time t(double seconds)
{
  return rclcpp::Time(
    static_cast<int32_t>(seconds),
    static_cast<uint32_t>((seconds - std::floor(seconds)) * 1e9),
    RCL_ROS_TIME);
}

// A single, static map <- lidar_link transform: lidar_link sits at (offset_x, offset_y) in map
// frame with no rotation, so a sensor-frame point (x, y) should land at
// (x + offset_x, y + offset_y) in map frame - a simple, exactly-checkable case.
std::shared_ptr<tf2_ros::Buffer> makeBufferWithStaticTranslation(
  double offset_x, double offset_y)
{
  auto buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = "map";
  tf_msg.header.stamp = t(0.0);
  tf_msg.child_frame_id = "lidar_link";
  tf_msg.transform.translation.x = offset_x;
  tf_msg.transform.translation.y = offset_y;
  tf_msg.transform.rotation.w = 1.0;
  // is_static=true: valid at any lookup time, so the test doesn't need to chase exact stamps -
  // the standard trick for testing TF-consuming code without a live, continuously-publishing
  // broadcaster.
  buffer->setTransform(tf_msg, "test", /*is_static=*/true);
  return buffer;
}

// 4 consecutive same-range returns, angle step 0.1 rad, centered on angle 0 - a compact cluster
// well inside the default human-width band, at a range comfortably inside [range_min, range_max].
struct SyntheticCluster
{
  std::vector<float> ranges;
  double angle_min;
  double angle_increment = 0.1;
  double sensor_frame_x;  // independently computed expected centroid, sensor frame
  double sensor_frame_y;
};

SyntheticCluster makeHumanWidthCluster(double range, double angle_min)
{
  SyntheticCluster c;
  c.ranges = {
    static_cast<float>(range), static_cast<float>(range),
    static_cast<float>(range), static_cast<float>(range),
  };
  c.angle_min = angle_min;
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (int i = 0; i < 4; ++i) {
    const double angle = angle_min + i * c.angle_increment;
    sum_x += range * std::cos(angle);
    sum_y += range * std::sin(angle);
  }
  c.sensor_frame_x = sum_x / 4.0;
  c.sensor_frame_y = sum_y / 4.0;
  return c;
}
}  // namespace

TEST(LidarHumanTrackerSource, HumanWidthClusterProducesADetectionInMapFrame)
{
  auto buffer = makeBufferWithStaticTranslation(1.0, 2.0);
  LidarPerceptionParams params;
  LidarHumanTrackerSource source(params, buffer);

  auto cluster = makeHumanWidthCluster(0.3, -0.15);
  source.processScan(
    cluster.ranges, cluster.angle_min, cluster.angle_increment, 0.1, 10.0, "lidar_link", t(1.0));

  auto humans = source.getHumans(t(1.0));
  ASSERT_EQ(humans.size(), 1u);
  EXPECT_NEAR(humans[0].x, cluster.sensor_frame_x + 1.0, 1e-6);
  EXPECT_NEAR(humans[0].y, cluster.sensor_frame_y + 2.0, 1e-6);
}

TEST(LidarHumanTrackerSource, ClusterWiderThanMaxHumanWidthIsRejected)
{
  auto buffer = makeBufferWithStaticTranslation(0.0, 0.0);
  LidarPerceptionParams params;
  params.max_cluster_width_m = 0.6;
  LidarHumanTrackerSource source(params, buffer);

  // A long, flat, wall-like return: 6 points spanning a wide angle at a constant range, wide
  // enough that its width_m exceeds max_cluster_width_m (0.6m) - this project's own depot world
  // has plenty of exactly this shape of return (walls, and shelf-pole rows close enough
  // together to blur into one cluster) and it must not be reported as a human.
  std::vector<float> ranges(6, 2.0f);
  source.processScan(ranges, -0.5, 0.2, 0.1, 10.0, "lidar_link", t(1.0));

  auto humans = source.getHumans(t(1.0));
  EXPECT_TRUE(humans.empty());
}

TEST(LidarHumanTrackerSource, ClusterBelowMinPointCountIsRejected)
{
  auto buffer = makeBufferWithStaticTranslation(0.0, 0.0);
  LidarPerceptionParams params;
  params.min_cluster_points = 3;
  LidarHumanTrackerSource source(params, buffer);

  std::vector<float> ranges = {0.5f, 0.5f};  // only 2 points, below min_cluster_points
  source.processScan(ranges, 0.0, 0.05, 0.1, 10.0, "lidar_link", t(1.0));

  EXPECT_TRUE(source.getHumans(t(1.0)).empty());
}

TEST(LidarHumanTrackerSource, UnknownTargetFrameIsCaughtNotThrownAndReturnsNoDetections)
{
  // Buffer knows nothing about "lidar_link" at all - simulates TF not being up yet (the exact
  // "Message Filter dropping message... timestamp earlier than all data in the transform
  // cache" condition observed live during this project's own manual testing).
  auto buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  LidarPerceptionParams params;
  LidarHumanTrackerSource source(params, buffer);

  auto cluster = makeHumanWidthCluster(0.3, -0.15);
  EXPECT_NO_THROW(
    source.processScan(
      cluster.ranges, cluster.angle_min, cluster.angle_increment, 0.1, 10.0, "lidar_link",
      t(1.0)));
  EXPECT_TRUE(source.getHumans(t(1.0)).empty());
}

TEST(LidarHumanTrackerSource, TrackPersistsAcrossConsecutiveScansOfTheSameStationaryCluster)
{
  auto buffer = makeBufferWithStaticTranslation(0.0, 0.0);
  LidarPerceptionParams params;
  LidarHumanTrackerSource source(params, buffer);

  auto cluster = makeHumanWidthCluster(0.3, -0.15);
  source.processScan(
    cluster.ranges, cluster.angle_min, cluster.angle_increment, 0.1, 10.0, "lidar_link", t(1.0));
  auto humans1 = source.getHumans(t(1.0));
  source.processScan(
    cluster.ranges, cluster.angle_min, cluster.angle_increment, 0.1, 10.0, "lidar_link", t(1.1));
  auto humans2 = source.getHumans(t(1.1));

  ASSERT_EQ(humans1.size(), 1u);
  ASSERT_EQ(humans2.size(), 1u);
  EXPECT_EQ(humans1[0].id, humans2[0].id);
}

TEST(LidarHumanTrackerSource, NumDegradedLastCallReflectsTracksLostNotSyntheticDropout)
{
  auto buffer = makeBufferWithStaticTranslation(0.0, 0.0);
  LidarPerceptionParams params;
  params.max_track_misses = 1;
  LidarHumanTrackerSource source(params, buffer);

  auto cluster = makeHumanWidthCluster(0.3, -0.15);
  source.processScan(
    cluster.ranges, cluster.angle_min, cluster.angle_increment, 0.1, 10.0, "lidar_link", t(1.0));
  EXPECT_EQ(source.numDegradedLastCall(), 0u);

  std::vector<float> empty_ranges;
  source.processScan(empty_ranges, 0.0, 0.1, 0.1, 10.0, "lidar_link", t(1.1));  // miss 1
  source.processScan(empty_ranges, 0.0, 0.1, 0.1, 10.0, "lidar_link", t(1.2));  // miss 2, dropped
  EXPECT_EQ(source.numDegradedLastCall(), 1u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
