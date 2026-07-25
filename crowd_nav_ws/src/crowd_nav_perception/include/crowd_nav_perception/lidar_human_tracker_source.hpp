#ifndef CROWD_NAV_PERCEPTION__LIDAR_HUMAN_TRACKER_SOURCE_HPP_
#define CROWD_NAV_PERCEPTION__LIDAR_HUMAN_TRACKER_SOURCE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "crowd_nav_perception/human_state_source.hpp"
#include "crowd_nav_perception/lidar_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"

namespace crowd_nav_perception
{

struct LidarPerceptionParams
{
  double break_distance_m = 0.15;
  // Width band a surviving cluster must fall in to be treated as a human-plausible detection -
  // at this robot's LiDAR mount height (README's Known Limitations, ~180 deg/8m budget sensor),
  // a single pedestrian typically presents as one contiguous cluster somewhere in this range;
  // narrower rejects sparse noise, wider rejects walls/shelf poles/pillars (this project's own
  // depot world - docs/phase0-findings.md - has plenty of those to reject).
  double min_cluster_width_m = 0.05;
  double max_cluster_width_m = 0.60;
  int min_cluster_points = 3;
  double gate_distance_m = 0.6;
  int max_track_misses = 5;
  double velocity_smoothing = 0.5;
  std::string map_frame = "map";
};

// The real, non-ground-truth HumanStateSource docs/lidar_perception-findings.md's audit exists
// to build: clusters the robot's actual /scan returns (lidar_clustering.hpp), width-filters
// clusters to plausible human leg/torso cross-sections, and transforms surviving centroids into
// map frame via the SAME tf2_ros::Buffer Nav2 already maintains for the robot's own
// localization - not a second, independently-sourced pose channel the way
// GroundTruthHumanSource's setRobotMapPose() fix had to correct for after the fact
// (docs/audit.md S1.3). That's deliberate, not incidental: using the robot's own real TF tree
// for both the robot's own pose and every human detection's frame conversion makes the map/
// world frame-mismatch bug class structurally impossible here, rather than merely fixed for one
// instance of it.
//
// setRobotMapPose() is intentionally NOT overridden - HumanObservations already come out of
// getHumans() in map frame via the TF transform in processScan(), exactly the "already reports
// positions relative to the robot's own localized frame, needs no conversion" case
// human_state_source.hpp's own interface comment anticipates for a real sensor-based source.
class LidarHumanTrackerSource : public HumanStateSource
{
public:
  // Templated on the node type (same reason as GroundTruthHumanSource, docs/phase7-findings.md/
  // phase8-findings.md): works with both rclcpp::Node and rclcpp_lifecycle::LifecycleNode, which
  // every nav2_core plugin actually receives. tf_buffer is the controller's own
  // std::shared_ptr<tf2_ros::Buffer> (the same one passed into nav2_core::Controller::configure,
  // never a second buffer this class builds itself) - see the class comment for why sharing it
  // matters, not just convenience.
  template<typename NodeT>
  LidarHumanTrackerSource(
    const std::shared_ptr<NodeT> & node,
    const std::string & scan_topic,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const LidarPerceptionParams & params)
  : params_(params),
    tf_buffer_(std::move(tf_buffer)),
    tracker_(params.gate_distance_m, params.max_track_misses, params.velocity_smoothing),
    logger_(node->get_logger())
  {
    scan_sub_ = node->template create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {onScan(msg);});
  }

  // Test-only constructor: no ROS subscription. tf_buffer must still be supplied (a standalone
  // tf2_ros::Buffer with no listener attached, pre-populated via setTransform() - the standard,
  // well-established way to unit-test TF-consuming code without a live broadcaster) so
  // processScan() below - the actual integration this class exists for, clustering + width
  // filtering + the real TF transform math + tracking, in one place - gets real test coverage
  // rather than relying on lidar_clustering.cpp/lidar_tracker.cpp's own unit tests to imply it
  // works end to end. Exactly the gap docs/audit.md S3.5 found in buildWorldState() - not
  // repeating it here.
  LidarHumanTrackerSource(
    const LidarPerceptionParams & params, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  std::vector<HumanObservation> getHumans(const rclcpp::Time & query_time) override;
  uint32_t numDegradedLastCall() const override {return last_reported_lost_;}

  // Pure(r) processing entry point - onScan is a thin field-extractor calling straight into
  // this, the same "pure core, thin ROS-owning wrapper" split as
  // GroundTruthHumanSource::ingestPedestrian() and every other production class in this project
  // (ControllerDecisionCore, SafetySupervisor).
  void processScan(
    const std::vector<float> & ranges, double angle_min, double angle_increment,
    double range_min, double range_max, const std::string & scan_frame_id,
    const rclcpp::Time & stamp);

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  LidarPerceptionParams params_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  LidarTracker tracker_;
  rclcpp::Logger logger_;

  std::vector<HumanObservation> latest_humans_;
  uint32_t last_reported_lost_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

}  // namespace crowd_nav_perception

#endif  // CROWD_NAV_PERCEPTION__LIDAR_HUMAN_TRACKER_SOURCE_HPP_
