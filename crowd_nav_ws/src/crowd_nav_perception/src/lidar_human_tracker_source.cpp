#include "crowd_nav_perception/lidar_human_tracker_source.hpp"

#include "crowd_nav_perception/lidar_clustering.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer_interface.hpp"

namespace crowd_nav_perception
{

LidarHumanTrackerSource::LidarHumanTrackerSource(
  const LidarPerceptionParams & params, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: params_(params),
  tf_buffer_(std::move(tf_buffer)),
  tracker_(params.gate_distance_m, params.max_track_misses, params.velocity_smoothing),
  logger_(rclcpp::get_logger("lidar_human_tracker_source"))
{
}

void LidarHumanTrackerSource::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  processScan(
    msg->ranges, static_cast<double>(msg->angle_min), static_cast<double>(msg->angle_increment),
    static_cast<double>(msg->range_min), static_cast<double>(msg->range_max),
    msg->header.frame_id, rclcpp::Time(msg->header.stamp));
}

void LidarHumanTrackerSource::processScan(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, const std::string & scan_frame_id,
  const rclcpp::Time & stamp)
{
  const auto clusters = clusterScan(ranges, angle_min, angle_increment, range_min, range_max,
      params_.break_distance_m);

  // One TF lookup per scan, not one per cluster: every cluster in this scan shares the same
  // stamp, so they share the same transform - looking it up once is both cheaper and avoids any
  // possibility of clusters within the same scan disagreeing about whether TF was available.
  //
  // Deliberately calling the tf2::TimePoint overloads (via tf2_ros::fromRclcpp), not the
  // rclcpp::Time convenience overloads Buffer also provides - even though the latter default
  // their own timeout parameter to zero, both route through the same underlying timed-wait
  // implementation, which unconditionally requires (and warns/misbehaves without)
  // setUsingDedicatedThread(true) on the buffer, regardless of the timeout value actually
  // passed. The tf2::TimePoint overloads are inherited straight from tf2::BufferCore (see
  // Buffer's own `using tf2::BufferCore::lookupTransform/canTransform` declarations) - a plain
  // synchronous "what's in the buffer right now" query with no timeout/threading concept at
  // all, structurally unable to hit that code path. Exactly the pattern tf2_ros's own
  // MessageFilter uses internally for the identical purpose (message_filter.hpp), not invented
  // here. Found live, not hypothetically, why this distinction matters: the rclcpp::Time
  // overload segfaulted instead of throwing tf2::TransformException when looking up an unknown
  // frame on a buffer with no dedicated thread (this class's own test-only constructor builds
  // exactly that kind) - inside the exact "TF not up yet" scenario this project's own manual
  // testing already observed live (RViz's "timestamp earlier than all data in the transform
  // cache" warnings). Independently of that crash, a blocking wait of any length inside this
  // call chain would itself be a latency risk - it feeds directly into the controller's
  // 30 ms-watchdog-bounded decision path - so "return immediately with
  // whatever's already buffered" is the correct choice on its own merits, not just a crash
  // workaround.
  std::vector<Detection> detections;
  bool have_transform = false;
  geometry_msgs::msg::TransformStamped scan_to_map;
  const tf2::TimePoint tf_time = tf2_ros::fromRclcpp(stamp);
  try {
    if (tf_buffer_->canTransform(params_.map_frame, scan_frame_id, tf_time)) {
      scan_to_map = tf_buffer_->lookupTransform(params_.map_frame, scan_frame_id, tf_time);
      have_transform = true;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *rclcpp::Clock::make_shared(RCL_ROS_TIME), 2000,
      "LidarHumanTrackerSource: TF lookup %s -> %s failed (%s)",
      scan_frame_id.c_str(), params_.map_frame.c_str(), ex.what());
  }

  if (have_transform) {
    detections.reserve(clusters.size());
    for (const auto & c : clusters) {
      if (c.num_points < params_.min_cluster_points) {continue;}
      if (c.width_m < params_.min_cluster_width_m || c.width_m > params_.max_cluster_width_m) {
        continue;
      }
      geometry_msgs::msg::PointStamped point_in;
      point_in.point.x = c.x;
      point_in.point.y = c.y;
      point_in.point.z = 0.0;
      geometry_msgs::msg::PointStamped point_out;
      tf2::doTransform(point_in, point_out, scan_to_map);
      Detection d;
      d.x = point_out.point.x;
      d.y = point_out.point.y;
      detections.push_back(d);
    }
  }
  // have_transform == false (TF not ready, or genuinely misconfigured): detections stays empty
  // for this whole scan. The tracker below still runs on that empty set, ages existing tracks
  // out normally via numLostLastCall(), and that IS the correct signal - "perception has
  // degraded to nothing" is real information the OOD detector's LOW_PERCEPTION_CONFIDENCE check
  // should see, not something to paper over by freezing stale positions forever.

  latest_humans_ = tracker_.update(detections, stamp);
  last_reported_lost_ = tracker_.numLostLastCall();
}

std::vector<HumanObservation> LidarHumanTrackerSource::getHumans(
  const rclcpp::Time & /*query_time*/)
{
  return latest_humans_;
}

}  // namespace crowd_nav_perception
