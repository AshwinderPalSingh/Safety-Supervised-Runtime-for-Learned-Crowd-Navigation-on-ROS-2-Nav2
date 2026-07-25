#ifndef CROWD_NAV_PERCEPTION__LIDAR_TRACKER_HPP_
#define CROWD_NAV_PERCEPTION__LIDAR_TRACKER_HPP_

#include <cstdint>
#include <vector>

#include "crowd_nav_perception/human_state_source.hpp"
#include "rclcpp/time.hpp"

namespace crowd_nav_perception
{

// A single position measurement, already in whatever frame HumanObservation reports positions
// in (map frame, by construction of LidarHumanTrackerSource - see that class's comment).
// LidarTracker itself is frame-agnostic; it never needs to know which frame this is.
struct Detection
{
  double x = 0.0;
  double y = 0.0;
};

// Frame-to-frame greedy nearest-neighbor tracker: associates each new Detection to the closest
// existing track within gate_distance_m, ages out unmatched tracks after max_misses consecutive
// updates with no match, and assigns a persistent id for a track's whole lifetime. A real
// sensor's detections have no ground-truth id to recover, so a monotonically increasing counter
// is the honest equivalent - not a byte-for-byte match to which real human is which, which no
// unmarked 2D LiDAR detector can promise anyway.
//
// Deliberately not a Kalman filter or multi-hypothesis tracker: greedy nearest-neighbor is the
// standard, well-understood minimum viable tracker at this scale (a handful of pedestrians, not
// a dense-crowd counting problem), and matches this project's own established preference
// (docs/lessons.md) for classical, inspectable methods over more powerful but harder-to-verify
// ones - especially for a first real, non-ground-truth perception pipeline.
class LidarTracker
{
public:
  explicit LidarTracker(
    double gate_distance_m = 0.6, int max_misses = 5, double velocity_smoothing = 0.5);

  // Associates detections against existing tracks, updates position/velocity for matched
  // tracks, creates new tracks for unmatched detections, and ages out (reporting via
  // numLostLastCall()) tracks that missed too many updates in a row. Returns one
  // HumanObservation per currently-alive track (including ones that missed this update but
  // haven't hit max_misses yet, holding their last known position/velocity - the standard
  // "coast through a brief occlusion" behavior), not one per raw detection.
  std::vector<HumanObservation> update(
    const std::vector<Detection> & detections, const rclcpp::Time & stamp);

  // Tracks aged out (crossed max_misses) on the most recent update() call - the closest honest
  // analogue to GroundTruthHumanSource's numDegradedLastCall() a real tracker has: a track
  // genuinely being lost (occlusion, a sensor gap, a human leaving FOV/range) is a real
  // perception-confidence event the LOW_PERCEPTION_CONFIDENCE OOD check should be able to see,
  // the same way a synthetic dropout event does today for GroundTruthHumanSource.
  uint32_t numLostLastCall() const {return lost_last_call_;}

private:
  struct Track
  {
    uint32_t id = 0;
    double x = 0.0;
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    int misses_in_a_row = 0;
    rclcpp::Time last_update_stamp;
    bool matched_this_call = false;
  };

  double gate_distance_m_;
  int max_misses_;
  double velocity_smoothing_;
  uint32_t next_id_ = 1;
  uint32_t lost_last_call_ = 0;
  std::vector<Track> tracks_;
};

}  // namespace crowd_nav_perception

#endif  // CROWD_NAV_PERCEPTION__LIDAR_TRACKER_HPP_
