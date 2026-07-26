#include "crowd_nav_perception/lidar_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace crowd_nav_perception
{

LidarTracker::LidarTracker(
  double gate_distance_m, int max_misses, double velocity_smoothing, double min_displacement_m)
: gate_distance_m_(gate_distance_m),
  max_misses_(max_misses),
  velocity_smoothing_(velocity_smoothing),
  min_displacement_m_(min_displacement_m)
{
}

std::vector<HumanObservation> LidarTracker::update(
  const std::vector<Detection> & detections, const rclcpp::Time & stamp)
{
  for (auto & t : tracks_) {t.matched_this_call = false;}

  // Greedy global nearest-neighbor: repeatedly take the closest still-available (track,
  // detection) pair under the gate, assign it, remove both from further consideration. Not
  // optimal assignment (Hungarian algorithm) - greedy is the standard, simple choice at this
  // scale (a handful of pedestrians, not a dense-crowd association problem), matching the same
  // "classical, inspectable over more powerful but harder to verify" choice clusterScan() makes.
  struct Candidate
  {
    double dist;
    size_t track_idx;
    size_t det_idx;
  };
  std::vector<Candidate> candidates;
  for (size_t ti = 0; ti < tracks_.size(); ++ti) {
    for (size_t di = 0; di < detections.size(); ++di) {
      const double d = std::hypot(
        tracks_[ti].x - detections[di].x, tracks_[ti].y - detections[di].y);
      if (d <= gate_distance_m_) {
        candidates.push_back({d, ti, di});
      }
    }
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {return a.dist < b.dist;});

  std::vector<bool> track_used(tracks_.size(), false);
  std::vector<bool> detection_used(detections.size(), false);
  for (const auto & c : candidates) {
    if (track_used[c.track_idx] || detection_used[c.det_idx]) {continue;}
    track_used[c.track_idx] = true;
    detection_used[c.det_idx] = true;
    Track & t = tracks_[c.track_idx];
    const auto & det = detections[c.det_idx];
    const double dt = (stamp - t.last_update_stamp).seconds();
    if (dt > 1e-6) {
      const double raw_vx = (det.x - t.x) / dt;
      const double raw_vy = (det.y - t.y) / dt;
      // Exponential smoothing, not a raw finite difference from one noisy detection to the
      // next: a single-frame velocity estimate off real clustered LiDAR centroids is noisy
      // enough (centroid jitter from which returns happened to cluster this tick) to make
      // PROXIMITY/RELATIVE_SPEED fire on tracker noise rather than real motion.
      t.vx = velocity_smoothing_ * raw_vx + (1.0 - velocity_smoothing_) * t.vx;
      t.vy = velocity_smoothing_ * raw_vy + (1.0 - velocity_smoothing_) * t.vy;
    }
    t.x = det.x;
    t.y = det.y;
    t.misses_in_a_row = 0;
    t.matched_this_call = true;
    t.last_update_stamp = stamp;
  }

  // Unmatched detections become new tracks - zero initial velocity (no prior sample to diff
  // against), which is the honest answer, not an extrapolation from nothing.
  for (size_t di = 0; di < detections.size(); ++di) {
    if (detection_used[di]) {continue;}
    Track t;
    t.id = next_id_++;
    t.x = detections[di].x;
    t.y = detections[di].y;
    t.origin_x = detections[di].x;
    t.origin_y = detections[di].y;
    t.last_update_stamp = stamp;
    t.matched_this_call = true;
    tracks_.push_back(t);
  }

  // Unmatched tracks coast (keep last known position/velocity) until max_misses, then drop.
  lost_last_call_ = 0;
  std::vector<Track> surviving;
  surviving.reserve(tracks_.size());
  for (auto & t : tracks_) {
    if (!t.matched_this_call) {
      t.misses_in_a_row += 1;
      if (t.misses_in_a_row > max_misses_) {
        lost_last_call_ += 1;
        continue;
      }
    }
    surviving.push_back(t);
  }
  tracks_ = std::move(surviving);

  std::vector<HumanObservation> out;
  out.reserve(tracks_.size());
  for (const auto & t : tracks_) {
    // Never-moved tracks are withheld entirely - not reported with reduced confidence, not
    // counted toward CROWD_SIZE, not fed to the policy at all - since a track that hasn't
    // proven it moves is, on the evidence available, indistinguishable from a static pillar.
    if (std::hypot(t.x - t.origin_x, t.y - t.origin_y) < min_displacement_m_) {continue;}
    HumanObservation obs;
    obs.id = t.id;
    obs.x = t.x;
    obs.y = t.y;
    obs.vx = t.vx;
    obs.vy = t.vy;
    // Fixed, conservative measurement-noise covariance rather than fake per-track precision - a
    // real value would come from the sensor's own range noise plus clustering centroid
    // variance; cov_xy stays 0.0, same "not filled in yet, schema ready for it" status
    // human_state_source.hpp's own comment already documents for GroundTruthHumanSource.
    obs.cov_xx = 0.05 * 0.05;
    obs.cov_yy = 0.05 * 0.05;
    obs.stamp = stamp;
    out.push_back(obs);
  }
  return out;
}

}  // namespace crowd_nav_perception
