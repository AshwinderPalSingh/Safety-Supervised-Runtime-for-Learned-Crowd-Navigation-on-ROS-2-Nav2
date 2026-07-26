#include "crowd_nav_perception/ground_truth_human_source.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace crowd_nav_perception
{

GroundTruthHumanSource::GroundTruthHumanSource(const DegradationParams & params)
: params_(params),
  rng_(params.degradation_seed),
  pos_noise_dist_(0.0, params.sigma_pos_m),
  vel_noise_dist_(0.0, params.sigma_vel_mps),
  dropout_dist_(params.dropout_prob)
{
}

void GroundTruthHumanSource::setRobotPose(double x, double y, double theta)
{
  robot_pose_ = std::make_tuple(x, y, theta);
}

void GroundTruthHumanSource::setRobotMapPose(double x, double y)
{
  robot_map_pose_ = std::make_pair(x, y);
}

uint64_t GroundTruthHumanSource::deriveSubstreamSeed(uint64_t scenario_seed, uint32_t subsystem_tag)
{
  std::seed_seq seq{
    static_cast<uint32_t>(scenario_seed & 0xFFFFFFFFu),
    static_cast<uint32_t>(scenario_seed >> 32),
    subsystem_tag};
  std::array<uint32_t, 2> result{};
  seq.generate(result.begin(), result.end());
  return (static_cast<uint64_t>(result[0]) << 32) | static_cast<uint64_t>(result[1]);
}

void GroundTruthHumanSource::onRobotPose(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  robot_pose_ = std::make_tuple(msg->position.x, msg->position.y, tf2::getYaw(msg->orientation));
}

void GroundTruthHumanSource::onPedestrianArray(
  const crowd_nav_pedestrians::msg::PedestrianArray::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  for (const auto & p : msg->pedestrians) {
    ingestPedestrian(p.id, p.x, p.y, p.vx, p.vy, stamp);
  }
}

void GroundTruthHumanSource::ingestPedestrian(
  uint32_t id, double x, double y, double vx, double vy, const rclcpp::Time & stamp)
{
  HumanObservation raw;
  raw.id = id;
  raw.x = x;
  raw.y = y;
  raw.vx = vx;
  raw.vy = vy;
  raw.stamp = stamp;

  auto degraded = degrade(raw);
  if (!degraded.has_value()) {
    return;  // dropped out or out of range this tick - simply absent, not synthesized.
  }

  auto & buf = history_[id];
  buf.emplace_back(stamp, degraded.value());

  // Bounded retention: keep only what a query could plausibly still need given the
  // configured latency, plus a small margin - not indexed by tick count (see class comment).
  const double retention_s = std::max(params_.latency_s, 0.0) + 1.0;
  while (!buf.empty() && (stamp - buf.front().first).seconds() > retention_s) {
    buf.pop_front();
  }
}

std::optional<HumanObservation> GroundTruthHumanSource::degrade(const HumanObservation & raw)
{
  // FOV/range restriction: what this robot's real sensor could
  // ever perceive, not a confidence signal - excluded here without touching the dropout
  // accumulator below, since a human legitimately outside sensor coverage is normal operation.
  if (robot_pose_.has_value()) {
    const auto & [rx, ry, rtheta] = robot_pose_.value();
    const double dx = raw.x - rx;
    const double dy = raw.y - ry;

    if (params_.max_range_m.has_value() && std::hypot(dx, dy) > params_.max_range_m.value()) {
      return std::nullopt;
    }
    if (params_.fov_half_angle_rad.has_value()) {
      const double angle_to_human = std::atan2(dy, dx) - rtheta;
      const double normalized_angle = std::atan2(
        std::sin(angle_to_human), std::cos(angle_to_human));
      if (std::abs(normalized_angle) > params_.fov_half_angle_rad.value()) {
        return std::nullopt;
      }
    }
  }

  // Always draw, regardless of whether sigma/prob is zero, so the RNG consumption sequence
  // per tick doesn't change shape based on degradation parameter values - only their scale
  // does. Keeps a degradation sweep's draw pattern consistent across sweep points.
  const double dx = pos_noise_dist_(rng_);
  const double dy = pos_noise_dist_(rng_);
  const double dvx = vel_noise_dist_(rng_);
  const double dvy = vel_noise_dist_(rng_);
  const bool dropped = dropout_dist_(rng_);

  if (dropped) {
    // Only the dropout model counts toward "degraded" (S4.8.5) - the FOV/range exclusion above
    // returns before this point and never touches this accumulator.
    ++dropped_since_query_;
    return std::nullopt;
  }

  HumanObservation out = raw;
  out.x += dx;
  out.y += dy;
  out.vx += dvx;
  out.vy += dvy;
  out.cov_xx = params_.sigma_pos_m * params_.sigma_pos_m;
  out.cov_yy = params_.sigma_pos_m * params_.sigma_pos_m;
  out.cov_xy = 0.0;
  return out;
}

std::optional<HumanObservation> GroundTruthHumanSource::delayedLookup(
  uint32_t id, const rclcpp::Time & query_time)
{
  auto it = history_.find(id);
  if (it == history_.end()) {
    return std::nullopt;
  }
  const rclcpp::Time target_time = query_time - rclcpp::Duration::from_seconds(params_.latency_s);

  // Freshest sample at or before target_time - a plain linear scan is fine here, the buffer is
  // bounded to (latency_s + 1s) worth of samples, not an unbounded history.
  std::optional<HumanObservation> best;
  for (const auto & [stamp, obs] : it->second) {
    if (stamp <= target_time) {
      best = obs;
    } else {
      break;
    }
  }
  return best;
}

std::vector<HumanObservation> GroundTruthHumanSource::getHumans(const rclcpp::Time & query_time)
{
  std::vector<HumanObservation> result;
  for (const auto & [id, buf] : history_) {
    auto obs = delayedLookup(id, query_time);
    if (obs.has_value()) {
      result.push_back(obs.value());
    }
  }

  // World-to-map frame correction (docs/audit.md S1.3): everything stored in history_ is in
  // Gazebo WORLD frame (raw.x/y as ingested from /pedestrians, itself world frame - see the
  // audit for the full trace). WorldState.robot is populated from Nav2's own pose argument,
  // which is MAP frame. These are NOT the same frame in general - confirmed by live measurement
  // to differ by a real, non-negligible offset for this project's scenarios - so returning raw
  // world-frame positions here silently fed the policy and the OOD PROXIMITY check a wrong
  // relative robot-human vector for as long as this class has existed. Correct by translating
  // every returned position by (current map pose - current world pose): the one offset that
  // makes "distance from state.robot" (map frame) and "this human's position" (now also map
  // frame) consistent. Passes through unchanged only if setRobotMapPose() was never called
  // (should not happen via the real controller, which always has a valid map-frame pose before
  // computeVelocityCommands() runs) or if world pose isn't known yet (same "not available yet"
  // window setRobotPose()/onRobotPose() already have) - neither silently invents a wrong number.
  if (robot_map_pose_.has_value() && robot_pose_.has_value()) {
    const auto & [rmx, rmy] = robot_map_pose_.value();
    const auto & [rwx, rwy, _] = robot_pose_.value();
    const double offset_x = rmx - rwx;
    const double offset_y = rmy - rwy;
    for (auto & h : result) {
      h.x += offset_x;
      h.y += offset_y;
    }
  }

  // Snapshot-and-reset (S4.8.5): whatever degrade() accumulated since the previous getHumans()
  // call becomes this call's reported figure, then the accumulator starts fresh for the next one.
  last_reported_dropped_ = dropped_since_query_;
  dropped_since_query_ = 0;
  return result;
}

}  // namespace crowd_nav_perception
