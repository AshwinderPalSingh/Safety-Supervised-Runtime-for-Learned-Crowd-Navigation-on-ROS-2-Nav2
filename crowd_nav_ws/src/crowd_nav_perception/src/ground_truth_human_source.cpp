#include "crowd_nav_perception/ground_truth_human_source.hpp"

#include <algorithm>
#include <array>
#include <cmath>

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

void GroundTruthHumanSource::setRobotPose(double x, double y)
{
  robot_xy_ = std::make_pair(x, y);
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
  robot_xy_ = std::make_pair(msg->position.x, msg->position.y);
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
  if (params_.max_range_m.has_value() && robot_xy_.has_value()) {
    const double dx = raw.x - robot_xy_->first;
    const double dy = raw.y - robot_xy_->second;
    if (std::hypot(dx, dy) > params_.max_range_m.value()) {
      return std::nullopt;
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
  return result;
}

}  // namespace crowd_nav_perception
