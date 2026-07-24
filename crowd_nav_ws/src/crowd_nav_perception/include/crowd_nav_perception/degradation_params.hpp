#ifndef CROWD_NAV_PERCEPTION__DEGRADATION_PARAMS_HPP_
#define CROWD_NAV_PERCEPTION__DEGRADATION_PARAMS_HPP_

#include <cstdint>
#include <optional>

namespace crowd_nav_perception
{

// IMPLEMENTATION_PLAN.md S4.2. All defaults are zero/off - oracle passthrough unless
// configured. degradation_seed is REQUIRED to be independent of whatever seed drives Phase 4's
// pedestrian-motion simulation (see GroundTruthHumanSource's constructor comment for why this
// matters and how it's enforced) - it is not the raw scenario seed, it's a value already
// derived into its own substream by the caller (or GroundTruthHumanSource derives it itself,
// see below).
struct DegradationParams
{
  double sigma_pos_m = 0.0;
  double sigma_vel_mps = 0.0;
  double dropout_prob = 0.0;
  double latency_s = 0.0;
  std::optional<double> max_range_m;
  // Angular half-FOV relative to the robot's current heading (IMPLEMENTATION_PLAN.md S4.8.1) -
  // e.g. M_PI/2 for this robot's real ~180 degree sensor. Unset (default) means no angular
  // restriction, matching every other field's "off unless configured" convention. Requires
  // setRobotPose()'s theta to have been set at least once; if it hasn't, this check is skipped
  // the same way the range check already skips when the robot's pose is unset.
  std::optional<double> fov_half_angle_rad;
  bool occlusion_check = false;  // ray-casts against the static costmap, not full 3D.
  // [DEFERRED, Phase 5 scope] - the brief's own "optional" framing of occlusion (S4.2) and this
  // plan's S5 cut-list both flag it as the first thing to drop under time pressure. Not
  // implemented in this phase: occlusion_check is accepted as a parameter and validated, but
  // GroundTruthHumanSource does not yet perform the raycast - see docs/phase5-findings.md.
  uint64_t degradation_seed = 0;
  double tick_period_s = 0.0;  // Must match the observation builder's actual tick rate - see
                                // the ring-buffer sizing note in GroundTruthHumanSource.
};

}  // namespace crowd_nav_perception

#endif  // CROWD_NAV_PERCEPTION__DEGRADATION_PARAMS_HPP_
