#include "crowd_nav_safety_supervisor/safety_supervisor.hpp"

#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"

namespace crowd_nav_safety_supervisor
{

using crowd_nav_observation::WorldState;
using crowd_nav_policy_adapters::Velocity2D;

namespace
{

// S4.8.3: point-cost lookup against the shared local costmap, exploiting the inflation layer
// (already expanded by this project's robot_radius) rather than sweeping a footprint polygon -
// matches MPPI's own ObstaclesCritic (consider_footprint: false against this same costmap).
// NO_INFORMATION is excluded explicitly: it satisfies ">= INSCRIBED_INFLATED_OBSTACLE" too
// (255 > 253), which would otherwise misread "haven't mapped this cell" as "this cell is
// lethal." A point outside the costmap's current bounds is unverifiable, not unsafe.
bool isLethal(nav2_costmap_2d::Costmap2D * costmap, double wx, double wy)
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap->worldToMap(wx, wy, mx, my)) {
    return false;
  }
  const unsigned char cost = costmap->getCost(mx, my);
  return cost != nav2_costmap_2d::NO_INFORMATION &&
         cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

// S4.8.3: secondary, best-effort lookup against the raw keepout mask (nav2_map_server's
// mask_server publishes a plain OccupancyGrid, standard occupied-cell convention: >=50 flagged,
// -1 unknown excluded) - used purely to distinguish KEEPOUT_VIOLATION from generic
// COSTMAP_COLLISION in the intervention log, never consulted for the safety decision itself.
// Found while testing (not hypothetical): OccupancyGrid.info.resolution is float32, while
// Costmap2D's own resolution_ is double - at a world point that lands exactly on a cell
// boundary, that precision difference can shift this lookup's cell index by one relative to the
// primary costmap's, mislabeling the cause at that specific boundary. Accepted as within this
// check's already-stated scope (S4.8.3: a stale/wrong secondary lookup only degrades log
// granularity, never the safety decision), not worth chasing bit-exact alignment for a
// cause-label only.
bool isKeepoutFlagged(const nav_msgs::msg::OccupancyGrid & mask, double wx, double wy)
{
  if (mask.info.resolution <= 0.0) {
    return false;
  }
  const double ox = mask.info.origin.position.x;
  const double oy = mask.info.origin.position.y;
  const int mx = static_cast<int>(std::floor((wx - ox) / mask.info.resolution));
  const int my = static_cast<int>(std::floor((wy - oy) / mask.info.resolution));
  if (mx < 0 || my < 0 || mx >= static_cast<int>(mask.info.width) ||
    my >= static_cast<int>(mask.info.height))
  {
    return false;
  }
  const size_t index = static_cast<size_t>(my) * mask.info.width + static_cast<size_t>(mx);
  if (index >= mask.data.size()) {
    return false;
  }
  return mask.data[index] >= 50;
}

}  // namespace

SafetySupervisor::SafetySupervisor(const SafetySupervisorConfig & config)
: config_(config)
{
}

SupervisorResult SafetySupervisor::checkOodCriteria(
  const WorldState & state, const Velocity2D & candidate, uint32_t num_degraded_this_tick) const
{
  // 1. CROWD_SIZE (S4.4) - operates on the real, pre-dummy-injection human list (S4.8.1).
  if (state.humans.size() > config_.max_train_humans) {
    return {false, InterventionCause::kCrowdSize};
  }

  // 2. PROXIMITY (S4.4) - center-to-center distance against the training-derived threshold
  // (discomfort_dist + policy_radius*2 = 0.8m), not bare collision geometry.
  for (const auto & h : state.humans) {
    const double dist = std::hypot(h.x - state.robot.px, h.y - state.robot.py);
    if (dist < config_.min_train_distance_m) {
      return {false, InterventionCause::kProximity};
    }
  }

  // 3. RELATIVE_SPEED (S4.4) - any human's own speed beyond the training distribution.
  for (const auto & h : state.humans) {
    const double speed = std::hypot(h.vx, h.vy);
    if (speed > config_.max_train_speed_mps) {
      return {false, InterventionCause::kRelativeSpeed};
    }
  }

  // 4. COMMAND_LIMIT (S4.4) - the policy's chosen raw holonomic speed, not the physical clamp
  // toTwistStamped() applies downstream (a distinct, already-safe-by-construction concern).
  const double commanded_speed = std::hypot(candidate.vx, candidate.vy);
  if (commanded_speed > config_.max_commanded_speed_mps) {
    return {false, InterventionCause::kCommandLimit};
  }

  // 5. LOW_PERCEPTION_CONFIDENCE (S4.8.5) - the dropout model's own count for this tick, not a
  // re-derived guess; FOV/range exclusions never reach this count (S4.8.1/S4.8.5).
  if (num_degraded_this_tick > 0) {
    return {false, InterventionCause::kLowPerceptionConfidence};
  }

  return {true, std::nullopt};
}

SupervisorResult SafetySupervisor::checkForwardSim(
  const WorldState & state, const Velocity2D & candidate,
  nav2_costmap_2d::Costmap2D * costmap, const nav_msgs::msg::OccupancyGrid * keepout_mask) const
{
  for (int step = 1; step <= config_.forward_sim_steps; ++step) {
    const double t = config_.forward_sim_dt_s * static_cast<double>(step);
    const double wx = state.robot.px + candidate.vx * t;
    const double wy = state.robot.py + candidate.vy * t;

    if (!isLethal(costmap, wx, wy)) {
      continue;
    }
    if (keepout_mask != nullptr && isKeepoutFlagged(*keepout_mask, wx, wy)) {
      return {false, InterventionCause::kKeepoutViolation};
    }
    return {false, InterventionCause::kCostmapCollision};
  }
  return {true, std::nullopt};
}

}  // namespace crowd_nav_safety_supervisor
