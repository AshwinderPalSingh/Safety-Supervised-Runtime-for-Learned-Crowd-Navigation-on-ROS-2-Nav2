#ifndef CROWD_NAV_SAFETY_SUPERVISOR__SAFETY_SUPERVISOR_HPP_
#define CROWD_NAV_SAFETY_SUPERVISOR__SAFETY_SUPERVISOR_HPP_

#include <optional>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "crowd_nav_observation/world_state.hpp"
#include "crowd_nav_policy_adapters/policy_adapter.hpp"
#include "crowd_nav_safety_supervisor/intervention_cause.hpp"
#include "crowd_nav_safety_supervisor/safety_supervisor_config.hpp"

namespace crowd_nav_safety_supervisor
{

struct SupervisorResult
{
  bool safe = true;
  // Unset iff safe. When !safe, this is the FIRST cause found - callers only need one reason to
  // reject a tick, not an exhaustive list.
  std::optional<InterventionCause> cause;
};

// Phase 9. Pure C++ (no live ROS node needed) - the same
// "separate testable core, thin ROS-owning wrapper" split already used for
// ControllerDecisionCore (S4.6) and GroundTruthHumanSource (S4.1/S4.2): depends on nav2_costmap_2d
// and nav_msgs *types* (Costmap2D, OccupancyGrid), not a live node, subscription, or lifecycle -
// a caller (CrowdNavController) owns fetching the live costmap pointer and the keepout mask
// message and passes them in per call, matching Nav2's own per-tick costmap-pointer convention
// (not cached once at construction, since the underlying buffer can be swapped/resized).
//
// Two independent checks, kept as separate methods rather than one combined evaluate():
// checkOodCriteria() needs no costmap at all (cheap, run it first); checkForwardSim() needs the
// live costmap and is the only one requiring nav2_costmap_2d's heavier types in a test.
class SafetySupervisor
{
public:
  explicit SafetySupervisor(const SafetySupervisorConfig & config);

  // S4.4 criteria 1/2/3/4/5 (CROWD_SIZE, PROXIMITY, RELATIVE_SPEED, COMMAND_LIMIT,
  // LOW_PERCEPTION_CONFIDENCE). `state` must be the ORIGINAL WorldState - i.e. never one that's
  // been through SarlAdapter::buildInputs()'s dummy-injection (S4.8.1) - or CROWD_SIZE/PROXIMITY
  // would be corrupted by counting a placeholder human that doesn't exist in reality.
  // `num_degraded_this_tick` is HumanStateSource::numDegradedLastCall() (S4.8.5) - the dropout
  // model's own count, not a re-derived guess.
  SupervisorResult checkOodCriteria(
    const crowd_nav_observation::WorldState & state,
    const crowd_nav_policy_adapters::Velocity2D & candidate,
    uint32_t num_degraded_this_tick) const;

  // S4.8.2/S4.8.3: forward-simulates `candidate` (the raw holonomic velocity, used as a
  // deliberately conservative constant-velocity world-frame proxy for the robot's actual
  // diff-drive-constrained motion - S4.8.3 argues why this can only be over-cautious, never
  // unsafely permissive) over a FIXED horizon/step count, checking each step's point cost
  // against `costmap` (the SAME Costmap2D* the embedded MPPI reads from - pass
  // costmap_ros_->getCostmap() fresh each call, S4.8.2). `keepout_mask` is optional
  // (best-effort, cause-labeling only, S4.8.3) - nullptr degrades gracefully to the generic
  // COSTMAP_COLLISION label; it is never consulted for the safety decision itself.
  SupervisorResult checkForwardSim(
    const crowd_nav_observation::WorldState & state,
    const crowd_nav_policy_adapters::Velocity2D & candidate,
    nav2_costmap_2d::Costmap2D * costmap,
    const nav_msgs::msg::OccupancyGrid * keepout_mask) const;

private:
  SafetySupervisorConfig config_;
};

}  // namespace crowd_nav_safety_supervisor

#endif  // CROWD_NAV_SAFETY_SUPERVISOR__SAFETY_SUPERVISOR_HPP_
