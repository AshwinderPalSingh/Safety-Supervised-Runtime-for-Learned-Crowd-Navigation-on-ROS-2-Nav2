#include "crowd_nav_policy_adapters/candidate_propagation.hpp"

#include "rclcpp/duration.hpp"

namespace crowd_nav_policy_adapters
{

using crowd_nav_observation::WorldState;

WorldState propagateCandidate(
  const WorldState & state, const CandidateAction & action, double time_step_s)
{
  WorldState next = state;

  next.robot.px = state.robot.px + action.vx * time_step_s;
  next.robot.py = state.robot.py + action.vy * time_step_s;
  next.robot.vx = action.vx;
  next.robot.vy = action.vy;
  // radius/gx/gy/v_pref/theta unchanged (holonomic propagate() doesn't touch them).

  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(time_step_s);
  for (auto & h : next.humans) {
    h.x = h.x + h.vx * time_step_s;
    h.y = h.y + h.vy * time_step_s;
    // vx/vy/cov_*/id unchanged - constant-velocity assumption, per propagate()'s
    // ObservableState branch.
    h.stamp = h.stamp + dt;
  }
  return next;
}

}  // namespace crowd_nav_policy_adapters
