#ifndef CROWD_NAV_SAFETY_SUPERVISOR__SAFETY_SUPERVISOR_CONFIG_HPP_
#define CROWD_NAV_SAFETY_SUPERVISOR__SAFETY_SUPERVISOR_CONFIG_HPP_

#include <cstdint>

namespace crowd_nav_safety_supervisor
{

// IMPLEMENTATION_PLAN.md S4.4/S4.8.3/S4.8.5. All thresholds independently toggleable/tunable -
// none of this is compiled in as a literal at the call site.
struct SafetySupervisorConfig
{
  // Forward-sim (S4.8.3): FIXED horizon/step count, no adaptive refinement, so worst-case cost
  // is known ahead of time. 4 steps at the candidate action space's own time_step_s (0.25s) is a
  // 1.0s look-ahead - reuse that value here rather than a second hand-typed one (set by the
  // caller from CandidateActionSpaceConfig::time_step_s, not duplicated as a literal default).
  int forward_sim_steps = 4;
  double forward_sim_dt_s = 0.25;

  // OOD criteria (S4.4) - crowd size/proximity/speed thresholds derived from the training
  // distribution, not this robot's physical limits.
  uint32_t max_train_humans = 5;
  double min_train_distance_m = 0.8;  // discomfort_dist(0.2) + policy_radius*2(0.3+0.3), S4.4.
  double max_train_speed_mps = 1.5;   // SARL trains humans at up to ~1 m/s; margin above that.

  // COMMAND_LIMIT (S4.4 item 4) - checked against the candidate's raw holonomic speed, mirrors
  // (but is independent of) CrowdNavController's own toTwistStamped() clamp - this is an OOD
  // signal about the policy's *chosen* action, not the physical safety clamp itself.
  double max_commanded_speed_mps = 1.0;
};

}  // namespace crowd_nav_safety_supervisor

#endif  // CROWD_NAV_SAFETY_SUPERVISOR__SAFETY_SUPERVISOR_CONFIG_HPP_
