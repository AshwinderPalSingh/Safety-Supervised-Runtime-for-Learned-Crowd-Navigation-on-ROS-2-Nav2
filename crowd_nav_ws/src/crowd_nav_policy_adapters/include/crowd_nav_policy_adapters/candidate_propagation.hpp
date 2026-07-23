#ifndef CROWD_NAV_POLICY_ADAPTERS__CANDIDATE_PROPAGATION_HPP_
#define CROWD_NAV_POLICY_ADAPTERS__CANDIDATE_PROPAGATION_HPP_

#include "crowd_nav_observation/world_state.hpp"
#include "crowd_nav_policy_adapters/candidate_action_space.hpp"

namespace crowd_nav_policy_adapters
{

// One-step lookahead reproducing cadrl.py's propagate() exactly (S4.3.1, holonomic branch
// only - the only kinematics mode the pinned checkpoint was trained with):
//   self:   px += action.vx * dt, py += action.vy * dt; vx/vy become action.vx/vy;
//           radius/gx/gy/v_pref/theta unchanged.
//   humans: px += vx * dt, py += vy * dt using EACH HUMAN'S OWN current velocity (constant-
//           velocity assumption) - not the candidate action, which only ever applies to self.
// This is the deployment-time path (cadrl.py's predict() also has a query_env=True branch that
// asks the training-time simulator for the true next human states; there is no simulator to
// query once this runs against Gazebo/real sensors, so this constant-velocity propagation is
// the only option actually available outside training - see S4.3.1).
crowd_nav_observation::WorldState propagateCandidate(
  const crowd_nav_observation::WorldState & state,
  const CandidateAction & action,
  double time_step_s);

}  // namespace crowd_nav_policy_adapters

#endif  // CROWD_NAV_POLICY_ADAPTERS__CANDIDATE_PROPAGATION_HPP_
