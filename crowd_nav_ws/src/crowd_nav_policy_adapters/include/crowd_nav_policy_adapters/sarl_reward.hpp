#ifndef CROWD_NAV_POLICY_ADAPTERS__SARL_REWARD_HPP_
#define CROWD_NAV_POLICY_ADAPTERS__SARL_REWARD_HPP_

#include <vector>

#include "crowd_nav_observation/world_state.hpp"

namespace crowd_nav_policy_adapters
{

// A single propagated human: position/velocity/radius only (no id/cov/stamp - this operates
// on one-step-ahead candidate states, not perception output).
struct PropagatedHuman
{
  double px = 0.0;
  double py = 0.0;
  double radius = 0.0;
};

// Reproduces multi_human_rl.py's compute_reward() exactly,
// verified against the actual reference source, not memory. Operates on the ONE-STEP-AHEAD
// propagated self state and propagated humans (candidate_propagation.hpp), matching the
// reference's own `self.compute_reward(next_self_state, next_human_states)` call inside its
// action-search loop.
//
// The 0.2 / 0.5 discomfort constants below are hardcoded exactly as the reference hardcodes
// them - NOT read from env.config's discomfort_dist/discomfort_penalty_factor keys, even
// though this checkpoint's config happens to match those values. Reproducing the code's
// actual behavior, not what the config's naming implies is configurable (S4.7).
double computeImmediateReward(
  const crowd_nav_observation::RobotSelfState & propagated_self,
  const std::vector<PropagatedHuman> & propagated_humans,
  double time_step_s);

}  // namespace crowd_nav_policy_adapters

#endif  // CROWD_NAV_POLICY_ADAPTERS__SARL_REWARD_HPP_
