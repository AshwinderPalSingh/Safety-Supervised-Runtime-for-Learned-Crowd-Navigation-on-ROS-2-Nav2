#include "crowd_nav_policy_adapters/sarl_adapter.hpp"

#include <cmath>
#include <limits>

#include "crowd_nav_policy_adapters/candidate_propagation.hpp"
#include "crowd_nav_policy_adapters/sarl_reward.hpp"
#include "crowd_nav_policy_adapters/sarl_rotate.hpp"

namespace crowd_nav_policy_adapters
{

using crowd_nav_observation::WorldState;

SarlAdapter::SarlAdapter(const CandidateActionSpaceConfig & config)
: config_(config)
{
}

ShapeSpec SarlAdapter::expectedShape() const
{
  ShapeSpec spec;
  spec.input_names = {kInputName};
  spec.input_shapes = {{-1, -1, 13}};  // (candidates, num_humans) both dynamic - S4.7
  spec.output_names = {kOutputName};
  spec.output_shapes = {{-1, 1}};
  return spec;
}

TensorBundle SarlAdapter::buildInputs(const WorldState & state)
{
  last_candidates_ = buildCandidateActionSpace(config_, state.robot.v_pref);
  last_num_humans_ = state.humans.size();
  last_rewards_.assign(last_candidates_.size(), 0.0);

  TensorBundle inputs;
  inputs.names = {kInputName};

  if (last_num_humans_ == 0) {
    // No humans to reason about this tick - caller must not run inference on this (S4.7).
    inputs.shapes = {{static_cast<int64_t>(last_candidates_.size()), 0, 13}};
    inputs.data = {{}};
    return inputs;
  }

  std::vector<float> flat;
  flat.reserve(last_candidates_.size() * last_num_humans_ * 13);

  for (size_t i = 0; i < last_candidates_.size(); ++i) {
    const WorldState propagated = propagateCandidate(state, last_candidates_[i], config_.time_step_s);

    std::vector<PropagatedHuman> reward_humans;
    reward_humans.reserve(propagated.humans.size());
    for (const auto & h : propagated.humans) {
      reward_humans.push_back({h.x, h.y, config_.human_radius_m});
    }
    last_rewards_[i] = computeImmediateReward(propagated.robot, reward_humans, config_.time_step_s);

    for (const auto & h : propagated.humans) {
      const auto row = rotate(propagated.robot, h.x, h.y, h.vx, h.vy, config_.human_radius_m);
      for (double v : row) {
        flat.push_back(static_cast<float>(v));
      }
    }
  }

  inputs.shapes = {
    {static_cast<int64_t>(last_candidates_.size()), static_cast<int64_t>(last_num_humans_), 13}};
  inputs.data = {std::move(flat)};
  return inputs;
}

Velocity2D SarlAdapter::selectAction(const TensorBundle & model_outputs, const WorldState & state)
{
  if (last_num_humans_ == 0 || model_outputs.data.empty() || model_outputs.data[0].empty()) {
    return {0.0, 0.0};  // safe stopgap - S4.7
  }

  const double discount = std::pow(config_.sarl_gamma, config_.time_step_s * state.robot.v_pref);

  double best_value = -std::numeric_limits<double>::infinity();
  CandidateAction best = last_candidates_[0];
  for (size_t i = 0; i < last_candidates_.size(); ++i) {
    const double network_value = static_cast<double>(model_outputs.data[0][i]);
    const double value = last_rewards_[i] + discount * network_value;
    if (value > best_value) {
      best_value = value;
      best = last_candidates_[i];
    }
  }
  return {best.vx, best.vy};
}

}  // namespace crowd_nav_policy_adapters
