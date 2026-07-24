#include "crowd_nav_policy_adapters/sarl_adapter.hpp"

#include <cmath>
#include <limits>

#include "crowd_nav_observation/observation_builder.hpp"
#include "crowd_nav_policy_adapters/candidate_propagation.hpp"
#include "crowd_nav_policy_adapters/sarl_reward.hpp"
#include "crowd_nav_policy_adapters/sarl_rotate.hpp"

namespace crowd_nav_policy_adapters
{

using crowd_nav_observation::WorldState;
using crowd_nav_perception::HumanObservation;

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
  last_rewards_.assign(last_candidates_.size(), 0.0);

  // Dummy-injection-on-empty (IMPLEMENTATION_PLAN.md S4.8.1): replicates the reference's own
  // JointState convention of never letting predict() see a genuinely empty human list - masked-
  // softmax attention over zero rows is a divide-by-zero in the pooling denominator, an
  // architecturally undefined input, not merely an untrained one. Placed along the robot's
  // current heading, far away (ObservationBuilder::kDummyDistanceM - this project's own already-
  // established padding-distance convention, reused for consistency rather than the reference's
  // arbitrary 11.9m/21.9m), stationary. dummy_radius_ below (0.0) is the one place this method
  // deliberately does not use config_.human_radius_m - matching the reference's own
  // ObservableState(dumX, dumY, 0, 0, 0) exactly, since this row has no physical human behind
  // it. Injection is structurally guaranteed to be the sole occupant of the batch whenever it
  // fires (only triggers when state.humans started empty), so there is no real/dummy mixing to
  // reason about.
  const bool dummy_injected = state.humans.empty();
  WorldState working_state = state;
  if (dummy_injected) {
    HumanObservation dummy;
    dummy.x = state.robot.px +
      crowd_nav_observation::ObservationBuilder::kDummyDistanceM * std::cos(state.robot.theta);
    dummy.y = state.robot.py +
      crowd_nav_observation::ObservationBuilder::kDummyDistanceM * std::sin(state.robot.theta);
    dummy.vx = 0.0;
    dummy.vy = 0.0;
    working_state.humans.push_back(dummy);
  }
  last_num_humans_ = working_state.humans.size();
  const double human_radius = dummy_injected ? 0.0 : config_.human_radius_m;

  TensorBundle inputs;
  inputs.names = {kInputName};

  std::vector<float> flat;
  flat.reserve(last_candidates_.size() * last_num_humans_ * 13);

  for (size_t i = 0; i < last_candidates_.size(); ++i) {
    const WorldState propagated =
      propagateCandidate(working_state, last_candidates_[i], config_.time_step_s);

    std::vector<PropagatedHuman> reward_humans;
    reward_humans.reserve(propagated.humans.size());
    for (const auto & h : propagated.humans) {
      reward_humans.push_back({h.x, h.y, human_radius});
    }
    last_rewards_[i] = computeImmediateReward(propagated.robot, reward_humans, config_.time_step_s);

    for (const auto & h : propagated.humans) {
      const auto row = rotate(propagated.robot, h.x, h.y, h.vx, h.vy, human_radius);
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
  // last_num_humans_ == 0 is no longer reachable here (buildInputs() always injects a dummy
  // human when perception is empty, S4.8.1) - this guard is purely against a malformed/failed
  // inference output (a genuinely possible ONNX Runtime failure mode independent of that fix),
  // not the zero-humans case this comment used to describe.
  if (model_outputs.data.empty() || model_outputs.data[0].empty()) {
    return {0.0, 0.0};
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
