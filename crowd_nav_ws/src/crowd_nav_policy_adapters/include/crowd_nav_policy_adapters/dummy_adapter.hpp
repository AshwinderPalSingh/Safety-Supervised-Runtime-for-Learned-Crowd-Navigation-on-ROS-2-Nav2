#ifndef CROWD_NAV_POLICY_ADAPTERS__DUMMY_ADAPTER_HPP_
#define CROWD_NAV_POLICY_ADAPTERS__DUMMY_ADAPTER_HPP_

#include <vector>

#include "crowd_nav_observation/observation_builder.hpp"
#include "crowd_nav_policy_adapters/candidate_action_space.hpp"
#include "crowd_nav_policy_adapters/policy_adapter.hpp"

namespace crowd_nav_policy_adapters
{

// IMPLEMENTATION_PLAN.md Phase 6: the throwaway *policy* (not the throwaway plumbing - the
// ONNX vendor package, shape validation, candidate generation/propagation, and observation
// builder it exercises are all real and reused unchanged by Phase 8's SarlAdapter). Proves the
// PolicyAdapter/ONNX/controller plumbing end to end with zero SARL-specific risk mixed in, and
// stays in the tree permanently afterward as a zero-checkpoint smoke test for the inference
// path (S3 Phase 8).
//
// buildInputs() must be called before selectAction() for a given decision - it stashes the
// candidate list selectAction() needs to map the network's per-row output back to a command.
// This mirrors how the Phase 7 controller plugin will always call them in that order.
class DummyAdapter : public PolicyAdapter
{
public:
  explicit DummyAdapter(const CandidateActionSpaceConfig & config);

  ShapeSpec expectedShape() const override;
  TensorBundle buildInputs(const WorldState & state) override;
  // Ignores model_outputs' actual values (the dummy network's output is meaningless by
  // design) and picks whichever stashed candidate's heading is closest to the goal direction -
  // a simple, deterministic decoder that only needs to prove data moved correctly end to end,
  // not a principled action-selection rule (that's SarlAdapter's job, Phase 8).
  Velocity2D selectAction(const TensorBundle & model_outputs, const WorldState & state) override;
  std::string name() const override {return "dummy";}

  static constexpr const char * kInputName = "candidates";
  static constexpr const char * kOutputName = "value";

private:
  CandidateActionSpaceConfig config_;
  crowd_nav_observation::ObservationBuilder builder_;
  std::vector<CandidateAction> last_candidates_;
};

}  // namespace crowd_nav_policy_adapters

#endif  // CROWD_NAV_POLICY_ADAPTERS__DUMMY_ADAPTER_HPP_
