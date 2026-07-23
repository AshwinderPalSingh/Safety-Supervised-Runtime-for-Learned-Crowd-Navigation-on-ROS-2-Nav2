#ifndef CROWD_NAV_POLICY_ADAPTERS__SARL_ADAPTER_HPP_
#define CROWD_NAV_POLICY_ADAPTERS__SARL_ADAPTER_HPP_

#include <vector>

#include "crowd_nav_policy_adapters/candidate_action_space.hpp"
#include "crowd_nav_policy_adapters/policy_adapter.hpp"

namespace crowd_nav_policy_adapters
{

// Real SARL candidate-action search (IMPLEMENTATION_PLAN.md S3 Phase 8 / S4.7), reproducing
// multi_human_rl.py's predict() exactly: propagate each candidate one step, rotate each human
// row, batch through the real exported value network, add the immediate reward and
// gamma-discounted network value, argmax.
//
// Deliberately UNPADDED - not built on crowd_nav_observation::ObservationBuilder, which pads to
// a fixed max_humans for DummyAdapter's static-shape model. Padding is wrong here: the
// network's masked-softmax attention only excludes a human row when its raw attention score is
// exactly 0.0, a property no padding convention guarantees by construction (S4.7). The batch
// tensor's num_humans axis is whatever WorldState actually has, matching the reference's own
// variable-length calling convention (verified: batching all candidates together in one call
// is mathematically identical to the reference's one-candidate-at-a-time convention, since
// ValueNetwork.forward()'s attention/mean pooling has no cross-batch-element coupling).
//
// Zero-humans case: buildInputs() returns an empty TensorBundle (no rows) - the caller
// (CrowdNavController) is expected to skip inference entirely on an empty batch and treat this
// tick as "nothing to decide," per S4.7's documented stopgap (a fuller response is Phase 9's
// OOD detector's job, not this adapter's).
class SarlAdapter : public PolicyAdapter
{
public:
  explicit SarlAdapter(const CandidateActionSpaceConfig & config);

  ShapeSpec expectedShape() const override;
  TensorBundle buildInputs(const WorldState & state) override;
  Velocity2D selectAction(const TensorBundle & model_outputs, const WorldState & state) override;
  std::string name() const override {return "sarl";}

  static constexpr const char * kInputName = "rotated_batch";
  static constexpr const char * kOutputName = "value";

private:
  CandidateActionSpaceConfig config_;
  std::vector<CandidateAction> last_candidates_;
  std::vector<double> last_rewards_;
  size_t last_num_humans_ = 0;
};

}  // namespace crowd_nav_policy_adapters

#endif  // CROWD_NAV_POLICY_ADAPTERS__SARL_ADAPTER_HPP_
