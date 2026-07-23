#ifndef CROWD_NAV_POLICY_ADAPTERS__CANDIDATE_ACTION_SPACE_HPP_
#define CROWD_NAV_POLICY_ADAPTERS__CANDIDATE_ACTION_SPACE_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace crowd_nav_policy_adapters
{

// Single source of truth for candidate-batch shape, loaded from policy_adapter.yaml
// (IMPLEMENTATION_PLAN.md S4.3.1) - consumed by both DummyAdapter (Phase 6) and SarlAdapter
// (Phase 8), and by generate_dummy_model.py at model-generation time. Values verified against
// the pinned checkpoint's own crowd_nav/configs/policy.config [action_space] and env.config
// [env] sections, not memorized - kinematics is always holonomic (the only mode the pinned
// checkpoint was trained with, S1.9), so it isn't a config knob here.
struct CandidateActionSpaceConfig
{
  int speed_samples = 5;
  int rotation_samples = 16;
  double time_step_s = 0.25;
  size_t max_humans = 5;
  double human_radius_m = 0.3;

  // Candidate count = speed_samples * rotation_samples + 1 (the +1 is the stop action) -
  // computed here, never hand-typed at a call site.
  size_t candidateCount() const
  {
    return static_cast<size_t>(speed_samples) * static_cast<size_t>(rotation_samples) + 1;
  }

  // Per-candidate raw observation-builder feature width (crowd_nav_observation
  // ::ObservationBuilder, Phase 5): 9 self-state fields + 5 fields per human slot.
  size_t featureDim() const {return 9 + 5 * max_humans;}
};

CandidateActionSpaceConfig loadCandidateActionSpaceConfig(const std::string & yaml_path);

// A single candidate's holonomic velocity (matches build_action_space()'s ActionXY).
struct CandidateAction
{
  double vx = 0.0;
  double vy = 0.0;
};

// Reproduces cadrl.py's build_action_space() exactly (S4.3.1): candidate 0 is always the stop
// action; the remaining speed_samples * rotation_samples candidates are the (rotation, speed)
// cartesian product (rotation outer, speed inner - matches itertools.product(rotations,
// speeds)'s iteration order), with speeds spaced *exponentially*, not uniformly:
//   speeds[i] = (exp((i+1)/speed_samples) - 1) / (e - 1) * v_pref,  i in [0, speed_samples)
//   rotations[j] = j * (2*pi / rotation_samples),                   j in [0, rotation_samples)
std::vector<CandidateAction> buildCandidateActionSpace(
  const CandidateActionSpaceConfig & config, double v_pref);

}  // namespace crowd_nav_policy_adapters

#endif  // CROWD_NAV_POLICY_ADAPTERS__CANDIDATE_ACTION_SPACE_HPP_
