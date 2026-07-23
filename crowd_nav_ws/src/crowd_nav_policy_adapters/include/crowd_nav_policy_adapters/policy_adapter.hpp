#ifndef CROWD_NAV_POLICY_ADAPTERS__POLICY_ADAPTER_HPP_
#define CROWD_NAV_POLICY_ADAPTERS__POLICY_ADAPTER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "crowd_nav_observation/world_state.hpp"

namespace crowd_nav_policy_adapters
{

using crowd_nav_observation::WorldState;

// Declared input/output tensor names and shapes an adapter expects an ONNX session to have.
// -1 in a shape entry means "dynamic, matches any extent at this position" (still checked for
// dimension count) - see IMPLEMENTATION_PLAN.md S4.3.
struct ShapeSpec
{
  std::vector<std::string> input_names;
  std::vector<std::vector<int64_t>> input_shapes;
  std::vector<std::string> output_names;
  std::vector<std::vector<int64_t>> output_shapes;
};

// A named batch of flat float tensors, row-major, one entry per name/shape/data triple - the
// currency PolicyAdapter and the ONNX inference helper (onnx_inference.hpp) pass between them.
struct TensorBundle
{
  std::vector<std::string> names;
  std::vector<std::vector<int64_t>> shapes;
  std::vector<std::vector<float>> data;
};

// Raw holonomic velocity of a chosen candidate action, in the robot's own frame - matches
// build_action_space()'s ActionXY (S4.3.1). Conversion to a diff-drive (v, omega) command is
// downstream (S1.9): deliberately not this interface's concern, since a future non-holonomic
// or non-CrowdNav-family policy might not need it at all.
struct Velocity2D
{
  double vx = 0.0;
  double vy = 0.0;
};

// IMPLEMENTATION_PLAN.md S4.3. Config (adapter type, model path, schema version, action-space
// discretization, kinematics mode) lives entirely in YAML - no code changes to select or tune
// an adapter. buildInputs() must be called before selectAction() for a given decision - an
// adapter is free to stash whatever intermediate state (e.g. the candidate list) it needs to
// map the network's per-row output back to a command.
class PolicyAdapter
{
public:
  virtual ShapeSpec expectedShape() const = 0;

  // Builds whatever tensor batch this policy family's network needs from canonical world
  // state. For SARL/CrowdNav-family adapters this is a batch of one-step-propagated candidate
  // joint-states, one row per candidate action - NOT a single observation vector.
  virtual TensorBundle buildInputs(const WorldState & state) = 0;

  // Consumes the network's raw output(s) plus the same world state to produce a command. For
  // SARL this is where the argmax over candidate actions + immediate-reward term lives.
  virtual Velocity2D selectAction(const TensorBundle & model_outputs, const WorldState & state) = 0;

  virtual std::string name() const = 0;

  virtual ~PolicyAdapter() = default;
};

}  // namespace crowd_nav_policy_adapters

#endif  // CROWD_NAV_POLICY_ADAPTERS__POLICY_ADAPTER_HPP_
