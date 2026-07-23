#include "crowd_nav_policy_adapters/dummy_adapter.hpp"

#include <cmath>
#include <limits>

#include "crowd_nav_policy_adapters/candidate_propagation.hpp"

namespace crowd_nav_policy_adapters
{

namespace
{
double wrapToPi(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}
}  // namespace

DummyAdapter::DummyAdapter(const CandidateActionSpaceConfig & config)
: config_(config), builder_(config.max_humans, config.human_radius_m)
{
}

ShapeSpec DummyAdapter::expectedShape() const
{
  const int64_t n = static_cast<int64_t>(config_.candidateCount());
  const int64_t f = static_cast<int64_t>(config_.featureDim());
  ShapeSpec spec;
  spec.input_names = {kInputName};
  spec.input_shapes = {{n, f}};
  spec.output_names = {kOutputName};
  spec.output_shapes = {{n, 1}};
  return spec;
}

TensorBundle DummyAdapter::buildInputs(const WorldState & state)
{
  last_candidates_ = buildCandidateActionSpace(config_, state.robot.v_pref);

  std::vector<float> flat;
  flat.reserve(last_candidates_.size() * config_.featureDim());
  for (const auto & candidate : last_candidates_) {
    const WorldState propagated = propagateCandidate(state, candidate, config_.time_step_s);
    const std::vector<double> row = builder_.build(propagated);
    for (double v : row) {
      flat.push_back(static_cast<float>(v));
    }
  }

  TensorBundle inputs;
  inputs.names = {kInputName};
  inputs.shapes = {
    {static_cast<int64_t>(last_candidates_.size()), static_cast<int64_t>(config_.featureDim())}};
  inputs.data = {std::move(flat)};
  return inputs;
}

Velocity2D DummyAdapter::selectAction(const TensorBundle & model_outputs, const WorldState & state)
{
  (void)model_outputs;  // deliberately ignored - the dummy network's output is meaningless.

  const double desired_heading =
    std::atan2(state.robot.gy - state.robot.py, state.robot.gx - state.robot.px);

  double best_diff = std::numeric_limits<double>::infinity();
  CandidateAction best{0.0, 0.0};
  for (const auto & candidate : last_candidates_) {
    const double heading = std::atan2(candidate.vy, candidate.vx);
    const double diff = std::fabs(wrapToPi(heading - desired_heading));
    if (diff < best_diff) {
      best_diff = diff;
      best = candidate;
    }
  }
  return {best.vx, best.vy};
}

}  // namespace crowd_nav_policy_adapters
