#ifndef CROWD_NAV_OBSERVATION__OBSERVATION_BUILDER_HPP_
#define CROWD_NAV_OBSERVATION__OBSERVATION_BUILDER_HPP_

#include <cstddef>
#include <vector>

#include "crowd_nav_observation/world_state.hpp"

namespace crowd_nav_observation
{

// Produces SARL's raw (pre-rotation) flat vector: 9 self-state fields followed by
// max_humans() blocks of 5 human fields each, ordered closest-to-robot first. Field order and
// semantics verified against tkkim-robot/Gazebo-CrowdNav (IMPLEMENTATION_PLAN.md S4.1.1) - the
// rotation into egocentric features (SarlAdapter's job, Phase 8) is NOT performed here.
//
// Padding to a fixed max_humans is THIS PROJECT'S OWN ADDITION for a static-shape ONNX export -
// the reference training code has no fixed-size padding at all (variable-length batches
// instead). Documented as an approximation, not attributed to upstream: the dummy/padding rows
// are out-of-training-distribution inputs, since the network was never exposed to phantom
// humans during training.
class ObservationBuilder
{
public:
  // human_radius: SARL's per-human schema slot (radius1) is a config constant here, not a
  // sensed quantity - HumanObservation (crowd_nav_perception) has no radius field because
  // perception never measures it, and crowd_nav_pedestrians' pedestrian_sim_node applies one
  // uniform ped_radius to every pedestrian (not per-individual) anyway. Same category as
  // RobotSelfState::v_pref: matched to the training/simulation distribution, not observed.
  ObservationBuilder(size_t max_humans, double human_radius);

  std::vector<double> build(const WorldState & state) const;

  size_t maxHumans() const {return max_humans_;}

  // Padding convention: place the dummy human this far from the robot, stationary, zero
  // radius, so its contribution to attention-weighted pooling is heavily down-weighted -
  // not a guaranteed-correct convention (see class comment), just this project's choice.
  static constexpr double kDummyDistanceM = 100.0;

private:
  size_t max_humans_;
  double human_radius_;
};

}  // namespace crowd_nav_observation

#endif  // CROWD_NAV_OBSERVATION__OBSERVATION_BUILDER_HPP_
