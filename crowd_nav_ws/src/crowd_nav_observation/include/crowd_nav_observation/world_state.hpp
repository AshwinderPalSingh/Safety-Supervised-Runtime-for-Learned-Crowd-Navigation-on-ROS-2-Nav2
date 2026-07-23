#ifndef CROWD_NAV_OBSERVATION__WORLD_STATE_HPP_
#define CROWD_NAV_OBSERVATION__WORLD_STATE_HPP_

#include <vector>

#include "crowd_nav_perception/human_state_source.hpp"

namespace crowd_nav_observation
{

// Raw field layout verified against tkkim-robot/Gazebo-CrowdNav's FullState
// (crowd_sim/envs/utils/state.py) - IMPLEMENTATION_PLAN.md S4.1.1. v_pref is a config parameter
// matching the training distribution, not measured from the real robot (same category as
// policy_radius, S4.3/S7). theta is the robot's actual current heading.
struct RobotSelfState
{
  double px = 0.0;
  double py = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double radius = 0.0;
  double gx = 0.0;
  double gy = 0.0;
  double v_pref = 0.0;
  double theta = 0.0;
};

// Canonical world state the observation builder consumes. Humans come from
// HumanStateSource::getHumans() (crowd_nav_perception) - already degraded/delayed/dropped by
// the time they reach here; this struct doesn't know or care which concrete source produced
// them.
struct WorldState
{
  RobotSelfState robot;
  std::vector<crowd_nav_perception::HumanObservation> humans;
};

}  // namespace crowd_nav_observation

#endif  // CROWD_NAV_OBSERVATION__WORLD_STATE_HPP_
