#include "crowd_nav_policy_adapters/sarl_reward.hpp"

#include <cmath>
#include <limits>

namespace crowd_nav_policy_adapters
{

using crowd_nav_observation::RobotSelfState;

double computeImmediateReward(
  const RobotSelfState & nav,
  const std::vector<PropagatedHuman> & humans,
  double time_step_s)
{
  double dmin = std::numeric_limits<double>::infinity();
  bool collision = false;
  for (const auto & human : humans) {
    const double dist =
      std::hypot(nav.px - human.px, nav.py - human.py) - nav.radius - human.radius;
    if (dist < 0.0) {
      collision = true;
      break;
    }
    if (dist < dmin) {
      dmin = dist;
    }
  }

  const bool reaching_goal = std::hypot(nav.px - nav.gx, nav.py - nav.gy) < nav.radius;

  if (collision) {
    return -0.25;
  } else if (reaching_goal) {
    return 1.0;
  } else if (dmin < 0.2) {
    return (dmin - 0.2) * 0.5 * time_step_s;
  }
  return 0.0;
}

}  // namespace crowd_nav_policy_adapters
