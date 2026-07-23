#include "crowd_nav_policy_adapters/sarl_rotate.hpp"

#include <cmath>

namespace crowd_nav_policy_adapters
{

using crowd_nav_observation::RobotSelfState;

std::array<double, 13> rotate(
  const RobotSelfState & self,
  double human_px, double human_py, double human_vx, double human_vy, double human_radius)
{
  const double dx = self.gx - self.px;
  const double dy = self.gy - self.py;
  const double rot = std::atan2(dy, dx);
  const double cr = std::cos(rot);
  const double sr = std::sin(rot);

  const double dg = std::hypot(dx, dy);
  const double v_pref = self.v_pref;
  const double vx = self.vx * cr + self.vy * sr;
  const double vy = self.vy * cr - self.vx * sr;
  const double radius = self.radius;
  const double theta = 0.0;  // holonomic branch only (S1.9)
  const double vx1 = human_vx * cr + human_vy * sr;
  const double vy1 = human_vy * cr - human_vx * sr;
  const double px1 = (human_px - self.px) * cr + (human_py - self.py) * sr;
  const double py1 = (human_py - self.py) * cr - (human_px - self.px) * sr;
  const double radius1 = human_radius;
  const double radius_sum = radius + radius1;
  const double da = std::hypot(self.px - human_px, self.py - human_py);

  return {dg, v_pref, theta, radius, vx, vy, px1, py1, vx1, vy1, radius1, da, radius_sum};
}

}  // namespace crowd_nav_policy_adapters
