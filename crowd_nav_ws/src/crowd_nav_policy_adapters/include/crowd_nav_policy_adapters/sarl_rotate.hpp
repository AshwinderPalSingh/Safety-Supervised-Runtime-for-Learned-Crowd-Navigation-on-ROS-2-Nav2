#ifndef CROWD_NAV_POLICY_ADAPTERS__SARL_ROTATE_HPP_
#define CROWD_NAV_POLICY_ADAPTERS__SARL_ROTATE_HPP_

#include <array>

#include "crowd_nav_observation/world_state.hpp"

namespace crowd_nav_policy_adapters
{

// Reproduces cadrl.py's rotate() exactly (holonomic branch only - the only kinematics mode the
// pinned checkpoint was trained with, /S4.7), verified against the
// actual reference source and a checked-in fixture (test/fixtures/sarl_rotate_reference.txt,
// generated the same way as Phase 5's - S4.1.2). This is the PRODUCTION rotation used by
// SarlAdapter; Phase 5's own round-trip test used a test-only copy of this same formula before
// this function existed, per that phase's own note that production rotation was deferred to
// Phase 8.
//
// Column order of the output: [dg, v_pref, theta, radius, vx, vy, px1, py1, vx1, vy1, radius1,
// da, radius_sum] - matches ValueNetwork's expected per-human row exactly (self_state_dim=6 +
// human_state_dim=7 = input_dim=13, verified against the checkpoint's own policy.config).
std::array<double, 13> rotate(
  const crowd_nav_observation::RobotSelfState & self,
  double human_px, double human_py, double human_vx, double human_vy, double human_radius);

}  // namespace crowd_nav_policy_adapters

#endif  // CROWD_NAV_POLICY_ADAPTERS__SARL_ROTATE_HPP_
