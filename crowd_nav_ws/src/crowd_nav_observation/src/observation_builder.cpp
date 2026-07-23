#include "crowd_nav_observation/observation_builder.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace crowd_nav_observation
{

ObservationBuilder::ObservationBuilder(size_t max_humans, double human_radius)
: max_humans_(max_humans), human_radius_(human_radius)
{
}

std::vector<double> ObservationBuilder::build(const WorldState & state) const
{
  std::vector<double> out;
  out.reserve(9 + 5 * max_humans_);

  const auto & r = state.robot;
  out.insert(out.end(), {r.px, r.py, r.vx, r.vy, r.radius, r.gx, r.gy, r.v_pref, r.theta});

  // Closest-to-robot first: if there are more real humans than max_humans_, this makes
  // truncation drop the farthest ones, not an arbitrary subset - the closest humans are the
  // most safety-relevant ones to keep.
  std::vector<size_t> order(state.humans.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(
    order.begin(), order.end(),
    [&](size_t a, size_t b) {
      const double da = std::hypot(state.humans[a].x - r.px, state.humans[a].y - r.py);
      const double db = std::hypot(state.humans[b].x - r.px, state.humans[b].y - r.py);
      return da < db;
    });

  for (size_t i = 0; i < max_humans_; ++i) {
    if (i < order.size()) {
      const auto & h = state.humans[order[i]];
      out.insert(out.end(), {h.x, h.y, h.vx, h.vy, human_radius_});
    } else {
      // Padding (this project's own addition, see class comment): far away, stationary, zero
      // radius.
      out.insert(out.end(), {r.px + kDummyDistanceM, r.py, 0.0, 0.0, 0.0});
    }
  }
  return out;
}

}  // namespace crowd_nav_observation
