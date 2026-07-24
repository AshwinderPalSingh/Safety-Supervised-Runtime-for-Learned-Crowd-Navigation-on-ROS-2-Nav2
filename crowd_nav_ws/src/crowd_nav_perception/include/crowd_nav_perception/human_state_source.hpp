#ifndef CROWD_NAV_PERCEPTION__HUMAN_STATE_SOURCE_HPP_
#define CROWD_NAV_PERCEPTION__HUMAN_STATE_SOURCE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/time.hpp"

namespace crowd_nav_perception
{

// IMPLEMENTATION_PLAN.md S4.1. cov_xy is always 0.0 today - only today's synthetic noise model
// (isotropic Gaussian) populates the diagonal. The field exists now, unfilled, because widening
// it later would touch every PolicyAdapter and the observation builder; a real tracker filling
// cov_xy in later shouldn't require a schema change here.
struct HumanObservation
{
  uint32_t id = 0;
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double cov_xx = 0.0;
  double cov_yy = 0.0;
  double cov_xy = 0.0;
  rclcpp::Time stamp;
};

class HumanStateSource
{
public:
  virtual std::vector<HumanObservation> getHumans(const rclcpp::Time & query_time) = 0;
  // How many raw detections this source's own degradation model dropped out (not: FOV/range-
  // excluded - a human genuinely outside a sensor's coverage isn't a degraded-confidence event,
  // see IMPLEMENTATION_PLAN.md S4.8.5) since the last getHumans() call. Non-pure, defaults to 0,
  // so a source with no degradation concept (e.g. TrackedHumanSource) needs no override, and no
  // existing call site is affected by this method existing.
  virtual uint32_t numDegradedLastCall() const {return 0;}
  // The robot's current pose in the SAME frame getHumans() must return human positions in -
  // i.e. whatever frame WorldState.robot is populated from (map frame, in this project's own
  // buildWorldState()). Non-pure, empty default: a real sensor-based source (TrackedHumanSource)
  // already reports positions relative to the robot's own localized frame and needs no
  // conversion. GroundTruthHumanSource overrides this - it's the one source that reads a
  // privileged Gazebo world-frame ground truth (a "cheat" channel that doesn't exist for a real
  // sensor), and world frame is NOT the same frame as WorldState.robot's map-frame pose; see
  // docs/audit.md S1.3 for the bug this fixes and why no existing test could have caught it.
  virtual void setRobotMapPose(double /*x*/, double /*y*/) {}
  virtual ~HumanStateSource() = default;
};

}  // namespace crowd_nav_perception

#endif  // CROWD_NAV_PERCEPTION__HUMAN_STATE_SOURCE_HPP_
