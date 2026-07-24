#ifndef CROWD_NAV_PERCEPTION__TRACKED_HUMAN_SOURCE_HPP_
#define CROWD_NAV_PERCEPTION__TRACKED_HUMAN_SOURCE_HPP_

#include <stdexcept>
#include <string>
#include <vector>

#include "crowd_nav_perception/human_state_source.hpp"

namespace crowd_nav_perception
{

// Documented stub, intentionally thin per IMPLEMENTATION_PLAN.md S6 - a real tracker-fed
// HumanStateSource is out of scope for this project (see the brief), but the interface seam
// exists so a real implementation could drop in later without touching anything that consumes
// HumanStateSource.
class NotImplementedYet : public std::logic_error
{
public:
  explicit NotImplementedYet(const std::string & what)
  : std::logic_error(what) {}
};

class TrackedHumanSource : public HumanStateSource
{
public:
  explicit TrackedHumanSource(const std::string & tracker_topic)
  : tracker_topic_(tracker_topic) {}

  std::vector<HumanObservation> getHumans(const rclcpp::Time & /*query_time*/) override
  {
    throw NotImplementedYet(
      "TrackedHumanSource is a documented stub (IMPLEMENTATION_PLAN.md S4.1/S6) - "
      "a real tracker-fed human source is out of this project's scope. Configured "
      "tracker topic was '" + tracker_topic_ + "'.");
  }

private:
  std::string tracker_topic_;
};

}  // namespace crowd_nav_perception

#endif  // CROWD_NAV_PERCEPTION__TRACKED_HUMAN_SOURCE_HPP_
