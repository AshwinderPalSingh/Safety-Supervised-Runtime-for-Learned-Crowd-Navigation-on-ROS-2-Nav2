#ifndef CROWD_NAV_SAFETY_SUPERVISOR__INTERVENTION_CAUSE_HPP_
#define CROWD_NAV_SAFETY_SUPERVISOR__INTERVENTION_CAUSE_HPP_

#include <cstdint>
#include <string>

namespace crowd_nav_safety_supervisor
{

// IMPLEMENTATION_PLAN.md S4.4/S4.8.5 - the full trigger taxonomy for the "intervention rate
// broken down by cause" headline metric. Mirrors InterventionEvent.msg's uint8 constants
// exactly (see toMsgValue() below) - kept as a real C++ enum rather than passing the msg's raw
// uint8 around internally, so SafetySupervisor's own logic/tests aren't coupled to the
// generated message header.
// Values assigned explicitly (not left to declaration order) so they stay tied to
// InterventionEvent.msg's own CROWD_SIZE=0.../INFERENCE_TIMEOUT=7 constants by construction,
// not by two files happening to list things in the same order.
enum class InterventionCause : uint8_t
{
  kCrowdSize = 0,
  kProximity = 1,
  kRelativeSpeed = 2,
  kCommandLimit = 3,
  kLowPerceptionConfidence = 4,
  kCostmapCollision = 5,
  kKeepoutViolation = 6,
  kInferenceTimeout = 7,
};

inline uint8_t toMsgValue(InterventionCause cause)
{
  return static_cast<uint8_t>(cause);
}

inline std::string toString(InterventionCause cause)
{
  switch (cause) {
    case InterventionCause::kCrowdSize: return "CROWD_SIZE";
    case InterventionCause::kProximity: return "PROXIMITY";
    case InterventionCause::kRelativeSpeed: return "RELATIVE_SPEED";
    case InterventionCause::kCommandLimit: return "COMMAND_LIMIT";
    case InterventionCause::kLowPerceptionConfidence: return "LOW_PERCEPTION_CONFIDENCE";
    case InterventionCause::kCostmapCollision: return "COSTMAP_COLLISION";
    case InterventionCause::kKeepoutViolation: return "KEEPOUT_VIOLATION";
    case InterventionCause::kInferenceTimeout: return "INFERENCE_TIMEOUT";
  }
  return "UNKNOWN";
}

}  // namespace crowd_nav_safety_supervisor

#endif  // CROWD_NAV_SAFETY_SUPERVISOR__INTERVENTION_CAUSE_HPP_
