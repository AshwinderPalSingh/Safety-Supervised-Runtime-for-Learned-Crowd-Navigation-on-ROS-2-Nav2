#include "crowd_nav_controller/controller_decision_core.hpp"

#include <chrono>

namespace crowd_nav_controller
{

using crowd_nav_policy_adapters::Velocity2D;

ControllerDecisionCore::ControllerDecisionCore(
  double policy_decision_period_s, double watchdog_window_s)
: policy_decision_period_s_(policy_decision_period_s), watchdog_window_s_(watchdog_window_s)
{
}

DecisionResult ControllerDecisionCore::decide(
  const rclcpp::Time & now,
  const std::function<Velocity2D()> & run_policy_decision)
{
  // Reap an outstanding decision if it has finished - discard a late result rather than
  // adopting it: it missed its watchdog window, so it's not trusted for the decision it was
  // meant to serve (see class comment). Clearing has_pending_ here, not adopting it as
  // last_command_, is what lets a fresh decision be attempted again on this or a later tick.
  if (has_pending_) {
    if (pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      pending_.get();  // discard - see above
      has_pending_ = false;
    } else {
      // Still outstanding - do not touch the adapter again this tick (race-avoidance, class
      // comment). Held last command isn't safe to keep asserting here either (we don't know
      // the policy is still healthy), so this stays in fallback until we know more.
      DecisionResult result;
      result.source = DecisionSource::kFallback;
      result.decision_attempted = false;
      return result;
    }
  }

  const bool decision_due =
    !has_decision_ || (now - last_decision_time_).seconds() >= policy_decision_period_s_;

  if (!decision_due) {
    DecisionResult result;
    result.source = DecisionSource::kPolicy;
    result.command = last_command_;
    result.decision_attempted = false;
    return result;
  }

  pending_ = std::async(std::launch::async, run_policy_decision);
  has_pending_ = true;

  const auto status = pending_.wait_for(
    std::chrono::duration<double>(watchdog_window_s_));

  DecisionResult result;
  result.decision_attempted = true;
  if (status == std::future_status::ready) {
    last_command_ = pending_.get();
    has_pending_ = false;
    has_decision_ = true;
    last_decision_time_ = now;
    result.source = DecisionSource::kPolicy;
    result.command = last_command_;
  } else {
    // Timed out - stays pending (has_pending_ remains true) to be reaped on a later tick.
    result.source = DecisionSource::kFallback;
  }
  return result;
}

}  // namespace crowd_nav_controller
