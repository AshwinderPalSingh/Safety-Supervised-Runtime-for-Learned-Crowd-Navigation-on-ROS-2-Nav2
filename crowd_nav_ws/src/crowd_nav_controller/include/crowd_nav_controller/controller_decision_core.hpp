#ifndef CROWD_NAV_CONTROLLER__CONTROLLER_DECISION_CORE_HPP_
#define CROWD_NAV_CONTROLLER__CONTROLLER_DECISION_CORE_HPP_

#include <functional>
#include <future>

#include "rclcpp/time.hpp"

#include "crowd_nav_policy_adapters/policy_adapter.hpp"

namespace crowd_nav_controller
{

enum class DecisionSource
{
  kPolicy,   // use `command` below
  kFallback  // caller must invoke the embedded fallback nav2_core::Controller itself - this
             // core has no costmap/tf/pose access, so it can't compute that command
};

struct DecisionResult
{
  DecisionSource source = DecisionSource::kFallback;
  crowd_nav_policy_adapters::Velocity2D command;
  // true if a fresh policy decision was started (successful or not) this tick, false if this
  // tick either held the last command or is waiting on an already-outstanding decision. Purely
  // informational (logging/testing), doesn't affect `source`.
  bool decision_attempted = false;
};

// Pure C++ (no live ROS node/costmap/tf/ONNX session needed) - IMPLEMENTATION_PLAN.md S4.6.
// Owns exactly two responsibilities: (1) hold the last successful policy command across
// controller ticks between policy decision points (the 4 Hz decision / 20 Hz control-loop rate
// mismatch), and (2) the inference-latency watchdog, implemented as a genuine bounded wait
// (std::async + wait_for), not after-the-fact measurement - a synchronous call would block the
// whole control loop for as long as inference takes, regardless of any deadline.
//
// Race-avoidance (S4.6): a decision that times out leaves its background thread running to
// completion (C++ has no safe forced-thread-termination). If a NEW decision were started on
// the same PolicyAdapter instance while that old one is still touching its mutable per-decision
// state (e.g. DummyAdapter's stashed candidate list), that's a real data race. Fix: never start
// a new decision while a previous one is still outstanding - poll it (non-blocking) each tick,
// and stay in kFallback until it resolves. This is also the conservative-correct safety
// default on its own merits, independent of the race-avoidance rationale.
class ControllerDecisionCore
{
public:
  ControllerDecisionCore(double policy_decision_period_s, double watchdog_window_s);

  // Called once per controller tick. `run_policy_decision` is invoked on a background thread,
  // bounded by watchdog_window_s, ONLY when a fresh decision is due AND no previous decision is
  // still outstanding - otherwise it isn't touched at all this tick.
  DecisionResult decide(
    const rclcpp::Time & now,
    const std::function<crowd_nav_policy_adapters::Velocity2D()> & run_policy_decision);

  // Lets the watchdog window be live-reconfigured (ros2 param set) without a lifecycle
  // restart - takes effect on the next decide() call, not retroactively on one in flight.
  void setWatchdogWindow(double watchdog_window_s) {watchdog_window_s_ = watchdog_window_s;}

private:
  double policy_decision_period_s_;
  double watchdog_window_s_;

  bool has_decision_ = false;
  rclcpp::Time last_decision_time_;
  crowd_nav_policy_adapters::Velocity2D last_command_;

  bool has_pending_ = false;
  std::future<crowd_nav_policy_adapters::Velocity2D> pending_;
};

}  // namespace crowd_nav_controller

#endif  // CROWD_NAV_CONTROLLER__CONTROLLER_DECISION_CORE_HPP_
