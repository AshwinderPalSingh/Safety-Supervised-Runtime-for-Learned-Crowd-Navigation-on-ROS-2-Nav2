// Tests drive ControllerDecisionCore directly with fake decision callables - no live Nav2 node,
// costmap, tf buffer, or ONNX session needed (IMPLEMENTATION_PLAN.md S4.6). `now` timestamps
// are synthetic sim time (scheduling is against sim time, per this project's established
// discipline - see crowd_nav_perception's latency tests); the watchdog bound itself is REAL
// wall-clock time (std::async + wait_for), since that's what actually bounds a controller
// tick's execution - fakes that need to "stall" use a real std::this_thread::sleep_for.
#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include "gtest/gtest.h"

#include "crowd_nav_controller/controller_decision_core.hpp"

using crowd_nav_controller::ControllerDecisionCore;
using crowd_nav_controller::DecisionSource;
using crowd_nav_policy_adapters::Velocity2D;

namespace
{
rclcpp::Time simTime(double seconds)
{
  return rclcpp::Time(
    static_cast<int32_t>(seconds),
    static_cast<uint32_t>((seconds - std::floor(seconds)) * 1e9),
    RCL_ROS_TIME);
}

double wallSecondsSince(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
}  // namespace

TEST(ControllerDecisionCore, HoldsLastCommandBetweenDecisionPeriodsWithoutReinvoking)
{
  ControllerDecisionCore core(/*policy_decision_period_s=*/0.25, /*watchdog_window_s=*/0.05);

  std::atomic<int> call_count{0};
  auto fake = [&]() {
      const int n = ++call_count;
      return Velocity2D{static_cast<double>(n), 0.0};
    };

  const auto r1 = core.decide(simTime(0.0), fake);
  EXPECT_EQ(r1.source, DecisionSource::kPolicy);
  EXPECT_TRUE(r1.decision_attempted);
  EXPECT_DOUBLE_EQ(r1.command.vx, 1.0);
  EXPECT_EQ(call_count.load(), 1);

  // Before the decision period elapses: held, fake not re-invoked.
  const auto r2 = core.decide(simTime(0.10), fake);
  EXPECT_EQ(r2.source, DecisionSource::kPolicy);
  EXPECT_FALSE(r2.decision_attempted);
  EXPECT_DOUBLE_EQ(r2.command.vx, 1.0);
  EXPECT_EQ(call_count.load(), 1);

  // After the decision period elapses: a fresh decision is attempted.
  const auto r3 = core.decide(simTime(0.30), fake);
  EXPECT_EQ(r3.source, DecisionSource::kPolicy);
  EXPECT_TRUE(r3.decision_attempted);
  EXPECT_DOUBLE_EQ(r3.command.vx, 2.0);
  EXPECT_EQ(call_count.load(), 2);
}

TEST(ControllerDecisionCore, FallsBackWhenDecisionExceedsWatchdogWindowAndDoesNotBlockTheFullDelay)
{
  ControllerDecisionCore core(/*policy_decision_period_s=*/0.25, /*watchdog_window_s=*/0.05);

  auto slow_fake = []() {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      return Velocity2D{1.0, 0.0};
    };

  const auto start = std::chrono::steady_clock::now();
  const auto result = core.decide(simTime(0.0), slow_fake);
  const double elapsed = wallSecondsSince(start);

  EXPECT_EQ(result.source, DecisionSource::kFallback);
  EXPECT_TRUE(result.decision_attempted);
  // Proves this is a genuine bounded wait, not "call synchronously then notice it was slow
  // afterward" - decide() must return well before the fake's 200ms sleep completes.
  EXPECT_LT(elapsed, 0.15);
}

TEST(ControllerDecisionCore, DoesNotStartASecondDecisionWhileOneIsStillOutstanding)
{
  ControllerDecisionCore core(/*policy_decision_period_s=*/0.05, /*watchdog_window_s=*/0.05);

  std::atomic<int> call_count{0};
  auto slow_once_fake = [&]() {
      ++call_count;
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      return Velocity2D{1.0, 0.0};
    };

  // First tick: times out, leaves a background thread outstanding.
  const auto r1 = core.decide(simTime(0.0), slow_once_fake);
  EXPECT_EQ(r1.source, DecisionSource::kFallback);
  EXPECT_EQ(call_count.load(), 1);

  // Second tick, immediately after (sim time also advanced past the decision period, so a
  // fresh decision WOULD otherwise be due) - must NOT invoke the fake again while the first
  // one is still running, and must return promptly (just a non-blocking poll).
  const auto poll_start = std::chrono::steady_clock::now();
  const auto r2 = core.decide(simTime(0.10), slow_once_fake);
  const double poll_elapsed = wallSecondsSince(poll_start);

  EXPECT_EQ(r2.source, DecisionSource::kFallback);
  EXPECT_FALSE(r2.decision_attempted);
  EXPECT_EQ(call_count.load(), 1) << "a new decision must not start while one is outstanding";
  EXPECT_LT(poll_elapsed, 0.02) << "checking an outstanding decision must not block";
}

TEST(ControllerDecisionCore, RecoversAfterTheOutstandingDecisionResolves)
{
  ControllerDecisionCore core(/*policy_decision_period_s=*/0.05, /*watchdog_window_s=*/0.05);

  std::atomic<int> call_count{0};
  auto fake = [&]() {
      const int n = ++call_count;
      if (n == 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
      }
      return Velocity2D{static_cast<double>(n), 0.0};
    };

  const auto r1 = core.decide(simTime(0.0), fake);
  EXPECT_EQ(r1.source, DecisionSource::kFallback);

  // Wait past the first decision's real completion time.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // This tick reaps the stale result (discarded, not adopted) and, since sim time also shows
  // the period has elapsed, starts and completes a fresh, fast decision.
  const auto r2 = core.decide(simTime(0.20), fake);
  EXPECT_EQ(r2.source, DecisionSource::kPolicy);
  EXPECT_TRUE(r2.decision_attempted);
  EXPECT_EQ(call_count.load(), 2);
  EXPECT_DOUBLE_EQ(r2.command.vx, 2.0) << "must be the fresh decision, not the discarded stale one";
}
