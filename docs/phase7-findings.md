# Phase 7 findings log

Per Phase 7 ("Controller plugin"), rescoped in v1.13 before any
implementation started (watchdog boundary structured for Phase 9 from day one, failover
*transition* added to the done-bar alongside the trigger, pluginlib conventions checked against
real Nav2 source first - see §4.6 and the v1.13 changelog entry). This is the first phase
producing a real `nav2_core` plugin, and it found more real integration gaps than any prior
phase - recorded here in full, including two that would have shipped silently without the live
Gazebo verification the user asked for.

**Status: DONE, done-bar met.** All items verified directly, not assumed:
- A `policy_raw` config running `DummyAdapter` drives the robot end-to-end in Nav2, in Gazebo
  (`NavigateToPose` to `(2.0, 0.0)` succeeded).
- Artificially stalling inference (`FollowPath.debug_inject_decision_delay_s`, live-settable)
  triggers failover to the embedded `nav2_mppi_controller::MPPIController` essentially
  immediately - confirmed via edge-triggered source-switch logging, not inferred.
- **The failover transition itself verified quantitatively**, not just the trigger: the raw
  controller output showed a genuine single-tick discontinuity at the switch (linear.x jumped
  -0.120 m/s, angular.z jumped +0.677 rad/s in one 0.05 s tick); the post-`nav2_velocity_smoother`
  stream - what the robot actually executes - bounded the same transition to exactly the
  configured `max_accel`/`max_decel` per-tick limits (0.075 m/s and 0.15 rad/s per tick).
- `ControllerDecisionCore`'s watchdog/hold-last-action/race-avoidance logic: 4/4 unit tests
  green, including a genuine bounded-wait proof (the core returns in <150 ms against a fake
  that sleeps 200 ms) and a race-avoidance proof (a second decision is never started while one
  is still outstanding).
- Full-workspace rebuild and re-run of every prior phase's test suite (30 tests total across 4
  packages) confirms no regression.

## What was built

New package `crowd_nav_controller`:
- `controller_decision_core.hpp/.cpp`: `ControllerDecisionCore`, pure C++ (no ROS/costmap/tf
  needed to test it). Owns hold-last-action scheduling and the watchdog, implemented as a
  genuine bounded wait (`std::async` + `future::wait_for`), not after-the-fact measurement of a
  synchronous call. A decision that times out leaves its background thread running; a second
  decision is never started against the same `PolicyAdapter` instance while one is still
  outstanding (avoids a real data race against `DummyAdapter`'s mutable per-decision state) -
  the core stays in fallback until the outstanding one resolves, polled non-blocking each tick.
- `crowd_nav_controller.hpp/.cpp`: `CrowdNavController`, the actual `nav2_core::Controller`.
  Builds a `WorldState` from the current pose/velocity/plan each tick, runs it through
  `DummyAdapter` inside `ControllerDecisionCore`'s watchdog boundary, converts the adapter's
  holonomic output to a diff-drive `Twist`, and delegates to a genuinely embedded
  `nav2_mppi_controller::MPPIController` (loaded via pluginlib) on fallback. Registers a dynamic
  parameter callback so `debug_inject_decision_delay_s` and the velocity-limit params are
  actually live-settable, not read once at configure time.

## Real bugs found via live verification, not caught by any unit test

Six real gaps surfaced during the live Gazebo/Nav2 verification - none of them would have been
caught by `ControllerDecisionCore`'s otherwise-thorough unit tests, which is itself the
argument for why this phase's done-bar requires the live check rather than stopping at gtest.

1. **Static library linked into a pluginlib `.so` - the exact class of pluginlib failure flagged
   before starting.** This workspace's CMake default is a static archive (no `-fPIC`);
   `crowd_nav_controller` built and linked fine as a static `.a`, and gtest linked against it
   fine too - but `controller_server` failed at runtime with "Could not find library
   corresponding to plugin... Make sure that the library 'crowd_nav_controller' actually
   exists," because `pluginlib::ClassLoader` loads plugins via `dlopen()`, which needs a real
   `.so`. Fixed by adding `SHARED` to `add_library()` in `crowd_nav_controller` and its three
   library dependencies (`crowd_nav_perception`, `crowd_nav_observation`,
   `crowd_nav_policy_adapters` - the last of which then hit a real `-fPIC` relocation error
   against `yaml-cpp`, confirming the root cause). No unit test would ever catch this - it's
   purely a `dlopen()`-time failure.
2. **`$(find-pkg-share ...)` doesn't resolve inside a raw YAML params file.** Launch
   substitution syntax only resolves inside a `.launch.py`; `nav2_params.yaml` is loaded
   directly as a params file with no such preprocessing (confirmed against how this project's
   own launch files pass it to `Node(parameters=[...])`). Fixed by leaving `onnx_model_path`
   empty in YAML and having `CrowdNavController` resolve a portable default via
   `ament_index_cpp::get_package_share_directory` when empty - the same pattern already used
   for `policy_adapter_config_path`.
3. **`DummyAdapter`'s "closest heading" tie-break picked the stop action whenever the goal was
   roughly ahead of the robot - the common case, not an edge case.** The stop action's
   `(vx,vy)=(0,0)` gives `atan2(0,0)==0`, a degenerate "heading" that tied with (and, by the
   original strict-less-than comparison, beat) any genuinely-facing-the-goal candidate. Found
   because Phase 6's own unit test happened to use a goal specifically NOT near heading 0
   (documented in that test's own comment as deliberately avoiding this exact case) - real
   end-to-end driving doesn't get to avoid it. The robot sat motionless, `cmd_vel` all zeros,
   until `controller_server`'s progress checker aborted the goal. Fixed by excluding the stop
   action from the search entirely (deciding when to stop is the goal checker's job, not this
   placeholder heuristic's); added `DummyAdapterEndToEnd.DoesNotStopWhenGoalIsRoughlyAheadOfTheRobot`
   as a permanent regression test.

   **This is the most instructive finding of the phase, and the strongest argument yet for
   keeping `DummyAdapter` in the tree permanently (§3 Phase 6/8), not just a Phase 6
   throwaway**: a dummy whose heuristic silently fails in the common case is exactly the
   defect that would have made Phase 8 unreadable. Swap in `SarlAdapter`, watch the robot
   behave oddly, and there would have been no way to tell whether the search reimplementation
   was wrong or the harness underneath it was - precisely the risk Phase 6 was built to
   eliminate (§4.6/S3 Phase 6: "guarantees a Phase 8 failure means the SARL search
   reimplementation is wrong and nothing else"). The eliminator itself had the bug. It's fixed
   now, and `DummyAdapterEndToEnd.DoesNotStopWhenGoalIsRoughlyAheadOfTheRobot` pins a real
   failure this project actually hit, not a hypothetical one - exactly the kind of regression
   coverage that makes the permanent-fixture argument concrete rather than aspirational.
4. **Runtime parameters were read once at `configure()` and never refreshed - `ros2 param set`
   updated the parameter server but had no effect on the running node.** This silently defeated
   the whole point of `debug_inject_decision_delay_s` being a live-settable diagnostic knob.
   Fixed by adding a dynamic parameter callback (`onSetParameters`, registered in `activate()`,
   reset in `deactivate()`), pattern verified against `nav2_rotation_shim_controller`'s own
   `dynamicParametersCallback` rather than invented from scratch.
5. **The "open-arena" world (`empty.sdf`) has no geometry at all beyond a ground plane** - no
   walls, no features. SLAM (and AMCL) need something to localize against; pairing SLAM with
   `empty.sdf` produced a persistent "Received map message is malformed" / "Robot is out of
   bounds of the costmap" failure that is a semantic mismatch, not a bug - there is nothing to
   map. Full end-to-end Nav2 verification (controller + planner + localization) used
   `amcl.launch.py`'s existing `depot_scaled.sdf`/`depot_scaled.yaml` combination instead, the
   same proven configuration prior phases used. `slam.launch.py`'s `world_file` was hardcoded;
   made it an overridable `DeclareLaunchArgument` (matching `amcl.launch.py`'s `map` arg
   pattern) as a small, permanent, generically useful fix along the way.
6. **Pluginlib registration verified end-to-end, not just checked against the docs**: the
   installed plugin XML resolves through `ament_index`'s `nav2_core__pluginlib__plugin`
   resource exactly as expected; `controller_server`'s own log confirms
   `Created controller : FollowPath of type crowd_nav_controller::CrowdNavController` and
   `CrowdNavController 'FollowPath': fallback controller 'nav2_mppi_controller::MPPIController'
   created` / `Activated MPPI Controller: FollowPath`.

## Fallback delegation - verified against real Nav2 source before writing any code

Per the process note going in: checked `nav2_rotation_shim_controller`'s actual source (fetched
at the pinned Humble commit, not just its header) rather than the docs, given the `::` vs `/`
pluginlib naming inconsistency that already bit this project once in Phase 2
(`docs/phase2-findings.md`). Confirmed the exact mechanism `CrowdNavController` now uses:
`pluginlib::ClassLoader<nav2_core::Controller> fallback_loader_{"nav2_core",
"nav2_core::Controller"}`, `fallback_controller_ = fallback_loader_.createUniqueInstance(name)`
where `name` is a string parameter (`FollowPath.fallback_controller_plugin`), and
`fallback_controller_->configure(parent, name, tf, costmap_ros)` called with the **same**
`name` the wrapper itself was given - meaning the embedded MPPI reads its parameters from the
same `FollowPath.*` namespace it always has, and its ~30 existing tuning parameters
(`time_steps`, `model_dt`, the critics, etc.) needed zero changes.

## Watchdog boundary - structured for Phase 9 now, not reworked later

Per the pinned requirement: `computeVelocityCommands()`'s `run_policy_decision` closure already
wraps the full "produce a command we're willing to send" sequence - build inputs, run
inference, select action - with an explicit comment marking where Phase 9's forward-sim/costmap
check will be inserted (a no-op today, since the supervisor doesn't exist yet). The watchdog
(`ControllerDecisionCore`) times the whole closure, not just an inference call, so adding the
supervisor check inside it later changes the closure's body, not the timing boundary itself.

## Failover transition - the actual captured evidence

Injected a live stall (`debug_inject_decision_delay_s` set to 0.2 s against a 0.03 s watchdog
window) while the robot was actively driving a hand-sent straight-line path via
`controller_server`'s own `/follow_path` action (bypassing `bt_navigator`/`planner_server` for a
controlled, sustained driving window). Captured both `/cmd_vel` (`controller_server`'s raw
output) and `/diff_drive_base_controller/cmd_vel_unstamped` (post-`nav2_velocity_smoother`) at
message-arrival resolution.

- **Trigger**: `CrowdNavController 'FollowPath': command source switched to FALLBACK` logged
  within the same decision cycle the stall was injected in (and `switched to policy` logged
  again once a later goal's first decision succeeded quickly) - added as edge-triggered
  (logs only on change) `RCLCPP_WARN` telemetry, genuinely useful beyond this test too.
- **Transition, raw stream**: last policy-sourced sample `t=1784789087.0264`
  `(linear=0.1208, angular=-0.7086)` -> first fallback-sourced sample `t=1784789087.1124`
  `(linear=0.0008, angular=-0.0319)` - a single-tick (0.086 s apart, close to one 0.05 s
  controller period) change of -0.120 m/s linear and +0.677 rad/s angular. Both exceed what the
  robot's configured limits allow it to execute instantaneously.
- **Transition, smoothed stream** (what the robot actually receives): the same window shows
  angular.z stepping `-0.708612 -> -0.558612 -> -0.408612 -> -0.258612` across three consecutive
  20 Hz ticks immediately after the switch - each step exactly `+0.15 rad/s`, matching
  `max_accel: 3.0` rad/s² × the 0.05 s smoothing period exactly, not approximately. Linear.x
  shows the equivalent `0.075 m/s`-per-tick bound. `nav2_velocity_smoother` - already configured
  in this project's own bringup, not built for this phase - is confirmed, by direct
  measurement, to actually absorb the failover discontinuity into a physically executable
  ramp, not just assumed to because it's "supposed to."

Raw capture data archived (not committed - ephemeral verification artifacts, matching Phase
3/4's own `capture_pedestrians.py`-style scripts): `/tmp/cmd_vel_raw.csv`,
`/tmp/cmd_vel_smoothed.csv` from this session.

## Design notes carried into Phase 8

- **§1.9 correction**: holonomic-to-diff-drive command conversion was originally scoped as
  `SarlAdapter`-specific. It isn't - `DummyAdapter` also emits a holonomic `Velocity2D` (Phase
  6's action space is holonomic regardless of which adapter is plugged in), so the conversion
  correctly lives in `CrowdNavController::toTwistStamped()` (adapter-agnostic), not duplicated
  inside any one adapter. The conversion itself is a simple proportional-heading controller,
  explicitly *not* tuned for SARL's specific action distribution - worth revisiting in Phase 8
  if it proves too crude once real SARL candidates are driving it.
- `RobotSelfState::radius`/`v_pref` (the network-facing constants, §4.3's policy_radius split)
  now come from `CandidateActionSpaceConfig::policy_radius_m`/`policy_v_pref_mps` - added to
  `crowd_nav_policy_adapters/config/policy_adapter.yaml` this phase (additive, doesn't change
  Phase 6's tested candidate-space/shape behavior) so Phase 8's `SarlAdapter` reads the exact
  same values rather than re-declaring them.
- No live `HumanStateSource` wired into `CrowdNavController` yet - `GroundTruthHumanSource`'s
  production constructor takes `rclcpp::Node::SharedPtr`, which
  `rclcpp_lifecycle::LifecycleNode` (what every `nav2_core` plugin actually receives) is not.
  `WorldState.humans` is empty for now (an already-tested, legitimate padding path from Phases
  5/6) - flagged for whichever phase first needs live perception inside a Nav2 plugin, most
  likely Phase 9's safety supervisor.
