# Phase 9 findings log

Per Phase 9 ("Safety supervisor"), rescoped in v1.17 before any
supervisor code was written (§4.8, three requirements from review: resolve the FOV filter
first; guarantee the supervisor and MPPI share the same costmap instance and footprint
treatment; bound the forward-sim cost by construction). The build order followed that
sequencing exactly - perception fix first, dummy-injection fix second, `crowd_nav_safety_
supervisor` third - not because it was convenient, but because review was explicit that OOD
thresholds tuned against a mismatched input pipeline are meaningless.

**Status: DONE, done-bar met.** All items verified directly, not assumed:
- The FOV/range filter and dummy-injection-on-empty fix both implemented and unit-tested
  **before any supervisor code existed**, per the explicit sequencing requirement.
- `crowd_nav_safety_supervisor` built as a pure C++ class (no live ROS node needed for its own
  logic), instantiated directly by `CrowdNavController` with the exact `costmap_ros_` pointer
  already shared with the embedded MPPI - "same costmap instance" is structural, not asserted.
- Forward-sim horizon fixed at 4 steps / 1.0s total, no adaptive refinement, reusing the
  candidate action space's own `time_step_s` rather than a second hand-typed default.
- 21 new unit tests (9 perception, 2 dummy-injection, 12 supervisor... see exact breakdown
  below) plus every prior phase's suite: 60 tests total across 5 packages, 0 failures.
- Live-verified in Gazebo: a clean baseline run (no pedestrians) reached its goal with **zero**
  intervention events - no false positives. In sessions with pedestrians present, three of the
  eight intervention causes were observed firing for real, non-engineered reasons - `CROWD_SIZE`,
  `PROXIMITY`, and a sustained `COSTMAP_COLLISION` episode (90+ consecutive ticks, ~20+ seconds,
  zero misses) - each logged with the correct cause and published on `/intervention_events`
  with the correct rejected/sent velocity pair.

## The FOV filter: a two-part decision, resolved first

Per review's explicit ask ("decide deliberately... the plan should state which and why," §4.8.1):

- **The FOV/range numbers**: this robot's real sensor geometry (±90° half-angle, 8m range - the
  same ~180°/8m figures already stated in the README's Known Limitations section), not the
  reference's D435I-specific 85.2°/12m. Neither choice matches the training distribution
  exactly (this robot's sensor genuinely differs from a D435I), so the tie-breaker was honesty
  about what this robot can actually perceive, not numerical proximity to a constant that was
  never this robot's number to begin with.
- **The dummy-injection-on-empty convention**: replicated regardless of the FOV numbers chosen,
  because it protects an architecturally-untested path in the network's own masked-softmax
  attention (a divide-by-zero in the pooling denominator over zero rows), not a training-
  distribution nicety.

**Implementation** (`crowd_nav_perception`): `DegradationParams` gained `fov_half_angle_rad`
(optional, off by default, matching every other field's convention). `GroundTruthHumanSource`
gained heading tracking (`setRobotPose(x, y, theta)`, `tf2::getYaw()` on `onRobotPose`'s
orientation) and an angular check in `degrade()`, placed *before* the noise/dropout draws so it
never perturbs the RNG consumption sequence. `CrowdNavController::configure()` now explicitly
sets both `fov_half_angle_rad` (default `M_PI_2`) and `max_range_m` (default `8.0`) on the
production `DegradationParams` - worth stating plainly: **before this phase, neither was
actually active in production**, even though `max_range_m` had existed as a capability since
Phase 5. Capability existing and capability active are different claims; this phase closed that
gap as a new `FollowPath.perception_fov_half_angle_rad` / `.perception_max_range_m` parameter
pair, not a hardcoded literal.

**Tests** (`test_ground_truth_human_source.cpp`, 9 total, 3 new): FOV exclusion behind the
robot, FOV rotating correctly with a non-zero heading (not just the `theta=0` case, which would
pass by accident even with a bug), and `numDegradedLastCall()` counting only genuine dropout
events, not FOV/range exclusions (`NumDegradedLastCallCountsOnlyDropoutNotFovExclusion`).

**A correction found while wiring `LOW_PERCEPTION_CONFIDENCE` to a real signal**: `degrade()`
discarded a dropped-out detection with no trace left anywhere in `WorldState` - the trigger as
originally scoped (§4.4) had no way to actually fire. Fixed with a new non-pure
`HumanStateSource::numDegradedLastCall()` (default `0`, no existing call site affected),
overridden by `GroundTruthHumanSource` via a snapshot-and-reset accumulator (`degrade()`
increments it on a real dropout; `getHumans()` snapshots-and-resets each call, so it always
reflects "since I was last asked"). This also corrected a v1.15 assumption: the zero-humans case
does **not** route through `LOW_PERCEPTION_CONFIDENCE` - a legitimately empty, correctly-
perceived scene isn't a degraded-confidence event, and treating it as one would fire the trigger
on every ordinary empty scene.

## Dummy-injection: `SarlAdapter` no longer returns an empty batch

`buildInputs()` now injects one synthetic placeholder human when `state.humans` is empty -
positioned along the robot's current heading at `ObservationBuilder::kDummyDistanceM` (100m,
this project's own existing padding-distance convention, reused for consistency rather than the
reference's arbitrary 11.9m/21.9m), stationary, and fed to the network with radius `0.0` (the
one place this method deliberately does not use `config_.human_radius_m` - matching the
reference's own `ObservableState(dumX, dumY, 0, 0, 0)` exactly, since this row has no physical
human behind it). Injection is structurally guaranteed to be the sole occupant of the batch
whenever it fires, so there is no real/dummy mixing to reason about. The old
`last_num_humans_ == 0` defensive branch in `selectAction()` is now dead code by construction
(removed) - the remaining `model_outputs.data.empty()` guard is a genuine defense against a
malformed/failed inference output, unrelated to this fix.

**Verified empirically, not assumed** (the specific risk §4.8.1 flagged: masked-softmax
attention over exactly one row is only degenerate if that row's raw attention score is exactly
`0.0`): `SarlAdapterDummyInjection.RealNetworkProducesFiniteNonDegenerateCommand` runs the real
exported `sarl_value_net.onnx` on the injected dummy and asserts every output value is finite,
and that `selectAction()`'s resulting command is finite. It does not degenerate in practice.

## `crowd_nav_safety_supervisor` - design and implementation

New package, `SafetySupervisor`: pure C++ (no live ROS node needed for its own logic, the same
"testable core, thin ROS-owning wrapper" split as `ControllerDecisionCore`/
`GroundTruthHumanSource`), two independently-testable methods:

- **`checkOodCriteria(state, candidate, num_degraded_this_tick)`** - the five §4.4 criteria
  (`CROWD_SIZE`, `PROXIMITY` at the training-derived 0.8m threshold, `RELATIVE_SPEED`,
  `COMMAND_LIMIT` against the candidate's raw holonomic speed, `LOW_PERCEPTION_CONFIDENCE` via
  the new dropout-count signal). `state` here is always the pre-dummy-injection `WorldState` -
  `CROWD_SIZE`/`PROXIMITY` can never be corrupted by `SarlAdapter`'s placeholder.
- **`checkForwardSim(state, candidate, costmap, keepout_mask)`** - forward-simulates the
  candidate's raw holonomic `(vx, vy)` as a constant-velocity world-frame proxy over a **fixed**
  4 steps at the candidate action space's own `time_step_s` (1.0s total, no adaptive
  refinement), checking each step's point cost against the shared costmap
  (`cost >= INSCRIBED_INFLATED_OBSTACLE && cost != NO_INFORMATION` - the exclusion matters
  because `NO_INFORMATION == 255 > 253` would otherwise misread "unmapped" as "lethal"). Using
  the raw holonomic velocity rather than `toTwistStamped()`'s diff-drive projection is a
  deliberate, argued-explicit approximation: the projection only ever *shrinks* effective
  forward progress, so this is a conservative over-estimate that can produce a false-positive
  stop but never a false-negative approval.

**Same costmap, same footprint - structural** (§4.8.2): `SafetySupervisor` is instantiated
directly in `CrowdNavController::configure()`, and `checkForwardSim()` is called with
`costmap_ros_->getCostmap()` fetched fresh each tick (not cached at construction, matching
Nav2's own per-tick convention) - the identical pointer already passed to
`fallback_controller_->configure()` two lines below. "Same footprint" resolves to no
independent footprint at all: this project's costmap uses a scalar `robot_radius`, and MPPI's
own `ObstaclesCritic` already runs `consider_footprint: false` against this same costmap, so a
plain point-cost lookup relying on the inflation layer is the primitive already shared with
MPPI, not a new one invented for the supervisor.

**`KEEPOUT_VIOLATION` vs. generic `COSTMAP_COLLISION`** (§4.8.3): a secondary, best-effort
lookup against `/keepout_filter_mask` (a plain `OccupancyGrid`, transient-local QoS to match
`nav2_map_server`'s own latched-map convention) labels which cause goes into the intervention
log; it is never consulted for the safety decision itself. **Found while testing, not
hypothetical**: `OccupancyGrid.info.resolution` is `float32` while `Costmap2D`'s own internal
resolution is `double` - at a world point sitting exactly on a cell boundary, that precision
difference shifted the mask lookup's cell index by one relative to the primary costmap's
(confirmed with a standalone debug harness: `Costmap2D::worldToMap()` and a
`Costmap2D`-from-`OccupancyGrid` constructed from the identical resolution/origin disagreed by
one cell at a point whose `(w-o)/res` was mathematically exact-integer). Fixed the *tests* by
offsetting the test costmap's origin by half a cell so test points are never boundary-exact;
documented in the source as an accepted characteristic of the real lookup, not a bug to chase -
it can only affect which cause label a genuinely-rejected command gets, never whether it's
rejected.

**Two-tier fallback, a thread-safety constraint** (§4.8.4): `ControllerDecisionCore`'s watchdog
already runs `run_policy_decision` on a background thread; if the supervisor's rejection called
into `fallback_controller_` (the embedded MPPI) from inside that closure, an orphaned thread
past the watchdog window could call into the same MPPI instance the main thread also calls on a
timeout - a real data race. So: watchdog timeout still defers to MPPI (main thread only,
unchanged from Phase 7); a supervisor rejection (checked inside `run_policy_decision`, on the
background thread) returns a direct `{0.0, 0.0}` stop instead, never touching
`fallback_controller_`.

**`InterventionEvent`** (§4.8.7): a new message (`crowd_nav_safety_supervisor/msg/
InterventionEvent`, generated and consumed in the same package via
`rosidl_get_typesupport_target` - message generation and the C++ library that publishes it live
in one package, since nothing else produces this message), published on `/intervention_events`
via a `LifecyclePublisher` (gated `on_activate()`/`on_deactivate()` in lockstep with the
plugin's own lifecycle, the publisher-side analogue of the `create_subscription` incompatibility
Phase 8 found) every rejected tick, plus an edge-triggered `RCLCPP_WARN` log line per cause.
`INFERENCE_TIMEOUT` is published from `CrowdNavController` directly (main thread, alongside the
existing source-switch log), not from `SafetySupervisor` - it's the one cause that never reaches
`run_policy_decision` at all.

**Tests** (`test_safety_supervisor.cpp`, 12 total): each of the five OOD criteria triggering
individually plus an all-clear case; forward-sim safe/collision/`NO_INFORMATION`-excluded/
out-of-bounds-excluded cases, plus `KEEPOUT_VIOLATION` vs. generic `COSTMAP_COLLISION` labeling
with and without a matching secondary mask.

## `CrowdNavController` wiring

`run_policy_decision`'s lambda now runs `checkOodCriteria()` (cheap, no costmap) then, only if
that passed, `checkForwardSim()` (fetching the live costmap pointer and a mutex-guarded snapshot
of the latest keepout mask - the one genuine cross-thread access in this phase, since
`onKeepoutMask()` runs on the executor thread servicing subscriptions while
`run_policy_decision` runs on `ControllerDecisionCore`'s background thread; every other new
per-tick member is touched from exactly one thread context and left as a plain field). A
rejection publishes the `InterventionEvent` and returns a stop; a pass clears the edge-triggered
log flag and returns the policy's candidate unchanged. New `FollowPath.*` parameters:
`supervisor_max_train_humans` (5), `supervisor_min_train_distance_m` (0.8),
`supervisor_max_train_speed_mps` (1.5), `supervisor_forward_sim_steps` (4) -
`forward_sim_dt_s`/`max_commanded_speed_mps` deliberately have no separate parameter, reusing
`action_space_config_.time_step_s`/`max_linear_vel_mps_` instead.

## Live Gazebo verification

Launched the full stack (`amcl.launch.py` - Gazebo, AMCL, the Nav2 servers, `crowd_nav_zones`'
keepout-mask pipeline - plus `pedestrians.launch.py` for the ground-truth robot-pose bridge and
pedestrian simulation where noted below) against the `depot_scaled` world, `adapter_type: "sarl"`.

- **Clean baseline, no pedestrians**: `NavigateToPose` to (2.0, 0.0) `SUCCEEDED`. Zero
  `/intervention_events` messages, zero `safety supervisor rejected` log lines. No regression
  from Phase 7/8's own baseline, and no false-positive supervisor activity during entirely
  ordinary driving.
- **`CROWD_SIZE`, real and non-engineered**: with 6 pedestrians present (above the default
  `max_train_humans=5`), a genuine `CROWD_SIZE` rejection fired and logged correctly the moment
  enough pedestrians were simultaneously within this robot's FOV/range - not staged, a direct
  consequence of the configured threshold being lower than the scene's pedestrian count.
- **`PROXIMITY`, real and non-engineered**: with 3 pedestrians present, 5 distinct
  edge-triggered `PROXIMITY` episodes logged as pedestrians moved within 0.8m of the robot.
- **`COSTMAP_COLLISION`, real, non-engineered, and sustained**: the policy repeatedly proposed
  the same candidate heading toward a genuinely lethal costmap cell; the supervisor rejected it
  on **every single tick for over 90 consecutive ticks (~20+ seconds)**, correctly logging the
  cause once (edge-triggered) and publishing an `InterventionEvent` on every one of those ticks
  with `sent_vx=sent_vy=0.0`. Zero misses across that whole window - not a single tick let an
  unsafe candidate through.
- **Totals across the session**: 1,176 `/intervention_events` messages (138 `PROXIMITY`, 1,038
  `COSTMAP_COLLISION`), 8 distinct edge-triggered `COSTMAP_COLLISION` episodes, 5 distinct
  `PROXIMITY` episodes, 1 `CROWD_SIZE` episode.
- **`AddZone`/`RemoveZone` plumbing confirmed functional**: both services succeeded, and
  `KeepoutFilter: New filter mask arrived... Updating old filter mask` confirmed the mask
  propagated into the live costmap each time. **Not independently isolated this phase**: the
  specific zone placements attempted (sized/positioned ad hoc during this session, not against
  a pre-surveyed map) either boxed in the robot's own position (which the *global* planner
  correctly refused to route through - the mechanism worked, but at a layer upstream of
  `FollowPath`/the supervisor) or left enough open room for Nav2's own planner to route around
  before the local controller/supervisor ever needed to reject anything. The genuinely
  non-engineered `COSTMAP_COLLISION` episode above already exercises the exact same forward-sim/
  costmap-rejection code path a corridor-blocking keep-out zone would (`KeepoutFilter` writes
  into the identical costmap as `LETHAL_OBSTACLE`, §4.8.3) - so the mechanism is verified, even
  though a specific engineered keep-out-zone trigger wasn't cleanly isolated live this session.
  A deliberately narrow, pre-measured corridor zone (matching Phase 3's own successful
  methodology exactly) is the natural way to close this specific gap and is a good Phase 10
  scenario-suite candidate rather than a repeat of this ad hoc attempt.
- **Not live-exercised this phase**: `RELATIVE_SPEED`, `COMMAND_LIMIT`, `LOW_PERCEPTION_
  CONFIDENCE`, `INFERENCE_TIMEOUT` did not fire during this session (no human moved fast enough,
  no candidate exceeded the physical speed limit, perception degradation was left at its
  oracle-passthrough default, and inference never missed the watchdog window). All four are
  covered by the unit-test suite above; none were observed live. Session environment also grew
  DDS network-write warnings partway through (`ddsi_udp_conn_write ... failed`) - transient,
  did not affect node liveness or the results captured, but is recorded here per this project's
  own "document what actually happened" discipline rather than a cleaned-up narrative.

Full-workspace rebuild plus every prior phase's test suite re-run clean (60 tests total across
5 packages, no regression).

## Design notes carried into Phase 10+

- **What this detector does not catch, stated explicitly** (§4.8.6): the five OOD criteria
  characterize world-state novelty (too many humans, too close, too fast, a suspicious command,
  a degraded tick) - none of them can detect an input-pipeline mismatch, where the observation
  was already wrong before any threshold looked at it. Phase 8's FOV-filter finding is exactly
  that class of bug, and resolving it (this phase) does not make the detector newly capable of
  catching a *different* pipeline bug of the same shape in the future - that needs differential
  testing against a reference, not a runtime threshold.
- A pre-measured, corridor-blocking keep-out zone test (Phase 3's own methodology) would give a
  cleaner, more direct live demonstration of `KEEPOUT_VIOLATION` specifically than this
  session's ad hoc placements managed - worth doing as part of Phase 10's scenario suite, where
  scenario geometry gets designed deliberately rather than improvised live.
- `RELATIVE_SPEED`, `COMMAND_LIMIT`, `LOW_PERCEPTION_CONFIDENCE`, and `INFERENCE_TIMEOUT` are
  unit-tested but were not observed firing live this phase - Phase 10's perception-noise sweep
  (dropout enabled) is the natural place `LOW_PERCEPTION_CONFIDENCE` gets its first live
  exercise; `RELATIVE_SPEED`/`COMMAND_LIMIT` would need a deliberately fast-moving pedestrian or
  an out-of-range candidate to observe live, neither of which arose naturally in this session.

## Addendum: reachability audit for the remaining causes, done before Phase 10

Per review: `LOW_PERCEPTION_CONFIDENCE` was a trigger cause that was defined in §4.4, enumerated
in the taxonomy, wired to a plausible-looking code path - and structurally could not fire, until
this phase's own fix. That's a class of bug where the design document and the implementation
both look correct in isolation; the only way it surfaces is asking "can this actually fire?" for
each enumerated cause rather than assuming enumeration implies reachability. Checked the same
question for the three remaining causes not yet exercised live, **before** Phase 10 starts
producing a results table that could hide the same defect as a silent zero:

- **`COMMAND_LIMIT` - inert under current defaults, confirmed by reading the actual formula, not
  assumed.** `buildCandidateActionSpace()`'s speed sampling (`candidate_action_space.cpp`) is
  `speeds[i] = (exp((i+1)/speed_samples) - 1) / (e - 1) * v_pref` - at `i = speed_samples - 1`
  (the fastest sampled candidate), this reduces to exactly `v_pref` (`policy_v_pref_mps`, 1.0
  m/s). `SafetySupervisorConfig::max_commanded_speed_mps` defaults to `max_linear_vel_mps_`,
  which is *also* 1.0 m/s - and independently grounded, not a copy-paste coincidence:
  `crowd_nav_control/config/diff_drive_controller.yaml`'s `linear.x.max_velocity` is genuinely
  1.0 m/s, this robot's real configured hardware limit. So the candidate generator's own speed
  ceiling and the physical-limit threshold are both real, correctly-sourced numbers that happen
  to be numerically equal - and since the check is a strict `>`, the maximum possible candidate
  speed can never exceed a threshold it's exactly equal to. Not a code bug (unlike
  `LOW_PERCEPTION_CONFIDENCE` - the check itself is correct and `SafetySupervisorOod.
  CommandLimitTriggersWhenCandidateTooFast` proves the logic works when handed an out-of-range
  candidate directly), but a real "silent zero" risk for Phase 10's results table: if
  `policy_v_pref_mps` and this robot's physical speed limit stay equal, `COMMAND_LIMIT` will
  report zero interventions for a structural reason that has nothing to do with how well the
  policy behaves. Not fixed by lowering the threshold artificially to force a trigger - that
  would be gaming a test rather than fixing a real gap - but flagged explicitly here and in
  Phase 10's plan (§4.9) so a zero in that column isn't misread as "the policy never asked for
  something unsafe."
- **`RELATIVE_SPEED` - reachable in principle, dormant under the default pedestrian scenario,
  now fixable.** `pedestrian_sim_node.py` explicitly clamps every pedestrian's speed to its own
  `max_speed` parameter (`if speed > self.max_speed: scale = self.max_speed / speed`), default
  1.0 m/s - below `SafetySupervisorConfig::max_train_speed_mps`'s 1.5 m/s default (deliberately
  set with margin above SARL's ~1 m/s training speed, §4.4). Unlike `COMMAND_LIMIT`, this one
  *was* a real, fixable gap: `max_speed` was already a full ROS parameter on the node but not
  exposed as a `pedestrians.launch.py` launch argument the way `seed`/`num_pedestrians`/`mode`
  already are - fixed by adding it to the launch file's own argument surface, so a Phase 10
  scenario can deliberately set it above 1.5 m/s and get a real, live `RELATIVE_SPEED` trigger
  rather than a permanently-dormant one.
- **`INFERENCE_TIMEOUT` - mechanism proven reachable (Phase 7, via the `debug_inject_decision_
  delay_s` diagnostic knob), not yet observed under real, non-injected latency.** Phase 0's own
  measurement put ONNX CPU inference at sub-millisecond to low-single-digit-milliseconds: the
  30ms watchdog window (`watchdog_window_s`) is a comfortable margin under light load, and the
  supervisor's own forward-sim adds only four trivial cost lookups per tick on top of that. This
  is a genuinely different case from the other two: there's no code or config reason it can't
  fire, and Phase 10's full matrix run (real system load - Gazebo, the harness, and however many
  scenarios run concurrently or back-to-back) is exactly the condition that could make it fire
  naturally for the first time. If it stays at zero even under that load, that is itself a real,
  reportable finding (the watchdog margin holds under realistic conditions), not a gap to force -
  worth explicitly checking for and stating either way in Phase 10's results, rather than
  silently assuming reachability from the Phase 7 debug-knob demonstration alone.
