# Safety-Supervised Runtime for Learned Crowd Navigation on ROS 2 Nav2
## Implementation Plan v1.20 (2026-07-21, updated 2026-07-24) — Phase 10 closed

This document is the living plan for the project, updated as each phase lands rather than
frozen at the start. Phases 0–10 are done as of this revision (§3 has current status per
phase); everything from Phase 11 onward is still plan, not implementation.

**v1.1 changes** (post-review, same day): inserted a synthetic-adapter phase before SARL to
decouple runtime-plumbing risk from SARL-search-correctness risk (§3, new Phase 6); confirmed
via real gz-sim issue tracker that SDF `<scale>` on collision meshes is a documented rough
edge, not a safe assumption (§1.1); widened `HumanObservation`'s covariance field instead of
an isotropic scalar (§4.1); derived the OOD proximity threshold from CrowdNav's actual
`env.config` (`discomfort_dist=0.2`, radii `0.3`) instead of bare collision radius, and added
the `policy_radius`/`robot_radius` split this surfaced (§4.3, §4.4); added a nightly
Gazebo smoke-test CI tier (§6, §3 Phase 11).

**v1.2 changes** (post-review, same day): confirmed `vita-epfl/CrowdNav` ships no trained
weights at all — training from scratch would have been a real, unbudgeted multi-hour phase —
but found a fork with a matching-config pretrained SARL checkpoint, so Phase 0/8 now target
loading and validating that checkpoint, with from-scratch training demoted to a documented
fallback (§1.8, §3 Phase 0/8, §5, §7).

**v1.3 changes** (Phase 0 execution, 2026-07-21/22): full findings in
`docs/phase0-findings.md`, committed incrementally. Headline items that change this plan:
the checkpoint was trained with **holonomic kinematics, not unicycle** as §7 had assumed —
`SarlAdapter` needs an explicit holonomic→unicycle command conversion for our diff-drive
robot, not just accel clamping (§1.4, §1.9 new). World-scale risk (§1.1) is **downgraded**:
aisle widths measured directly from the SDF (~2.58 m row spacing, ~13× our robot's width,
confirming the mismatch with real numbers), RTF measured at ~1.0, and the depot's collision
layer turns out to be pure primitives (no mesh) — the gz-sim mesh-scale bug class doesn't
apply to it at all, so rescaling is deterministic arithmetic, not a real risk. Found a new,
real risk not in the original list: headless rendering hangs on this machine without
Xvfb/EGL, which will hit Phase 1's LiDAR sensor too (§1.10 new). ONNX Runtime version bumped
1.17.3 → 1.20.1 after the vendor package's own plumbing test caught a real IR-version
incompatibility (§1.3). `gz_ros2_control` needs `sudo apt install` and this machine has no
passwordless sudo — blocked, needs you (§3 Phase 0).

**v1.4 changes (Phase 0 closed, 2026-07-22)**: tkkim-robot's `rl_model.pth` failed validation
(0.21 vs 0.99 reported); root-caused via three differential test runs to that specific file,
not the harness (§1.8). **Final decision: `il_model.pth`** from the same repo (0.96/0.02/0.02),
chosen over LeeKeyu's working RL checkpoint (0.72/0.18/0.10) because an 18%-collision-rate
base policy undermines this project's core "is the supervisor catching genuine OOD behavior"
evaluation question — full reasoning and provenance in §1.8 and `docs/phase0-findings.md`.
Added **Phase 12** (second policy integration, HEIGHT) as the explicit reusability proof, not
a replacement. Corrected a mid-investigation mis-attribution about
`CrowdNav_Prediction_AttnGraph`'s test logs, recorded rather than silently fixed (§1.8).

**v1.5 changes (Phase 1 done, 2026-07-22)**: `crowd_nav_description`/`crowd_nav_control`/
`crowd_nav_gazebo` built, and verified end to end in Gazebo — URDF valid, robot spawns,
hardware interface + controllers activate, `/scan` publishes at spec, robot drives under a
velocity command (odometry-confirmed). Hit and fixed several real environment issues (XML
comment syntax, this workspace's space-containing path breaking two different launch-file
substitutions, a `pkill -f` self-match footgun) — full trail in `docs/phase1-findings.md`.
Root-caused (not just noted) a `gpu_lidar` rear-hemisphere self-detection artifact via a
decisive test: rotating the sensor's yaw 180° moved the artifact with it, proving it's
sensor-frame-relative, confirming a gz-sim rendering limitation rather than a modeling bug.
Left open at this revision, resolved in v1.6 below.

**v1.6 changes (2026-07-22)**: LiDAR artifact resolved by masking at the sensor (`<min_angle>`/
`<max_angle>` narrowed to a clean ±90°), not filtering downstream — avoids a two-topics-diverge
failure mode where one consumer reads raw and another reads filtered. Verified fully clean
(0/180 self-hits, was 176/360). Real cost recorded, not hidden: the robot now runs an
effectively-180°, not 360°, LiDAR, compounding with the already-deliberate 8 m range cap —
documented as a stated limitation in a new `README.md`, not left as an implicit accident.
Confirmed exact gz-sim version (6.18.0) and searched upstream for a matching issue — none
found exactly, closest is the still-open `gazebosim/gz-sim#2743` (general `gpu_lidar` accuracy
degradation). Phase 2's plan updated to treat AMCL tuning and costmap raytracing/decay as
first-class tasks given the reduced FOV, not afterthoughts discovered later in Phase 10.

**v1.7 changes (2026-07-22)**: Phases 2 and 3 closed (their own done-bars fully met, not
partially) — see `docs/phase2-findings.md` and `docs/phase3-findings.md` for the full trails;
§3's Phase 2/3 entries below carry the summary. Phase 4 rescoped before any implementation
started, per the reasoning below — plan changed first, code follows, not the other way around.

**§1.5 correction**: Phase 3 found this section's original "no surprises" claim was wrong on
two counts, not zero — see the rewritten §1.5 below and `docs/phase3-findings.md` for detail.
Worth stating plainly: the resulting global-costmap fix was **a plan bug, not an
implementation detail** — §3's Phase 3 text specified "stock KeepoutFilter in the local
costmap" as if that were sufficient, and it produced exactly the failure that specification
predicts (global planner keeps handing MPPI a path through the zone, MPPI can't execute it,
Nav2 spins in recovery forever). A bug caught by implementation and fixed in the plan is the
right direction for bugs to travel; recorded here rather than only in the phase findings doc
so the plan document itself carries the correction, not just the implementation.

**Phase 4 rescoped** (before implementation, per the above): HuNav dependency **dropped
entirely** — §1.2 rewritten below. Pedestrian state moves to a single ROS node (ORCA or
social-force based), with `reactive`/`non_reactive` as **config flags on that one
implementation**, not two separate mechanisms as originally planned. The robot's pose is
injected from Gazebo ground truth, not `/odom` (odometry drift would leak into pedestrian
reactive behavior for no good reason - the pedestrian model should react to where the robot
actually is). Seeded from the scenario seed, with **byte-identical trajectory reproduction
across same-seed runs added to the done-bar** - this is the entire justification for dropping
HuNav (determinism a HuNav-wrapped social-force engine doesn't cleanly offer), so it needs to
be proven now, not assumed and discovered broken at Phase 10's evaluation matrix. Also added
to the done-bar: the pedestrian node must step on **sim time, not wall clock** - with this
project's measured RTF sometimes below 1.0 under load (Phase 2's heavier physics-step
experiments), a wall-clock-driven simulator would silently desynchronize from Gazebo and
seeded reproducibility would be meaningless regardless of the RNG. New: a visual-only Gazebo
actor **mirror node**, launch-toggleable and off by default, for visualization/demo purposes
only - the authoritative pedestrian state is the ROS node's, never the mirror's. Done-bar
additions from this: pedestrians must run correctly headless with mirroring disabled, and the
RTF impact of running the mirror must be measured and recorded, not assumed negligible.

**§5 risk #6 replaced**: HuNav's non-reactive gap (the original risk) no longer applies - there
is no HuNav integration to have a gap in. New risk in its place: the mirror node drifting out
of sync with the authoritative pedestrian state (visualization lying about what the simulation
actually did) - see §5 below.

**Forward-note added to Phase 9** (§3): the same class of error as the §1.5 correction above -
a safety-critical component checking against state/a costmap that the thing it's supposed to
agree with doesn't share - is explicitly flagged as likely to recur there, not just noted once
and forgotten.

**v1.8 changes (2026-07-22)**: Phase 4 implemented and closed per the v1.7 rescope - full trail
in `docs/phase4-findings.md`. All four done-bar items verified with direct measurement, not
assumed from the design: determinism (byte-identical across two independent same-seed runs,
diffed message-for-message), sim-time stepping (confirmed by pausing the Gazebo world and
measuring `sim_time` barely advance during an 8s real-time sleep), and mirror-node correctness
plus its RTF cost (markers spawn and track correctly; RTF unaffected at 6-pedestrian scale).
Also found and fixed along the way: no Python gz-transport bindings are installed (ruled out
direct topic subscription without a bridge), and the right ground-truth-pose mechanism
(Gazebo's per-model `PosePublisher` system plugin) was confirmed by checking what's actually
installed and testing its output topic/type directly, rather than assuming a topic name or
message shape from memory - the same discipline as the LoadMap.srv finding in v1.7.

**v1.9 changes (2026-07-22)**: Phase 5 rescoped before implementation, per review, in three
concrete ways (§4.1.1, §4.1.2, §4.2 above have the full detail):
1. The observation builder's schema is now pinned to the actual reference implementation
   (`tkkim-robot/Gazebo-CrowdNav`, commit `9cad128d124f86bafe48d2cd11b5eee74bec77d9` - cloned
   and read directly, not assumed) rather than left as "a documented schema" with the document
   not yet written. Found along the way: the reference training code has **no fixed-size
   padding at all** - that's this project's own addition for a static-shape ONNX export, not
   something to reverse-engineer from upstream.
2. The degradation model's noise/dropout RNG now requires its own substream, independent of
   Phase 4's pedestrian-motion RNG - both seeded from the same scenario seed, but sharing one
   stream would let a noise-parameter sweep silently perturb pedestrian trajectories too,
   turning a controlled experiment into an uncontrolled one.
3. The latency ring buffer must be derived from (or keyed to) sim time, not a hardcoded tick
   count - otherwise the effective latency in seconds silently changes if the observation
   builder's own tick rate ever does.

Phase 5's done-bar gained a fourth item: an observation-builder round-trip test against the
reference implementation's own `CADRL.rotate()`, not just a hand-computed expected-value unit
test - the same class of check Phase 8's `SarlAdapter` verification already commits to, moved
one phase earlier since it's the highest-leverage place in this project for a silent bug (a
wrong human ordering or off-by-one padding produces a policy that behaves plausibly but
wrongly, with nothing downstream able to flag it).

**v1.10 changes (2026-07-23)**: Phase 5 implemented and closed per the v1.9 rescope - full trail
in `docs/phase5-findings.md`. All four done-bar items verified, not assumed: unit tests green;
**round-trip against the actual reference implementation passes** (re-cloned
`tkkim-robot/Gazebo-CrowdNav` at the pinned commit since the prior session's clone didn't
survive, re-read `cadrl.py`'s `rotate()` directly rather than trusting a memorized
transcription, generated a checked-in fixture from the real `CADRL.rotate()`, verified all 5
cases to `1e-6`); RNG-substream independence confirmed by test (same-tag streams identical,
different-tag streams diverge immediately); latency ring buffer verified with a test
specifically constructed to fail under a tick-counting implementation (uneven-interval
ingestion, correct freshest-sample-at-or-before-target-time selection). Found and fixed before
first build, not after: `ObservationBuilder` originally referenced a `radius` field on
`HumanObservation` that doesn't exist - perception never measures human radius, and
`pedestrian_sim_node` (Phase 4) already treats it as one uniform config constant, not a
per-individual sensed value, so the fix was a `human_radius` config parameter on
`ObservationBuilder` (same category as `RobotSelfState::v_pref`), not a schema change to
`HumanObservation`. Also switched the round-trip fixture from JSON to plain whitespace-delimited
text after `rosdep resolve` was unavailable to confirm `nlohmann-json-dev` actually resolves to
the installed `nlohmann-json3-dev` package - avoids shipping an unverified dependency.

**v1.11 changes (2026-07-23)**: Phase 6 rescoped before implementation, per review, in two
concrete ways (§4.3.1 has the full detail):
1. The candidate action space (speed/rotation sample counts, exponential speed spacing,
   one-step propagation) is now a single config (`policy_adapter.yaml`) that both the Python
   dummy-model generator and the C++ adapters (`DummyAdapter` now, `SarlAdapter` in Phase 8)
   read - the trivial ONNX model's declared batch dimension is computed from it, never
   hand-typed as `81` in two places that could silently drift apart. The underlying values
   (`speed_samples=5`, `rotation_samples=16`, `time_step=0.25`, exponential sampling) were
   re-verified by fetching the pinned checkpoint's actual `policy.config`/`env.config` and
   `cadrl.py` directly, not recalled from the §7 assumption that first stated them.
2. `DummyAdapter` is now explicitly scoped to stay in the tree permanently after Phase 8 lands,
   as a zero-checkpoint smoke test for the inference plumbing - not deleted once real SARL
   weights are available (§3 Phase 8).

Also recorded: `crowd_nav_onnxruntime_vendor`'s packaging risk (§5 risk #5) is already
retired as of Phase 0 - Phase 6 consumes an already-built, already-tested vendor package, not
fresh packaging risk. The documented fallback (`ros-industrial/epd_onnxruntime_vendor`) still
applies if a *future* ONNX Runtime version bump ever breaks the CMake wrapper.

**v1.12 changes (2026-07-23)**: Phase 6 implemented and closed per the v1.11 rescope - full
trail in `docs/phase6-findings.md`. New package `crowd_nav_policy_adapters`: `PolicyAdapter`
interface, `CandidateActionSpaceConfig`/`buildCandidateActionSpace()`/`propagateCandidate()`
(all verified against the pinned checkpoint's actual `policy.config`/`env.config`/`cadrl.py`,
re-fetched directly rather than trusting the plan's own §7 assumption), `validateSessionShapes()`
(proven to reject 5 different deliberately-wrong shapes, not just accept the correct one),
`runInference()`, and `DummyAdapter` wired to a config-shape-derived trivial ONNX model
(`generate_dummy_model.py`). 16/16 new tests pass; full-workspace rebuild plus every prior
phase's test suite re-run clean (25 tests total, no regression). One environment gap found and
recorded, not silently worked around: this machine's PyTorch (2.13) dynamo-based ONNX exporter
needs `onnxscript`, not installed by default here - documented in the findings so a fresh
machine doesn't hit the same failure unexplained.

**v1.13 changes (2026-07-23)**: Phase 7 rescoped before implementation, per review, in three
concrete ways (§4.6 has the full detail):
1. The watchdog is structured around the full "produce a command we're willing to send" path
   from day one - inference *and* a supervisor-check call site, the latter a no-op placeholder
   until Phase 9 exists - so Phase 9 slots into an existing boundary instead of forcing a
   rework of Phase 7's own timing code.
2. The failover done-bar now requires verifying the *transition*, not just the trigger:
   capturing both the raw controller output and the post-smoother command across an injected
   stall, confirming the handover never exceeds the robot's configured accel/decel limits -
   observed directly in Gazebo, not inferred later from Phase 10 results.
3. Checked pluginlib registration/lifecycle/parameter conventions against a real controller
   plugin's actual source (`nav2_rotation_shim_controller`, fetched at the pinned Humble
   commit) before writing any code, given the `::` vs `/` naming inconsistency that already
   bit this project once in Phase 2. Found real precedent for embedding a second
   `nav2_core::Controller` instance via pluginlib as a delegate/fallback - `CrowdNavController`
   uses this exact mechanism to embed real `nav2_mppi_controller::MPPIController` as its
   fallback, rather than reimplementing a worse one.

Also found before implementation, not after: this project's own `nav2_velocity_smoother` is
already configured and lifecycle-managed (`crowd_nav_bringup/config/nav2_params.yaml`), which
means the "velocity smoothing / acceleration clamping" duty this plan's §3 originally assigned
to the controller plugin itself is already handled downstream - `CrowdNavController`'s own
scope narrows to hold-last-action and the watchdog/failover decision, per the "don't
reimplement the stack" principle carried forward since Phase 2/3/9.

**v1.14 changes (2026-07-23)**: Phase 7 implemented and closed per the v1.13 rescope - full
trail in `docs/phase7-findings.md`. New package `crowd_nav_controller`: `ControllerDecisionCore`
(bounded-wait watchdog, hold-last-action, race-avoidance - 4/4 unit tests green) and
`CrowdNavController` (embeds `nav2_mppi_controller::MPPIController` via pluginlib exactly the
way `nav2_rotation_shim_controller` does, verified against that package's real source). Live
Gazebo verification met every done-bar item: end-to-end `NavigateToPose` succeeded with
`DummyAdapter`; a stall injection triggered failover within the same decision cycle (confirmed
via added source-switch logging); the failover *transition* was captured quantitatively - the
raw stream showed a real single-tick discontinuity (-0.120 m/s linear, +0.677 rad/s angular)
that the already-configured `nav2_velocity_smoother` bounded to exactly its configured
`max_accel`/`max_decel` per-tick limits in the stream the robot actually executes. Found and
fixed six real integration gaps along the way, none catchable by unit tests alone: a static-
library pluginlib load failure (fixed by building `crowd_nav_controller` and its three library
dependencies `SHARED`), a launch-substitution string silently not resolving inside a raw YAML
params file, a `DummyAdapter` heading tie-break bug that left the robot motionless whenever the
goal was roughly ahead (the common case), stale runtime parameters that `ros2 param set`
couldn't actually change (fixed with a dynamic parameter callback), a SLAM+`empty.sdf` pairing
that can't work (no geometry to map), and pluginlib registration reconfirmed end-to-end against
the real controller_server log. Full-workspace rebuild plus every prior phase's test suite
re-run clean (30 tests total, no regression).

**v1.15 changes (2026-07-23)**: Phase 8 rescoped before implementation, per review, in three
concrete ways plus one more found while pinning them (§4.7 has the full detail):
1. The ONNX export is verified standalone in Python - three-way check (wrapper vs. original
   checkpoint, ONNX Runtime vs. wrapper, batched-vs-sequential calling convention), all on real
   varied joint states, plus a load check against the actual pinned C++ ONNX Runtime 1.20.1 -
   **before any C++ adapter code is written**, so an export problem is caught in an hour, not
   discovered after a day spent on the search loop. All three checks passed (max abs diffs
   0.0 / 1.9e-7 / 2.4e-7).
2. The action-match done-bar now requires adversarial cases - scenarios where the top two
   candidates' values are genuinely close, mined from the reference implementation's own
   `action_values`, not hand-picked - since an obviously-best action matches even with a wrong
   discount factor or reward term.
3. A dedicated test asserts the radius value actually reaching the network is `policy_radius_m`
   (0.3), never `robot_collision_radius` (0.14) - Phase 2's undersized-radius bug is the
   precedent for how quietly a wrong radius can propagate.
4. **Found while verifying the network architecture, not anticipated going in**: reusing Phase
   5/6's `ObservationBuilder` padding for `SarlAdapter` would be a real bug, not a style
   mismatch - the network's masked-softmax attention excludes a human row only when its raw
   attention score is exactly `0.0`, a property no padding convention guarantees by
   construction. `SarlAdapter` gets its own unpadded, dynamic-`num_humans`-shaped input
   construction instead, matching the reference's true variable-length behavior. A zero-humans
   tick (possible via this project's own perception degradation model, never exercised by the
   reference) falls back to the stop action as a deliberate, documented stopgap - full handling
   belongs to Phase 9's `LOW_PERCEPTION_CONFIDENCE` OOD trigger.

**v1.16 changes (2026-07-23)**: Phase 8 implemented and closed per the v1.15 rescope - full
trail in `docs/phase8-findings.md`. All three pinned requirements verified with concrete
numbers (export bit-close before any C++ code; 10 adversarial action-match cases down to a
0.0018% top-two gap, one loosest-margin case's divergence independently confirmed as a genuine
cross-platform floating-point boundary rather than a logic bug; `policy_radius`/`robot_radius`
split asserted directly). `CrowdNavController` gained an `adapter_type` config switch
(`PolicyAdapter` held as a base-class pointer, not a concrete `DummyAdapter`) and live
perception wiring (`GroundTruthHumanSource`, fixing the `LifecycleNode` incompatibility Phase
7 found and deferred, via a templated constructor). Live-verified in Gazebo with real
pedestrians: goal reached, no fallback triggers, zero call-site changes needed to switch
adapters. **The most consequential finding of the phase**: the checkpoint's own source repo
silently FOV-filters and augments its human list inside `JointState.__init__` before SARL ever
sees it (an Intel D435I-simulating filter, this repo's own addition on top of upstream
CrowdNav) - found because a naive fixture comparison produced a false mismatch. Fixed the
fixture to capture what `predict()` actually evaluated; this project's own `SarlAdapter` does
not yet replicate an equivalent filter matching this robot's actual sensor, a real gap flagged
for resolution before Phase 10's evaluation numbers are treated as meaningful. Full-workspace
rebuild plus every prior phase's test suite re-run clean (39 tests total, no regression).

**v1.17 changes (2026-07-23)**: Phase 9 rescoped before implementation, per review, in three
concrete ways plus one correction found while pinning them (§4.8 has the full detail):
1. The FOV/range filter (Phase 8's top carry-forward finding) must be resolved in code *before*
   any supervisor code exists, since the OOD thresholds are meaningless against a mismatched
   input pipeline. Decided as two independent questions with different answers: this robot's
   real ~180°/8m sensor geometry for the restriction itself (not the reference's D435I-specific
   85.2°/12m), but the reference's dummy-injection-on-empty convention replicated regardless,
   since it protects an architecturally-untested path in the network's attention mechanism, not
   a training-distribution nicety.
2. The supervisor is structurally guaranteed to share MPPI's exact costmap instance - it's
   instantiated by `CrowdNavController` with the same `costmap_ros_` pointer already passed to
   the embedded MPPI, not a second subscription to cross-check. "Same footprint" resolves to no
   independent footprint at all: both rely on the same inflated costmap, where this project's
   `robot_radius` already lives exactly once, matching MPPI's own `consider_footprint: false`
   configuration.
3. The forward-sim horizon and step count are fixed at implementation time (4 steps, the
   existing `time_step_s`, no adaptive refinement) so the watchdog window covers a known
   worst-case cost, per §1.7. The candidate's raw holonomic velocity is used as a deliberately
   conservative (never unsafely-permissive, argued explicitly in §4.8.3) proxy for the robot's
   real diff-drive-constrained motion, rather than re-deriving `toTwistStamped()`'s projection
   inside the watchdog-timed closure.
4. **Found while wiring `LOW_PERCEPTION_CONFIDENCE` to a real signal, not anticipated going
   in**: `GroundTruthHumanSource::degrade()` currently discards a dropped-out detection with no
   trace left in `WorldState` - there was no way to implement this trigger as originally
   described. Fixed with a new non-pure `HumanStateSource::numDegradedLastCall()` (default 0,
   no existing call site affected). Also corrects a v1.15 assumption that the zero-humans case
   would route through this same trigger - it doesn't; a legitimately empty, correctly-perceived
   scene isn't a degraded-perception event, and treating it as one would fire constantly on
   ordinary empty scenes. A new §4.8.6 states explicitly, per review, what this detector cannot
   catch: input-pipeline mismatches (Phase 8's own FOV-filter bug is exactly that class), as
   opposed to world-state novelty, which is all five criteria actually characterize.

**v1.18 changes (2026-07-24)**: Phase 9 implemented and closed per the v1.17 rescope - full
trail in `docs/phase9-findings.md`. All three pinned requirements verified in the actual build
order they demanded: the FOV/range filter and dummy-injection fix landed and were unit-tested
(12 new tests) **before any `crowd_nav_safety_supervisor` code existed**; `SafetySupervisor` is
instantiated directly by `CrowdNavController` with the identical `costmap_ros_` pointer already
shared with the embedded MPPI, making "same costmap instance" structural rather than asserted;
the forward-sim is fixed at 4 steps / 1.0s, no adaptive refinement. Live-verified in Gazebo, not
just unit-tested: a clean baseline run (no pedestrians) reached its goal with zero intervention
events, and with pedestrians present three of the eight causes fired for real, non-engineered
reasons and were logged/published correctly - `CROWD_SIZE` (6 pedestrians over the configured
limit), `PROXIMITY` (5 episodes), and a sustained `COSTMAP_COLLISION` that correctly rejected
the same unsafe candidate on every one of 90+ consecutive ticks (~20+ seconds, zero misses,
1,176 `/intervention_events` messages published total this session). A specific engineered
keep-out-zone trigger wasn't cleanly isolated live this session (ad hoc zone placements either
blocked the global planner upstream of the supervisor or left room to route around) - flagged
honestly rather than glossed over, with a pre-measured corridor-zone test (Phase 3's own
methodology) proposed as a Phase 10 scenario. `RELATIVE_SPEED`/`COMMAND_LIMIT`/
`LOW_PERCEPTION_CONFIDENCE`/`INFERENCE_TIMEOUT` are unit-tested but weren't observed live this
session either. Full-workspace rebuild plus every prior phase's test suite re-run clean (60
tests total across 5 packages, no regression).

**v1.19 changes (2026-07-24)**: Phase 10 rescoped before implementation, per review, in three
concrete ways plus two carried process notes (§4.9 has the full detail):
1. The matrix gets piloted first - one scenario, one seed, all three configs, end to end,
   checked by hand - before any seed count above one is attempted.
2. `N=8` seeds per core-matrix cell and a 5-point `dropout_prob` noise sweep (on
   `policy_supervised`/`open_arena`/`reactive` only, to isolate the perception effect from
   depot's own navigability difficulty) are committed to now, in this revision, not chosen after
   looking at any results.
3. `policy_supervised` underperforming `baseline_mppi` on depot efficiency is documented here as
   the *expected* result before any episode has run - the headline number is the intervention-
   rate-by-cause comparison across scenario families, not a win/loss table.
4. A named, permanent `depot_keepout_block` scenario replaces Phase 9's inconclusive ad hoc zone
   placements, with an explicit expected outcome per config stated in advance - including
   `policy_raw` **expected to violate the zone**, since SARL has no concept of a static keep-out
   region at all.
5. Two carried-forward items get resolved in this rescope rather than deferred again: §5 risk
   #4's never-implemented "log AMCL covariance alongside supervisor decisions" note (§4.9.6, a
   harness-level `interventions.csv` correlating two already-published topics, not a schema
   change), and a stated contingency (§4.9.5) for isolating §4.8.1's FOV-restriction choice from
   the policy itself if depot results look bad enough to need it - one supplementary run using
   already-exposed parameters, zero new code.

**v1.20 changes (2026-07-24)**: Phase 10 implemented and closed - full findings in
docs/phase10-findings.md, summary in §3. Two bugs found and fixed mid-phase, both significant
enough to change what the matrix's own results meant: a Gazebo `PosePublisher` bug that had
left `/ground_truth/robot_pose` silently dead since Phase 4 (retroactively meaning Phase 9's
FOV filter had been inert the whole time it was "verified"), and a map/world coordinate-frame
mismatch that made `depot_keepout_block`'s first run meaningless (zero supervisor interventions
across all three configs). Both root-caused via direct instrumentation rather than guessed at,
fixed, and re-verified. The matrix itself surfaced one result harder to report than "underperforms
on efficiency as predicted": under reactive pedestrians, `policy_supervised` has a *higher*
collision rate than both `baseline_mppi` and `policy_raw`, in both scenario families - reported
in full per the standing "report it even if my method didn't win" instruction, alongside the
`depot_keepout_block` result that shows the supervisor's mechanism does work exactly as
designed (425/425 correct rejections, zero violations) even while the collision-rate result
shows it isn't a strict safety improvement in every condition.

---

## 0. TL;DR of decisions and pushback

- **tugbot_depot is the wrong scale for this robot as-is**, and I'm not going to pretend
  otherwise. Tugbot is 661×598×630 mm; our robot is 190×190×80 mm — a ~3.5× linear ratio.
  A depot built for a 660 mm AMR will have aisles wide enough that a 190 mm robot barely
  notices the crowd. Plan below: try uniform SDF rescaling first (cheap), keep tugbot_depot
  natively-scaled as the SLAM/AMCL/visual demo world regardless, and author a small
  correctly-scaled depot-style world as the actual "structured" **evaluation** world if
  rescaling doesn't hold up physically. Details in §1.1.
- **SARL is not an obs→action network.** It's a *value network* plus a Python-side
  discrete-action-enumeration + one-step constant-velocity lookahead + argmax search. The
  ONNX graph is just the value net (MLPs + attention pooling — exports cleanly). The
  candidate-action search has to be reimplemented in C++ inside `SarlAdapter`. This is more
  adapter code than the brief's "~100 lines" estimate for future policies, but it's exactly
  what the adapter seam is for, and it only has to be paid once. Details in §1.4.
- **HuNav's behavior modes don't give a true non-reactive control condition.** Even
  "impassive" still runs through HuNav's own social-force engine for the pedestrian's own
  path. I'm adding a second, independent, much simpler scripted-waypoint actor mechanism for
  the `non_reactive` pedestrian mode instead of trying to bend HuNav to do it. Details in §1.2.
- **ONNX Runtime packaging**: vendoring our own minimal CMake wrapper (ExternalProject-style,
  pinned to ONNX Runtime 1.17.x CPU-only Linux-x64 prebuilt) rather than depending on
  `ros-controls/onnxruntime_vendor`, whose build-tool dependency (`ament_cmake_vendor_package`)
  has no confirmed Humble track record. CPU-only is correct here — the value net is tiny.
- **`vita-epfl/CrowdNav` ships no trained weights** — confirmed by listing its actual git
  tree, not assumed. Training from scratch (its own `train.config`: 3,000 imitation-learning
  episodes + 10,000 RL episodes) would have been a real, unbudgeted phase. Found a fork,
  `tkkim-robot/Gazebo-CrowdNav`, with a pretrained SARL checkpoint whose `env.config` matches
  ours exactly — using that as the primary weight source, from-scratch training as a
  documented fallback only. Details in §1.8.
- Cuts I'm making relative to a literal reading of the brief, all reversible later: SLAM mode
  is a demo, not wired into the evaluation harness; HEIGHT/OracleNav get an interface + a
  documented walkthrough, not stub classes; CI runs build + unit tests only, no Gazebo-in-CI;
  zone CRUD is a single service, not "service or topic". Full reasoning in §6.
- I initialized a git repo in the project directory (previously not one) so the phased work
  has history from the start. Nothing else touched.

---

## 1. Feasibility assessment

### 1.1 World: tugbot_depot scale

Confirmed: `tugbot_depot` (MovAI, Fuel, CC-BY-4.0, ~3.7 MB) is Fortress-native and pairs with
the Tugbot robot, whose real-world sibling (TUGBOT2) is 661×598×630 mm, 90 kg, 2 m/s top
speed. That is the AMR class this depot's aisles, doorways, and shelf spacing are
proportioned for. Our robot is 190 mm — about 3.5× smaller in every linear dimension. I could
not get exact aisle-width numbers out of Fuel's metadata (it doesn't expose SDF geometry), so
the precise severity is a Phase 0 measurement, not something I can assert a number for — but
the prior from Tugbot's footprint is strong enough that I'd bet on "aisles proportioned for a
~0.6–0.9 m robot," i.e. 6–15× our robot's width. At that ratio the crowd-navigation problem
does partly evaporate, exactly as you suspected.

**Plan:**
1. Phase 0 spike: load `tugbot_depot` in Fortress, measure real-time factor, and measure
   actual aisle/doorway widths (ruler tool or SDF geometry inspection).
2. Try `<include><scale>0.27 0.27 0.27</scale></include>` (0.27 ≈ 190/661, matching the ratio
   Tugbot experiences) on the included world model. This is cheap to try, and if the meshes
   and collisions scale together cleanly, it's the best outcome — full visual richness at the
   right relative difficulty. **This must be an explicit Phase 0 check, not an assumption**:
   gz-sim has open, documented issues (gazebosim/gz-sim#2656, gazebosim/ros_gz#587) where
   `<scale>` on a mesh applies to the visual but mishandles the collision geometry. Test
   procedure: spawn the robot in the scaled world and drive it into a rescaled shelf/wall
   edge; measure where contact actually occurs. If it occurs at the *visually*-scaled boundary,
   scaling is safe to use. If contact happens at the original full-scale boundary (or not at
   all, or the robot clips through), collision didn't scale — stop immediately and fall to
   step 3 rather than debugging it, since a world that looks right but collides wrong is
   exactly the failure mode that wastes a week before anyone notices it in evaluation results.
   Time-box this check to one day.
3. If scaling breaks (fragmented collision meshes, physics jitter, plugin poses now
   misaligned), keep `tugbot_depot` **only** as the native-scale SLAM/AMCL/visual demo world
   (localization-against-static-structure is scale-agnostic, so it's still useful there — see
   §5), and author a small, correctly-scaled depot-style world for the actual **structured**
   evaluation family: a compact floor plan (shelving units, corridors ~0.5–0.7 m, a couple of
   pillars) assembled from simple primitive/box models at the right scale. This is a narrow,
   deliberate exception to "pre-made not hand-authored," and I'll say so in the README rather
   than hide it.
4. Either way, also keep a genuinely open, obstacle-free arena world (trivial to author) for
   the in-distribution control condition per §5.8 of the brief — this one has no scale
   ambiguity since it's just an empty circle.

**Phase 0 results (measured, not estimated — full detail in `docs/phase0-findings.md`):**
RTF ≈ 0.996–1.0, physics-only, sampled directly off the live `/stats` topic. Aisle widths
pulled straight from the SDF: three parallel shelf-pole rows at 2.58 m center-to-center
spacing (~2.52 m clear) — **≈13× our robot's 190 mm width**, vs. ≈4.2× for Tugbot's actual
598 mm width. Confirms the mismatch with real numbers instead of the footprint-based estimate
above.

Better news than expected on the rescale mechanism: the depot's *collision* layer
(`depot_collision`) turns out to be a single hand-authored model built entirely from
primitive `<box>`/`<cylinder>` shapes with literal pose/size numbers — no mesh collision at
all. The gz-sim mesh-scale bugs cited above (gz-sim#2656, ros_gz#587) are specific to `<mesh>`
geometry and **don't apply here** — rescaling is just multiplying already-known numbers by
0.27, not exercising any flaky engine code path. The only mesh in the world is a separate
`<include>` of `OpenRobotics/models/Depot` for the *visual* appearance at the same pose; if
scaling that glitches, it's cosmetic only (collision, costmap, AMCL, and every eval metric key
off `depot_collision`, not the visual mesh) and has a cheap fallback (synthesize simple
box/cylinder visuals matching the already-extracted primitive dimensions). **Net effect: this
risk is downgraded** from "verify carefully, may need the custom-world fallback" to "known-safe
arithmetic rescale, cosmetic-only residual risk." tugbot_depot at native scale remains the
SLAM/AMCL/visual demo world; the 0.27×-rescaled `depot_collision` becomes the structured-depot
evaluation world (not yet built — that's Phase 1/2 world-authoring work).

Side finding, independent of scale: the shelf collision is only 3 cm-radius corner posts, not
full shelf footprints — worth knowing before assuming the depot is collision-realistic at any
scale, though it doesn't change the scale conclusion.

### 1.2 Pedestrian simulation: single ROS node, HuNav dropped (rescoped in v1.7)

**Original plan (superseded, kept below for the record): `hunav_gazebo_fortress_wrapper`
(robotics-upo) for `reactive`, a second independent scripted-actor mechanism for
`non_reactive`.** Real and Fortress+Humble native, but two independent mechanisms sharing only
a topic schema means two independent sources of nondeterminism, two things that can drift out
of sync with each other, and no clean story for byte-identical seeded reproduction across a
whole evaluation matrix (Phase 10) - HuNav's social-force engine isn't built around that
guarantee. Rescoped before implementation started (v1.7, prompted by review, applied to the
plan document itself rather than only in code - see the v1.7 changelog entry above).

**Current plan**: pedestrian state lives in a single ROS node, not a Gazebo plugin - an
ORCA-based or social-force-based simulator (implementation detail, either is fine; the
requirement is determinism, not which algorithm), with `reactive` and `non_reactive` as
**config flags on that one implementation**, not two separate mechanisms. The robot's pose fed
into the pedestrian model comes from **Gazebo ground truth, not `/odom`** - odometry drift has
no business leaking into how simulated pedestrians react to the robot's actual position. The
node is seeded from the scenario's seed (Phase 10's seeded scenario suite, §3), and **must
produce byte-identical pedestrian trajectories across two runs with the same seed** - this is
the actual justification for dropping HuNav, so Phase 4's done-bar proves it directly rather
than assuming it. The node **steps on sim time** (subscribes to `/clock`, not wall-clock
timers) - with RTF sometimes below 1.0 under this project's heavier physics-step
configurations (Phase 2), a wall-clock-driven pedestrian simulator would silently desynchronize
from Gazebo, making seeded reproducibility meaningless regardless of how careful the RNG
seeding is.

A new, separate **Gazebo actor mirror node** provides visual-only representation of the
ROS node's authoritative pedestrian state, for demo/visualization purposes - **launch-toggleable
and off by default**. It never feeds anything back into the simulation; it only reads the
ROS node's published state and moves Gazebo actors to match, so it can drift or lag without
corrupting anything downstream (`GroundTruthHumanSource`, the observation builder, the
evaluation harness) - they all consume the ROS node's topic directly, never the mirror. Both
`reactive`/`non_reactive` mode and mirror-on/off are launch-time switches, not code forks.

### 1.3 ONNX Runtime packaging

`onnxruntime_vendor` (ros-controls) exists and vendors ONNX Runtime 1.24.3 via
`ament_cmake_vendor_package`. I can't confirm that build-tool package has a solid Humble
track record (it reads as written for the Jazzy/Rolling-era vendor-package convention), and I
don't want the ONNX packaging risk item to secretly depend on an external repo I don't
control. Recommendation: vendor a small (~100-line) CMake package ourselves —
`ExternalProject_Add` or plain `FetchContent` pulling the official prebuilt
`onnxruntime-linux-x64-1.17.x.tgz` CPU release from Microsoft's GitHub releases, exposing an
imported target. This is a well-worn pattern (same shape as the vendor packages above; I read
both for reference). CPU-only is the right call — SARL's value net is a few hundred thousand
parameters, sub-millisecond to low-single-digit-millisecond inference on CPU. No CUDA, no GPU
packaging surface. Agree with your instinct to prefer ONNX Runtime over LibTorch: LibTorch's
binary footprint and ABI fragility inside an ament/colcon workspace is a materially bigger
packaging risk for no benefit here.

**Phase 0 result**: built this package for real and it works — `colcon build
--packages-select crowd_nav_onnxruntime_vendor` succeeds, and a real inference plumbing test
(not just a compile check: loads a `Linear(4,1)` ONNX model with known weights, runs it, checks
the output against the known-correct answer) passes. Caught one real bug in the process: the
originally-pinned **1.17.3 rejects models with ONNX IR version 10** outright (a hard crash,
`Ort::Exception`, not a warning) — and this machine's PyTorch (2.13, current dynamo-based
exporter) produces IR version 10 by default on the very first model exported. **Bumped the
pinned version to 1.20.1** (IR v10 support landed in ONNX Runtime 1.19). Also: the originally
sketched `crowd_nav_onnxruntime_vendor::onnxruntime` namespaced target doesn't work for a
vendored prebuilt `.so` (`install(TARGETS ...)` requires a real build target, not a CMake
`IMPORTED` one) — downstream packages consume this via the older
`ament_export_include_directories()`/`ament_export_libraries()` pattern instead (works,
verified). See §4.3 — this doesn't change the `PolicyAdapter` interface, only how
`crowd_nav_policy_adapters`'/`crowd_nav_controller`'s `CMakeLists.txt` link against it.

### 1.4 CrowdNav / SARL export — the architecture surprise

This is worth stating plainly because it changes what "exporting SARL to ONNX" means.

Reading `vita-epfl/CrowdNav`'s `sarl.py`: SARL (and the whole CADRL/multi-human-RL family) is
**not** a network that maps observation → velocity command directly. At decision time, the
Python `predict()` method:
1. Builds a fixed discrete action set (typically ~1 stop action + several speeds × headings,
   under either holonomic or unicycle kinematics — CrowdNav supports both).
2. For each candidate action, propagates the robot one step under that action and propagates
   every human one step assuming constant velocity (closed-form, no network involved).
3. Batches the resulting joint states through the network — `mlp1`/`mlp2` per-human encoding,
   a masked-softmax attention pooling over humans (handles variable crowd size natively via a
   `(scores != 0)` mask, no python loop, exports fine to ONNX), concatenated with self-state,
   through `mlp3` to a scalar **value**.
4. Picks the action maximizing `immediate_reward + γ^(Δt·v_pref) · value`, where the immediate
   reward is a cheap closed-form collision/goal check, not part of the network.

The only thing that needs to leave Python is step 3 — a clean tensor-in/scalar-out graph once
a debug `.cpu().numpy()` line (used only for attention-weight visualization, not decision
logic) is stripped from the forward path. Steps 1, 2, and 4 have to be reimplemented in C++
inside `SarlAdapter`: candidate-action generation, one-step kinematic/constant-velocity
propagation, a batched ONNX Runtime call, and the argmax. That's genuinely more code than a
"~100-line adapter," but it's a one-time cost for the first (and hardest, ironically, despite
being called the "easy" policy) integration — see §6 for why I'm not worried this contradicts
the brief's reusability goal.

### 1.5 Nav2 KeepoutFilter — two real surprises, corrected in v1.7 after Phase 3 implementation

**Originally claimed "no surprises" here — wrong, on two counts, both found only once actually
implemented and tested against the phase's real done-bar, not assumed from reading Nav2's
docs.** Full trail in `docs/phase3-findings.md`; summary:

1. **"Stock KeepoutFilter in the local costmap" (this plan's own original Phase 3 text) is not
   sufficient** for a zone to actually cause a replan, only for it to be locally respected.
   The global costmap/planner (NavFn) has no knowledge of a zone that's only in the local
   costmap, so it keeps handing MPPI the same path straight through it - MPPI correctly
   refuses to violate the keepout, and Nav2 spins in an infinite recovery loop instead of
   replanning. This was **a plan bug, not an implementation detail** - the fix (KeepoutFilter
   in both costmaps) had to happen in this document, not just in `nav2_params.yaml`. The
   general shape of this error - a component checking against costmap/state that the thing
   it needs to agree with doesn't share - is flagged as likely to recur in Phase 9 (§3).
2. The zone-manager node here is **not** "a plain publisher with a CRUD service in front of
   it," as originally written - it's a CRUD service in front of a node that writes a map file
   and asks a real `map_server` instance to reload it via Nav2's own `LoadMap` service,
   deliberately avoiding hand-rolling `OccupancyGrid` publishing logic Nav2 already provides
   (see `docs/phase2-findings.md`'s "don't reimplement the stack," applied here). That path
   itself had a real surprise: `LoadMap.srv`'s own doc comment describes a `file://`-prefixed
   URL form that this Nav2 build's `map_server` actually rejects (`RESULT_INVALID_MAP_METADATA`)
   - a plain absolute path works. Verified directly against the running service, not assumed
   from the message definition's comment - documented in the README's "Known upstream API/doc
   discrepancies" section.

Everything else in the original claim held: the mask topic does use latched/transient-local
QoS specifically so a new `OccupancyGrid` can be republished at runtime and picked up without
restarting the filter, and no Nav2 patches or custom `CostmapFilter` plugin were needed.

### 1.6 ros2_control / gz_ros2_control

`gz_ros2_control` (current name; `ign_ros2_control` is now a compatibility shim) is available
for Humble and is the standard Fortress bridge. Using it as specified — no surprises here,
just confirming the package name to use in code (`gz_ros2_control`, not the legacy
`ign_ros2_control`).

### 1.7 Additional risk not in your original list: control-loop tick budget

The safety supervisor's forward-simulation is *itself* work done inside (or immediately after)
the controller tick, on top of policy inference. If the watchdog only covers inference time
and not supervisor-check time, a slow supervisor can silently blow the tick budget the same
way slow inference can. Fix: the watchdog window covers the full "produce a command we're
willing to send" path — inference *and* supervisor check — and the supervisor's forward-sim
uses a fixed, small horizon/step count (no adaptive refinement) so its worst-case cost is
bounded and known ahead of time, not just typically fast.

### 1.8 A trained SARL checkpoint — confirmed missing upstream, found elsewhere

Raised in review: Phase 0's export spike verifies the *value network exports and matches
PyTorch*, but silently assumed a trained checkpoint would exist to export. Checked this
directly rather than assume it either way — `vita-epfl/CrowdNav`'s git tree has no
`trained_models/` directory and no `.pth`/`.tar` file anywhere, and its README doesn't mention
a checkpoint download. Its own `train.config` specifies 3,000 imitation-learning episodes (50
epochs, warm-started from ORCA demonstrations) followed by 10,000 RL episodes at 100 training
batches each — on the order of hours on CPU given how small the network is, but genuinely a
separate phase with its own environment-setup risk (the repo is old enough — Python
2.7/3.5-era artifacts show up in related forks — that pinning a compatible `gym`/`torch`
version, per the known `crowd_sim` import issue in the upstream tracker, is its own small
yak-shave).

Rather than eat that cost, I looked for a fork that already ships trained weights for this
exact codebase, and found two: `LeeKeyu/sarl_star` (`rl_model.pth`, 349 KB) and
`tkkim-robot/Gazebo-CrowdNav` (`data_sarl/output/rl_model.pth`, 392 KB — plus CADRL and
LSTM-RL checkpoints alongside it). Confirmed both are real binaries, not LFS pointer stubs,
by checking blob sizes directly. `tkkim-robot/Gazebo-CrowdNav` is the better source: its
`data_sarl/output/env.config` matches the parameters this plan already assumed *exactly*
(`discomfort_dist=0.2`, `robot_radius=human_radius=0.3`, `human_num=5`, `v_pref=1`) — it's
essentially the paper authors' own reference training run, not a divergent fork. Upstream
`vita-epfl/CrowdNav` is MIT-licensed; the fork carries no separate license file, so I'll
attribute both the original paper/repo and the specific fork the checkpoint file came from in
the README rather than treat it as unlicensed.

**Revised plan:** Phase 0 loads `tkkim-robot/Gazebo-CrowdNav`'s `data_sarl/output/rl_model.pth`
in a plain Python/PyTorch environment (no ROS involved yet), confirms it's a bare
`state_dict` (not a fully pickled module — those are far more version-fragile) and loads
cleanly on a current PyTorch, and reproduces a reasonable success rate using CrowdNav's own
`test.py` against its own 500 test cases as a sanity check before trusting it as "the policy."
**Only if that fails** (corrupt file, incompatible pickle, success rate far off the paper's
reported numbers) does training-from-scratch become a real phase — and if it does, it runs as
a background job from day one of Phase 0 (it has zero dependency on ROS/Gazebo/ONNX, so it
doesn't block anything else) rather than being discovered as a blocker at Phase 8.

**Phase 0 result**: while setting up validation, pulled the checkpoint's actual
`policy.config` alongside its `env.config` — `[action_space] kinematics = holonomic`. This
checkpoint was trained assuming the robot can move in any direction instantly, not a
differential-drive unicycle. See §1.9 — this is a real, separate finding from the
env.config parameters already used in §4.4, and changes what `SarlAdapter` has to do beyond
candidate-action search.

**tkkim-robot's `rl_model.pth` (the RL-fine-tuned checkpoint) failed validation** — 0.21
success rate at n=500, vs. the paper's reported 0.99. Three cheap differential runs (n=50)
root-caused it: `LeeKeyu/sarl_star`'s independently-trained checkpoint (0.72/0.18/0.10,
unicycle) and **tkkim's own `il_model.pth`** (0.96/0.02/0.02, identical holonomic/
with-global-state architecture) both behave normally through the identical harness — ruling
out a reproduction bug. The problem is specific to that one `rl_model.pth` file (looks like an
undertrained/interrupted RL run, given tkkim's CADRL checkpoints are numbered training
snapshots — `rl_model_1000.pth` … `rl_model_10000.pth` — while SARL's folder ships only one
unnumbered file with no way to confirm convergence).

**FINAL DECISION: `tkkim-robot/Gazebo-CrowdNav`'s `il_model.pth`**, not either RL-trained
option. An 18%-collision-rate base policy (LeeKeyu's) is a bad foundation for this project
specifically — §5.8's whole evaluation design depends on distinguishing "supervisor caught a
genuinely OOD/unsafe command" from "supervisor is compensating for a mediocre base policy,"
and that distinction is already hard to read with a policy that collides in 1 of ~5.5
in-distribution episodes before OOD scenarios are even introduced. `il_model.pth`'s
0.96/0.02/0.02 is a much cleaner substrate for demonstrating the supervisor's value at the
boundaries, which is the actual point.

**Real cost, stated plainly, not hidden**: `il_model.pth` is pure behavior-cloning of ORCA. It
will not show genuinely RL-refined crowd-interaction timing distinct from the classical ORCA
baseline it's evaluated against in §5.8 — the "learned policy" arm will behave closer to its
own comparison point than a fully-converged RL policy would. State this in the project README
as a known limitation. This is exactly the gap Phase 12 (§3) exists to eventually close with a
policy that has real, author-validated RL performance (HEIGHT) instead of a workaround
checkpoint. Also: this choice stays inside the config family §4.4's OOD thresholds were
already derived against (holonomic, `with_global_state=true`) — switching to LeeKeyu's
checkpoint would have meant re-deriving those thresholds against a different architecture on
top of accepting the worse collision rate. Full provenance (SHA256, commit, exact configs) in
`docs/phase0-findings.md`.

**A related correction, recorded rather than quietly fixed**: while surveying alternatives to
SARL (per your question about whether reproducing an old checkpoint was even the right move),
I initially misread `Shuijing725/CrowdNav_Prediction_AttnGraph`'s shipped test logs as
evidence of that repo's own learned policy's quality. Those logs
(`ORCA_no_rand`/`SF_no_rand`) are actually for the paper's **classical baseline comparisons**,
not their PPO+prediction method (`my_model`, which has no test log at all, only a training
progress CSV). AttnGraph was never actually validated as a candidate — corrected before
presenting the final comparison. Full detail in `docs/phase0-findings.md`.

### 1.9 Holonomic-trained policy, nonholonomic robot — a real gap, not just accel clamping

The brief's §5.5 already flags "handle nonholonomic constraints... clamp to the robot's
configured limits," which reads as an acceleration-limit problem. Phase 0 surfaced something
more specific: the actual checkpoint we're using (§1.8) was trained with
`kinematics = holonomic` (confirmed in its shipped `policy.config`), meaning SARL's
one-step action propagation and its resulting `(vx, vy)` command assume the agent can move in
any direction instantly, independent of current heading. Our robot is a differential-drive
unicycle — it cannot execute an arbitrary `(vx, vy)` directly; it can only command
`(v, ω)` (forward speed, turn rate).

This needs an explicit conversion step, not just clamping. Standard approach (used by other
real-robot SARL deployments, e.g. `sarl_star`): treat the network's holonomic `(vx, vy)`
output as a desired heading + speed, and derive `(v, ω)` by turning toward that heading while
moving forward at (up to) the commanded speed — `ω` proportional to heading error, `v` the
commanded speed scaled down as heading error grows (don't drive fast while turning
sharply). This conversion lives in `SarlAdapter::selectAction` (policy-family-specific, per
§4.3), not in the generic controller plugin or the accel-clamp step, which still separately
clamps whatever `(v, ω)` comes out of this conversion to the robot's physical limits.
Documented here now so Phase 8 doesn't rediscover it as a surprise.

### 1.10 Headless rendering hangs on this machine — relevant beyond Phase 0

Found while measuring RTF: running `tugbot_depot` server-only (`ign gazebo -s -r`) with
Tugbot's stock camera/depth sensors enabled hangs indefinitely with no console output (a 45 s
run had to be killed). Stripping the `Sensors`/`Imu` system plugins fixed it immediately —
clean startup, runs to completion, RTF ≈ 1.0. Root cause: no Xvfb/EGL headless-rendering setup
on this machine, and the `Sensors` system's ogre2 backend blocks trying to acquire a GL
context. This is bigger than a Phase 0 RTF-measurement inconvenience: our own robot's 2D
LiDAR (§3 of the brief) is very likely implemented as a `gpu_lidar` sensor type in
gz-sim, which goes through this same rendering system. **Phase 1 will hit this identical hang
the moment the LiDAR sensor is added**, unless fixed first. Fallback options, cheapest first:
(1) install `Xvfb` and run under a virtual display; (2) confirm/force Mesa software
rendering (`LIBGL_ALWAYS_SOFTWARE=1`) works headless for ogre2; (3) if neither works cleanly,
use a CPU raycast-based lidar sensor type instead of `gpu_lidar` (slower, but doesn't touch the
rendering system at all — acceptable given our LiDAR spec is already deliberately modest, §3).
Flagging this now, before Phase 1, rather than letting it surface as a mystery hang later.

---

## 2. Workspace and package layout

Everything below lives in one colcon workspace, `crowd_nav_ws/src/`.

| Package | Responsibility | Depends on Nav2? |
|---|---|---|
| `crowd_nav_description` | URDF/xacro digital twin (§3), LiDAR xacro macro, MEASUREMENTS.md | No |
| `crowd_nav_control` | ros2_control YAML, `diff_drive_controller` config, `.ros2_control.xacro` hardware-plugin swap point | No |
| `crowd_nav_gazebo` | World files (open-arena, tugbot_depot integration, compact depot if needed), spawn launch | No |
| ~~`crowd_nav_msgs`~~ | **Superseded (v1.9), not built as planned** — `AddZone`/`RemoveZone` ended up defined inside `crowd_nav_zones` (Phase 3), the ground-truth human-state message (`Pedestrian`/`PedestrianArray`) inside `crowd_nav_pedestrians` (Phase 4), and `InterventionEvent` inside `crowd_nav_safety_supervisor` (Phase 9) - each colocated with its producing package rather than centralized. Standard ROS2 practice for a project this size, and it happened without the plan saying so first - correcting here rather than leaving the table wrong. | — |
| `crowd_nav_onnxruntime_vendor` | Minimal vendored ONNX Runtime CMake package | No |
| `crowd_nav_perception` | **Built and closed in Phase 5.** `HumanStateSource` interface, `GroundTruthHumanSource` (+ degradation model, consuming `crowd_nav_pedestrians`' `PedestrianArray` via the message-adapter seam in §4.1), `TrackedHumanSource` stub. **Production constructor templated in Phase 8** so it works from any `nav2_core` plugin's `LifecycleNode`, not just `rclcpp::Node` (see §4.7). **Phase 9 (§4.8.1)**: heading tracking + angular-FOV term added to the degradation model (this robot's real ~180°/8m sensor, now actually active in production, not just a capability); `numDegradedLastCall()` added to `HumanStateSource` so `LOW_PERCEPTION_CONFIDENCE` (§4.8.5) is a real, checkable signal. | No |
| `crowd_nav_observation` | **Built and closed in Phase 5.** Observation builder library, canonical `WorldState` struct | No |
| `crowd_nav_policy_adapters` | **`PolicyAdapter` interface, candidate action-space/propagation, shape validation, `DummyAdapter` (Phase 6), `SarlAdapter` built and closed in Phase 8, see §4.7.** Real SARL value net (`models/sarl_value_net.onnx`), production `rotate()`/`computeImmediateReward()`. **Phase 9 (§4.8.1)**: `SarlAdapter::buildInputs()` now injects a synthetic placeholder human on an empty perceived-human list instead of returning an empty batch, verified not to degenerate against the real network. | Onnxruntime only |
| `crowd_nav_controller` | **Built and closed in Phase 7/8, see §4.6/§4.7.** `nav2_core::Controller` plugin: hold-last-action rate handling, inference-latency watchdog, failover to a genuinely embedded `nav2_mppi_controller::MPPIController` via pluginlib (not accel clamping/smoothing - `nav2_velocity_smoother` already owns that), `adapter_type` config switch (`dummy`/`sarl`), live perception via `GroundTruthHumanSource`. **Phase 9 (§4.8)**: instantiates `SafetySupervisor` directly (sharing its own `costmap_ros_`), calls it inside `run_policy_decision` after `selectAction()`, publishes `InterventionEvent` via a `LifecyclePublisher`. | Yes (nav2_core) |
| `crowd_nav_safety_supervisor` | **Built and closed in Phase 9, see §4.8.** Forward-sim + costmap/keepout check, 5-criteria OOD detector, per-cause intervention logging (`InterventionEvent`). Pure C++ core (`SafetySupervisor`), no live ROS node needed for its own logic. | Yes (costmap_2d) |
| ~~`crowd_nav_costmap_filters`~~ | **Built as `crowd_nav_zones` instead (Phase 3)** - same responsibility (zone-manager node: mask gen + reload via a real `map_server` instance + `AddZone`/`RemoveZone`, stock `KeepoutFilter`), different name chosen during implementation; correcting the table rather than leaving two names for one package. | Yes (map_server/costmap_2d) |
| `crowd_nav_pedestrians` | **Rescoped v1.7 - HuNav dropped entirely, see §1.2.** A single deterministic seeded social-force ROS node (`reactive`/`non_reactive` as one config flag), ground-truth robot pose (not `/odom`) via `robot_pose_extractor.py`, plus an off-by-default visual-only Gazebo actor mirror node. Built and closed in Phase 4. **Phase 10**: the original `PosePublisher`-plugin pose source was found to never actually publish on this gz-sim version; replaced with a `scene_broadcaster`/`TFMessage`-based `robot_pose_extractor.py` node, same topic/type, zero downstream changes (docs/phase10-findings.md). | No (Gazebo only, no HuNav dependency) |
| `crowd_nav_evaluation` | **Built and closed in Phase 10, see docs/phase10-findings.md.** Scenario suite (`scenarios.py`), per-episode harness runner (`run_episode.py`, fresh process tree per episode, own stray-process sweep), matrix driver (`run_matrix.py`), in-episode ground-truth monitor (`episode_monitor.py`), plots (`make_plots.py`). | Yes (transitively) |
| `crowd_nav_bringup` | Top-level launch files tying it all together, baseline MPPI config, AMCL/SLAM launch args | Yes |

Dependency direction is deliberate: perception/observation/adapters have zero Nav2
dependency, so they're unit-testable in plain colcon/gtest without spinning up simulation —
that's most of what CI actually runs (§6).

---

## 3. Phased build order

Each phase has an independent "done" check — no phase depends on a later one being started.

**Phase 0 — Feasibility spikes (no lasting code, throwaway scripts OK)**
Measure tugbot_depot RTF and aisle widths; test SDF uniform rescale (with the explicit
collision check from §1.1); stand up `crowd_nav_onnxruntime_vendor` and confirm it links a
trivial ONNX model in a bare CMake target; obtain a SARL checkpoint (§1.8) — load
`tkkim-robot/Gazebo-CrowdNav`'s `data_sarl/output/rl_model.pth` in plain PyTorch, confirm it's
a bare state_dict, run CrowdNav's own `test.py` against its 500 test cases and sanity-check
the success rate, only falling back to training from scratch (kicked off immediately as a
background job, not blocking anything else in this phase) if that fails; then run the SARL
export spike proper (strip the numpy debug line, export the value net, load it in a Python
`onnxruntime` session, confirm outputs match the checkpointed PyTorch model bit-close on a
handful of test states); confirm `gz_ros2_control` diff-drive demo runs on this machine.
**Done:** world-scale decision made and written up; a validated SARL checkpoint is in hand;
ONNX round-trip numerically verified against it; diff-drive demo spins in Gazebo.

**Phase 1 — Robot digital twin — DONE, one open issue carried forward**
`crowd_nav_description` URDF/xacro per §3 of the brief (confirmed geometry + `[PENDING]`
estimates, computed inertia tensors, LiDAR xacro macro with range/rate/resolution/noise as
params), `crowd_nav_control` ros2_control block with the hardware-plugin swap point, spawn
launch in an empty world. **Done:** URDF valid (`check_urdf`), robot spawns, hardware
interface + both controllers activate cleanly, `/scan` publishes at the exact configured
5 Hz/360°/~1°/8 m spec, robot drives under a velocity command (odometry confirmed:
`(0,0)` → `(1.44, 0.93)`, yaw rotated, under a combined forward+turn command — interactive
keyboard teleop itself can't run in this headless environment, so this `cmd_vel` test is the
equivalent verification of the same control chain). Full diagnostic trail in
`docs/phase1-findings.md`, including several environment/tooling issues (space-in-path launch
bugs, a `pkill -f` self-match footgun) hit and fixed along the way.

**Open issue carried into Phase 2, not resolved**: the `gpu_lidar` sensor exhibits a
~176°-wide self-detection artifact confined to its own rear hemisphere (confirmed
sensor-frame-relative, not caused by robot geometry — rotating the sensor's yaw 180° moved the
artifact with it) — a rendering limitation in this gz-sim 6.18.0 install, not a modeling
mistake. Neither raising the mount standoff (tested 0.005/0.03/0.08 m) nor the alternative
CPU-raycast `lidar` sensor type (found completely non-functional on this install) fixed it.
Phase 2 needs to either mask the known-bad sector in software or find an actual renderer-level
fix before AMCL/costmap work can trust the full 360° scan.

**Phase 2 — Baseline Nav2 (MPPI) + AMCL + SLAM toolbox — DONE, done-bar met**
Stock Nav2 with `nav2_mppi_controller`, AMCL against a saved map, `slam_toolbox` online
mapping as an alternate launch mode, on the scaled depot world Phase 0 planned and this phase
built (`depot_scaled.sdf`, generated from Phase 0's extracted primitive geometry). This *is*
the `baseline_mppi` eval config's runtime. **Agreed done-bar** (set explicitly before starting,
per the process note this phase raised): five consecutive successful goals from different
start poses, in both AMCL mode (saved map) and SLAM mode (online map), with no manual
intervention. **Met**, with independent ground-truth verification (not just odometry) that the
robot was never physically compromised during either run — full diagnostic trail in
`docs/phase2-findings.md`.

A first pass left three things open (a SLAM map "ghost obstacle", one unexplained navigation
overshoot, and recurring session-level DDS/lifecycle hangs) and correctly stopped rather than
paper over them. A follow-up session root-caused all three:

- The ghost obstacle and the overshoot shared a root cause the prior session couldn't see from
  odometry alone: the robot was **physically tipping over during driving** (confirmed via
  Gazebo ground-truth pose queries showing ~33° pitch, not the 0° odometry implied). Traced to
  four compounding bugs in `nvis_3302ard.xacro`/`diff_drive_controller.yaml` — a wheel/caster
  joint z-offset that buried the wheels 3 cm below the floor at spawn, caster friction too high
  for in-place rotation, and (the real structural fix) the caster's mass being small enough
  that torque balance put the robot's center of mass essentially at the wheel axle, so the
  caster carried only ~2% of the robot's static weight — enough that ordinary braking/turning
  could momentarily unweight it and tip the chassis. Fixed by correcting the joint geometry,
  giving the caster near-zero friction, and raising its mass share to ~15% (still physically
  reasonable for a real caster assembly). Verified with a full 32-segment driving sweep showing
  zero tip events, pitch/roll exactly 0.0 throughout.
- A second, independent SLAM issue remained after the physics fix: fast 180°/360° in-place
  spins combined with Phase 1's deliberately-narrowed ~180° LiDAR FOV caused scan-matcher
  tracking loss (a double-mapped, rotated-duplicate room outline). Fixed by retiring the
  hand-rolled `cmd_vel`-scripting approach to mapping sweeps in favor of `NavigateToPose`
  waypoint goals — using Nav2's own MPPI controller and costmap-based planner instead of
  re-implementing a worse version of the same capability. Produced a clean, visually-verified
  map (correct room outline, pillars and shelf poles in their right places, no ghosts).
- The overshoot was NavFn's known near-goal degenerate-path brittleness (already partially
  fixed once before) recurring at a slightly larger stall distance; fixed by loosening
  `xy_goal_tolerance`/planner `tolerance` further, kept in sync per the config's own comment.

Also confirmed: AMCL localizes and tracks accurately (~1.3 cm against odometry ground truth
after driving); the DDS/lifecycle hangs did not recur after the CycloneDDS switch + hygiene
tooling from the first session. Full bug-by-bug trail, including the several intermediate
fixes that were necessary but not sufficient on their own, is in `docs/phase2-findings.md` —
worth reading before touching robot dynamics or the mapping scripts again.

**AMCL tuning is a first-class task in this phase, not an afterthought** — Phase 1's LiDAR
mask (§ Phase 1 above, `docs/phase1-findings.md`) reduced the sensor from 360° to a clean
180°, and that reduction bites first here: workable (plenty of real robots run 180° LiDARs)
but with weaker rotational constraint on scan matching than a 360° setup, needing more
particles and more deliberate `laser_model_type` tuning than would otherwise be a default-and-
forget config. Budget real time for this rather than discovering degraded localization
silently later, in Phase 10's results. Also carries forward into costmap config: the reduced
FOV means the local costmap won't clear obstacles behind the robot as it passes them (they
persist until they age out) — `obstacle_layer` raytracing and decay parameters need to account
for this, and reversing recoveries should be treated as less safe as a result.

**Phase 3 — Dynamic keep-out zones — DONE, done-bar met**
`crowd_nav_zones` package: zone-manager node (mask generator + `AddZone`/`RemoveZone`
services, backed by a real `map_server` instance reloaded via its stock `LoadMap` service, not
a hand-rolled publisher — see `docs/phase2-findings.md`'s "don't reimplement the stack"),
`costmap_filter_info_server` + that `map_server` instance for the mask, stock `KeepoutFilter`.
**Done, verified with real evidence, not just presence at launch**: a zone added mid-navigation
genuinely blocked the corridor (confirmed via ground-truth trajectory, not just the mask
topic), the robot found and executed an actual detour (`Reached the goal!` / `Goal succeeded`
in the logs — not a stall reported as success), and removing the zone let a fresh goal through
the same corridor directly (20.7s vs 150s+ with the detour). Full trail, including one
deviation from this phase's own original text, in `docs/phase3-findings.md`.

**Deviation from the plan worth flagging explicitly**: the first real test (not just checking
the mask exists) found that `KeepoutFilter` in the local costmap *alone* — as originally
planned above — doesn't satisfy this phase's own done-bar. The global costmap/planner had no
knowledge of the zone, so it kept handing MPPI the same blocked path, producing an infinite
recovery loop (`Failed to make progress`, repeating) rather than a replan. Fixed by adding
`KeepoutFilter` to the global costmap too — necessary for "MPPI visibly replans" to be true in
any meaningful sense, not scope creep for its own sake. Also found and fixed: a stock Nav2
message's own doc comment (`LoadMap.srv`'s `map_url` field) describing a `file://` URI form
that this build actually rejects — verify against the real service, not the docstring, even
for code that isn't this project's own.

**Phase 4 — Pedestrian simulation (rescoped v1.7, see §1.2) — DONE, done-bar met**
`crowd_nav_pedestrians` package: a single deterministic Helbing-style social-force ROS node
(`pedestrian_sim_node.py`) owning pedestrian state, `reactive`/`non_reactive` as one config
flag (not two separate mechanisms - HuNav dropped entirely), robot pose injected from Gazebo
ground truth via a `PosePublisher` plugin + `ros_gz_bridge` (not `/odom`), seeded from a single
`random.Random(seed)` instance, stepping in fixed `dt` increments keyed to accumulated
`/clock` time (not wall-clock timers). A separate, launch-toggleable, off-by-default Gazebo
actor mirror node (`actor_mirror_node.py`) provides visual-only representation for demos -
never authoritative, never read by anything downstream. Full trail: `docs/phase4-findings.md`.

**Done, verified with direct measurement (docs/phase4-findings.md has the full trail):**
- Pedestrians publish the ground-truth human-state topic/schema; `reactive`/`non_reactive` and
  mirror-on/off are independent launch-time switches, not code forks.
- **Determinism proven, not assumed**: two independent fresh launches (full teardown between
  them), same seed, captured and diffed the first 100 published messages from each - every
  overlapping sim-time-indexed message was byte-identical (position/velocity to 6 decimal
  places). This is the entire justification for dropping HuNav, verified here rather than
  first discovered broken at Phase 10's evaluation matrix.
- **Sim-time stepping confirmed**, not assumed: paused the running Gazebo world mid-simulation
  via its `WorldControl` service and measured `sim_time` directly - it advanced only ~0.47s
  during an 8-second real-time sleep while paused, and the pedestrian node (which steps
  exclusively from accumulated `/clock` values) froze in lockstep and resumed correctly from
  the frozen point on unpause.
- **Headless correctness + mirroring cost measured**: the full pedestrian simulation runs
  correctly with the mirror disabled (the default). With it enabled, all marker models spawned
  and tracked their pedestrians' positions correctly; sampled `real_time_factor` showed no
  measurable difference (~0.9999-1.0 in both conditions) at this scale (6 pedestrians) - a
  real measurement, not an assumption, worth re-checking if pedestrian count scales up a lot
  later.

**Phase 5 — HumanStateSource, perception degradation, observation builder (rescoped v1.9) — DONE, done-bar met**
`GroundTruthHumanSource` wrapping the Phase 4 topic, degradation model (Gaussian
position/velocity noise, dropout, latency, max-range; costmap-based occlusion check declared
but not implemented this phase — the one sub-feature cut first under time pressure, see §6),
`TrackedHumanSource` stub, observation builder producing SARL's flat padded vector with a
schema pinned to the actual reference implementation, not just documented in the abstract
(§4.1.1 - human ordering, padding as this project's own addition not upstream's, frame, units,
exact field layout verified against `tkkim-robot/Gazebo-CrowdNav`). Full trail:
`docs/phase5-findings.md`.

**Done, verified with direct measurement (docs/phase5-findings.md has the full trail):**
- Unit tests (fixed input → known output vector) green, both packages: `crowd_nav_perception`
  (6/6) and `crowd_nav_observation` (3/3).
- **Round-trip verified against the reference implementation, not just hand-computed
  expectations** (§4.1.2): re-cloned `tkkim-robot/Gazebo-CrowdNav` at the pinned commit, called
  its actual `CADRL.rotate()` on 5 synthetic robot+human states to generate a checked-in
  fixture, then fed the same states through this project's observation builder plus a
  test-only transcribed `rotate()` and diffed against the fixture to `1e-6` - all 5 cases
  match. Caught a real gap before the first build attempt: `ObservationBuilder` referenced a
  nonexistent `radius` field on `HumanObservation`; fixed by making human radius a config
  constant (matching how `pedestrian_sim_node` already treats it - one uniform value, not
  per-individual), not a schema addition to the perception message.
- A manual degradation sweep (dropout=0.3 over 1000 synthetic ticks) produces the expected
  missing-detection rate within 5-sigma binomial sampling tolerance, **with RNG-substream
  independence confirmed by test** (§4.2) - same-tag streams reproduce byte-identical, a
  different-tag stream diverges immediately - confirming the split actually isolates
  perception noise from pedestrian motion, not just asserted to by construction.
- Degradation latency's ring buffer verified against sim time with a test built specifically to
  fail under a tick-counting implementation: uneven-interval ingestion (t=0.0/0.3/1.7/2.0),
  query at t=2.0 with latency=1.0 correctly returns the t=0.3 sample, not a naive
  "3-ticks-back" or "latest" answer.

**Phase 6 — Synthetic adapter + ONNX plumbing validation (rescoped v1.11) — DONE, done-bar met**
Added after review: starting the policy-integration work directly with SARL bundles two
independent risks together — "does the ONNX/controller-plugin plumbing work" and "did I
correctly reimplement SARL's candidate-action search" — and a failure could be either. What
this phase is actually for, stated plainly: it's what guarantees a Phase 8 failure means "the
SARL search reimplementation is wrong" and nothing else. Everything it builds is real and
permanent, reused unchanged by Phase 8 — the vendored ONNX package (already built and proven
in Phase 0), the shape-validation helper, the `PolicyAdapter` interface, and the candidate-
action generation/one-step propagation code (§4.3.1, verified against the actual reference
implementation, not reconstructed from memory). `DummyAdapter` is the one throwaway *policy*,
wired to a **hand-written trivial ONNX model** (`generate_dummy_model.py`) whose declared input
shape is computed from the same `policy_adapter.yaml` config the C++ side reads — not a
hand-typed `81` on either side (§4.3.1) — with trivial-but-checkable internals (a single linear
layer with known weights, so inference output has a known-correct answer, not just "didn't
crash"). `buildInputs` produces a real batch of one-step-propagated candidates through this
project's own observation builder (Phase 5, unchanged); `selectAction` does something simple
and deterministic (ignore the network's meaningless output, pick the candidate whose heading is
closest to the goal direction) just to prove data moves correctly end-to-end. Full trail:
`docs/phase6-findings.md`.

**Done, verified with direct measurement (docs/phase6-findings.md has the full trail):**
- A standalone C++ harness (gtest, `crowd_nav_policy_adapters`) loads the checked-in dummy ONNX
  model, runs shape validation, performs inference, and decodes an action, deterministically
  and reproducibly, with no crashes.
- **Shape validation proven to actually reject a mismatch, not just accept the correct case**:
  5 deliberately-wrong `ShapeSpec`s (wrong feature dim, wrong candidate count, wrong name,
  wrong rank, an extra input) against the real model each throw `ShapeValidationError`; a
  dynamic-batch (`-1`) marker is separately confirmed to still be accepted.
- Inference output matches an independently-computed known-correct answer (sum of each input
  row, from the trivial model's known weight=1/bias=0), not just "didn't crash."
- Candidate-action generation and one-step propagation both verified against values computed
  independently in Python, not by re-invoking this project's own C++ formula against itself —
  and the underlying discretization (`speed_samples=5`, `rotation_samples=16`, exponential
  speed spacing, `time_step=0.25`) was re-confirmed by fetching the pinned checkpoint's actual
  `policy.config`/`env.config`/`cadrl.py` directly, not recalled from §7's own assumption.
- Two independent `DummyAdapter` instances given the same synthetic input produce
  byte-identical output.

**Phase 7 — Controller plugin (rescoped v1.13, see §4.6) — DONE, done-bar met**
`nav2_core::Controller` plugin (`CrowdNavController`): hold-last-action across the policy/
controller-tick rate mismatch, and an inference-latency watchdog (structured now to cover
supervisor-check time too, per §1.7, even though Phase 9's supervisor is a no-op placeholder
until it exists) with failover to a **genuinely embedded `nav2_mppi_controller::MPPIController`
instance** loaded via pluginlib — not a hand-rolled fallback, and not a second reimplementation
of velocity smoothing/acceleration clamping, which `nav2_velocity_smoother` already does,
already configured, downstream of every command regardless of source (§4.6). Built and
validated against `DummyAdapter` from Phase 6, not SARL — this proves the controller plugin's
rate handling and watchdog/failover mechanics independently of whether SARL's search logic is
correct. Full trail: `docs/phase7-findings.md`.

**Done, verified with direct measurement (docs/phase7-findings.md has the full trail):**
- A `policy_raw` config running `DummyAdapter` drives the robot end-to-end in Nav2, in Gazebo
  (`NavigateToPose` to a real goal succeeded).
- Artificially stalling inference (a live-settable diagnostic parameter,
  `FollowPath.debug_inject_decision_delay_s`) demonstrably triggers failover, confirmed via
  edge-triggered source-switch logging within the same decision cycle the stall was injected.
- **The failover *transition*, not just the trigger, verified quantitatively**: the raw
  `controller_server` output showed a genuine single-tick discontinuity at the switch
  (linear.x jumped -0.120 m/s, angular.z jumped +0.677 rad/s in one 0.05 s tick) that the
  robot cannot execute instantaneously; the post-`nav2_velocity_smoother` stream — what the
  robot actually receives — bounded the same transition to *exactly* the configured
  `max_accel`/`max_decel` per-tick limits (0.075 m/s and 0.15 rad/s per tick, confirmed to the
  decimal against the smoother's own config), not inferred from Phase 10 results.
- The `ControllerDecisionCore` watchdog/hold-last-action/race-avoidance logic (§4.6) unit
  tested directly against injected fake decision callables (controllable delay) — 4/4 green,
  including a genuine bounded-wait proof (returns in <150 ms against a 200 ms-sleeping fake)
  and a race-avoidance proof (a second decision never starts while one is still outstanding).
- **Six real integration gaps found and fixed via the live verification, none catchable by unit
  tests alone**: a static-vs-shared-library pluginlib load failure (the exact "library not
  found" class of error flagged before starting), a launch-substitution string that silently
  doesn't resolve inside a raw YAML params file, a `DummyAdapter` tie-break bug that picked the
  stop action whenever the goal was roughly ahead of the robot (the common case, not an edge
  case — the robot simply never moved), runtime parameters that were read once at `configure()`
  and never refreshed by `ros2 param set`, a SLAM+`empty.sdf` pairing that can't work because
  the "open-arena" world has no geometry to map, and reconfirming pluginlib registration end to
  end against the real controller_server log. Full-workspace rebuild plus every prior phase's
  test suite re-run clean (30 tests total, no regression).

**Phase 8 — SARL ONNX export + SarlAdapter (rescoped v1.15, see §4.7) — DONE, done-bar met**
Real export of the Phase 0-validated checkpoint (strip the debug `.cpu().numpy()` line, export
the value net, verify bit-close against the original PyTorch model **in Python, before any C++
adapter code is written**), then `SarlAdapter` implementing the candidate-action generation +
one-step propagation + batched inference + argmax from §1.4, including the `policy_radius`/
`robot_radius` split from §4.3 and an **unpadded, dynamic-shape** input construction (§4.7 -
`ObservationBuilder`'s fixed-`max_humans` padding is `DummyAdapter`-specific and would be a
real correctness bug here). Swapped in for `DummyAdapter` via a new `adapter_type` config
switch (`"dummy"` | `"sarl"`) — no changes to the Phase 7 controller plugin's call sites, which
is itself a live test of the adapter-swap promise the whole `PolicyAdapter` design exists for,
before HEIGHT or ORACLE-Nav ever show up. Full trail: `docs/phase8-findings.md`.

**Done, verified with direct measurement (docs/phase8-findings.md has the full trail):**
- The ONNX export verified bit-close against the original PyTorch checkpoint in Python, on real
  varied joint states, *before* any C++ adapter code (max abs diffs: 0.0 wrapper-vs-original,
  1.9e-7 ONNX-vs-wrapper, 2.4e-7 batched-vs-sequential) — plus a load check against the actual
  pinned C++ ONNX Runtime 1.20.1, not just the Python-side runtime used for the export check.
- A standalone C++ harness (no Nav2, no Gazebo) feeds 15 scenarios through `SarlAdapter` and its
  chosen action matches the original Python SARL's action — **including 10 adversarial cases
  mined from the reference's own `action_values`** (top-two gaps from 0.0018% to 0.05%), not
  just cases where one action is obviously best. One case at the loosest adversarial margin
  (0.05%) diverged; verified from the C++ side too that the two candidates' network values are
  themselves within ~1% at that exact case — a genuine cross-platform floating-point precision
  boundary (PyTorch/numpy vs. ONNX Runtime CPU reduction order), not a logic bug, since 9 other
  cases with tighter margins (down to 0.0018%, 25x tighter) matched exactly.
- A dedicated test asserts the self-state radius value actually reaching the network is
  `policy_radius_m` (the training value), never `robot_collision_radius` (the URDF value).
- Flipping the Phase 7 controller's `adapter_type` from `dummy` to `sarl` required zero call-site
  changes; live-verified end to end in Gazebo with real pedestrians present (Phase 4's
  simulation, real perception via `GroundTruthHumanSource`) — goal reached, no fallback triggers.
- **A major architectural finding, not anticipated going in**: the checkpoint's own source repo
  applies a simulated depth-camera FOV filter and injects a synthetic "dummy" human inside its
  own `JointState` construction *before* SARL ever sees the human list — found while generating
  the action-match fixture, when a naive apples-to-oranges comparison (this project's raw
  perceived humans vs. the reference's filtered-and-augmented list) produced a false mismatch.
  Fixed the fixture to capture what `predict()` actually evaluated; **this project's
  `SarlAdapter` does not yet replicate any FOV/range filter of its own** — a real, explicitly
  flagged gap (not this checkpoint source's specific D435I spec, which doesn't match this
  robot's actual ~180°/8m sensor, but the *principle* of restricting to what the robot could
  actually perceive) worth resolving before Phase 10's evaluation numbers are treated as
  meaningful, most naturally alongside Phase 9's own perception-awareness work.
- `DummyAdapter` **stays in the tree after this phase**, not deleted once real weights land —
  it's the fastest possible smoke test for the whole inference path (no checkpoint, no PyTorch,
  no real policy) and the tool of choice for isolating "is the plumbing still working" from "is
  the policy behaving correctly" if a later integration (Phase 12's HEIGHT) breaks something.
  Kept as a permanent fixture in the test suite, not a Phase 6 throwaway — Phase 7 already found
  a real bug in it this way (`docs/phase7-findings.md`), not a hypothetical one.

**Phase 9 — Safety supervisor (rescoped v1.17, see §4.8) — DONE, done-bar met**
**Principle carried forward from Phase 2** (`docs/phase2-findings.md`, "don't reimplement what
the Nav2 stack already does better"): the forward-simulation/collision-check loop below should
reuse Nav2's own collision-checking primitives (costmap cost lookup, `nav2_costmap_2d`'s
footprint collision checker, MPPI's `ObstaclesCritic` as a reference) rather than a from-scratch
trajectory rollout — a hand-rolled version found real, hard-to-predict interaction bugs in
Phase 2 (mapping sweep × narrow LiDAR FOV × scan matching) that reusing the stack's own,
already-hardened logic would have avoided. Reserve genuinely custom code for what's actually
novel here — the OOD detector and the intervention-logging/fallback decision — not for
re-solving "is this position in collision." §4.8.2/§4.8.3 land on a plain point-cost lookup
against the shared costmap (not `FootprintCollisionChecker`'s polygon sweep), since this
project's costmap uses a scalar `robot_radius` and MPPI's own `ObstaclesCritic` already runs
`consider_footprint: false` against the same costmap - the point lookup is the primitive
already shared with MPPI, not a new one invented for the supervisor alone.

**Second principle carried forward, from Phase 3's §1.5 correction**: the same failure shape
recurs whenever two components need to agree on state but only one of them actually has it.
Phase 3's version was a global planner computing paths against a costmap that didn't know
about active keep-out zones (only the local costmap did), producing an infinite
disagree-and-retry loop instead of a real replan. The supervisor here is at genuine risk of the
same class of bug if its forward-simulation collision check runs against a *different* costmap
snapshot (different update rate, different layer set, different timing) than what the
controller being supervised is actually acting on — the supervisor could reject or accept
commands based on a picture of the world the controller doesn't share, producing exactly the
kind of silent disagreement Phase 3 found the hard way. Verify explicitly that the supervisor
reads the *same* costmap (or an explicitly-justified equivalent) the controller uses, don't
assume a second subscription to "the costmap topic" is automatically the same picture. §4.8.2
resolves this structurally: `SafetySupervisor` is instantiated by `CrowdNavController` itself
with the same `costmap_ros_` pointer already shared with the embedded MPPI, not a second,
independently-subscribed copy to cross-check against.

**Three further requirements from review, resolved before any supervisor code (§4.8 has the
full reasoning for each) - this is why the build order below starts with perception, not the
supervisor package**:
1. **The FOV/range filter must be resolved first**, not deferred to Phase 10, because the OOD
   thresholds below are meaningless if tuned against an input pipeline that doesn't match what
   the policy was trained on. §4.8.1: this robot's real ~180°/8m sensor geometry for the
   restriction itself (not the reference's D435I-specific 85.2°/12m - a deliberate,
   justified divergence, not an oversight), but the reference's dummy-injection-on-empty
   convention *is* replicated regardless of FOV numbers chosen, since it protects an
   architecturally-untested (not just untrained) path in the network's own attention
   mechanism. Corrects a v1.15 assumption that the zero-humans case would route through
   `LOW_PERCEPTION_CONFIDENCE` - it doesn't; a genuinely empty, correctly-perceived scene isn't
   a degraded-perception event.
2. **Supervisor and MPPI must share the same costmap instance and the same footprint
   treatment.** §4.8.2: structural, by construction (same `costmap_ros_` pointer), not an
   assertion to add. "Same footprint" resolves to "no independent footprint at all" - both the
   supervisor and MPPI's `ObstaclesCritic` already rely on the same inflated costmap values,
   which is where this project's `robot_radius` already lives exactly once.
3. **The forward-sim's cost must be bounded by construction, not typical-case fast.** §4.8.3:
   fixed 4 steps at the existing `time_step_s` (1.0s look-ahead total), no adaptive refinement,
   using the candidate's raw holonomic velocity as a conservative (never unsafely-permissive,
   per §4.8.3's argument) proxy for the robot's actual diff-drive-constrained motion, so the
   watchdog window (§1.7) covers a known worst case.

Forward-simulation of commanded velocity, costmap + active-keep-out-zone check (§4.8.3), OOD
detector (§4.4/§4.8.5 - concrete criteria, each wired to a real, checkable signal, including a
fix to make `LOW_PERCEPTION_CONFIDENCE` reachable at all, §4.8.5), a two-tier fallback - watchdog
timeout still defers to the embedded MPPI unchanged from Phase 7, but a supervisor rejection is
a direct controlled stop rather than a second call into MPPI, for the thread-safety reason
§4.8.4 gives - and per-cause intervention logging (`InterventionEvent`, §4.8.7). **Explicitly
out of scope for what this catches**: §4.8.6 - the OOD detector characterizes world-state
novelty, not input-pipeline correctness; it would not, by itself, have caught Phase 8's FOV-
filter bug, and shouldn't be read as a general guarantee against that class of error recurring
elsewhere. Full trail: `docs/phase9-findings.md`.

**Done, verified with direct measurement (docs/phase9-findings.md has the full trail):**
- The FOV/range filter and dummy-injection fix were implemented and unit-tested (12 new tests
  across `crowd_nav_perception`/`crowd_nav_policy_adapters`) **before any supervisor code was
  written**, per the explicit sequencing requirement - not just described as done first, actually
  landed first, in that order, in separate commits.
- `SafetySupervisor` (12 unit tests) verified geometrically: each of the 5 OOD criteria
  triggering individually plus an all-clear case; forward-sim safe/collision/`NO_INFORMATION`-
  excluded/out-of-bounds-excluded cases; `KEEPOUT_VIOLATION` vs. generic `COSTMAP_COLLISION`
  labeling with and without a matching secondary mask. A real, non-hypothetical test confirms
  `LOW_PERCEPTION_CONFIDENCE` fires on a dropout and does not fire on a legitimately empty scene.
- **Live-verified in Gazebo, not just unit-tested**: a clean baseline run (no pedestrians)
  reached its goal with zero intervention events - no false positives. With pedestrians present,
  three of the eight causes fired for real, non-engineered reasons and were logged/published
  correctly: `CROWD_SIZE` (6 pedestrians exceeding the configured limit), `PROXIMITY` (5 distinct
  episodes), and a sustained `COSTMAP_COLLISION` - the same candidate rejected on every one of
  90+ consecutive ticks (~20+ seconds), zero misses, `sent_vx=sent_vy=0.0` every time. 1,176
  `/intervention_events` messages published across the session. A specific engineered keep-out-
  zone trigger wasn't cleanly isolated this session (ad hoc zone placements either blocked the
  global planner upstream of the supervisor or left room for Nav2 to route around them) - but
  `KeepoutFilter` writes into the identical costmap as `LETHAL_OBSTACLE` (§4.8.3), so the
  non-engineered `COSTMAP_COLLISION` episode already exercises the exact same rejection code
  path a corridor-blocking zone would; flagged as a good Phase 10 scenario-suite candidate for a
  cleaner, pre-measured version of that specific test (Phase 3's own methodology).
- `RELATIVE_SPEED`, `COMMAND_LIMIT`, `LOW_PERCEPTION_CONFIDENCE`, and `INFERENCE_TIMEOUT` are
  unit-tested but were not observed firing live this session - honestly flagged, not glossed
  over, as carry-forward items for Phase 10 (§4.8.6 already states the detector's own limits;
  this is a coverage note, not a new one).
- Full-workspace rebuild plus every prior phase's test suite re-run clean (60 tests total across
  5 packages, no regression).

**Phase 10 — Evaluation harness (rescoped v1.19, see §4.9). CLOSED, see docs/phase10-findings.md.**
Seeded scenario suite (open-arena × structured-depot) × (`baseline_mppi`, `policy_raw`,
`policy_supervised`) × (`reactive`, `non_reactive`), clean episode termination on
collision/timeout, CSV + metrics + plots, perception-noise sweep. All three requirements from
review met, each verified rather than assumed:
1. **Piloted before the full run** (§4.9.1) - and the pilot caught real bugs (a lost executable
   bit, a stray-Gazebo-process corruption risk) before they could waste an overnight run.
2. **N and the noise-sweep points decided in §4.9.2 before any episode ran**: `N=8` seeds/cell
   (96 core episodes), `dropout_prob ∈ {0.0, 0.1, 0.2, 0.3, 0.5}` sweep on
   `policy_supervised`/`open_arena`/`reactive` (40 episodes) - both numbers held unchanged
   through the full run.
3. **The expected-result-shape framing held, and then some**: `policy_supervised` underperforms
   `baseline_mppi` on efficiency as predicted, but the matrix also surfaced a less flattering,
   unpredicted result - a **higher** collision rate than both alternatives under reactive
   pedestrians, in both scenario families - reported prominently in the findings doc rather than
   the efficiency gap alone.

`depot_keepout_block` (§4.9.3) initially produced a meaningless result: zero supervisor
interventions across all three configs, contradicting the plan's own stated expectation that
`policy_raw` would violate the zone. Root-caused (not assumed) to a coordinate-frame bug: the
harness's own ground-truth zone check compared a Gazebo-world-frame pose against a map-frame-
intended zone spec, while the safety supervisor's own forward-sim (correctly, internally
consistently) operated entirely in map frame. Fixed in both the harness (`episode_monitor.py`
now checks `/amcl_pose`, not `/ground_truth/robot_pose`) and the scenario definition (the zone's
map-frame position was itself wrong, chosen with world-frame intuition). Re-run to a decisive
result matching the original hypothesis: `baseline_mppi` routes around the zone via the global
costmap planner, `policy_raw` drives straight into it, `policy_supervised`'s forward-sim
rejects the identical approach 425/425 times (zero violations) but has no way to route around
it, so it gets stuck rather than succeeding. Full root-cause trace and the noise-sweep cliff's
rate-normalized saturation analysis (ruling out "supervisor floor" as an alternative
explanation) are both in docs/phase10-findings.md, not left implicit in a CSV.

A previously-undiagnosed Gazebo bug was also found and fixed this phase: `PosePublisher` never
actually published on this gz-sim version, meaning `/ground_truth/robot_pose` had never worked
- which retroactively means Phase 9's FOV-filter feature had been silently inert since it was
built (same topic, same dependency). Fixed via a `scene_broadcaster`/`TFMessage`-based
replacement (`robot_pose_extractor.py`), confirmed to fix both the harness's own metrics and
(as a side effect, same topic) the FOV filter.

**Phase 11 — CI and docs**
GitHub Actions per-PR gate: build workspace + compile plugins + run unit tests + lint (no
Gazebo, see §6). Separate nightly/manually-triggered workflow: launch the full stack
(Gazebo + Nav2 + one pedestrian) and drive one goal to completion in the open-arena world —
not a correctness check, just a smoke test for launch-file and parameter-schema rot, which
unit tests structurally can't catch and which is exactly what silently breaks in ROS 2
workspaces over time. **Pinned before implementation, from Phase 10's own experience**: the
smoke test must assert on the actual topics the stack depends on being live (starting with
`/ground_truth/robot_pose` - the exact one that stopped publishing silently for three phases,
`docs/phase10-findings.md`), not just that the launch succeeds and a goal is reached - a
handful of "topic X published at least N messages" checks would have caught that bug in a day
instead of three phases, since a dead topic with a passing goal is exactly the failure mode
this tier exists to catch. README rewrite: lead with the results and the project's actual
thesis (a bounded, verifiable safety mechanism with a real, measured capability cost and a
derivable detection-margin limit - `docs/phase10-findings.md`'s overall assessment, not a
sentence about installation), written for a reader who won't run it - the `docs/` findings
chain is the evidence, the README is the argument built on it. Also: a "how to add a new
policy" walkthrough (documented, using HEIGHT as the worked example, not implemented). Final
MEASUREMENTS.md pass.

**Phase 12 — Second policy integration: HEIGHT**
This is the reusability proof `PolicyAdapter` was built for, not a replacement for the SARL
work above. Explicitly out of scope until Phases 1–11 are done and the runtime is proven end
to end against `SarlAdapter`. Scope: (1) actually validate `Shuijing725/CrowdNav_HEIGHT`'s
official checkpoint (Google Drive link, §1.8 — currently unverified, treat with the same
skepticism `tkkim-robot`'s `rl_model.pth` got before it was tested) using the same kind of
differential-testing discipline Phase 0 used for SARL — don't assume it's good just because
it's official; (2) write `HeightAdapter` against the heterogeneous spatio-temporal graph
observation HEIGHT expects (real new work: this is not a flat vector, and won't reuse
`SarlAdapter`'s candidate-action-search pattern — HEIGHT outputs actions directly via PPO, per
Phase 0's research, so `selectAction` is simpler than SARL's, but `buildInputs` is harder,
needing a graph-structured tensor bundle); (3) export to ONNX and validate the same way Phase
8 validated SARL's export (bit-close match against the original PyTorch model); (4) swap
`policy_supervised`'s adapter config from `sarl` to `height` and confirm **zero changes** to
`crowd_nav_controller`, `crowd_nav_safety_supervisor`, the observation-builder interface, or
the evaluation harness — that's the actual thing being tested here. **Done:** HEIGHT drives
the robot through the same evaluation scenario suite Phase 10 built, using the same
`PolicyAdapter` seam, with only a config change and a new adapter file — proving the
reusability requirement (brief §6) with a second real policy, not just an interface that
looks reusable on paper.

Hardware (ESP32 `hardware_interface::SystemInterface`) is explicitly out of scope for all of
the above — the interface boundary in Phase 1 is designed so it drops in later with zero
changes above it, but nothing in this plan depends on it existing.

---

## 4. Interface definitions

### 4.1 `HumanStateSource`

```cpp
struct HumanObservation {
  uint32_t id;
  double x, y;           // map frame, metres
  double vx, vy;         // map frame, m/s
  double cov_xx, cov_yy, cov_xy;  // full symmetric 2x2; only the diagonal is populated by
                                   // today's synthetic noise model (cov_xy = 0). Widening
                                   // this later would touch every adapter, so the field
                                   // shape is fixed now even though only isotropic noise
                                   // fills it until a real tracker needs the off-diagonal.
  rclcpp::Time stamp;
};

class HumanStateSource {
public:
  virtual std::vector<HumanObservation> getHumans(const rclcpp::Time & query_time) = 0;
  virtual ~HumanStateSource() = default;
};
```

- `GroundTruthHumanSource(topic, degradation_params)`: subscribes to a ground-truth topic via
  a small internal message-adapter function (not hardcoded to HuNav's or the scripted actor's
  message type — this is the actual seam), applies the degradation model below, returns the
  degraded set.
- `TrackedHumanSource`: constructor takes a tracker topic name; `getHumans()` is a documented
  stub (throws `NotImplementedYet` with a clear message) — this is intentionally thin per §6.

### 4.1.1 Observation builder schema — verified against the actual reference implementation

**Added in v1.9, before implementation, per review**: the original plan asserted the
observation builder produces "SARL's flat padded vector with a documented schema" without the
schema actually being pinned down against real source. Fixed by cloning
`tkkim-robot/Gazebo-CrowdNav` (commit `9cad128d124f86bafe48d2cd11b5eee74bec77d9`, matching the
checkpoint's source repo) and reading `crowd_nav/policy/cadrl.py`/`multi_human_rl.py`/
`crowd_sim/envs/utils/state.py` directly, rather than assuming a schema and hoping Phase 8
reconciles it later.

**What's actually verified, concretely:**
- `FullState` (self/robot): 9 raw fields, in this exact order —
  `px, py, vx, vy, radius, gx, gy, v_pref, theta`.
- `ObservableState` (each human): 5 raw fields — `px, py, vx, vy, radius`.
- The network's actual input is **not fixed-size/padded at all** in the reference training
  code — the batch dimension is literally the number of humans present that tick, handled by
  `SARL.ValueNetwork.forward`'s attention pooling over however many rows exist. **Padding to a
  fixed max-human-count is this project's own addition**, needed because our target is a
  static-shape ONNX export, not a training-time variable-length batch — it is not something to
  reverse-engineer from the reference repo, because the reference repo doesn't do it. Document
  it as our own design choice, not attribute it to upstream.
- `CADRL.rotate(state)` (used unchanged by SARL) is the one piece with an exact, unambiguous
  reference: given a batch of concatenated `(self_state + one_human_state)` rows (14 columns:
  `px,py,vx,vy,radius,gx,gy,v_pref,theta,px1,py1,vx1,vy1,radius1`), it produces the 13-column
  egocentric feature row `[dg, v_pref, theta, radius, vx, vy, px1, py1, vx1, vy1, radius1, da,
  radius_sum]` via a fixed rotation onto the robot-to-goal heading — see `cadrl.py` lines
  187-222 for the exact formula (goal distance `dg`, ego-rotated self velocity, ego-rotated
  relative human position/velocity, distance-to-human `da`, summed radii). This function is
  what the round-trip test (§4.1.2 below, Phase 5's done-bar) checks our raw field construction
  against - not the fork's own environment/FOV-simulation code in `state.py`, which is
  training-environment perception simulation specific to that fork, not part of the policy's
  own observation contract.
- Robot `v_pref`/`theta`: `v_pref` is a **config parameter matching the training distribution**
  (not measured from the real robot — same category of number as `policy_radius`, §4.3/§7),
  `theta` is the robot's actual current heading (`atan2(vy, vx)` per `JointState.__init__`,
  or the robot's real orientation if directly available - equivalent for a robot actually
  moving in its heading direction).

### 4.1.2 Round-trip verification against the reference implementation (Phase 5 done-bar)

**Added in v1.9, per review**: a fixed-input/hand-computed-expected-output unit test (the
original plan's only check) catches transcription errors, but the expected values in that test
are ones a person wrote by hand - it can't catch "I misunderstood the schema" bugs, which are
exactly the class of bug most likely here and least likely to be caught by anything downstream
(a wrong human ordering or an off-by-one in padding produces a policy that behaves plausibly
but wrongly, silently). The done-bar therefore requires, in addition to the hand-computed unit
tests: feed an identical synthetic robot+human state through (a) this project's observation
builder, through to the raw (pre-padding) field vector, and (b) the reference repo's own
`CADRL.rotate()` called directly on the equivalent raw vector constructed the same way in
Python - assert the two rotated outputs match to floating-point tolerance. This is checked
against the actual reference implementation, not against this project's own understanding of
it - the same class of check Phase 8's `SarlAdapter` verification (§3) already commits to, done
one phase earlier where it's cheaper to fix if the schema understanding is wrong.

### 4.2 Perception degradation model

Applied per-human, per-tick, inside `GroundTruthHumanSource`, all defaulting to zero (oracle
passthrough) unless configured:

```cpp
struct DegradationParams {
  double sigma_pos_m = 0.0;
  double sigma_vel_mps = 0.0;
  double dropout_prob = 0.0;
  double latency_s = 0.0;
  std::optional<double> max_range_m;
  bool occlusion_check = false;   // ray-casts against the static costmap, not full 3D
};
```

- **Noise/dropout RNG uses its own substream, separate from Phase 4's pedestrian-motion RNG**
  (added in v1.9, per review) - both are ultimately seeded from the same scenario seed, but if
  they drew from one shared stream, changing a noise parameter (e.g. sweeping `sigma_pos_m`)
  would consume a different number of random draws and shift *which* random numbers the
  pedestrian motion model sees too - silently changing where the humans walk at every noise
  level. That turns a noise sweep into an uncontrolled experiment (humans on different paths
  at each setting, not just differently-perceived), and the effect would be subtle enough to
  survive to Phase 10's results without anyone noticing the sweep wasn't isolating what it
  claimed to. Fixed by deriving two independent substreams from the one scenario seed (e.g.
  `std::seed_seq{scenario_seed, 0}` for pedestrian motion, `std::seed_seq{scenario_seed, 1}`
  for degradation noise/dropout - any equivalent independent-substream derivation is fine, the
  requirement is that changing degradation params cannot perturb the pedestrian RNG's own draw
  sequence, and vice versa).
- Noise: seeded Gaussian draw per human per tick, from the degradation substream above.
- Dropout: seeded Bernoulli draw per human per tick, same substream as noise (dropout and noise
  are both "degradation," and don't need mutual isolation from each other - only from the
  pedestrian-motion stream).
- **Latency ring buffer sized and verified against sim time, not a raw tick count** (added in
  v1.9, per review): `latency_s` is a duration, but a ring buffer's natural size is a count of
  slots. If the buffer length is a hardcoded tick count and the observation builder's own tick
  rate ever changes, the *effective* latency silently changes with it - the buffer would still
  compile and run, just be quietly wrong. Fixed by deriving `buffer_length_ticks` from
  `latency_s / tick_period_s` at configuration time (not hardcoding it) and asserting the
  division is exact (or documenting and rounding deliberately, not silently) - alternatively,
  keying the buffer to stored sim timestamps directly (each slot carries its own stamp; "the
  degraded state" is whichever slot's stamp is `>= now - latency_s`) removes the tick-rate
  coupling entirely and is the more robust of the two options if the extra bookkeeping is
  cheap, which it is here.
- Max-range / occlusion: cheap 2D checks (Euclidean distance; line-of-sight raycast against
  the static costmap) — occlusion is the one piece I'd cut first if time runs short, per the
  brief's own "optional" framing of it.

### 4.3 `PolicyAdapter`

```cpp
struct ShapeSpec { std::vector<std::string> input_names; std::vector<std::vector<int64_t>> input_shapes;
                    std::vector<std::string> output_names; std::vector<std::vector<int64_t>> output_shapes; };

class PolicyAdapter {
public:
  virtual ShapeSpec expectedShape() const = 0;
  // Builds whatever tensor batch this policy family's network needs from canonical world
  // state. For SARL this is a batch of one-step-propagated candidate joint-states, one row
  // per candidate action — NOT a single observation vector.
  virtual TensorBundle buildInputs(const WorldState & state) = 0;
  // Consumes the network's raw output(s) plus the same world state to produce a command.
  // For SARL this is where the argmax over candidate actions + immediate-reward term lives.
  virtual Velocity2D selectAction(const TensorBundle & model_outputs, const WorldState & state) = 0;
  virtual std::string name() const = 0;
  virtual ~PolicyAdapter() = default;
};
```

At load time, the controller plugin queries the ONNX Runtime session's actual input/output
tensor names and shapes and asserts they match `expectedShape()` — mismatch fails the
lifecycle transition loudly rather than segfaulting or silently misinterpreting a tensor.
Config (adapter type, model path, schema version, action-space discretization, kinematics
mode) lives entirely in YAML — no code changes to select or tune an adapter.

**Trained radius vs physical radius.** SARL's self-state input includes the agent's own
radius as a literal feature, and CrowdNav's public training config (`env.config`, confirmed)
sets `robot_radius = 0.3` m — roughly 2× our actual robot's physical collision radius.
Feeding the network our true radius would itself be a distribution shift, independent of
everything else in §1.1/§1.4. `SarlAdapter` therefore takes two separate config values:
`policy_radius` (default 0.3 m, matching CrowdNav's training distribution — fed into the
network's self-state feature) and `robot_radius` (the physical footprint — used in exactly
**two** places: costmap inflation and the safety supervisor's forward-sim collision check).
**Correction (2026-07-22, caught while updating this section after a Phase 2 bug — see
below): this originally, incorrectly, also listed the OOD proximity threshold as a
`robot_radius` consumer. It isn't — §4.4 below is explicit that the proximity threshold uses
`policy_radius`, not the physical radius, and that's correct: it needs to match what the
policy was trained against, not the real robot's size.** This split is a `SarlAdapter`-specific
detail; the generic `PolicyAdapter` interface doesn't need to know about it.

**`robot_radius`'s actual value — diagonal, not face-normal half-width, confirmed the hard way
in Phase 2.** The chassis is a 190×190 mm square. The face-normal half-width is 0.095 m, but a
circular collision-radius approximation needs the **diagonal** (`sqrt(0.095²+0.095²) =
0.1344 m`) — using the smaller face-normal figure under-represents the footprint at the
corners specifically, which is exactly the bug Phase 2 found live (§ Phase 2 above,
`docs/phase2-findings.md`): an undersized `robot_radius: 0.12` let the planner treat squeezes
as clear that the real square corners would have clipped. Fixed to `0.14` m (small margin over
the exact diagonal). Single source of truth is now the `robot_collision_radius` xacro property
in `crowd_nav_description/urdf/nvis_3302ard.xacro`; `crowd_nav_bringup/config/nav2_params.yaml`
already matches it (both costmaps) with a comment pointing back to it. **Requirement for
Phase 9**: the safety supervisor's forward-sim collision check must use this same value/
property, not re-derive or hardcode its own — this is exactly the kind of value that's cheap to
get right once and expensive to have silently drift between two places that both matter for
physical safety.

### 4.3.1 Candidate action space — single config, no hand-copied constants (added v1.11, before Phase 6 implementation)

Review flagged a real risk: if the trivial dummy ONNX model's declared shape and the C++
candidate generator's batch size are both hand-typed (`81`) in two separate places, shape
validation only proves the dummy model agrees with itself — a genuine mismatch wouldn't surface
until Phase 8. Fixed by making both derive from one config, with the discretization values
themselves verified against the pinned checkpoint's actual config files (not memory):

- `crowd_nav/configs/policy.config`'s `[action_space]` section, fetched directly at the pinned
  commit: `kinematics = holonomic`, `speed_samples = 5`, `rotation_samples = 16`,
  `sampling = exponential`. This is where the plan's already-pinned §7 assumption ("5 speeds ×
  16 headings + 1 stop") actually comes from — confirmed at the source here, not just asserted.
- `crowd_nav/configs/env.config`'s `[env]` section: `time_step = 0.25` — the propagation
  interval both self and human one-step lookahead use.
- `cadrl.py`'s `build_action_space()`, read directly: speeds are **exponentially**, not
  uniformly, spaced — `speeds[i] = (exp((i+1)/speed_samples) - 1) / (e - 1) * v_pref` for
  `i` in `[0, speed_samples)`; `rotations = linspace(0, 2π, rotation_samples, endpoint=False)`
  for holonomic. Candidate 0 is always the stop action `(0, 0)`; the remaining
  `speed_samples * rotation_samples` candidates are every `(rotation, speed)` pair — 81 total
  with the values above, computed, never hand-typed.
- `cadrl.py`'s `propagate()`, read directly: self (`FullState`) advances under the candidate
  action itself (`px += vx·dt`, `py += vy·dt`; radius/gx/gy/v_pref/theta unchanged for
  holonomic); humans (`ObservableState`) advance under their **own current velocity**
  (`px += vx·dt`, `py += vy·dt`, constant-velocity assumption) — matching §1.4's step 2
  exactly, now confirmed against the source rather than just described in the abstract.
  Worth recording: `predict()` has a `query_env=True` branch (and the pinned `policy.config`
  does set `query_env = true`) that asks the *training-time simulator* for the true next human
  states instead of assuming constant velocity — not usable at deployment (there is no
  simulator to query once this is running against Gazebo/real sensors), so the deployed adapter
  necessarily takes the `else` branch's constant-velocity propagation. Not a new decision, just
  confirming the plan's existing choice is the only one actually available outside training.

**Single config file**, `crowd_nav_policy_adapters/config/policy_adapter.yaml`
(`speed_samples`, `rotation_samples`, `time_step_s`, `max_humans`, `human_radius_m`) is the one
place these numbers live. `generate_dummy_model.py` (Phase 6) reads it to compute the trivial
ONNX model's declared input shape (`candidateCount() × featureDim()`, both computed, not
typed); `DummyAdapter` (Phase 6) and `SarlAdapter` (Phase 8) read the same file at construction
to compute their own `expectedShape()` and to size `buildInputs()`'s batch — so shape
validation is checking the model file against the runtime config, and a drift between them
(someone edits the YAML, forgets to regenerate the `.onnx`, or vice versa) is exactly the kind
of mismatch it's designed to catch.

**Feature vector per candidate**: this project's own raw (pre-rotation) observation vector
(`crowd_nav_observation::ObservationBuilder::build()`, Phase 5, run on each candidate's
propagated `WorldState`) — reused unchanged, not a new format invented for the dummy.
Deliberately **not** the rotated 13-column SARL feature: Phase 5's own findings
(`docs/phase5-findings.md`) explicitly reserved rotation for Phase 8's `SarlAdapter`, and Phase
6 shouldn't quietly implement it a phase early under a different name just because a batch of
candidates happens to flow through here too.

### 4.4 OOD detector — sharpened

The brief flagged its own OOD criteria as under-specified and asked me to say so if they're
too vague to build well. They are, as stated ("crowd size beyond training range" etc. — true,
but not a threshold). Concrete version I'll implement, each independently toggleable and each
its own logged trigger cause:

1. **Crowd size**: `num_humans_visible > max_train_humans` (config, default = SARL's training
   max, typically 5).
2. **Proximity**: nearest human center-to-center distance `< min_train_distance`, derived from
   CrowdNav's own training *reward*, not from bare collision geometry. Its public `env.config`
   (confirmed) sets `discomfort_dist = 0.2` m on top of `robot_radius = human_radius = 0.3` m
   — the policy was never rewarded for tolerating a center-to-center distance below
   `0.3 + 0.3 + 0.2 = 0.8` m, and that's the default threshold. Deriving this from bare
   collision radius (`0.6` m) instead — which is what I originally had here — would set the
   trigger *inside* the policy's own trained comfort margin, where it would rarely fire, since
   the policy actively steers away from that zone during training; it would look like a
   working check while measuring almost nothing. Uses the *training* radius (0.3 m), not the
   robot's physical radius — see the `policy_radius`/`robot_radius` split in §4.3.
3. **Relative speed**: any human's speed, or the robot-human closing speed, exceeds
   `max_train_speed` (config; SARL trains humans at up to ~1 m/s, so a config default a bit
   above that with margin).
4. **Command magnitude**: policy output velocity/turn-rate outside the robot's configured
   physical limits (belt-and-suspenders with the clamp in §5.5 of the brief — this one is a
   pure sanity check, not really an OOD signal).
5. **Perception confidence** (only meaningful when degradation is enabled): if the
   observation builder had to synthesize a value for a dropped-out detection, treat that tick
   as reduced-confidence and count it toward intervention stats even if no other trigger
   fires — this is what makes the noise sweep in §5.8 legible (you can attribute interventions
   to perception quality specifically).

Each of 1–4 is a hard threshold with a config default and a named enum value in the
intervention log (`CROWD_SIZE`, `PROXIMITY`, `RELATIVE_SPEED`, `COMMAND_LIMIT`,
`LOW_PERCEPTION_CONFIDENCE`, plus `COSTMAP_COLLISION` and `KEEPOUT_VIOLATION` from the
forward-sim check and `INFERENCE_TIMEOUT` from the watchdog). That's the full trigger
taxonomy for the "intervention rate broken down by cause" headline metric.

**Stated limit of this detector, per review (§4.8.6 has the full reasoning)**: these criteria
characterize *world-state novelty* - a crowd, distance, or speed unlike training. None of them
can detect an *input-pipeline* mismatch, where the observation fed to the policy was already
wrong before any threshold got to look at it - Phase 8's FOV-filter finding (§4.7) is exactly
that class of bug, and no amount of tuning these five thresholds would have caught it. Worth
keeping in mind before treating a clean OOD-trigger rate as evidence the input pipeline itself
is correct.

### 4.5 Hardware boundary (ros2_control)

Today: `<hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>` in
`crowd_nav_control`'s `.ros2_control.xacro`, with `diff_drive_controller` talking to velocity
command interfaces and position/velocity state interfaces on `left_wheel_joint` and
`right_wheel_joint`. Documented contract for the future ESP32 plugin: implement
`hardware_interface::SystemInterface` exporting the identical interface names/types, register
via `pluginlib`, and swap only the `<plugin>` tag (plus whatever serial/I2C params the ESP32
interface needs) — `diff_drive_controller`, the URDF joint structure, and everything above the
hardware layer is unchanged. Nothing beyond this contract is built now.

### 4.6 Controller plugin design (added v1.13, before Phase 7 implementation)

Rescoped before implementation, per review, in the ways below. This is the first phase
producing a real `nav2_core` plugin — pluginlib registration, lifecycle transitions, and
parameter declaration are three places Nav2 fails silently or with unhelpful errors, and the
`::` vs `/` pluginlib naming inconsistency already hit once in Phase 2
(`docs/phase2-findings.md`) lives in exactly this area. Checked against a real, working
controller plugin's actual source rather than the docs before writing any code:

- **Fallback is a genuinely embedded `nav2_core::Controller` instance, not a hand-rolled
  simplified one** — reusing Nav2's own hardened MPPI rather than reimplementing a worse
  version of it, per the principle carried forward since Phase 2/3/9
  (`docs/phase2-findings.md`, "don't reimplement what the Nav2 stack already does better").
  Precedent found and verified by fetching the actual source (not just the header) of
  `nav2_rotation_shim_controller` at the pinned Humble commit: it holds a
  `pluginlib::ClassLoader<nav2_core::Controller> lp_loader_{"nav2_core", "nav2_core::Controller"}`,
  loads a second controller via `lp_loader_.createUniqueInstance(primary_controller)` (a string
  parameter, `<plugin_name>.primary_controller`), and calls `configure()`/`activate()`/
  `deactivate()`/`cleanup()` on it in lockstep with its own lifecycle calls — passing the
  **same** `name` argument through, so the inner controller's parameters live in the same
  `<plugin_name>.*` namespace as the wrapper's own, not a separate sub-namespace. This project's
  `CrowdNavController` follows the identical mechanism, just with the trigger condition
  inverted: `RotationShimController` delegates to the wrapped controller by default and does
  its own thing (rotate-to-heading) only in a specific geometric condition; `CrowdNavController`
  runs its own logic (policy inference) by default and delegates to the wrapped controller
  (`nav2_mppi_controller::MPPIController`) only on watchdog trip. Concretely: `FollowPath`'s
  `plugin:` value in `nav2_params.yaml` becomes `crowd_nav_controller::CrowdNavController`, and
  a new `FollowPath.fallback_controller_plugin: "nav2_mppi_controller::MPPIController"`
  parameter is added — **MPPI's existing ~30 tuning parameters
  (`FollowPath.time_steps`, `FollowPath.model_dt`, etc.) are kept exactly as configured today**,
  since the embedded instance reads them from the same `FollowPath.*` namespace it always has.
- **`nav2_velocity_smoother` already exists in this project's own bringup, fully configured and
  lifecycle-managed** (`crowd_nav_bringup/config/nav2_params.yaml`'s `velocity_smoother:`
  section — `max_accel: [1.5, 0.0, 3.0]`, `max_decel: [-1.5, 0.0, -3.0]`,
  `smoothing_frequency: 20.0` — already in the `lifecycle_manager`'s managed node list, already
  wired downstream of `controller_server`'s raw `cmd_vel` output before it reaches
  `/diff_drive_base_controller/cmd_vel_unstamped`). Missed on the first read of this plan's own
  §3 Phase 7 text, which described the controller plugin doing "velocity smoothing... /
  acceleration clamping" as if that duty were still unclaimed. It isn't: reimplementing
  acceleration-limited smoothing inside `CrowdNavController` would duplicate what
  `nav2_velocity_smoother` already does correctly, downstream, on every command regardless of
  source (policy or fallback) - exactly the "don't reimplement the stack" principle again.
  `CrowdNavController`'s own job narrows to: hold-last-action across the policy/controller rate
  mismatch, and the watchdog/failover decision. Absolute velocity-magnitude clamping to
  hardware limits (not smoothing - a different concern) is still applied at the source, since a
  bad candidate should never be able to physically exceed the robot's hard limits regardless of
  what downstream smoothing does. This finding directly shapes the failover-transition test
  below: verify the *existing* smoother actually bounds the transition within real accel/decel
  limits, not a from-scratch test of hand-rolled smoothing code that no longer exists.
- **Decision logic is a separate, pure-C++, unit-testable core** (`ControllerDecisionCore`),
  wrapped by a thin `nav2_core::Controller`-derived class that owns only ROS/lifecycle/pluginlib
  concerns - the same split already used successfully for `GroundTruthHumanSource` (Phase 5,
  two-constructor design specifically so degradation/latency logic didn't need a live ROS
  pub/sub to test). `ControllerDecisionCore::decide(now, run_policy_decision)` is called once
  per controller tick; `run_policy_decision` is an injected callable (the real path: build
  candidate batch -> ONNX inference -> `selectAction`; test path: a fake with a controllable
  delay) so the watchdog/hold-last-action mechanics are testable without a live Nav2 node,
  costmap, tf buffer, or ONNX session.
- **Watchdog implementation: a bounded wait, not after-the-fact measurement.** On a tick where a
  fresh policy decision is due, `run_policy_decision` runs on `std::async(std::launch::async,
  ...)`; the calling (controller) thread waits at most `watchdog_window_s` via
  `future::wait_for()` before giving up and using the fallback for that tick - this is a real
  timeout, not "call synchronously, then notice afterward that it took too long," which matters
  because a synchronous call blocks the entire 20 Hz control loop for as long as inference takes
  regardless of any deadline. **Honest limitation, stated rather than glossed over**: C++ has no
  safe way to forcibly terminate a running thread, so a *genuinely hung* (not just slow) call
  leaves an orphaned background thread running to completion rather than being killed outright.
  Real ONNX CPU inference is a bounded deterministic computation (never truly infinite, per
  Phase 0's sub-millisecond-to-low-single-digit-millisecond measurement), so this only matters
  for a hypothetical hang, not the realistic slow-inference risk this phase is built for - a
  process-level supervisor would be needed to cover true hangs, out of scope here, worth
  flagging as a Phase 9/hardening follow-up rather than silently assumed covered.
- **The orphaned-thread case creates a real data race if left unhandled, so it's handled
  explicitly**: `DummyAdapter` (and `SarlAdapter` later) carry mutable per-decision state
  (`last_candidates_`, stashed between `buildInputs()` and `selectAction()`) that is not
  thread-safe against a second concurrent decision. If a decision times out, its background
  thread may still be running against that same adapter instance. Fix: `ControllerDecisionCore`
  tracks whether a previous decision's future is still outstanding and, if so, **does not start
  a new one** even if a fresh decision is due - it stays in fallback until the outstanding
  future resolves (checked with a non-blocking `wait_for(0s)` each tick), then resumes normal
  decisioning. This is also the conservative-correct safety behavior on its own merits: if it's
  not confirmed that a previous inference call has actually finished touching shared state,
  continuing to trust the classical controller rather than gambling on concurrent adapter
  access is the right default, independent of the race-avoidance rationale.
- **Failover-transition done-bar, not just failover-trigger**: per review, "stalling inference
  triggers failover within one watchdog window" checks the *trigger*, not what the robot
  actually does at the handover moment - a held policy command for several ticks followed by an
  abrupt MPPI command can itself be a velocity discontinuity the robot can't execute, and that's
  exactly what `nav2_velocity_smoother` exists to bound. Done-bar item added: with the full
  stack running in Gazebo (open-arena world, `policy_raw` config), inject a stall via a single
  clearly-labeled test-only parameter (`FollowPath.debug_inject_decision_delay_s`, default
  `0.0`, documented as a diagnostic knob, not a production one), and capture both
  `cmd_vel` (`controller_server`'s raw output - where the actual command-source switch is
  visible) and `/diff_drive_base_controller/cmd_vel_unstamped` (post-smoother - what the robot
  actually executes) across the transition. Verify directly, not inferred from Phase 10 results:
  failover is visible in the raw stream at the trigger point, and the smoothed stream's
  tick-to-tick velocity change never exceeds the configured `max_accel`/`max_decel` even across
  that transition.

### 4.7 SarlAdapter design (added v1.15, before Phase 8 implementation)

Rescoped before implementation, per review, in the ways below. This is the highest-uncertainty
phase remaining — everything under it is proven; what's unproven is whether the C++
reimplementation of SARL's candidate-action search matches the Python original. Three things
pinned before writing any adapter code, plus one more found in the process of pinning them.

**Export verified standalone in Python, before any C++ code, per the explicit requirement.**
Loaded the real `il_model.pth` checkpoint into the actual `SARL`/`ValueNetwork` classes
(re-cloned `tkkim-robot/Gazebo-CrowdNav` at the pinned commit), wrapped the network to drop the
non-exportable debug line (`self.attention_weights = weights[0, :, 0].data.cpu().numpy()` -
found by reading `sarl.py` directly; the numpy conversion is on an instance attribute that
never feeds the returned value, so dropping it is safe, not a behavior change), and exported to
ONNX (opset 18, dynamic `batch` and `num_humans` axes - see the no-padding finding below).
Checked three things, all on real varied synthetic joint states (1/3/5/8 humans, several
seeds each), not zeros:
1. Wrapper (PyTorch, debug line dropped) vs. the original checkpoint's `ValueNetwork` (PyTorch,
   debug line intact): **max abs diff = 0.0** - dropping the debug line changes nothing.
2. ONNX Runtime output vs. the wrapper (PyTorch): **max abs diff = 1.9e-7** - the actual export
   fidelity check the requirement asked for, at floating-point noise level.
3. **Batching all 81 candidates in one `(81, num_humans, 13)` call vs. calling the network once
   per candidate with `batch=1`: max abs diff = 2.4e-7.** This is the reference's own real
   calling convention (`multi_human_rl.py`'s `predict()` calls `self.model(rotated_batch_input)`
   once per action, `batch_size=1`) - confirms the plan's "batched inference" design is
   mathematically identical to the reference's one-candidate-at-a-time convention (no
   cross-batch-element coupling in `ValueNetwork.forward()`'s attention/mean pooling, which
   operate per batch index via `view`/`keepdim`), not just "close enough."
Also confirmed the exported model loads and runs correctly under this project's actual pinned
ONNX Runtime **1.20.1** (not just the newer Python-side runtime used for the export check
itself) - a real, previously-bitten class of version mismatch (§1.3).

**Verified network architecture and reward/discount formula against the checkpoint's own
config, not memory**: `crowd_nav/data_sarl/output/policy.config`'s `[sarl]` section -
`mlp1_dims=150,100`, `mlp2_dims=100,50`, `attention_dims=100,100,1`, `mlp3_dims=150,100,100,1`,
`with_global_state=true`, `with_om=false`; `[rl]` `gamma=0.9`. `self_state_dim=6`,
`human_state_dim=7`, `input_dim=13` (`cadrl.py`) - matches the rotated feature layout exactly
(§4.1.1: 6 self fields + 7 human fields). `multi_human_rl.py`'s `compute_reward()`, read
directly: collision (`dist < 0` between propagated self and any human, using `nav.radius +
human.radius`) → `-0.25`; else goal reached (`dist to goal < nav.radius`) → `1`; else
`dmin < 0.2` → `(dmin - 0.2) * 0.5 * time_step`; else `0`. **Worth flagging**: the `0.2`/`0.5`
discomfort constants are hardcoded literally in this function, not read from `env.config`'s
`discomfort_dist`/`discomfort_penalty_factor` keys despite matching them in this specific
config - the C++ reimplementation hardcodes `0.2`/`0.5` too, matching what the code actually
does, not what the config file's naming implies is configurable. Final value:
`reward + gamma^(time_step * v_pref) * network_value`, argmax over all 81 candidates.

**No padding for `SarlAdapter` - a real correctness finding, not a style choice.** Phase 5/6's
`ObservationBuilder` pads to a fixed `max_humans` for `DummyAdapter`'s static-shape ONNX model.
Reusing that padding for `SarlAdapter` would be a real bug: the network's masked-softmax
attention (`scores_exp = torch.exp(scores) * (scores != 0).float()`) excludes a human row only
when its *raw attention score* is *exactly* `0.0` - not a property any padding convention
(ours or otherwise) can guarantee by construction. Feeding padded rows to the real network
risks the padding silently influencing the value estimate instead of being ignored. Fixed by
giving `SarlAdapter` its own unpadded per-human row construction (self 9 fields + each real
human's 5 fields, closest-first for consistency with `ObservationBuilder` but no padding past
the actual count) and exporting/consuming the ONNX model with a **dynamic** `num_humans` axis,
matching the reference's own variable-length behavior exactly rather than approximating it.
`ObservationBuilder` remains `DummyAdapter`-specific; it is not reused here.
**Zero-humans edge case**: the reference never exercises `predict()` with zero humans
(`env.config`'s `human_num=5` is fixed), but this project's own perception degradation model
(Phase 5) can transiently drop every detection in one tick - a genuine, if rare, possibility
here that doesn't exist in the reference's world. Attention pooling over zero humans is
degenerate (softmax normalization divides by a zero sum). `SarlAdapter` handles this as an
explicit, documented special case: skip the network entirely and return the stop action for
that tick - the conservative-safe default when perception has nothing to report, not "drive
blindly." A fuller response belongs to Phase 9's OOD detector (`LOW_PERCEPTION_CONFIDENCE` is
already one of its planned trigger categories, §4.4) - this is a deliberate stopgap, not that.

**Adversarial action-match test, mined from the reference implementation itself, not guessed.**
"Chosen action matches Python SARL" only stresses a wrong discount factor or reward term if the
top two candidates' values are genuinely close - an obviously-best action matches even with a
wrong `gamma`. Rather than hand-pick scenarios and hope, the fixture generator runs the real
`predict()` over many synthetic scenarios, captures the full `action_values` array `predict()`
already stores, and keeps the ones where the top two candidates' values differ by less than a
small fraction of a percent - these are exactly where a subtle discrepancy in the immediate-
reward term or the `gamma^(time_step * v_pref)` discount would surface as a different argmax,
not just a different score. Both adversarial (near-tied) and typical (clear-margin) cases are
checked; the done-bar requires the chosen action to match on both.

**`policy_radius`/`robot_radius` split, asserted directly, not just configured correctly by
construction.** §4.3's split already exists (`CandidateActionSpaceConfig::policy_radius_m`,
wired into `RobotSelfState::radius` since Phase 7's `CrowdNavController::buildWorldState()`) -
but Phase 2's undersized-`robot_radius` bug (§4.3) shows how quietly a wrong radius propagates
silently until something concrete fails. A dedicated test asserts the *actual value reaching
the network* (the self-state radius field in `SarlAdapter`'s raw per-candidate row, post-
rotation) equals `policy_radius_m` (0.3, the training value) and is never equal to
`robot_collision_radius` (0.14, the URDF value) - a regression here would silently feed the
network an out-of-distribution self-radius without any load-time error, the exact quiet-failure
shape Phase 2 already demonstrated once.

### 4.8 Safety supervisor design (added v1.17, before Phase 9 implementation)

Rescoped before implementation, per review, in the three ways the review asked for. Per the
explicit instruction, the first of these (the FOV filter) must be **resolved in code before any
`crowd_nav_safety_supervisor` code is written** - the OOD thresholds below are meaningless if
tuned against an input pipeline that doesn't match what the policy was trained on. The build
order in §3's Phase 9 entry reflects this: perception fix first, supervisor second.

**4.8.1 The FOV/range filter - a two-part decision, not one.** Phase 8 found that the
checkpoint's own source repo restricts and augments the human list inside `JointState.__init__`
before SARL ever sees it (§4.7, `docs/phase8-findings.md`): an 85.2°/12m simulated-D435I FOV
filter, then an unconditional synthetic "dummy" human appended to whatever survives (even an
empty list). These are two independent design questions with different answers:

- **The FOV/range *numbers* themselves: use this robot's real sensor geometry (~180°, i.e.
  ±90° half-angle from robot heading; 8m range), not the reference's 85.2°/12m.** The D435I
  spec is an artifact of a different codebase's simulated camera, unrelated to this project's
  robot. Copying it would trade one arbitrary mismatch (no filter at all) for a different
  arbitrary mismatch (the wrong camera's filter) - neither matches the training distribution
  exactly, since this robot's sensor genuinely differs from a D435I (§1's LiDAR finding: a
  confirmed rear-hemisphere rendering artifact masked at the source, `docs/phase1-findings.md`;
  plus the project's own already-deliberate 8m range cap, §3). Given that neither choice is
  training-faithful, the tie-breaker is honesty about what this robot can actually perceive, not
  numerical proximity to a constant that was never this robot's number to begin with. The
  ~180°/8m figures are already stated as this project's real sensor spec in the README's Known
  Limitations section - reusing them here keeps one honest story about what the robot can see,
  instead of a second, different fictional sensor spec invented just for the policy's input.
- **The dummy-injection-on-empty convention: replicate it, regardless of which FOV numbers are
  used.** This one is not about matching a sensor - it's about matching an untested code path in
  the network itself. Nothing in the reference's own evaluation ever calls `predict()` with a
  genuinely empty human list (`env.config`'s `human_num` is fixed and non-zero, and the
  fallback-dummy branch exists specifically because *some* list is always non-empty by
  construction before `predict()` runs). Masked-softmax attention over zero rows is a divide-
  by-zero in the pooling denominator - not "untrained," but architecturally undefined. Whatever
  this project decides its own FOV/range numbers should be, an empty perceived-human list must
  still become a one-row batch before it reaches the network, matching the *shape* of the
  reference's own defensive convention even though the specific placement numbers (11.9m/21.9m)
  are this reference implementation's own arbitrary choice, not a trained parameter either.
  **This corrects an assumption in v1.15's own changelog entry above**, which expected the zero-humans case
  to route through `LOW_PERCEPTION_CONFIDENCE`. On reflection that's wrong: a genuinely empty
  scene (no humans anywhere nearby, or all correctly outside this robot's real FOV/range) is not
  a degraded-perception event, it's the ordinary, expected case in an open-arena world with no
  pedestrians nearby - flagging every such tick as low-confidence would make that OOD trigger
  fire constantly for entirely legitimate scenes, which is worse than not having the signal at
  all. `LOW_PERCEPTION_CONFIDENCE` stays scoped to the degradation model's own dropout event
  (§4.8.5 below); dummy-injection is a separate, silent, architectural necessity.

**Where each lives**: the FOV/range restriction is a perception-layer concern
(`crowd_nav_perception::GroundTruthHumanSource::degrade()`, extending the existing
`DegradationParams::max_range_m` machinery with a new angular-FOV term relative to robot
heading), because it's a property of *what the robot's sensor can see*, shared by every
consumer of `HumanStateSource` - including this same supervisor's own `CROWD_SIZE` OOD check
below, which must only count humans the robot could actually perceive, not the ground-truth
count. Dummy-injection lives inside `SarlAdapter::buildInputs()` specifically, *not* the shared
perception layer, for the opposite reason: a supervisor `CROWD_SIZE` check that counted a fake
placeholder human as real would be corrupted by construction. `GroundTruthHumanSource` currently
tracks only robot position (`robot_xy_`, `setRobotPose(x,y)`) - the angular check needs heading
too, added via `setRobotPose(x,y,theta)` and `tf2::getYaw()` on the existing
`/ground_truth/robot_pose` subscription (same extraction pattern already used in
`CrowdNavController::buildWorldState()`).

One more honest gap this surfaces: `DegradationParams` defaults are all off (oracle passthrough,
by explicit design, per its own header comment) and `CrowdNavController::configure()`'s
production perception wiring (added Phase 8) constructs a bare default-initialized
`DegradationParams` - meaning **today, in production, neither the range cap nor the (new)
FOV term is actually enabled**, even though `max_range_m` has existed as a capability since
Phase 5. Capability existing and capability *active* are different claims; this phase makes the
production `CrowdNavController::configure()` call explicitly set both
(`perception_fov_half_angle_rad` default `M_PI/2`, `perception_max_range_m` default `8.0`, both
new `FollowPath.*` parameters, not hardcoded literals, matching how `pedestrian_topic` etc. are
already exposed) rather than leaving them as a theoretical, never-exercised code path.

**Verification required at implementation time, not assumed**: attention pooling over exactly
one human row degenerates only if that row's raw attention score is exactly `0.0` (§4.7's
masked-softmax finding); for a single injected dummy this is a real, checkable numerical
question, not a hypothetical - confirm empirically against the actual ONNX network with the
dummy's specific injected values that the score isn't exactly zero (and thus the softmax
denominator isn't zero), the same "verify, don't assume" discipline Phase 8 already applied to
the padding question. If it is exactly zero, perturb the injected placeholder's position by a
fixed, documented, non-round offset rather than trusting a coincidence not to recur.

**4.8.2 Same costmap, same footprint - structural, not asserted.** Per review: two components
disagreeing about world state produced Phase 3's infinite-recovery-loop bug (a global planner
that didn't know about a keep-out zone the local costmap did); the same shape recurs if the
supervisor forward-simulates against different costmap state than MPPI plans against. The fix
here is structural rather than a runtime cross-check: `SafetySupervisor` is a plain C++ class,
**instantiated directly by `CrowdNavController::configure()`**, constructed with the *exact same*
`costmap_ros_` shared pointer already held there - the identical pointer already passed to
`fallback_controller_->configure(parent, name, tf, costmap_ros)` two lines below where it would
be constructed. There is no second subscription, no independent costmap topic, and no "assert
these match" check to write, because there is only one `Costmap2DROS` instance in this process
and both the embedded MPPI and the new supervisor read directly from it. This also answers the
"same footprint" half of the ask without needing a footprint at all: this project's costmap
uses a scalar `robot_radius` (`0.14m`, `nav2_params.yaml`), not an explicit footprint polygon,
and MPPI's own `ObstaclesCritic` is already configured `consider_footprint: false` (confirmed
in Phase 7's own research into this project's actual MPPI config) - meaning MPPI itself already
treats "cost at a point" (relying on the inflation layer, which was computed *from* that same
`robot_radius`) as its own collision signal against this costmap. The supervisor's forward-sim
check (§4.8.3) does the same plain point-cost lookup against the same costmap, which is the
most consistent choice available: not a second, independently-defined footprint check that
could quietly drift from MPPI's own, but literally the same primitive MPPI already relies on.

**4.8.3 Bounded forward-simulation, fixed by construction.** Per review and §1.7: the horizon
and step count are fixed at implementation time, not adaptively refined, so the supervisor's
worst-case per-tick cost is a known constant rather than a typical one. Concretely: 4 steps at
`time_step_s = 0.25s` (reusing `CandidateActionSpaceConfig::time_step_s`, already loaded by
`CrowdNavController` - one source for this number, not a second hand-typed one), i.e. a fixed
1.0s look-ahead - four `Costmap2D::getCost()` lookups per tick, a bounded, trivial cost. Each
step forward-integrates using the **candidate's raw holonomic `(vx, vy)`** as a constant-velocity
world-frame displacement (`x += vx * dt`, `y += vy * dt`), *not* the diff-drive-projected twist
`toTwistStamped()` would actually send. This is a deliberate approximation, not an oversight:
the diff-drive projection only ever *shrinks* effective forward progress relative to the
holonomic target (`cos(heading_error) <= 1`, plus the existing velocity clamp) and only
gradually turns the robot toward the target heading - so simulating with the full holonomic
vector is always a conservative over-estimate of how far the robot actually travels toward a
given point in a given step. The failure direction of this approximation is one-sided: it can
produce a false-positive stop (rejecting a command the real, more cautious diff-drive-
constrained robot would have executed safely), never a false negative (approving a command whose
real, further-foreshortened trajectory turns out more dangerous than what was checked). Stated
explicitly as an accepted, conservative-by-construction simplification, not silently assumed
equivalent to the real kinematics - reusing `toTwistStamped()`'s own projection here instead
would require either baking a stale twist into the held-command state (a regression against
Phase 7's hold-last-action design, which re-projects the held holonomic vector against the
*current* pose every tick) or re-running the projection on every 20Hz control tick instead of
only at 4Hz decision points, which the watchdog boundary isn't scoped to cover.

Per-step check, reusing Nav2's own cost-value semantics (`nav2_costmap_2d/cost_values.hpp`)
rather than inventing a new threshold: `worldToMap()` first - if the point falls outside the
local costmap's current rolling window (3m x 3m, `nav2_params.yaml`), that step is *unverifiable*
and treated as not-unsafe (logged, not silently ignored) rather than either extreme; asserting
danger for "off the edge of a 1.5m-radius rolling window" would manufacture false positives
near the window boundary during ordinary driving, and the 4-step/1.0s horizon at this robot's
configured speeds keeps this rare in practice, not eliminated by assumption. When the point
*is* in bounds: cost `>= INSCRIBED_INFLATED_OBSTACLE` (253) **and** cost `!= NO_INFORMATION`
(255) is unsafe - the exclusion matters because `NO_INFORMATION == 255 > 253` would otherwise
also satisfy the raw `>=` comparison, incorrectly flagging "haven't mapped this cell" the same
as "this cell is lethal." `FootprintCollisionChecker<CostmapT>` (confirmed to exist,
`footprint_collision_checker.hpp`) is deliberately *not* used here, even though it's the more
general primitive - it exists to sweep an actual footprint polygon, which this project's
costmap configuration doesn't have (`consider_footprint: false` above); using it would mean
defining a second footprint representation for the supervisor alone, precisely the kind of
drift-risk §4.8.2 exists to avoid.

**Cause labeling, not the safety decision itself**: the pass/fail decision above uses one
authoritative source (the shared local costmap). A *separate*, secondary, best-effort
subscription to `/keepout_filter_mask` (already published by `crowd_nav_zones`' mask server,
confirmed at `crowd_nav_zones/launch/zones.launch.py`) is used purely to label *which* of
`COSTMAP_COLLISION` or `KEEPOUT_VIOLATION` goes into the intervention log, since
`nav2_costmap_2d::KeepoutFilter` (Phase 3) writes the keep-out mask directly into the same local
costmap's cost values as `LETHAL_OBSTACLE` - a single cost lookup can't tell which cause
produced a given lethal cell. If this secondary lookup is stale or unavailable, the log
degrades to the generic `COSTMAP_COLLISION` label; it never changes whether the command is
rejected, since it isn't consulted for that decision at all.

**4.8.4 Two-tier fallback, not one - a thread-safety constraint, not a style choice.** Per
§4.6, the watchdog (`ControllerDecisionCore`) already runs `run_policy_decision` on a background
thread via `std::async`, bounded by `wait_for()`; a genuinely hung (not just slow) call leaves
that thread running to completion, since C++ has no safe forced termination. If the supervisor's
rejection response called `fallback_controller_->computeVelocityCommands()` (the embedded MPPI)
from *inside* `run_policy_decision`, an orphaned background thread past the watchdog window
could call into the same `fallback_controller_` instance the main thread *also* calls into once
`decide()` times out - two concurrent calls into one `nav2_core::Controller` instance, the exact
class of data race §4.6 already went out of its way to prevent for `PolicyAdapter`'s own mutable
state. So the two triggers get two different responses, not one:
- **Watchdog timeout** (inference too slow or hung): unchanged from Phase 7 - `decide()` returns
  `kFallback`, and `CrowdNavController::computeVelocityCommands()` (main thread only, never the
  background one) calls the embedded MPPI. This is the "give it to a planner that can afford to
  think longer" response.
- **Supervisor rejection** (the policy answered in time, but the forward-sim or an OOD check
  says don't trust or send this): checked *inside* `run_policy_decision`, after
  `selectAction()` produces a candidate, still on the background thread, still inside the same
  watchdog-timed window (§1.7). The response is a direct, hard stop (`{0.0, 0.0}`) returned from
  that same closure - never a call into `fallback_controller_`. This is deliberately not "hand
  it to MPPI instead": a stop cannot make the immediate-horizon risk the supervisor just detected
  *worse* (a stationary point re-checks trivially safe at every future forward-sim step, barring
  a moving obstacle closing in regardless of the robot's own command, which no choice of command
  prevents anyway) - stated as the honest scope of what this response guarantees (arresting
  policy-induced risk this tick), not a general recovery mechanism (Nav2's own recovery behaviors
  already own resolving a pre-existing bad state, out of scope here).

**4.8.5 OOD criteria, implemented against what's actually available.** §4.4's five criteria,
plus one correction found while wiring `LOW_PERCEPTION_CONFIDENCE` to a real signal: `degrade()`
currently returns `std::optional<HumanObservation>` and a dropped-out detection is simply absent
from `getHumans()`'s result - there is no signal anywhere in `WorldState` today that a dropout
happened versus a human simply not being nearby. Fixed with the smallest change that makes the
trigger real rather than a permanently-unreachable enum value: a new non-pure virtual,
`HumanStateSource::numDegradedLastCall()` (default `0`, so `TrackedHumanSource`'s stub and every
existing call site are unaffected), overridden by `GroundTruthHumanSource` to report how many
raw detections its *dropout* model discarded on the most recent `getHumans()` call - FOV/range
exclusions do not count toward this, since a human legitimately outside this robot's real sensor
coverage is normal operation, not degraded confidence, and conflating the two would make this
trigger fire on every empty-scene tick regardless of perception quality. `CROWD_SIZE`,
`PROXIMITY` (0.8m, §4.4), `RELATIVE_SPEED`, and `COMMAND_LIMIT` (checked against the candidate's
raw holonomic speed, no twist-projection needed) are otherwise implemented exactly as §4.4
specifies. Each of the 5 OOD causes and the 2 forward-sim causes reject the tick the same way as
§4.8.4's supervisor-rejection path (hard stop, not MPPI) - `INFERENCE_TIMEOUT` is the only cause
that uses the embedded-MPPI path, since it's the only one that fires on the main thread via the
existing watchdog, not inside `run_policy_decision`.

**4.8.6 What this supervisor does and doesn't catch - stated explicitly, per review.** The OOD
detector's 5 criteria (§4.4) characterize *world-state novelty* relative to the training
distribution (too many humans, too close, too fast, a suspicious command, a degraded tick) - none
of them can detect an *input-pipeline* mismatch, where the world state the policy reasons about
was already wrong before any threshold got a chance to look at it (Phase 8's FOV-filter finding,
§4.7, is exactly this class of bug: the reference itself pre-processes its input before the
network sees it, invisibly to any downstream OOD check). §4.8.1's fix resolves the *specific*
pipeline mismatch found so far; it does not make the OOD detector newly capable of catching
*future* pipeline bugs of the same shape - that would need a different mechanism (e.g.
differential testing against the reference implementation, which is what actually caught this
one), not a runtime threshold. Recorded here so this isn't rediscovered as a surprise limitation
later, the same way the LiDAR FOV and IL-only-checkpoint limitations are recorded in the
README's Known Limitations section rather than left implicit.

**4.8.7 `InterventionEvent` - the home §2's forward-note already reserved.** A small message,
`crowd_nav_safety_supervisor/msg/InterventionEvent.msg`: timestamp, `cause` (the 8-value
taxonomy in §4.4's closing paragraph), the rejected candidate command, and the substituted
command actually sent. Published on `/intervention_events` (Phase 10's evaluation harness is
the eventual consumer, for the "intervention rate broken down by cause" headline metric - a real
message, not a placeholder for one, is what a future harness can actually subscribe to and log
to CSV) *and* logged via `RCLCPP_WARN` at the point of rejection, edge-triggered per cause the
same way Phase 7's source-switch logging already is, for interactive visibility without needing
to echo a topic during manual testing.

### 4.9 Evaluation harness design (added v1.19, before Phase 10 implementation)

Rescoped before implementation, per review. This is the phase the whole project has been
building toward, and the one most likely to produce results that can't be interpreted -
review's three requirements are all aimed at that specific risk, not at the harness's mechanics.

**4.9.1 Pilot before the full matrix.** Before any seeded run, one scenario × one seed × all
three configs (`baseline_mppi`, `policy_raw`, `policy_supervised`), end to end, checked by hand:
each episode terminates (not hangs), the CSV row has plausible values (nonzero duration, a real
path length, `policy_supervised`'s intervention columns populated and the other two configs'
left at zero/NA), and nothing in the harness's own orchestration (launch, goal-sending,
zone-placement for the named scenario below, termination detection, teardown between episodes)
silently swallows a failure. This is deliberately the *first* thing built and run, before the
scenario suite is filled out to its full size - the point is finding a harness bug after twenty
minutes, not after an overnight unattended run. **Done for this sub-step specifically**: three
pilot episodes complete, their CSV rows inspected by hand and judged sane, before any seed count
above one is attempted.

**4.9.2 N and the noise-sweep points, decided now, not after looking at results.** Per review:
choosing sample size after glancing at the numbers is how a results table stops being
trustworthy. Committing here, before any episode has run:
- **`N = 8` seeds per (scenario × config × pedestrian-mode) cell** for the core matrix. 2
  scenario families × 3 configs × 2 modes × 8 seeds = 96 episodes. Reasoning: large enough to
  report a real distribution (mean *and* spread, per the existing done-bar wording), small
  enough to be tractable in one extended session given each episode is a full Gazebo/Nav2
  simulation, not a cheap unit test. If session time makes the full 96 infeasible, that will be
  stated as an explicit, reasoned scope reduction in `docs/phase10-findings.md` (e.g. "N=8
  planned, N=5 actually completed, here's why") - never silently reduced, and never chosen
  *because* partial results looked a particular way.
- **Noise sweep: `dropout_prob ∈ {0.0, 0.1, 0.2, 0.3, 0.5}` (5 points), `policy_supervised` only,
  `open_arena`/`reactive` only, N=8 seeds per point (40 episodes).** Scoped to one axis and one
  scenario/config/mode combination deliberately, not the full matrix crossed with the sweep -
  the sweep's stated purpose (§4.4/§4.8.5) is attributing interventions to perception quality
  specifically, which needs `LOW_PERCEPTION_CONFIDENCE`'s own trigger varied in isolation, not a
  combinatorial explosion of every axis at once. `open_arena`/`reactive` isolates the perception
  effect from the navigability difficulty the depot family already introduces on its own (mixing
  the two would make it impossible to attribute a rate change to noise vs. terrain). `sigma_pos_
  m`/`sigma_vel_mps`/`latency_s` sweeps are real, legitimate follow-on work but out of scope for
  this phase - noted as a Phase 12+ candidate rather than attempted here under time pressure.
  This sweep is also the first live exercise of `LOW_PERCEPTION_CONFIDENCE` at all (§4.8's own
  addendum found it was never observed live through Phase 9) - the RNG-substream isolation from
  Phase 5 (§4.2) is exactly what makes this a controlled sweep rather than "humans on different
  paths at each setting too."

**4.9.3 A named, permanent keep-out-block scenario, not an ad-hoc check.** Phase 9's live
session confirmed the rejection mechanism works (a real, non-engineered `COSTMAP_COLLISION`
episode) but never cleanly isolated a keep-out-zone-specific trigger - ad hoc zone placements
either blocked the global planner upstream of the supervisor or left room to route around them.
Per review, the fix is a scenario in the suite, not a repeat of that improvisation: `depot_
keepout_block` - a fixed start/goal pair in the depot world, in a corridor with no viable
detour, with a zone placed (via `AddZone`, at scenario setup, matching Phase 3's own successful
methodology - a pre-measured corridor-width zone, not a guess) directly across the only path.
Run once per config (not part of the N=8 statistical matrix - its job is a decisive
demonstration plus a permanent regression test, not a distribution), with an explicit expected
outcome per config, stated here so a future run that doesn't match it is caught immediately:
- **`baseline_mppi`**: succeeds via a detour, or times out if the corridor genuinely has none -
  either way, `KeepoutFilter` is wired into both costmaps (Phase 3), so MPPI is aware of the
  zone and never drives through it.
- **`policy_raw`**: **expected to violate the zone.** SARL's observation model has no concept of
  a static keep-out region - it reasons about humans, not costmap obstacles - so nothing in the
  policy itself prevents driving straight through. This is the sharpest, most direct
  demonstration of the project's own motivating premise (a learned policy doesn't know about
  hard constraints; something else has to enforce them), and the scenario is designed
  specifically to make that failure visible, not to flatter the baseline comparison.
- **`policy_supervised`**: rejects the candidate that would enter the zone and logs
  `KEEPOUT_VIOLATION` (not the generic `COSTMAP_COLLISION` - this scenario is also the first
  live test of that specific cause-label path, §4.8.3's secondary lookup). The harness's own
  termination checker gains a fourth outcome category for this scenario specifically:
  `keepout_violation` - the episode ends the moment ground-truth robot pose actually enters the
  zone polygon (only reachable by `policy_raw`, given the other two configs are expected never
  to enter it), rather than running the episode out to an uninformative timeout.

**4.9.4 Expected result shape, stated before running anything, per review.** `policy_supervised`
underperforming `baseline_mppi` on path length/duration in the depot family is the *expected*
result, not a bug to chase: the policy is out-of-distribution in a structured environment by
construction (trained on CrowdNav's open, unstructured scenarios), which is exactly why the
open-arena family exists as a contrast. A results table where the supervised config wins
everything would be more suspicious than one that wins on safety and loses on efficiency -
worth stating in `docs/phase10-findings.md` explicitly rather than letting a reader assume an
unqualified win was intended. The headline number is the **intervention-rate-by-cause
breakdown, compared across the two scenario families** - near-zero in open-arena and materially
higher in depot is a clean, honest characterization of transfer failure (a real finding), not
just supporting evidence for "the supervisor works." Report that comparison prominently, not as
a footnote to the collision/success-rate table.

**4.9.5 If depot results look catastrophic, isolate the FOV decision before concluding
anything about the policy.** §4.8.1 chose this robot's real ~180°/8m sensor geometry over the
reference checkpoint's D435I-specific 85.2°/12m for the production default - a deliberate,
justified call, but it means Phase 10 is the *first* time the policy runs against a narrower
FOV than it saw in training, and a bad depot result could mean either "the policy doesn't
transfer to structured environments" or "the narrower FOV alone hides too much of the scene in
tight corridors." These are different claims requiring different fixes, and depot's own tight
geometry makes the second one plausible enough to check before assuming the first. If depot
numbers look bad enough to warrant investigation: one supplementary run (not part of the N=8
matrix, not re-litigating N) with `FollowPath.perception_fov_half_angle_rad`/`.perception_max_
range_m` set to the reference's own 85.2°/12m (already-exposed parameters, §4.8.1 - zero new
code required) against the same depot scenario/seeds. If that run looks meaningfully better,
the FOV choice - not the policy itself - is implicated, and that's a distinct, honestly-reported
finding, not something to quietly fold into "the policy struggles in depot."

**4.9.6 AMCL covariance alongside interventions, per §5's original (unimplemented until now)
risk note.** §5 risk #4 flagged logging AMCL's own covariance alongside every supervisor
decision, to separate "the supervisor was right to intervene" from "localization was degraded
and misled it," but nothing before this phase actually built that correlation. The harness's
metrics collector subscribes to both `/intervention_events` and `/amcl_pose` and writes a
secondary `interventions.csv` (one row per event: episode id, timestamp, cause, the rejected/
sent velocities already in the message, and the AMCL pose covariance trace at that timestamp) -
not a new field on `InterventionEvent` itself (that message is already shipped and tested;
correlating two already-published topics by timestamp in the harness is the right layer for
this, not a schema change to a component that isn't the harness's own).

**Found while designing the harness, not anticipated going in: `policy_raw` had no way to
actually mean "no supervisor."** `CrowdNavController` unconditionally ran the supervisor check
whenever it was the active plugin - there was no config axis distinguishing "policy with the
safety net" from "policy without it," which the whole `policy_raw` vs. `policy_supervised`
comparison depends on being a real difference, not just a label. Fixed with a new
`FollowPath.supervisor_enabled` parameter (default `true`), launch-time only (not exposed via
`onSetParameters` - each evaluation episode gets a fresh launch, not a live reconfiguration
mid-run): when `false`, `run_policy_decision` returns the adapter's raw candidate directly, no
OOD/forward-sim check, no `InterventionEvent`. `policy_raw` sets it `false`; `policy_supervised`
leaves it at the default.

**Scenario suite, metrics, and package layout**: `crowd_nav_evaluation` (§2) - scenario
definitions (world, start/goal, pedestrian mode, seed, and for `depot_keepout_block` the zone
spec) as data, not hardcoded per-scenario branches in the runner; a harness runner that launches
the stack per episode (reusing `amcl.launch.py`/`pedestrians.launch.py` as-is, not a parallel
launch mechanism), sends the goal, watches for termination (success/collision/timeout, plus
`keepout_violation` for the named scenario), and tears down cleanly between episodes (reusing
`scripts/ros2_teardown.sh`'s SIGINT-then-SIGTERM-then-SIGKILL discipline, not a fresh ad hoc
kill sequence); a metrics collector producing per-episode `episodes.csv` (scenario family, mode,
config, seed, outcome, duration, path length, minimum human distance, intervention count total
and by cause) and the `interventions.csv` above; plots (variance/distributions, not just means,
per the existing done-bar wording) generated from the CSVs, not computed inline during the run.
**Done:** the full matrix (96 core episodes + 40 noise-sweep episodes + the named scenario per
config) runs unattended after the pilot passes, producing both CSVs and plots; the intervention-
rate-by-cause comparison across scenario families and the `depot_keepout_block` per-config
outcomes are both reported explicitly in `docs/phase10-findings.md`, not left implicit in a
CSV nobody reads.

---

## 5. Riskiest parts, ranked, with fallbacks

1. **World scale (tugbot_depot vs 190 mm robot)** — highest risk because it can silently
   invalidate the entire "structured depot" evaluation family if unaddressed. Fallback:
   custom-authored correctly-scaled depot world (§1.1); tugbot_depot demoted to demo-only.
2. **SARL export architecture** — de-risked twice: the Phase 0 spike (§1.4) verifies the
   export itself early, and Phase 6's synthetic `DummyAdapter` proves the ONNX/controller
   plumbing before Phase 8 has to touch SARL's actual search logic, so a Phase 8 failure can
   only mean the search reimplementation is wrong, not "something in the pipeline is broken."
   If the numpy line isn't the only non-exportable piece, fallback is tracing
   (`torch.jit.trace`) instead of `torch.onnx.export`, which tolerates more Python-side
   flexibility at the cost of losing dynamic-shape generality (acceptable here since the
   candidate-action batch size is fixed by config anyway).
3. **Control-loop tick budget / watchdog correctness** (§1.7) — fallback if the supervisor
   check itself proves too slow to fit safely: run the supervisor's forward-sim on a bounded
   fixed-horizon, fixed-step lookup-table-style check rather than a general dynamics
   integrator, and if even that doesn't fit, move the supervisor to run one tick behind
   (validate the *previous* command before it's superseded) rather than in the hot path —
   documented as a fallback, not the default.
4. **AMCL degradation in crowds** (§8 of the brief) — mitigate by logging AMCL's own
   covariance alongside every supervisor decision, so post-hoc analysis can separate
   "supervisor was right to intervene" from "localization was degraded and misled the
   supervisor." Not solved, but measured — I don't think this is fully solvable within this
   project's scope, and the brief doesn't ask for a fix, just consideration.
5. **ONNX Runtime packaging inside colcon** — de-risked by vendoring our own minimal package
   instead of depending on an externally-maintained one of unconfirmed Humble compatibility
   (§1.3). Fallback: `ros-industrial/epd_onnxruntime_vendor` as a second reference
   implementation if our own CMake wrapper hits an unexpected snag.
6. **Pedestrian mirror-node drift** (replaces the original "HuNav non-reactive gap" risk, which
   no longer applies now that HuNav is dropped entirely — §1.2, v1.7) — the visual-only Gazebo
   actor mirror node (off by default) reads the authoritative ROS pedestrian node's published
   state and moves Gazebo actors to match; if it lags or desyncs, what's *rendered* could
   diverge from what the simulation actually did. Mitigated by design: the mirror never feeds
   anything back into the simulation and nothing downstream (`GroundTruthHumanSource`, the
   observation builder, the evaluation harness) reads from it — they all consume the
   authoritative node's topic directly, so a drifting mirror is a visualization bug, not a
   correctness bug. Still worth a sanity check once built (mirror position vs. authoritative
   position at a few sampled ticks) rather than assuming the design guarantee holds in practice.
7. **Reduced 8 m LiDAR interacting with costmap/MPPI/AMCL sizing in a large depot** — mostly
   dissolves if the world-scale fallback in §1.1 lands on a compact custom world (8 m comfortably
   covers a small floor plan); only a live risk if tugbot_depot ends up used at native scale
   for the actual evaluation, which is precisely what §1.1's decision tree tries to avoid.
8. **`gpu_lidar` rear-hemisphere self-detection artifact** (Phase 1, `docs/phase1-findings.md`)
   — confirmed sensor-rendering-internal, not a modeling bug, and not fixed by standoff height
   or switching sensor type. Fallback: mask the known-bad ~176° sector in the observation/
   perception layer (a software workaround, not a real fix) if no renderer-level solution turns
   up during Phase 2. Real risk to AMCL/costmap quality if left unaddressed.
9. **No trained SARL checkpoint upstream** (§1.8) — mitigated by using
   `tkkim-robot/Gazebo-CrowdNav`'s matching-config pretrained checkpoint instead of training
   from scratch. Fallback if it doesn't validate in Phase 0: train from scratch using
   upstream `vita-epfl/CrowdNav`'s own `train.config` (3,000 IL + 10,000 RL episodes), run as
   a background job starting day one of Phase 0 so it's never discovered as a Phase 8 blocker.

---

## 6. Cuts, redundancies, and things I think are over-scoped as literally stated

- **SLAM mode wired into the evaluation harness**: the brief correctly scopes evaluation to
  AMCL-only for repeatability. I'm treating SLAM mode as a launch-file demo with no metrics
  collection, not a second fully-integrated eval path — building full harness support for a
  mode you've already said isn't used for measurement would be pure churn.
- **HEIGHT/OracleNavAdapter as code now**: the brief explicitly defers these ("come later
  through their own adapters"). Building empty stub classes for them today is speculative
  scaffolding with no test coverage behind it. I'll define the `PolicyAdapter` interface
  generally enough to fit them (per §6 of the brief's own reasoning about heterogeneous
  graphs vs flat vectors) and write the "how to add a new policy" README walkthrough using
  HEIGHT as the worked example — as documentation, not code. This satisfies the reusability
  requirement without maintaining dead code.
- **Gazebo-in-CI as a per-PR gate**: full Gazebo + Nav2 + HuNav simulations on every commit is
  expensive and flaky (headless GPU/rendering quirks, timing-dependent flakiness), and isn't
  what most of this project's logic needs — the observation builder, schema adapters,
  supervisor geometry, OOD thresholds, and degradation model are pure computation with no
  simulation dependency, and that's where correctness bugs actually live. Per-PR CI stays
  build + unit tests + lint. Revised after review, though: full sim-in-CI entirely was too
  blunt a cut. A single nightly/manually-triggered workflow that launches the full stack and
  drives one goal to completion is cheap and catches a different, real failure class — launch
  file and parameter-schema rot — that unit tests structurally cannot catch. Added as Phase 11;
  it's a smoke test (did it run to completion), not a behavioral correctness gate, and it
  doesn't block PRs.
- **"Service or topic interface" for zone CRUD**: implementing both is redundant. One service
  (`AddZone`/`RemoveZone`, request/response with success/failure) is strictly better for CRUD
  semantics than a fire-and-forget topic, and it's what the zone-manager node exposes. The
  mask *republish* is still topic-based (that's the KeepoutFilter's own mechanism, a
  different concern from the CRUD API).
- **Occlusion-based dropout**: the brief itself marks this "optional" in §5.4. I'm keeping it
  in scope but explicitly last-priority — a simple costmap raycast, not full 3D occlusion
  reasoning — and it's the first thing to cut if Phase 5 runs long.
- **(Reversed after review) Isotropic covariance on human position estimates**: v1 of this
  plan used a single scalar here, reasoning that a full covariance matrix was over-engineering
  for synthetic noise. On reflection that's the wrong place to economize: the struct shape is
  what's expensive to change later (it touches every adapter and the observation builder),
  not the two extra doubles now. `HumanObservation` carries a full symmetric 2×2 (§4.1); only
  the diagonal is populated until `TrackedHumanSource` needs the off-diagonal for a real
  tracker. This is cheap insurance, not scope creep.

Nothing else in the brief reads as redundant or over-engineered to me — the two-scenario-family
requirement, the versioned schema documentation, the per-cause intervention breakdown, and the
noise sweep are all load-bearing for the results meaning anything, and I'd resist cutting any
of them.

---

## 7. Assumptions log (things I decided rather than asked, per your standing instruction)

- ONNX Runtime version pinned to **1.20.1** (CPU-only Linux x64 prebuilt) — revised from the
  original 1.17.3 plan after Phase 0 found 1.17.3 hard-rejects the ONNX IR version this
  machine's PyTorch actually produces. See §1.3.
- SARL action-space discretization defaults to the original paper's config (5 speeds × 16
  headings + 1 stop). **Kinematics mode is holonomic**, confirmed from the checkpoint's own
  `policy.config` (§1.8/§1.9) — corrected from this plan's original "unicycle" assumption.
  `SarlAdapter` converts the holonomic output to `(v, ω)` for our diff-drive robot; see §1.9.
- `policy_radius` (fed to the network) defaults to CrowdNav's training value, 0.3 m;
  `robot_radius` (used for actual collision/costmap geometry, and required for the Phase 9
  supervisor too) is the `robot_collision_radius` xacro property in
  `crowd_nav_description/urdf/nvis_3302ard.xacro` — 0.14 m, the chassis diagonal plus a small
  margin, not the smaller face-normal half-width (a real bug, found and fixed in Phase 2) —
  see §4.3.
- SARL weights sourced from `tkkim-robot/Gazebo-CrowdNav`'s `data_sarl/output/rl_model.pth`
  (config-matched, MIT-licensed lineage via upstream `vita-epfl/CrowdNav`) rather than
  training from scratch — see §1.8. Both repos will be credited in the README next to the
  vendored/converted model file.
- `max_vel_x` is a single config parameter defaulting to 1.0 m/s (policy-training-matched
  regime), with the 0.3 m/s hardware-matched value documented in MEASUREMENTS.md as the value
  to switch to if/when physical deployment happens — per §7 of the brief.
- Git repository initialized locally now; no remote configured, no commits made yet (waiting
  for you to confirm before any push, per standing safety practice around shared state).

---

## Next step

Waiting on your go-ahead to start Phase 0. Nothing destructive or irreversible happens in
Phase 0 — it's measurement and a couple of throwaway spike scripts — so I'll proceed directly
into it once you confirm, unless you want to redirect anything above first.
