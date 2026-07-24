# Hard adversarial audit

Commissioned after Phase 11 closed the project, before writing `explanation.pdf`. Assume
something is wrong and go find it - not a review pass. Findings are tagged by severity
(critical / significant / minor / cosmetic) and by disposition (fixed / deferred / won't-fix,
with reasoning). Per explicit standing instruction: if a finding invalidates a headline number,
that is reported as-is, not reframed as "defensible under a different reading."

This document is written incrementally as the audit proceeds, not compiled after the fact.

## Summary of findings

| # | Finding | Severity | Disposition |
|---|---|---|---|
| §1.1 | 4 of 8 OOD trigger causes (CROWD_SIZE, RELATIVE_SPEED, COMMAND_LIMIT, INFERENCE_TIMEOUT) never fired across the 139-episode matrix; 3 are structurally unreachable given the actual scenario/config values, not just rare | CRITICAL | Fix in Part 2 |
| §1.2 | Topic liveness: all project-dependency topics confirmed live; one methodology false alarm during the audit itself, resolved | none | No action |
| §1.3 | `WorldState.robot` (map frame) and `WorldState.humans` (world frame) are different, non-equal frames combined in `buildWorldState()` - affects every relative robot-human distance computed by the policy and the `PROXIMITY` OOD check since Phase 7 | **CRITICAL** | **Fix in Part 2, highest priority** |
| §1.4 | `policy_radius`/`robot_radius` split | none | No action, already correctly guarded |
| §1.5 | Float32/double precision | none | No action, only the one already-documented instance exists |
| §1.6 | Seeded determinism | none | Re-confirmed holding, after 2 self-diagnosed test-methodology false alarms |
| §1.7 | Executable-bit git-mode sweep | minor | 4th occurrence of this bug class found (`.github/scripts/run_smoke_test_episode.py`), fixed |
| §2 | Results integrity - every headline number re-derived from CSVs | none | All match exactly; no arithmetic/transcription errors |
| §3 | Contamination (rogue run / pilot data) | none | None found |
| §3.5 | Test quality: `buildWorldState()` has zero unit coverage; `COMMAND_LIMIT`'s own test uses an unreachable candidate value | SIGNIFICANT | Add integration + reachability-aware tests in Part 2 |
| §4 | Documentation drift | none | Spot-checked, none found |
| §5 | Code quality | none | None found at reasonable-depth pass |

Two CRITICAL findings (§1.1, §1.3) and one SIGNIFICANT finding (§3.5, which follows directly
from §1.3) drive Part 2. Everything else: no action needed, or already minor/fixed inline.

---

## 1. Correctness

### 1.1 OOD trigger reachability - all 8 causes, individually

Real occurrence counts across the full 139-episode Phase 10 matrix (`episodes.csv`, all
`intervention_count_cause_N` columns summed):

| Cause | Count | Status |
|---|---|---|
| CROWD_SIZE (0) | **0** | **Structurally unreachable in the actual matrix - see below** |
| PROXIMITY (1) | 6218 | Reachable, fired extensively, timing-analyzed in phase10-findings.md |
| RELATIVE_SPEED (2) | **0** | **Structurally unreachable in the actual matrix - see below** |
| COMMAND_LIMIT (3) | **0** | **Structurally unreachable by construction - proven below** |
| LOW_PERCEPTION_CONFIDENCE (4) | 6082 | Reachable, fired extensively (noise sweep) |
| COSTMAP_COLLISION (5) | 7 | Reachable, fired (all in the keepout re-run) |
| KEEPOUT_VIOLATION (6) | 425 | Reachable, fired (keepout re-run, verified mechanism) |
| INFERENCE_TIMEOUT (7) | **0** | **Never observed - mechanism exists, not exercised** |

**Half the taxonomy - 4 of 8 causes - never fired across the entire matrix.** This is a more
severe version of the exact failure mode the Phase 9→10 review already flagged once
(`LOW_PERCEPTION_CONFIDENCE` was unreachable when built, found by accident). The pre-Phase-10
reachability audit (`docs/phase9-findings.md` addendum) checked three of these four and
concluded: `COMMAND_LIMIT` is inert by construction (correct, confirmed again below);
`RELATIVE_SPEED` was "fixed" by exposing `pedestrian_sim_node`'s `max_speed` as a launch
argument; `INFERENCE_TIMEOUT` was flagged as "mechanism-reachable, not yet observed, watch
during Phase 10." **CROWD_SIZE was not flagged as a concern at all** (Phase 9 had observed it
fire live, so it was treated as proven). None of the three assessments checked whether the
*actual, configured Phase 10 scenario suite* would ever create the triggering condition - "the
parameter is exposed" and "the mechanism fired once in ad hoc testing" are not the same claim
as "the matrix that produced the headline results exercised it." It didn't, for three of them.

**CROWD_SIZE** - `SafetySupervisorConfig::max_train_humans = 5` (default). Every scenario in
`scenarios.py` (`CORE_SCENARIOS['open_arena']`, `['depot']`, `KEEPOUT_BLOCK_SCENARIO`) sets
`num_pedestrians` to either 4 or 0 - confirmed via direct grep, no scenario anywhere in the
harness ever configures more than 4. Since no mechanism in the pipeline can make the perceived
human count *exceed* the spawned pedestrian count (dummy-injection only fires on an *empty*
list, injecting exactly one, never inflating a non-empty list), the perceived count is capped
at 4 for every episode in the matrix - strictly below the threshold of 5. **Structurally
unreachable given the current scenario suite**, even though the check itself is correct and
was previously observed to fire in Phase 9's own ad hoc, non-harness testing (a different
pedestrian count than any Phase 10 scenario uses).

**RELATIVE_SPEED** - `SafetySupervisorConfig::max_train_speed_mps = 1.5`. `run_episode.py`'s
`ped_cmd` construction was checked directly (`grep -n "max_speed" run_episode.py scenarios.py`)
- **neither file ever passes a `max_speed` argument to `pedestrians.launch.py`**, meaning every
episode in the matrix runs pedestrians at that launch file's own default, 1.0 m/s - below the
1.5 m/s threshold. The earlier fix (exposing `max_speed` as a launch argument, so it *could* be
set above threshold) was necessary but not sufficient: nothing in the harness that actually
produced the Phase 10 results ever sets it there. This is the same class of bug the review
explicitly asked to check for, and it survived the check that was supposed to catch it, because
that check verified the parameter was *settable*, not that any real episode *set* it.

**COMMAND_LIMIT** - re-derived directly from source, not from memory of the earlier finding.
`candidate_action_space.cpp`'s speed formula: `speeds[i] = (exp((i+1)/speed_samples) - 1) /
(e - 1) * v_pref`. At `i = speed_samples - 1`: `(i+1)/speed_samples = 1`, so
`speeds[last] = (e - 1)/(e - 1) * v_pref = v_pref` **exactly**. `policy_v_pref_mps = 1.0`
(`policy_adapter.yaml`) and `max_commanded_speed_mps` reuses `max_linear_vel_mps_`, which reads
`max_linear_vel_mps: 1.0` from `nav2_params.yaml` - also exactly 1.0. Every candidate's
Cartesian speed magnitude at the top speed index is `speed * sqrt(cos^2+sin^2) = speed`
regardless of rotation, so the maximum candidate speed across the *entire* action space is
exactly 1.0 m/s. The check is `commanded_speed > config_.max_commanded_speed_mps` (strict).
1.0 is never greater than 1.0. **Confirmed structurally unreachable by construction** - this
one was already documented as such and remains correct on re-derivation.

**INFERENCE_TIMEOUT** - `watchdog_window_s` default 0.03 s (30 ms). Fires via
`ControllerDecisionCore`'s watchdog when a policy decision (candidate enumeration + batched
ONNX inference) exceeds that window. The mechanism is real and unit-tested
(`test_controller_decision_core.cpp` exercises the watchdog path with an artificially slow fake
decision function) - this is not a "silent zero" in the same sense as the three above, since
the *test suite* does prove the mechanism works. But it has now gone through an entire 139-
episode live matrix, on real Gazebo+ONNX inference, without firing once - meaning on this
project's actual hardware, SARL/Dummy inference plus candidate enumeration reliably completes
within 30 ms. Status: reachable in principle (test-proven), never observed in practice under
this project's real operating conditions, after being explicitly flagged to watch for in Phase
10 and not watched (no log/metric was added to track near-miss decision latencies, only whether
the watchdog actually fired).

**Severity: CRITICAL.** Three of these (CROWD_SIZE, RELATIVE_SPEED, COMMAND_LIMIT) mean any
claim that "the safety supervisor's full trigger taxonomy was exercised by the evaluation
matrix" is false, and the intervention-rate-by-cause headline numbers in
`docs/phase10-findings.md` implicitly represent 4 of 8 causes as clean zeros rather than
untested zeros - a categorically different, weaker claim. Disposition: **deferred to Part 2**
(fixing requires either a new scenario/config that legitimately exercises CROWD_SIZE and
RELATIVE_SPEED, re-running affected episodes, or explicit documentation reframing - decided in
Part 2, not here).

### 1.2 Topic liveness - every topic this project's own code depends on

Live-checked directly (not inferred), via a real Gazebo/Nav2 session (`open_arena`,
`baseline_mppi`/`dummy`, supervisor disabled) with an actual `navigate_to_pose` goal sent, using
three independent methods where the first gave an ambiguous result: `ros2 topic hz`,
`ros2 topic echo --once`, and a raw `rclpy` subscriber with a real spin loop (to rule out CLI
tool artifacts).

**Confirmed live, all topics this project's own code actually subscribes to or depends on**:
`/ground_truth/robot_pose`, `/pedestrians`, `/amcl_pose`, `/scan` (already tracked by
`episode_monitor.py`'s own counters, Phase 11), plus checked fresh here: `/clock` (100 Hz),
`/tf` (~75 Hz), `/joint_states` (~100 Hz), `/diff_drive_base_controller/odom` (~50 Hz),
`/world/<world>/dynamic_pose/info` (~50 Hz, the scene_broadcaster feed upstream of
`/ground_truth/robot_pose`), `/cmd_vel` (~20 Hz during active navigation),
`/diff_drive_base_controller/cmd_vel_unstamped` (confirmed live during active navigation - see
below for the false alarm this one produced), and the transient_local/latched topics `/map`,
`/tf_static`, `/costmap_filter_info`, `/keepout_filter_mask` (each confirmed to deliver their
retained message immediately to a freshly-joined subscriber, the correct behavior for a latched
topic, not a dead one).

**A methodology false alarm worth documenting** (the audit process itself, not a code finding):
`/diff_drive_base_controller/cmd_vel_unstamped`, `/plan`, `/transformed_global_plan`, and
`/particle_cloud` initially all showed zero messages across multiple check methods, including a
15-second raw `rclpy` subscription window - a real scare, since a dead velocity-smoother output
would mean the robot physically cannot move despite Nav2 believing it commanded motion. Traced
to the audit's own test methodology: the `navigate_to_pose` goal sent via the CLI had already
completed (confirmed via the CLI's own "Goal finished with status: SUCCEEDED" and the robot's
`/ground_truth/robot_pose` sitting almost exactly at the commanded goal) *before* the topic
checks ran - the checks were made after the episode was already idle, not during active
navigation. Re-run immediately after a fresh goal send, `/diff_drive_base_controller/cmd_vel_unstamped`
confirmed live. This is exactly the kind of false positive a hasty audit could report as a
finding without the deeper check; recorded here as a caution rather than silently discarded.

**Confirmed genuinely out of scope, not a dependency**: `/plan`, `/transformed_global_plan`,
`/particle_cloud` - grepped across the entire `crowd_nav_ws/src` tree, none of this project's
own code subscribes to any of the three. They are stock Nav2/AMCL diagnostic/RViz-visualization
topics; whether they are live has no bearing on this project's correctness or safety claims, so
they are outside "topics the stack depends on" as the audit itself frames the requirement.

**Severity: none found** (the one real scare was a false alarm, resolved). No fix needed.

### 1.3 Coordinate frame audit - CRITICAL finding: `WorldState.robot` and `WorldState.humans` are in different frames

**This is the single most severe finding in this audit.** `crowd_nav_controller.cpp::buildWorldState()`
sets `state.robot.px/py` from `computeVelocityCommands()`'s own `pose` argument - the pose Nav2
supplies in the local costmap's global frame, which for this system is `'map'` (the same frame
AMCL publishes, the same frame the safety supervisor's forward-sim and the keepout costmap
consume - confirmed correct and internally consistent in isolation, §1.2's Phase 10 keepout fix
above).

`state.humans` is populated from `human_source_->getHumans(now)` -
`GroundTruthHumanSource::getHumans()`, which returns positions built entirely from
`ingestPedestrian(id, x, y, ...)`'s raw arguments, which trace directly to the `/pedestrians`
topic's own `x`/`y` fields. `pedestrian_sim_node.py` spawns and moves pedestrians inside an
absolute box (`min_x=-3.5, max_x=3.4, min_y=-1.7, max_y=1.5`, checked directly against source)
with **no transform to map frame anywhere** - this box is sized to match the room's actual
Gazebo **world**-frame extent (confirmed: matches the wall positions in both `open_arena.sdf`
and `depot_scaled.sdf`), and the only other pose this node ever consumes
(`/ground_truth/robot_pose`, used purely for the reactive repulsion force) is also world frame.
So `state.humans[i].x/y` is Gazebo **world** frame.

**Verified live, not just traced through code** (since a claim this severe needs more than a
source read): launched the real stack, drove the robot with real motion (to force AMCL past its
initial-pose prior, ruling out a startup-transient false read), and compared `/amcl_pose` (map
frame) against `/ground_truth/robot_pose` (world frame) at the same real moment via a raw
`rclpy` subscriber (1507 ground-truth samples, 4 AMCL samples over a 10s window with the robot
moving): **difference (world − map) = (−2.93, −0.07)** - a real, non-negligible, non-zero
offset, matching this scenario's known spawn/map correspondence (world `(−3, 0)` localizes to
map `(0, 0)`) almost exactly. This is not the "map ≈ world" case where the bug would be
practically inert; the offset is large relative to every threshold in the system
(`min_train_distance_m=0.8`, `COLLISION_DISTANCE_M≈0.39`).

**Consequence**: every relative-position calculation between the robot and any human that goes
through `WorldState` - the `PROXIMITY` OOD check (`hypot(h.x - state.robot.px, h.y -
state.robot.py)`), every distance term `ObservationBuilder` feeds the policy, every
candidate-propagation reward computation - has been computing a **wrong** relative
robot-human vector, offset by this scenario's world/map difference, for the entire time this
project has run any live Gazebo integration test (Phase 7 onward). This is not confined to one
scenario the way the keepout bug was; it is structural to `buildWorldState()` itself and would
affect every core-matrix and noise-sweep episode in the Phase 10 results.

**Why this was never caught**: confirmed directly - `test_safety_supervisor.cpp`'s `baseState()`
helper sets `state.robot = {0.0, 0.0, ...}` and places humans at hand-picked coordinates
described in comments as relative to that same origin (e.g. `"0.5m from robot at origin"` for
the PROXIMITY test). Every unit test in the suite hand-constructs both `state.robot` and
`state.humans` in one implicit, consistent frame, because a pure C++ unit test has no Gazebo or
Nav2 to source two independently-framed poses from in the first place. **This is exactly the
class of bug the audit was asked to find**: a test that would pass under a naive wrong
implementation, because the very thing that makes the implementation wrong (combining a
real map-frame Nav2 pose with a real world-frame Gazebo pose) cannot occur in the test's own
construction.

**What is NOT affected, stated precisely to avoid overclaiming**:
- The SARL ONNX export/checkpoint validation (Phase 0, Phase 8's bit-close PyTorch comparison)
  used hand-constructed, synthetic `JointState`/`WorldState` fixtures with no Gazebo or Nav2
  involvement at all - unaffected by this bug, still valid evidence the exported network matches
  the reference model.
- The evaluation harness's own **outcome determination** (`episode_monitor.py`) computes
  collision/success independently, comparing `/ground_truth/robot_pose` (world) against
  `/pedestrians` (world) - both genuinely world frame, internally consistent, *not* affected by
  this bug. The core success/collision/timeout **outcome** numbers in `docs/phase10-findings.md`
  are not directly falsified by this finding.
- What IS directly undermined: the **mechanism explanation** for why - `PROXIMITY` firing
  6218 times, its timing relative to collisions, the "detection-margin gap" explanation given
  for the collision-rate finding - all of that reasoning assumed `PROXIMITY`'s internal distance
  computation was a real (if imperfect) proximity estimate. It was actually computing a
  distance to a **phantom human position** offset by ~(2.9, 0.07) m from where the humans
  really were, relative to the robot. Whether it still happened to fire near real close
  encounters, by the specific geometry of a robot and humans both traversing the same
  world/map-shifted box, needs to be re-examined, not assumed.

**Severity: CRITICAL.** This is the highest-priority fix in Part 2. Disposition: **fix required**
- `buildWorldState()` must source `state.robot.px/py` from the same frame `state.humans` is in
(world), or transform `state.humans` into map frame before populating `WorldState` - the second
option is architecturally cleaner (keeps `state.robot` matching what the safety supervisor's
forward-sim/costmap needs) and is the direction Part 2 will take. This will change `PROXIMITY`'s
actual trigger behavior, and very likely the intervention-rate numbers and possibly the
collision-rate result itself - **episodes affected by this fix will be re-run, and any changed
headline number will be reported as changed, not reframed.**

### 1.4 `policy_radius`/`robot_radius` split - verified, holds everywhere

Grepped every file referencing either radius across the workspace. `policy_radius_m=0.3`
(training value) is used consistently for everything fed to the network (`state.robot.radius`
via `ObservationBuilder`, the OOD `min_train_distance_m` threshold derivation) and NEVER for
physical/costmap geometry. `robot_collision_radius=0.14` (`nvis_3302ard.xacro`) is used
consistently for the real costmap (`nav2_params.yaml`'s `robot_radius: 0.14`, both
`local_costmap` and `global_costmap` sections confirmed identical) and nowhere fed to the
network. A dedicated regression test (`test_sarl_adapter.cpp`,
`FeedsTrainingRadiusNotUrdfRadiusToTheNetwork` or similarly named) explicitly asserts the
network-fed value equals `policy_radius_m` and is *not* close to the URDF value. **No bug,
already correctly guarded.**

### 1.5 Float32-vs-double precision sweep

Grepped every `float`/`static_cast<float>` usage and every `OccupancyGrid` consumer in the
codebase. All `float` casts are ONNX tensor marshaling (`onnx_inference.cpp`,
`dummy_adapter.cpp`, `sarl_adapter.cpp`) - a deliberate, necessary conversion to the network's
own float32 tensor format, not a precision bug (position/velocity magnitudes in this
application are well within float32's precision for the comparisons that matter). The only code
anywhere that does its own arithmetic on an `OccupancyGrid`'s `float32` fields is
`isKeepoutFlagged()` in `safety_supervisor.cpp` - the single, already-documented instance (its
own code comment states the class and scope of the issue). **No second instance found.**

### 1.6 Seeded determinism re-verification

**Confirmed still holding** - with the correct methodology, reached only after two of my own
test-methodology mistakes, recorded honestly rather than discarded:

1. First attempt: a hand-scripted synthetic `/clock` publisher against one already-running
   Gazebo instance, re-launching only `pedestrian_sim_node.py` between captures. Produced
   completely different trajectories. Wrong test, not a real bug: the node's own "catch-up"
   design (documented in `pedestrian_sim_node.py`, matching Phase 4's rationale) means a freshly
   started node catches up from `next_step_time=0` to whatever absolute sim-time it first
   observes - reusing one continuously-running Gazebo instance across two separate node
   launches means each one starts its catch-up from a *different* absolute baseline, so nothing
   about their outputs was ever expected to align.
2. Second attempt: fixed that (fresh Gazebo restart per run, giving both a matching baseline),
   but compared captured messages by raw arrival index. Still mismatched, starting a few
   messages in. Also the wrong test: two separate Gazebo launches don't advance sim time at
   exactly the same rate relative to real time (confirmed: 180 vs 157 messages captured in the
   same 5 real-time seconds), so the *n*-th message received doesn't correspond to the same
   simulated moment across runs.
3. Third attempt: fresh Gazebo restart per run **and** comparison aligned by the message's own
   sim-time-derived key (`header.stamp`), taking only the overlapping range - the same
   methodology Phase 4's original finding used ("every overlapping sim-time-indexed line"),
   not something invented fresh here. Result: **108 of 108 overlapping sim-time-keyed entries
   byte-identical** (position and velocity, 6 decimal places) across two independently-launched
   Gazebo sessions with the same seed.

Recorded at this length deliberately: comparison methodology for this specific kind of check
is easy to get wrong in a way that produces a false "bug," and the audit's own standing rule
(don't reframe an inconvenient result) cuts both ways - a false positive dismissed without this
level of scrutiny would have been just as dishonest as a real positive explained away. **No
bug. Determinism claim re-confirmed, not just re-asserted.**

### 1.7 Executable-bit git-mode sweep (explicit addition per review) - every script, not just previously-failed ones

Per the explicit instruction not to spot-check only the files that already broke something: swept
`git ls-files -s` against every `.py` file in `crowd_nav_ws/src/*/scripts/` and `*/test/`
(the directories actually installed as `PROGRAMS` or invoked as executables), every `.sh` script
repo-wide, and `.github/scripts/`.

**crowd_nav_ws**: zero non-executable entries found - the Phase 10/11 fixes (four scripts across
two phases) were applied correctly and completely at the git level, confirmed by this sweep
finding nothing left over in that tree.

**A fourth instance found**: `.github/scripts/run_smoke_test_episode.py` (written this session,
for the Phase 11 nightly workflow) was itself committed non-executable (`100644`) - the exact
same class of oversight, just in a file too new to have been caught by the sweep that fixed the
other four. Functionally harmless here specifically (`nightly-smoke-test.yml` invokes it as
`python3 .github/scripts/run_smoke_test_episode.py`, an explicit interpreter call that doesn't
depend on the exec bit or shebang) - but inconsistent with every other script in the repo and
exactly the pattern this check exists to close permanently. Fixed (`chmod +x` + staged).
**This is the fourth occurrence of this exact bug class across this project.** Worth a permanent
process note, not just a fix - see `docs/lessons.md` (Part 3).

---

## 2. Results integrity

Re-derived every headline number in `docs/phase10-findings.md` directly from `episodes.csv`,
independently of the prose - not spot-checked, the full set.

**Sample sizes**: 139 total rows, 139 unique `episode_id` values (no duplicates). Core=96,
sweep=40, keepout=3 (all match the claimed split exactly). Core per-cell N=16 for all six
(scenario, config) cells (8 seeds × 2 pedestrian modes, as claimed). Zero `pilot_`-prefixed rows
present in the main `episodes.csv` - pilot data lives only in the separate `results_pilot/`
directory, confirming it was never merged into the reported matrix.

**Collision-rate headline table** (reactive mode only): re-derived independently -
`depot`: baseline_mppi 1/8 (12%), policy_raw 1/8 (12%), policy_supervised 2/8 (25%);
`open_arena`: baseline_mppi 1/8 (12%), policy_raw 1/8 (12%), policy_supervised 4/8 (50%).
**Matches the findings doc exactly.**

**Noise sweep** (outcome distribution and rate-normalized intervention counts): re-derived
independently for all 5 dropout points - outcomes and rates (0.81, 3.01, 3.58, 3.66, 3.60
interventions/sec) **match the findings doc exactly**.

**Keepout scenario**: re-derived directly - `baseline_mppi` success/path=6.37m,
`policy_raw` keepout_violation/path=0.64m, `policy_supervised` timeout/428 interventions
(425 KEEPOUT_VIOLATION, 3 COSTMAP_COLLISION). **Matches the findings doc exactly.**

**No arithmetic or transcription errors found anywhere in the headline numbers.** The one
result-integrity issue this audit found is not a wrong number - it's the OOD-reachability gap
(§1.1) and the frame-mismatch bug (§1.3), both of which affect what the numbers *mean*, not
whether they were computed correctly from the data that exists.

## 3.5 Test quality - which tests would pass under a naive wrong implementation

**`crowd_nav_controller` has zero unit test coverage for `buildWorldState()` or the overall
`computeVelocityCommands()` integration.** Confirmed directly: the package's only test file,
`test_controller_decision_core.cpp`, covers exclusively the watchdog/decision-timing logic (4
tests). The function that actually combines Nav2's pose argument with
`GroundTruthHumanSource::getHumans()` output into one `WorldState` - where the §1.3 frame bug
lives - has never been exercised by anything except live Gazebo observation, where "the robot
seems to navigate reasonably" was never going to surface a constant-offset error in relative
geometry. This is the direct, mechanical reason the bug went undetected for as long as it did,
not a coincidence.

**Every test that hand-constructs a `WorldState` does so with `state.robot` and `state.humans`
in one implicit, mutually-consistent frame - by construction, not by choice.** Confirmed across
`test_safety_supervisor.cpp`, `test_sarl_adapter.cpp`, `test_observation_builder.cpp`,
`test_candidate_propagation.cpp`: none of these tests *could* exercise the frame-mismatch bug,
because a pure C++ unit test has no independent Nav2-pose and Gazebo-pose sources to disagree
in the first place. These tests are all individually valid for what they claim to test (the
pure logic given self-consistent input) - the issue is a documentation/framing one: nothing
states that "the OOD/policy logic is correct" and "the OOD/policy logic receives correct input
in production" are two separate claims, and only the first one has test coverage.

**`COMMAND_LIMIT`'s own unit test demonstrates the same gap concretely.**
`CommandLimitTriggersWhenCandidateTooFast` feeds a hand-picked candidate `{2.0, 0.0}` (2.0 m/s)
against `max_commanded_speed_mps=1.0` and correctly asserts rejection - but §1.1 already proved
the real candidate action space, built from the real `policy_v_pref_mps=1.0`, can never produce
a candidate faster than exactly 1.0 m/s. The test verifies the threshold-crossing *logic* is
correct; it says nothing about whether real production candidates can ever cross that
threshold, and gives a false impression that "`COMMAND_LIMIT` is tested and verified" when the
actually-important question - can this fire in production - was never asked by the test suite,
only by this audit.

**Severity: SIGNIFICANT** (both the `buildWorldState()` gap and the reachability-vs-logic
conflation). Disposition: fixing the frame bug (§1.3) is Part 2's priority; adding a
`buildWorldState()`-level integration test and reachability-aware assertions for
`COMMAND_LIMIT`/`RELATIVE_SPEED`/`CROWD_SIZE` (asserting against the *actual* production config
values, not arbitrary ones) are both scoped into Part 2's fix list.

## 3. Contamination check - rogue matrix run / archived pilot data

Confirmed via the sample-size check above (zero `pilot_`-prefixed rows in the reported
`episodes.csv`) and via the exact count match (96+40+3=139 with no duplicate or extra IDs) that
the results are not contaminated by either the rogue premature `--phase all` run (deleted
before the real matrix ran, per `docs/phase10-findings.md`'s own account) or the archived pilot
episodes (kept in a separate `results_pilot/` directory, never merged). **No contamination
found.**

## 4. Documentation drift

Spot-checked rather than re-verified line-by-line (the plan document alone is 2000+ lines) -
focused on claims most likely to have drifted: test counts (`IMPLEMENTATION_PLAN.md`'s "60 tests
total" claim - re-run `colcon test-result --all` directly, confirmed still exactly 60, 0
failures), phase-completion status (README's status framing, confirmed consistent - "11
complete" appears in both the Future Work section and Build history section, no stale "Phase N
of 12" leftover from an earlier revision), and a repo-wide `TODO`/`FIXME`/`XXX` sweep across
`IMPLEMENTATION_PLAN.md`, `README.md`, and every `docs/phase*-findings.md` (zero matches - no
forgotten markers). The one already-known historical drift instance (§1.5's "no surprises"
claim, corrected in Phase 3) is itself documented as a past correction, not a currently-stale
claim. **No new documentation drift found** at this level of check; a full claim-by-claim
re-verification of 11 phases of findings docs was judged disproportionate given the audit's
time budget and lower expected yield relative to the categories above.

## 5. Code quality

**Unused parameters**: checked `crowd_nav_controller.cpp` (the largest, most parameter-heavy
file) - every ROS parameter name declared appears at least twice (declare + read, three times
for the ones with dynamic-reconfigure callbacks). No declared-but-never-read parameters found.

**Reimplemented stack features**: no new instances found beyond what prior phases already
document reusing correctly (KeepoutFilter/mask_server for zones, pluginlib for the MPPI
fallback, `nav2_velocity_smoother` for acceleration limiting rather than hand-rolled clamping).

**Hardcoded-should-be-config duplication**: the one place two files must be kept manually in
sync (`robot_radius: 0.14` in `nav2_params.yaml`, both costmap sections, matching
`robot_collision_radius` in the xacro) is already flagged with an explicit cross-referencing
comment in both files ("Must match... single source of truth... Update both together") and
confirmed identical in §1.4 above - a manual-sync point, not an unmanaged duplication.

**Severity: none found at this pass.** Given the scope already consumed by §§1-3.5 and their
higher-severity findings, this category was checked at reasonable, not exhaustive, depth.
