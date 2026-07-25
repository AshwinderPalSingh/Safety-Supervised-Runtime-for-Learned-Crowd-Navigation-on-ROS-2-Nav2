# LiDAR-based human perception: audit and fixes

Commissioned directly, not as part of any planned phase: the project's default human perception
had always been `GroundTruthHumanSource` - real, privileged Gazebo positions, never anything the
robot's own sensors actually measured. That is a legitimate, deliberate oracle mode for isolating
policy-logic bugs from perception bugs (§4 below), but it is also, unqualified, a cheat: a policy
and a safety supervisor validated only against ground truth have never been tested against
anything resembling what a real robot's own sensors will actually hand them. With physical
hardware bring-up approaching (a real 360° 2D LiDAR, not the simulated, gz-sim-bug-masked ~180°
sensor this project has run against everywhere else), this was audited and closed out before that
transition, not after.

## Summary

**A real, non-ground-truth `HumanStateSource` - `LidarHumanTrackerSource` - already existed in
the source tree, untracked, before this audit started.** Adaptive-breakpoint clustering
(`lidar_clustering.cpp`, a standard, citable classical method - Borges & Aguiar 2004), greedy
nearest-neighbor tracking with exponential velocity smoothing (`lidar_tracker.cpp`), and a
TF-based sensor-frame-to-map-frame conversion sharing Nav2's own buffer
(`lidar_human_tracker_source.cpp`) - all genuinely well-designed, all already unit-tested. What
this audit found and fixed:

- **Never wired into production.** `CrowdNavController::configure()` unconditionally constructed
  `GroundTruthHumanSource` - `LidarHumanTrackerSource` was fully built, fully tested, and dead
  code from the running system's own perspective. One file's own launch-file docstring already
  *claimed* the wiring existed ("the default HumanStateSource as of this change"). It didn't.
  Fixed - see §2.
- **A real crash**, found by actually running the existing test suite, not just reading the code:
  `test_lidar_human_tracker_source` segfaulted. Root cause and fix in §3.1 - also a real
  watchdog-latency risk independent of the crash.
- **A real correctness gap for the actual physical sensor**, not the simulated one: a human
  standing near the scan array's angular seam would be split into two half-width detections on a
  true 360° LiDAR (this robot's real sensor), a failure mode the simulated ~180°-masked sensor can
  structurally never expose. Root cause and fix in §3.2.
- **One test-authoring bug** (wrong expected angle in a hand-derived test case, not a bug in the
  code under test) - §3.3, for completeness, not because it was consequential.

Post-fix: 25 tests across `lidar_clustering`/`lidar_tracker`/`lidar_human_tracker_source`, full
workspace rebuild clean, 91 tests total across the workspace, 0 failures, and a live Gazebo run
(§6) confirming the wiring genuinely takes effect and survives a real navigation attempt without
crashing - which also surfaced one real, honest, non-blocking finding worth carrying into the
real-hardware calibration pass: a purely geometric width filter cannot always distinguish a
static pillar from a human, and this project's own depot world has pillars sized right inside the
human-width band. `human_source_type` is
now a real, config-selectable `FollowPath` parameter (`"ground_truth"` | `"lidar_tracked"`),
defaulting to `"lidar_tracked"` - the evaluation harness explicitly pins every existing scenario
to `"ground_truth"` so nothing about the already-reported 142-episode matrix changes as a result
(§4).

## 1. Design, as found (not designed fresh by this audit)

- **`clusterScan()`** (`lidar_clustering.hpp/.cpp`): walks a `LaserScan`'s valid returns in
  angular order, starts a new cluster whenever the Cartesian distance between consecutive points
  exceeds `break_distance_m` (default 0.15 m). Deliberately not a fancier method (RANSAC circle
  fits, learned leg detectors like DR-SPAAM/DROW) - this robot's budget LiDAR spec doesn't have
  the angular resolution to make those meaningfully more accurate, and a classical, inspectable
  method is the honest choice for a first real perception pipeline.
- **`LidarTracker`** (`lidar_tracker.hpp/.cpp`): greedy global nearest-neighbor association
  (closest (track, detection) pair under a gate distance, repeatedly, not Hungarian-optimal),
  exponential velocity smoothing (raw finite-difference velocity off single-frame cluster
  centroids is too noisy to feed `PROXIMITY`/`RELATIVE_SPEED` directly), coast-through-brief-gaps
  track aging, monotonic per-track IDs (a real sensor has no ground-truth ID to recover).
- **`LidarHumanTrackerSource`** (`lidar_human_tracker_source.hpp/.cpp`): clusters the scan, width-
  filters clusters to a plausible human cross-section (0.05-0.60 m, `min_cluster_points=3`),
  transforms surviving centroids into map frame, feeds the tracker. Width-filtering exists
  specifically to reject this project's own depot world's shelf poles and pillars, which cluster
  too (confirmed by a dedicated test using this project's own known depot geometry).
- **Frame handling is structural, not a repeat of the audit's own worst finding**: this class
  transforms every detection through the *same* `tf2_ros::Buffer` Nav2 itself passed into the
  controller plugin's `configure()` - not a second, independently-sourced pose channel the way
  `GroundTruthHumanSource::setRobotMapPose()` had to be added after the fact to correct for
  (`docs/audit.md` §1.3). Using the robot's own real TF tree for both the robot's own pose and
  every human detection's frame conversion makes that entire bug class structurally impossible
  here, not merely fixed for one instance of it.

## 2. The wiring gap

`crowd_nav_controller.cpp` had zero references anywhere to `LidarHumanTrackerSource`,
`TrackedHumanSource`, or any `human_source_type` selector - confirmed by a direct repo-wide grep,
not assumed from reading one file. `configure()` constructed `GroundTruthHumanSource`
unconditionally, exactly as it had since Phase 8. Meanwhile `pedestrians.launch.py`'s own
docstring already asserted the opposite: *"LidarHumanTrackerSource (the default HumanStateSource
as of this change, crowd_nav_controller's human_source_type)."* That claim was false at the time
it was written - a real instance of the same class of defect this project's own `docs/lessons.md`
(#1) already generalizes: a component can be fully built, fully tested, and still never actually
run, and nothing about reading its own code or its own doc comments would tell you that.

**Fixed**: `CrowdNavController` now declares a real `human_source_type` parameter
(`"ground_truth"` | `"lidar_tracked"`, mirroring `adapter_type`'s existing dummy/sarl pattern
exactly, including the same fail-loudly-on-unknown-value handling) and constructs whichever
source is actually configured. Seven additional `lidar_*` parameters expose every tunable field
of `LidarPerceptionParams` individually - deliberately, since none of them are validated against
the real physical LiDAR yet (only the simulated sensor, and only informally); a real-hardware
calibration pass should need a launch-argument change, not a rebuild.

## 3. Bugs found by actually running the existing tests, not just reading the code

### 3.1 A real segfault: blocking TF lookup on a non-dedicated-thread buffer

**Symptom**: `test_lidar_human_tracker_source` crashed (SIGSEGV, return code -11) on
`UnknownTargetFrameIsCaughtNotThrownAndReturnsNoDetections` - a test whose entire point was to
prove a missing/unready TF tree degrades gracefully. It didn't; it crashed the process instead.

**Root cause**: `tf_buffer_->transform(point_in, params_.map_frame,
tf2::durationFromSec(params_.tf_timeout_s))` (a nonzero-timeout lookup) requires the buffer to
have a dedicated thread servicing incoming transforms while the call blocks and waits -
`tf2_ros::Buffer`'s own runtime warning says so explicitly (*"Do not call canTransform or
lookupTransform with a timeout unless you are using another thread for populating data"*). A
standalone buffer with no listener (exactly what this class's own test-only constructor builds,
and plausibly what a not-yet-fully-initialized real buffer looks like too, momentarily, at
startup) doesn't satisfy that precondition, and the misuse crashed instead of throwing the
`tf2::TransformException` the surrounding `catch` block was written to handle.

**Also, independently of the crash**: a nonzero timeout (default 0.1 s) sitting inside this call
chain is a real latency risk on its own merits - `processScan()` feeds directly into the
controller's 30 ms-watchdog-bounded decision path (§7 of `explanation.pdf`), so a call that could
legitimately block for up to 100 ms has no business being in it, whether or not it happens to
crash first.

**Fix**: switched to the `tf2::TimePoint`-based `canTransform`/`lookupTransform` overloads
(inherited from `tf2::BufferCore` via `Buffer`'s own `using` declarations) instead of the
`rclcpp::Time`-based convenience overloads - both default their *own* timeout parameter to zero,
but both still route through the same timed-wait implementation that requires the dedicated
thread; the `tf2::TimePoint` overloads are a plain synchronous "what's buffered right now" query
with no timeout/threading concept at all, structurally unable to hit that code path. This is not
a novel workaround - it is the exact pattern `tf2_ros::MessageFilter` uses internally
(`message_filter.hpp`) for the identical purpose. One TF lookup per scan (not one per cluster) was
also applied while fixing this: every cluster in a scan shares the same stamp and therefore the
same transform, so batching the lookup is both cheaper and removes any chance of clusters within
one scan disagreeing about TF availability.

**Verified**: the same test now passes cleanly with a one-time, expected, benign console warning
(`tf2` printing "Invalid frame ID... frame does not exist" for the deliberately-unknown-frame
case) instead of crashing. All 6 `test_lidar_human_tracker_source` cases pass.

### 3.2 A real correctness gap for the actual physical sensor: the angular seam

**Symptom**: none observed live - found by reasoning about the sim/hardware mismatch directly,
not from a failing test, since the simulated sensor structurally cannot expose this.

**The gap**: `clusterScan()`'s main pass only ever compares each sample to the one immediately
before it in array order - it never compares the array's last sample to its first. On this
project's own simulated LiDAR (masked to ~180° at the source, for an unrelated gz-sim rendering
bug, README's Known Limitations), that's harmless: the array's two ends sit in the always-empty
region behind the robot, genuinely far apart physically, not just far apart in array index. On
this project's real physical sensor - a full 360° LiDAR, confirmed with the person doing the
hardware bring-up - there is no such gap: the array's last and first samples are physically
adjacent angles. A human standing near that seam would be reported as two separate half-width
clusters, each likely too narrow individually to pass the human-width filter, rather than one
correct detection.

**Fix**: after the main clustering pass, if (a) the scan's total angular span is within one
sample of a full 2π (a genuine 360° scan, checked explicitly - never applied to a partial-FOV
scan like the sim's own) and (b) the first and last surviving point-groups' adjacent endpoints
are within `break_distance_m` of each other, they are merged into one cluster. Concatenating the
groups in true angular order (last-array-group first, then first-array-group) means the existing
centroid/width formulas apply unchanged to the merged group - no separate wraparound-specific math
needed.

**Verified**: two new tests, one positive (a synthetic 8-sample full-circle scan with a human-
width cluster straddling the seam merges into exactly one 4-point cluster, centroid/width checked
against independently-computed trig values to `1e-9`) and one adversarial-negative (endpoints
constructed to sit well within `break_distance_m` of each other on a scan that does *not* span a
full circle - proving the full-circle gate itself prevents merging, not incidental distance).
Both pass; all 9 pre-existing `clusterScan` tests continue to pass unchanged.

### 3.3 A test-authoring bug, for completeness

`test_lidar_clustering.SinglePointClusterHasExactCentroidAndZeroWidth` asserted an exact centroid
of `(2.0, 0.0)` for a return placed at array index 1 with `angle_min=0.0` - but index 1's actual
angle is `angle_min + 1*angle_increment = 0.1`, not `0`, per `clusterScan()`'s own documented,
correct formula. `clusterScan()` was computing `(1.9900..., 0.19967...)` - exactly
`2.0*cos(0.1)`/`2.0*sin(0.1)` - correctly, per its own contract; the test's own inline comment had
an off-by-one assumption about which array index its `angle_min=0.0` reference point applied to.
Fixed by correcting `angle_min` to `-0.1` so the intended point lands at angle 0 exactly, matching
the test's own stated intent. Not a defect in the code under test - recorded here anyway, per this
project's own standing discipline of reporting what running the actual test suite found, not just
what reading the code suggested.

## 4. Backward comparability with the already-reported evaluation matrix

Changing `CrowdNavController`'s own default away from `GroundTruthHumanSource` is a substantive
behavior change - left unaddressed, it would have silently altered every future bare invocation
of this project's own evaluation harness relative to the 142-episode matrix already reported in
`docs/phase10-findings.md` and audited in `docs/audit.md`. Handled explicitly, not left to
accident: `crowd_nav_evaluation/scripts/scenarios.py`'s `CONFIGS` dict now pins
`"human_source_type": "ground_truth"` on every existing config (`baseline_mppi`, `policy_raw`,
`policy_supervised`), and `run_episode.py` passes it through with a defensive
`config.get('human_source_type', 'ground_truth')` (fails safe to the oracle, not to whatever
`CrowdNavController`'s own default happens to be, if a future config entry ever forgets to set
it). The already-reported matrix remains exactly reproducible; nothing about this change alters
any number in `docs/phase10-findings.md`.

## 5. What this audit did not do (explicitly, not by oversight)

- **No real-hardware validation.** Every `lidar_*` parameter default (cluster width band, break
  distance, gate distance, track-miss tolerance) is carried over from whatever the pre-existing
  implementation already had, itself apparently sized against this project's own simulated sensor
  spec (~180°/8 m/~1° resolution), not the real 360° hardware LiDAR. A real calibration pass -
  mount the sensor, record real scans, check for a hardware self-detection artifact near the
  mount (a different, real-hardware analogue of this project's own simulated LiDAR rendering bug,
  README's Known Limitations), retune the width/break-distance parameters against real noise
  characteristics - is real, necessary follow-on work, not done here. Every value needed for it is
  already a config parameter (§2), specifically so that pass is a launch-argument change, not a
  rebuild.
- **No re-run of the evaluation matrix against `LidarHumanTrackerSource`.** §4 keeps the existing
  ground-truth-based matrix reproducible; it deliberately does not also produce a new, sensor-in-
  the-loop matrix. That is a real, valuable, separate measurement - the honest question "how much
  of the ground-truth-measured collision-rate result survives real (simulated) perception noise
  and clustering error, not just the synthetic degradation model" - left as explicit future work,
  not attempted here.
- **The synthetic degradation model (`GroundTruthHumanSource`'s noise/dropout/latency) and this
  pipeline's own real clustering/tracking noise are not the same thing.** The former simulates
  imperfect perception on top of true positions; the latter is genuinely imperfect perception,
  with its own real failure modes (a human occluded by another human producing no detection at
  all, two humans close together undersegmenting into one cluster, a real sensor's own range
  noise). Comparable in spirit, not identical, and not claimed to be.

## 6. Live Gazebo verification

Everything above was verified by unit test. This section is the live check on top of that -
this project's own standing discipline (`docs/lessons.md` #2, #7) is that a correct-looking unit
suite is not evidence a real integration seam works, and `buildWorldState()`'s own frame bug
(`docs/audit.md` §1.3) is the concrete precedent for exactly that gap. Launched the full stack
(`amcl.launch.py`, depot world, every default - i.e. `human_source_type` at its new
`"lidar_tracked"` default, not forced) plus `pedestrians.launch.py` with 3 reactive pedestrians,
`mirror_enabled` at its own new default (real collidable bodies, see the Files section below).

**Confirmed directly, not assumed**:
- `ros2 param get /controller_server FollowPath.human_source_type` returned `lidar_tracked` on
  the live node - the parameter wiring genuinely took effect, not just compiled.
- `/scan` publishing at the expected ~5 Hz throughout.
- A real `InterventionEvent` was observed on `/intervention_events` with a real, non-zero
  rejected velocity - the safety supervisor genuinely acted on `LidarHumanTrackerSource`-derived
  data, not a synthetic or empty feed.
- `controller_server` never crashed, at any point across stack startup (including the exact "TF
  not up yet" window §3.1's fix targets - confirmed live via the same benign, expected
  `[INFO]`-level "Timed out waiting for transform... odom... frame does not exist" messages
  Nav2 already prints during ordinary startup, not an unhandled exception) through a full
  navigation attempt to resolution.

**One real, honest finding, not swept under "it didn't crash"**: the single `InterventionEvent`
observed was cause `CROWD_SIZE` (threshold 5), with only 3 real pedestrians spawned. The most
likely explanation, not yet confirmed to the standard the rest of this document holds itself to:
this project's own depot world has 4 static pillars sized 0.126 m × 0.126 m
(`crowd_nav_gazebo/worlds/depot_scaled.sdf`) - squarely inside `LidarHumanTrackerSource`'s
0.05-0.60 m human-width band (§1). A purely geometric, single-scan width filter has no way to
distinguish a static pillar from a human-sized dynamic object; if one or more pillars are within
LiDAR range at the same time as the 3 real pedestrians, the perceived count can exceed 3 without
any real crowd-size violation happening. Weighing against this explanation: a 12-second follow-up
window showed zero further `CROWD_SIZE` events, which is not what a *constantly*-misclassified
static pillar would produce - so this reads as intermittent (angle/range-dependent, or transient
tracker noise while a track initializes) rather than a persistent miscount, but the single
occurrence is real, live, first-time evidence, not hypothetical. The navigation attempt itself
ended in `Goal failed` (not a crash - `controller_server` remained healthy throughout) - a
plausible, honest consequence of the supervisor now reasoning about noisier, real (if imperfect)
perception instead of a clean oracle, not investigated further here.

**Not fixed in this pass, deliberately scoped as follow-on** (§5 already flagged the general
version of this before this live run produced concrete evidence for it): distinguishing static
from dynamic detections - either by differencing against the already-loaded static map (this
project already has one, for localization) or by requiring a minimum displacement across several
tracked updates before a detection counts toward `CROWD_SIZE`/feeds the policy - is the direct,
now-motivated next step, not a hypothetical nice-to-have.

## Files

- `crowd_nav_ws/src/crowd_nav_perception/`: `lidar_clustering.hpp/.cpp`,
  `lidar_tracker.hpp/.cpp`, `lidar_human_tracker_source.hpp/.cpp` (all pre-existing, audited and
  fixed here), `test/test_lidar_clustering.cpp`, `test/test_lidar_tracker.cpp`,
  `test/test_lidar_human_tracker_source.cpp`.
- `crowd_nav_ws/src/crowd_nav_controller/src/crowd_nav_controller.cpp`: `human_source_type` and
  seven `lidar_*` parameters, conditional source construction (new, this audit).
- `crowd_nav_ws/src/crowd_nav_bringup/launch/amcl.launch.py`: `human_source_type`/`scan_topic`
  launch arguments (new, this audit).
- `crowd_nav_ws/src/crowd_nav_evaluation/scripts/scenarios.py`, `run_episode.py`:
  `human_source_type` pinned to `"ground_truth"` for backward comparability (new, this audit).
- `crowd_nav_ws/src/crowd_nav_pedestrians/scripts/actor_mirror_node.py`,
  `crowd_nav_ws/src/crowd_nav_pedestrians/launch/pedestrians.launch.py`: pedestrian bodies made
  genuinely collidable and launched by default (pre-existing, not authored by this audit) - the
  simulation-side reason `LidarHumanTrackerSource` has anything real to detect at all in Gazebo.
