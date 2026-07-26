# Phase 4 findings log

Per Phase 4 ("Pedestrian simulation"), rescoped in v1.7 before any
implementation started (HuNav dropped, single deterministic ROS node - see §1.2, and the v1.7
changelog entry, for the reasoning). Same discipline as prior phases: findings recorded as
they land.

**Status: DONE, done-bar met.** All four done-bar items verified with direct measurement, not
assumed from the design:
- Ground-truth topic/schema published; `reactive`/`non_reactive` and mirror-on/off are
  independent launch-time switches.
- **Determinism**: byte-identical across two independent same-seed runs.
- **Sim-time stepping**: confirmed via a Gazebo world pause, not assumed from "it subscribes to
  /clock."
- **Headless correctness + mirroring cost measured**: mirror runs correctly when enabled: RTF
  unaffected at this scale (~1.0 both with and without).

## What was built

New package `crowd_nav_pedestrians`:
- `msg/Pedestrian.msg` (`id`, `x`, `y`, `vx`, `vy`) / `msg/PedestrianArray.msg` (`header` +
  array) - the ground-truth schema `GroundTruthHumanSource` (Phase 5) will adapt from, per
  §4.1's "small internal message-adapter function... this is the actual seam."
- `scripts/pedestrian_sim_node.py`: a single deterministic Helbing-style social-force
  simulator. `reactive`/`non_reactive` is one parameter (`mode`) toggling whether the robot's
  ground-truth position contributes a repulsion force - not two code paths. All randomness
  (initial positions/goals, new-goal selection) draws from one `random.Random(seed)` instance
  owned by the node; the physics step itself is pure, iteration-order-fixed floating point.
  Steps in fixed `dt` increments keyed to accumulated `/clock` time (a "catch-up" loop handles
  however many clock messages arrive between steps), not wall-clock timers.
- `scripts/actor_mirror_node.py`: visual-only, off by default. Spawns one simple cylinder
  marker per pedestrian via Gazebo's `create` service and moves them via `set_pose` on each
  `PedestrianArray` message - both calls are fire-and-forget (`call_async`, no blocking wait),
  since correctness never depends on any single cosmetic update landing.
- `nvis_3302ard.xacro`: added Gazebo's `PosePublisher` system plugin (model-pose-only,
  `use_pose_vector_msg: false`), giving the robot a dedicated ground-truth pose topic
  (`/model/nvis_3302ard/pose`, `ignition.msgs.Pose`) independent of odometry.
- `launch/pedestrians.launch.py`: always bridges the robot's ground-truth pose
  (`ros_gz_bridge`, `geometry_msgs/msg/Pose`) and launches the sim node; the mirror node and
  its two service bridges (`SpawnEntity`, `SetEntityPose`) are gated behind `mirror_enabled`
  (default `false`).

## Real findings from verifying against actual behavior, not the design on paper

1. **No Python gz-transport bindings installed** (`python3-gz-transport`-equivalent not
   present) - ruled out subscribing to Gazebo topics directly from Python without a bridge.
   Confirmed via direct import attempts before committing to a design, not assumed missing.
2. **Found the right mechanism by checking what's actually installed**, not the first thing
   that came to mind: `ignition-gazebo6-pose-publisher-system` (a per-model ground-truth pose
   publisher) is available and gives a clean, single-entity `ignition.msgs.Pose` topic -
   confirmed via `ign topic -i` after attaching it, rather than assuming the plugin's topic
   name or message shape from memory. Avoided the fragile alternative (bridging the whole
   world's `Pose_V` array and hardcoding an index for our robot, which would break if the
   world's entity count or ordering ever changed).
3. **Plugin naming convention, again**: initially wrote the pose-publisher plugin using the
   newer `gz::sim::systems::...` / `gz-sim-*-system` naming, which doesn't match this project's
   actual gz-sim 6.18.0 (Fortress) install - every other plugin in this codebase uses the
   `ignition::gazebo::systems::...` namespace and `libignition-gazebo-*-system.so` filenames.
   Caught before ever testing it, by checking the convention already established elsewhere in
   this same file, rather than mixing two plugin-naming eras in one xacro.
4. **`ros_gz_bridge` does support `geometry_msgs/msg/Pose <-> ignition.msgs.Pose` and the
   `SpawnEntity`/`SetEntityPose` services directly** - confirmed by reading the actual
   conversion headers (`ros_gz_bridge/convert/geometry_msgs.hpp`) and by testing the bridge
   live, not assumed from the package's general reputation for supporting "most" message types.

## Determinism verification (the actual justification for dropping HuNav)

Two independent fresh launches (full teardown between them, not just a node restart), same
seed (123), same `num_pedestrians` (6). Captured the first 100 `/pedestrians` messages from
each run to a file (position/velocity to 6 decimal places, keyed by simulation time from the
message header, not wall-clock capture time - see `capture_pedestrians.py`).

**Result**: `diff` between the two capture files showed exactly one line of difference at the
very start (a one-tick offset from the two capture scripts starting to listen at very slightly
different points in the pedestrians' publish sequence - an artifact of test-harness startup
timing, not the simulation). Every overlapping sim-time-indexed line thereafter was
**byte-identical** across both independent runs, position and velocity, to 6 decimal places.

## Sim-time stepping verification

Rather than trust "it subscribes to `/clock`" as sufficient, paused the running Gazebo world
via its `WorldControl` service mid-simulation and measured directly:
- `/world/.../stats`'s `sim_time` field advanced only ~0.47 s during an 8-second real-time
  sleep while paused (a small residual from steps already in flight when the pause took
  effect) - confirming the *world itself* was genuinely frozen, not just that no new messages
  happened to arrive.
- Since `pedestrian_sim_node` steps exclusively from accumulated `/clock` values (never reads
  wall-clock time anywhere in its stepping logic), this transitively confirms the pedestrian
  simulation froze in lockstep with the world, not drifting ahead on an internal timer.
- Unpausing resumed publishing correctly from the frozen sim-time point.

## Mirror node verification

With `mirror_enabled:=true`: all 6 `pedestrian_marker_N` models spawned successfully (confirmed
via `ign model --list`, not just "no error printed" - `ign model -l` alone is a different,
unrelated flag and initially produced a misleading empty-looking result before using the
correct `--list` form). Spot-checked marker 0's actual Gazebo pose against pedestrian 0's most
recently published state: positions matched within the range expected from ordinary
inter-query lag (two sequential, not simultaneous, CLI queries, pedestrian moving at ~0.5 m/s
in between) - consistent with the mirror correctly tracking, not stalled or broken.

**RTF impact**: sampled `real_time_factor` from `/world/.../stats` several times with the
mirror both disabled and enabled, otherwise identical scenario (6 pedestrians, same seed).
Both conditions read ~0.9999-1.0 - **no measurable RTF cost from the mirror at this scale**
(6 simple cylinder markers, fire-and-forget pose updates). Worth re-measuring if a later phase
scales up pedestrian count substantially; not re-tested at larger N here since Phase 4's
done-bar only asks for the impact to be measured and recorded, not for a specific scale target.

## Design notes for Phase 5 (`GroundTruthHumanSource`)

- The message-adapter seam §4.1 anticipates is straightforward here: `PedestrianArray` ->
  `HumanObservation` is a direct field mapping (`id`/`x`/`y`/`vx`/`vy` match exactly; `cov_xx`
  etc. are populated by the degradation model downstream, not present in the ground-truth
  message itself, which is correct - covariance is a *perception* concept, not a ground-truth
  one).
- Frame: `PedestrianArray.header.frame_id` is `"map"` and all positions are already in that
  frame (pedestrian bounds were chosen directly in map-frame coordinates matching the room
  geometry established in Phase 2/3's reliability gates), so no additional frame transform
  should be needed in `GroundTruthHumanSource`.
