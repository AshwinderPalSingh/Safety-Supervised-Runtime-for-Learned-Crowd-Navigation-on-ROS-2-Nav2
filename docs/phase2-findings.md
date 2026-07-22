# Phase 2 findings log

Live log, per IMPLEMENTATION_PLAN.md's Phase 2 ("Baseline Nav2 + AMCL + SLAM toolbox"), same
discipline as Phases 0-1: findings recorded as they land, real bugs kept in the record rather
than quietly fixed.

**Status at the end of this session: substantial real progress, agreed done-bar (5
consecutive goals, both modes, no manual intervention) NOT yet met.** Recorded honestly below,
including the specific reasons, rather than claiming completion or silently narrowing scope.

## What was built

- `crowd_nav_gazebo/worlds/depot_scaled.sdf`: the 0.27x-scaled depot world Phase 0 planned but
  didn't build - generated programmatically from the exact primitive geometry extracted from
  the original tugbot_depot in Phase 0 (4 walls, 18 shelf poles in 3 rows, 4 pillars; boxes/
  pallets/stairs not reproduced, out of scope for this phase's functional testing needs).
  Validated loading cleanly in `ign gazebo`.
- `crowd_nav_bringup` package (new): `config/nav2_params.yaml` (AMCL, MPPI, costmaps,
  planner, behaviors, bt_navigator, velocity_smoother), `config/slam_toolbox.yaml`,
  `launch/slam.launch.py`, `launch/amcl.launch.py`, `maps/depot_scaled.{yaml,pgm}`.
- `crowd_nav_gazebo/launch/spawn_robot.launch.py` extended with `world_file`/`spawn_x`/
  `spawn_y`/`spawn_yaw` arguments (was hardcoded to `empty.sdf` at the origin in Phase 1).

## Real bugs found and fixed (recorded, not silently patched)

1. **MPPI's `ObstaclesCritic.consider_footprint: true` needs an explicit costmap footprint
   polygon** - we only configure a circular `robot_radius`. Fixed: `consider_footprint: false`
   (use the circular radius for collision checking, consistent with how the costmap itself is
   configured).
2. **Pluginlib lookup names are inconsistently `namespace::Class` vs. `package/Class` across
   different Nav2 packages** - not discoverable from the config alone, only from the actual
   registered-types list in each failure. Confirmed empirically, package by package:
   - `nav2_navfn_planner`, `nav2_behaviors`: **slash** format (`nav2_navfn_planner/NavfnPlanner`,
     `nav2_behaviors/Spin`, `/BackUp`, `/Wait`).
   - `nav2_controller`, `nav2_costmap_2d`, `nav2_mppi_controller`: **`::`** format
     (`nav2_controller::SimpleGoalChecker`, `nav2_costmap_2d::ObstacleLayer`, etc.)
   Worth a permanent note for anyone extending this config: don't assume one convention,
   check the actual error's "Declared types" list if a plugin fails to load.
3. **AMCL's `initial_pose` is in MAP frame, not world frame** - a real conceptual bug, not a
   typo. Map frame's origin coincides with wherever SLAM/localization considered its own
   start (the robot's spawn pose), so spawning at world `(-3.0, 0.0)` means the map-frame
   initial pose is `(0, 0, 0)`, not `(-3.0, 0.0, 0)`. Symptom was "Robot is out of bounds of
   the costmap!" repeating - not an obviously pose-related error message, worth remembering
   for next time this class of symptom shows up.
4. **SLAM map coverage left zero margin at the robot's own start pose.** The scripted
   coverage-driving sweep (see below) only ever moved `+x`/lateral from spawn, so the saved
   map's boundary sat almost exactly at the start pose (`origin: [3.21e-05, ...]`, essentially
   0). Combined with bug #3's fix (initial pose now correctly at map `(0,0,0)`), the robot's
   own starting position was right at the map's edge - not comfortably inside it. Fixed by
   padding the saved `.pgm` with a 30px (0.9 m) border of "unknown" on all sides and shifting
   `origin` accordingly, rather than re-driving the whole sweep.
5. **Circular `robot_radius` was undersized relative to the actual chassis.** Used `0.12` m
   (roughly the face-normal half-width, 0.095 m, plus a little margin) but the chassis is
   square (190x190mm) - the actual worst-case corner-to-center distance is
   `sqrt(0.095^2+0.095^2) = 0.1344` m. A circular collision-radius approximation smaller than
   the real diagonal under-represents the footprint at the corners specifically. Fixed:
   `robot_radius: 0.14` (both costmaps). Found while investigating an apparent physical
   stall (see below) - plausible contributing cause, though not confirmed as the sole cause.
6. **`xy_goal_tolerance: 0.1` combined with NavFn triggered a real, reproducible failure
   mode near the goal.** The robot would close to within ~0.15-0.2 m of a goal, and NavFn
   would then fail to plan the remaining short hop ("GridBased: failed to create plan with
   tolerance 0.20") - a known category of NavFn brittleness on very short/near-degenerate
   start-goal distances. The robot never got a chance to be declared "close enough" before
   the last-mile replan attempt failed, and the resulting cancel/retry/spin/wait/retry loop
   never converged within the wait budget. Fixed: loosened `xy_goal_tolerance` to `0.2` and
   planner `tolerance` to `0.25`, giving the goal checker a chance to accept before that NavFn
   edge case triggers. **Confirmed fixed** for the one goal tested after the change (see
   below) - not yet proven across the full reliability run.
7. **Repeated `kill -9` across ~15+ restart cycles in one session degraded DDS reliability**,
   visible as `/dev/shm` accumulating 87 stale `fastrtps_*` shared-memory segments and,
   separately, the Nav2 lifecycle manager hanging indefinitely at "Configuring map_server"
   with no error - just silence. Cleaning `/dev/shm/fastrtps_*` (safe once no ROS processes
   are running) and restarting the `ros2` daemon fixed it once, but **the hang recurred on the
   next fresh launch** after another clean restart. This looks like genuine session-level
   infrastructure fragility (many hours of continuous heavy ROS2/Gazebo process churn) rather
   than something fixable by continuing to restart - flagged as unresolved, not chased further
   in this session.

## SLAM mapping - completed

Scripted a 12-segment coverage-driving sequence (forward/turn segments via direct `cmd_vel`
publishing, since interactive teleop can't run headlessly) covering a diagonal swath of the
depot. Every segment's odometry delta matched its commanded velocity/duration exactly - no
signs of collision or getting stuck during the sweep itself. Saved a real, non-degenerate map
via `slam_toolbox`'s `save_map` service (worked around the same space-in-path bug as before -
see below - by saving to `/tmp` and copying in): 336x196 px @ 0.03 m/px, 2.4% occupied, 38.0%
free, 59.6% unknown (padded afterward per bug #4 above, final size 396x256).

**Same space-in-path bug, a third occurrence**: `slam_toolbox`'s `save_map` service internally
invokes `map_saver_cli` as a subprocess, which choked on this workspace's space-containing
path exactly like `launch`'s `Command()` and `ros_gz_sim`'s `gz_args` did in Phase 1. Same
workaround as before: generate to a space-free path (`/tmp`), copy into the project after.
Three independent tools tripped by the identical bug class now - noting again, as in Phase 1,
this is a real, recurring cost of this project's directory name, not a one-off.

## AMCL localization - confirmed working well (positive finding)

After fixing bugs #3/#4, AMCL converges and **tracks accurately**: drove the robot ~0.72 m
forward (odometry ground truth) and AMCL's estimate was within ~1.3 cm of that figure - real
scan-matching convergence, not just trusting odometry. This is exactly the kind of result
worth recording as a genuine positive, not just noting the absence of errors: a ~180 deg FOV
localizes reliably here, at least for straight-line motion in a modest, non-adversarial
region of the map.

## A genuine, unresolved map-quality issue: a "ghost obstacle"

While debugging why a goal at map `(2.0, 0.0)` failed to plan, directly inspected
`/global_costmap/costmap` values along the straight line from `(0,0)` to `(2,0)` rather than
guessing further. Found: clean (value 0) from x=0 to x≈1.50, then a jump to 96-100
(near-lethal, inflation-adjacent) from x≈1.55 to x≈1.95, then back to 0 exactly at the goal.
**No designed world geometry exists anywhere near that map location** (nearest real features -
the shelf field, pillars - are 1+ m away in world coordinates). This is a spurious artifact in
the SLAM-built map, not a real obstacle, and not a Nav2 configuration issue - most likely a
SLAM pose-drift/scan-registration error during the single-pass coverage sweep (no loop closure
was deliberately engineered into the sweep beyond one incidental spin). Confirmed its
existence and extent directly; did not root-cause the exact mechanism further.

**Practical handling in this session**: rather than re-run the whole coverage sweep hoping for
a cleaner result, verified individual goal legs directly against the live costmap
(`check_costmap_line.py`, a small diagnostic script written for this) before attempting them,
to distinguish "real planning/config bug" from "known map artifact in this specific region."
This is a reasonable engineering workaround for *this session*, but it is not a fix - the
underlying map still has the artifact, and any goal search that doesn't pre-check against it
will hit the same wall. **Open item for whoever picks this up next**: either re-run SLAM
mapping with a more deliberate loop-closure-friendly path (e.g., closing at least one loop
early and often, not just at the very end), or accept the current map for demo purposes and
regenerate a clean one only when this matters (e.g., before Phase 10's evaluation runs).

## Reliability testing - partial, not the full bar

Pre-verified 5 chained goal-to-goal legs against the live costmap (all clean, avoiding the
ghost-obstacle region) before attempting them via the actual `NavigateToPose` action:
`(0,0)->(1.2,0)->(0.7,0.8)->(0,1.5)->(-0.5,0.8)->(0,-1.5)`.

- **Goal 1** `(0,0)->(1.2,0)`: failed twice before the tolerance fix (bug #6) - confirmed via
  direct costmap inspection that the path was clean throughout, and via `amcl_pose` that the
  robot got to within ~0.2 m of the goal both times, consistent with the NavFn short-hop
  diagnosis. **After the tolerance fix and a `/dev/shm` cleanup + daemon restart, succeeded
  cleanly in 5.5 s.**
- **Goal 2** `(1.2,0)->(0.7,0.8)`: failed. The robot ended up at `(3.42, 0.64)` per AMCL -
  well past the intended target in an unexplained direction. Not root-caused before the
  session's infrastructure issues (below) took over as the blocking concern; plausibly related
  to the ghost-obstacle region (which sits between these two points on some candidate paths)
  causing NavFn to plan a long detour that then went wrong, but this is a hypothesis, not a
  confirmed diagnosis.
- **Goals 3-5**: not attempted - the Nav2 lifecycle manager began hanging indefinitely at
  "Configuring map_server" (bug #7) on the next restart, and recurred after the first
  mitigation attempt. Stopped live debugging at that point rather than continue an open-ended
  chase of what looks like session-level infrastructure fragility.

**SLAM-mode 5-goal test: not attempted this session** - AMCL-mode testing alone consumed the
remaining time/reliability budget.

## Hygiene tooling added (post-review)

The `/dev/shm` degradation isn't a one-off - Phase 10's evaluation harness will run the full
stack hundreds of times unattended, so this needed to become automatic, not a manual cleanup
step to remember. Added:
- `scripts/ros2_teardown.sh`: graceful shutdown (SIGINT, wait, SIGTERM, wait, SIGKILL only for
  stragglers) followed by `/dev/shm/fastrtps_*` cleanup, but *only* once nothing ROS/Gazebo-
  related is confirmed still running (never deletes shared memory a live process might hold).
  Safe to run as a saved script, unlike the inline `pkill -f` self-match footgun documented in
  Phase 1 findings - the search patterns don't appear in this script's own invocation line.
- `scripts/check_dds_health.sh`: counts stale `fastrtps_*` segments and fails loudly (nonzero
  exit, clear message) if above a threshold (default 20 - Phase 2 saw 87 right before the
  lifecycle hang). Wired directly into `spawn_robot.launch.py` (and therefore both
  `slam.launch.py` and `amcl.launch.py`, which include it) as the first thing
  `generate_launch_description()` does - a dirty session now fails the launch immediately,
  before Gazebo/Nav2 even start, instead of hanging 40+ seconds later with no clear cause.

## Recommendation: switch to CycloneDDS

Raised as a question by the reviewer; researched rather than guessed at. FastRTPS's shared-
memory transport creates ad-hoc per-participant segments in `/dev/shm` with no built-in
recovery if a participant is killed uncleanly - exactly this session's failure mode. CycloneDDS
by default doesn't create this kind of persistent `/dev/shm` clutter at all; its shared-memory
support (when explicitly enabled) is iceoryx-based, a separate, more mature shared-memory IPC
framework built around a daemon specifically designed to handle crashed participants
gracefully. Independent of the shared-memory question: multiple real-world reports specifically
describe Nav2 reliability issues being resolved by switching FastRTPS to CycloneDDS. It's also
a reasonable, well-supported choice if hardware deployment over WiFi ever happens - not
necessarily the theoretical fastest option on a lossy link (Zenoh reportedly does better there,
but is a bigger ecosystem shift, out of scope for now), but a safe, mainstream improvement over
the default. **Recommended.** Needs `sudo apt install ros-humble-rmw-cyclonedds-cpp` (confirmed
available, not yet installed) plus `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` - low cost,
easily reversible by unsetting the env var.

## Honest assessment of the done-bar

The agreed bar (5 consecutive successful goals, both AMCL and SLAM modes, no manual
intervention) is **not met**. What is confirmed, with real evidence rather than assumption:
the world, Nav2 stack, and SLAM-built map all function; AMCL localizes and tracks accurately;
one full navigation goal succeeded end-to-end after fixing real, now-documented bugs. What
remains open: a map-quality artifact, one unexplained overshoot, and recurring session-level
DDS/lifecycle infrastructure flakiness that needs either a fresh session or further
investigation to resolve - not something to paper over by quietly redefining "done."
