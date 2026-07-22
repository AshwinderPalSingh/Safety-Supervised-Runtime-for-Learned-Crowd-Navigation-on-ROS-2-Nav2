# Phase 1 findings log

Live log, per IMPLEMENTATION_PLAN.md's Phase 1 ("Robot digital twin"). Committed as findings
landed, same discipline as Phase 0.

## Packages built

- `crowd_nav_description`: `urdf/nvis_3302ard.xacro` (chassis/wheels/caster, all inertias
  computed from mass+geometry, not magic numbers), `urdf/lidar.xacro` (parameterized LiDAR
  macro), `MEASUREMENTS.md`.
- `crowd_nav_control`: `urdf/nvis_3302ard.ros2_control.xacro` (hardware swap point, §4.5),
  `config/diff_drive_controller.yaml`.
- `crowd_nav_gazebo`: `worlds/empty.sdf`, `launch/spawn_robot.launch.py`.

All three build clean with `colcon build`.

## Environment/tooling issues hit and fixed (recorded so they aren't rediscovered)

1. **XML comments can't contain a literal `--`.** `<!-- ESTIMATES -- to be replaced -->` broke
   the xacro parser outright (line/column pointed straight at it). Fixed; grepped the whole
   xacro tree for stray `--` afterward to confirm no others.
2. **This workspace's path contains spaces** (the project directory name), and `launch`'s
   `Command()` substitution `shlex.split()`s its resolved string — fragmenting the path into
   multiple bogus arguments. Hit this twice: once building the `robot_description` parameter
   (fixed by wrapping the path substitution in literal quote characters), and once via
   `ros_gz_sim`'s `gz_sim.launch.py` internally re-splitting its `gz_args` string the same way
   (fixed by bypassing that launch file entirely and invoking `ign gazebo` via a plain
   `ExecuteProcess` with pre-tokenized `cmd=[...]` — those list elements map 1:1 to argv with
   no further splitting). Confirmed `ign gazebo` itself loads the space-containing path fine
   when invoked directly; the bug was in the two launch-file layers, not the simulator.
3. **`robot_description` needs explicit `ParameterValue(..., value_type=str)`** — without it,
   `robot_state_publisher` tries to auto-infer the parameter's YAML type and fails on the URDF
   XML content.
4. **Bypassing `gz_sim.launch.py` loses its `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` setup** (needed for
   gz-sim's system-plugin loader to find `gz_ros2_control-system.so` — a separate search path
   from the dynamic linker/`LD_LIBRARY_PATH`, confirmed by testing: sourcing ROS setup alone
   left both `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH` empty). Fixed by
   setting it explicitly in the launch file from `LD_LIBRARY_PATH` before spawning `ign gazebo`.
5. **`pkill -f "pattern"` can match its own invoking shell** when the pattern text appears
   literally in the command line that's running the `pkill` call itself (it does, if you type
   the pattern as a literal string in the same script) — this silently killed the current
   shell mid-script more than once here, with no output and a bare nonzero exit code, which
   looked like a harness quirk before the actual cause was found. Switched to explicit-PID
   kills (`ps aux` first, then `kill -9 <pid>`) for all process cleanup from here on.

## LiDAR self-detection artifact — root-caused, still an open issue

Spawned cleanly, controllers activate, `/scan` publishes at the configured 5 Hz /
360°(±π) / ~1° resolution / 8 m range spec — all confirmed via `ros2 topic hz` and
`ros2 topic echo`. But in a completely empty world (nothing but a ground plane, which sits
below the sensor's horizontal scan plane and shouldn't be visible to it at all), roughly half
the 360 rays reported finite hits instead of `inf`.

**Investigation** (each step cheap, done before concluding anything):
1. Standoff sweep: 0.005 m → self-hits ~0.2–0.44 m (wide spread, up to 3.5 m at some angles).
   0.03 m → tighter band ~0.36–0.44 m. 0.08 m → ~0.49–0.58 m. Distance scales *up* with
   standoff, meaning more clearance does not converge toward eliminating it — ruled out
   "just mount it higher."
2. Explicit `<vertical><samples>1</samples><min_angle>0</min_angle><max_angle>0</max_angle>`
   block added (was previously left to gz-sim's default) — no change. Ruled out an implicit
   default vertical spread.
3. **Decisive test**: rotated the sensor's own yaw 180°. The affected angular region moved
   with it — same ~176° arc in the *sensor's own frame*, now occupying the opposite side in
   the *world* frame. Robot geometry (chassis, wheels, caster) is fixed in the world frame and
   did not move; if any of those were the cause, the artifact would have stayed put. This
   proves the artifact is internal to the `gpu_lidar` sensor's own rear-hemisphere rendering,
   not our model's geometry.
4. Tried the alternative CPU-raycast `type="lidar"` sensor as a possible clean fix — it's
   **non-functional on this install**: no error, but also no gz-transport topic at all, checked
   via `ign topic -l`. Not usable as-is.

**Current state**: reverted to `gpu_lidar` (the only working sensor type here) with a modest,
physically-defensible `lidar_standoff = 0.03` m (raising it further doesn't help, since
clearance isn't the actual cause). The front ~176° arc is clean; the rear ~176° reports a
self-detection artifact at a distance that shifts with standoff height, in an empty world.

**This is a genuine open issue, not resolved, flagged clearly rather than papered over.** It
does not block Phase 1's own done-criteria (spec-compliant publishing, confirmed), but it will
matter as soon as Phase 2 feeds this into AMCL/costmap — a phantom ~0.3–0.5 m ring of
"obstacles" behind the robot at all times would corrupt local costmap generation and likely
paralyze planning. Options for Phase 2, not decided yet:
- Live with gz-sim's rendering limitation and mask/ignore the known-bad rear sector in the
  observation/perception layer (software workaround, doesn't fix the sensor but makes it usable).
- Investigate further with actual visual tools (a GUI or rendered image dump — not possible
  headlessly with what's set up so far) to see if there's an ogre2/gz-sim config flag that
  fixes the rear-frustum rendering specifically.
- Re-check whether a future patch-level gz-sim 6.x update (still Fortress, not a distro bump)
  fixes this, since it looks like a real renderer bug rather than a config mistake.

## Motion verified

Published `{linear: {x: 0.5}, angular: {z: 0.3}}` to `/diff_drive_base_controller/cmd_vel_unstamped`
for 4 s. Odometry moved from `(0, 0)`, yaw 0 to `(1.439, 0.931)`, quaternion `z: 0.514` —
confirms real combined forward+turning motion, not just clean startup logs.

## Phase 1 status: functionally done, one open issue carried forward

URDF valid (`check_urdf` passes), robot spawns, hardware interface + both controllers
activate cleanly, LiDAR publishes at the exact configured spec, robot drives under a
velocity command (teleop keyboard itself can't be driven headlessly in this environment —
this cmd_vel test is the equivalent verification of the same control chain a human would
exercise interactively). `MEASUREMENTS.md` lists every `[PENDING]` value. The LiDAR
rear-hemisphere artifact above is carried forward as a named risk into Phase 2, not silently
dropped.
