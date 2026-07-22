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

**Confirmed version**: `libignition-gazebo6` `6.18.0-1~jammy` (`ign gazebo --versions` →
`6.18.0`). Recorded precisely per the standing ask, since "which gz-sim version" is exactly
the kind of detail that matters if this is ever reported upstream or re-checked after an
update.

**Upstream issue search**: no exact match found for this specific symptom (a clean, roughly
2-way angular split where the affected region is tied to the *sensor's own frame*, confirmed by
the yaw-rotation test). The closest related report is
[`gazebosim/gz-sim#2743`](https://github.com/gazebosim/gz-sim/issues/2743) ("Inaccurate GPU
Lidar") — general `gpu_lidar` accuracy degradation vs. Gazebo Classic, affecting both Fortress
and Harmonic, still open as of this check. That confirms `gpu_lidar` accuracy problems are a
known, acknowledged, still-unresolved category upstream, which fits what was found here, but
it doesn't describe this specific sensor-frame-relative rear-hemisphere pattern. This looks
like it could be a distinct, reportable finding — recommended, not yet done (filing an issue
is a public action; raised to the user rather than done unilaterally).

## LiDAR fix: masked at the sensor, not filtered downstream

Two options were on the table: mask the affected arc directly in the `gpu_lidar` sensor's
`<min_angle>`/`<max_angle>` (so `/scan` never contains those rays), or publish the full scan
and filter it downstream (a `laser_filters` node, or via Nav2's `obstacle_layer` config).
**Went with masking at the sensor.** Reasoning: a downstream filter means two representations
of the scan exist simultaneously (raw and filtered) — a straightforward way to end up with one
consumer (say, AMCL) reading the wrong one, producing a localization bug that looks like a
tuning problem and is expensive to trace back to its actual cause. One topic, one truth.

Implemented as two new xacro properties in `nvis_3302ard.xacro`:
`lidar_horizontal_fov` (`${pi}`, was `${2*pi}`) and `lidar_samples` (`180`, was `360`,
scaled down with the FOV to hold the ~1°/sample resolution spec constant) — both at a single
call site, with an inline comment pointing back to this file and noting they restore to
`${2*pi}`/`360` in one place if a fixed gz-sim version is ever confirmed.

**Verified clean**: respawned with the masked FOV — `angle_min`/`angle_max` now ±90°, 180
samples, **0 of 180 finite ("unexpected hit") readings** in the empty world (previously 176 of
360). Scan rate still confirmed at ~5 Hz. Fully resolved as a practical matter, at the cost of
field of view.

**This is a real, stated cost, not a free fix** — the robot now runs a ~184°-clean, masked
down to a clean ±90° (180°) forward-facing sensor, not the original 360° spec, on top of the
already-deliberate 8 m range cap. That combination (180° / 8 m) is closer to a real budget
LiDAR than the original spec and should be documented as a deliberate, known limitation in the
project README, not discovered later as an accident. Three concrete consequences flagged for
Phase 2, not yet acted on:
1. **AMCL will be materially worse** with ~180° than 360° — workable (plenty of real robots
   run 180° LiDARs) but weaker rotational constraint; expect to need more particles and more
   careful `laser_model_type` tuning than a 360° setup would.
2. **The local costmap won't clear behind the robot** — obstacles passed will persist until
   they age out. `obstacle_layer` raytracing and the costmap's decay behavior need to account
   for this; reversing recoveries are less safe as a result.
3. **Compounds with the 8 m range cap** — deliberately pessimistic in both range and field of
   view now. Defensible (closer to a real budget sensor), but a deliberate limitation to state
   plainly, not an accident to discover later.

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
