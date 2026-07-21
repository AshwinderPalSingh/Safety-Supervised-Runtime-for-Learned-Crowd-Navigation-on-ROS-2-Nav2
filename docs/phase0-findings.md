# Phase 0 findings log

Live log, per finding, per IMPLEMENTATION_PLAN.md's Phase 0. Each entry below was committed
as it landed, not written up after the fact. See IMPLEMENTATION_PLAN.md for what each check
is for and what "done" means.

## Environment survey (2026-07-21)

- ROS 2 Humble installed at `/opt/ros/humble`.
- Gazebo: `gz sim` reports **Gazebo Sim 6.18.0 = Fortress**. Confirmed correct distro, not
  Garden/Harmonic.
- Python 3.10.12, no PyTorch installed system-wide.
- No GPU (`nvidia-smi` absent) — confirms the CPU-only ONNX Runtime call in
  IMPLEMENTATION_PLAN.md §1.3 is correct for this machine, not just in general.
- `colcon` present. `cmake`/`g++`/`libboost-all-dev` present (needed to build Python-RVO2 for
  the CrowdNav checkpoint validation).
- `ros-humble-nav2-*` (mppi-controller, amcl, costmap-2d, etc.) and `ros-humble-slam-toolbox`
  already installed. `ros-humble-gz-ros2-control` is **available via apt but not installed**.
- **Blocked, needs you**: installing `gz_ros2_control` requires `sudo apt install
  ros-humble-gz-ros2-control`, and this machine has no passwordless sudo. I can't run this
  myself — please run it (or grant a way for me to), and I'll pick the diff-drive-demo check
  back up. Nothing else in Phase 0 depends on it, so it's not blocking the rest of this phase.

## World scale (tugbot_depot)

Downloaded via `ign fuel download -u https://fuel.gazebosim.org/1.0/MovAi/worlds/tugbot_depot`
(the correct Fortress-era CLI on this machine — `/usr/bin/gz` here is actually **Gazebo
Classic 11**'s introspection tool, a separate package installed alongside Fortress; the
Fortress binary/CLI is `ign gazebo` / `ign fuel`. Worth knowing before anyone reaches for `gz`
expecting Fortress verbs).

**RTF, measured directly** (not estimated): sampled the live `/world/world_demo/stats` topic
on a running headless server. **`real_time_factor` ≈ 0.996–1.0 (essentially real-time)**,
physics-only. Not a risk on this machine.

**Headless rendering hangs** — a real blocker, found by accident: running the world
server-only (`ign gazebo -s -r`) with Tugbot's stock sensor plugins enabled (depth cameras via
the `Sensors`/ogre2 system) **hangs indefinitely** with zero console output; a 45 s run had to
be killed (exit 124). Stripping the `Sensors`/`Imu` system plugins from a scratch copy of the
world fixed it immediately — full clean startup, runs to completion. Root cause: no
Xvfb/EGL headless-rendering setup on this machine, and ogre2 blocks trying to get a GL
context. **This matters beyond Phase 0**: our own robot's 2D LiDAR is very likely implemented
as a `gpu_lidar` sensor, which goes through the same rendering system — Phase 1 will hit this
identical hang unless headless rendering is fixed first (install Xvfb, or confirm EGL software
rendering, or use a CPU raycast lidar sensor type instead of the GPU one). Flagging this now so
it doesn't get rediscovered as a mysterious Phase 1 hang.

**Aisle widths — measured directly from the SDF, not estimated.** The depot's collision
layer (`model name='depot_collision'`) is a single hand-authored model made entirely of
primitive `<box>`/`<cylinder>` collisions with literal numbers — no mesh collision at all.
Three parallel rows of shelf corner-posts (`shelfs1..18`, 0.03 m-radius cylinders) sit at
y = -0.018, 2.566, 5.150 m. Row-to-row spacing: **2.58 m center-to-center (~2.52 m clear)**.
Cross-aisles between shelf clusters within a row: ~2.57–2.68 m. For our 190 mm robot that's
**≈13× its own width** — for Tugbot (598 mm) it's ≈4.2×, a sensible clearance for that AMR
class. This is the same ~3.5× ratio from §1.1's Tugbot-footprint argument, now confirmed with
real numbers pulled straight from the world file instead of inferred.

Side finding, independent of scale: the shelf "collision" is only 3 cm corner posts — the
robot can drive straight through what looks like a solid shelf row today between the posts.
Doesn't change the scale conclusion, but worth knowing before assuming the depot is
collision-realistic at any scale.

**Rescale plan, revised — better than what IMPLEMENTATION_PLAN.md §1.1 anticipated.** The
gz-sim mesh/collision-scale bugs I flagged (gz-sim#2656, ros_gz#587) are specific to `<mesh>`
geometry. They're irrelevant here: `depot_collision` has no mesh, so a uniform rescale is just
multiplying every collision primitive's pose and box/cylinder size by 0.27 directly in the SDF
— arithmetic on literal numbers, not the engine's mesh-scale code path, so there's no bug class
to trigger. The *visual* appearance is a separate `<include>` of `OpenRobotics/models/Depot`
(a mesh) at the same base pose — that's the only place a mesh-scale glitch could show up, and
it would be purely cosmetic (collision, costmap, AMCL, and the eval metrics all key off
`depot_collision`, not the visual mesh). Plan: scale `depot_collision`'s primitives by 0.27×
for the evaluation world; try `<include><scale>` on the visual Depot mesh for visual
consistency, and if that glitches, fall back to synthesizing simple box/cylinder visuals
matching the already-known-exact scaled primitives (cheap, since every dimension is already
extracted above) rather than debugging the mesh. **Net effect: risk #1 in the plan is
downgraded** — collision-layer correctness no longer depends on any known-flaky gz-sim
behavior at all.
tugbot_depot stays the native-scale SLAM/AMCL/visual demo world exactly as planned; the scaled
copy (not yet built — that's Phase 1/2 world-authoring work, out of scope for Phase 0) becomes
the structured-depot evaluation world.

## Process correction: environment isolation

Initially installed the first few CrowdNav Python dependencies (torch, gym, cython, and an
upgrade of setuptools/packaging to fix an unrelated metadata bug) via `pip install --user`
directly into the shared user site-packages, before switching to an isolated `virtualenv`
(the stdlib `venv` module needs `python3.10-venv`, which needs sudo — same blocker as
gz_ros2_control below, so used the pure-Python `virtualenv` package instead, no sudo needed).
That initial `--user` install touched `setuptools`/`packaging` versions in the same
site-packages `colcon` uses. Checked immediately (`colcon version-check`, `colcon --help`) —
both still worked — but flagging this plainly rather than treating "still works" as "no
harm done": if anything colcon-related looks off later, this is the first place to look.
Everything after that point ran inside `crowdnav_venv`, fully isolated.

## gz_ros2_control diff-drive demo

Needed two `sudo apt install` runs from you (no passwordless sudo on this machine, so I
couldn't run them myself): `ros-humble-gz-ros2-control` then `ros-humble-gz-ros2-control-demos`.
Both verified installed after you ran them (checked `apt list --installed` and the actual
plugin `.so` on disk, not just trusting the report).

The stock demo launch (`diff_drive_example.launch.py`) defaults to `-r -v 1 empty.sdf`
(GUI client, no `-s`) — on this machine (no Xvfb/display, see the headless-rendering finding
above) that would need a display. Copied the launch file to the scratchpad and added `-s`
(server-only) to `gz_args`; everything else identical to the stock demo, no changes to the
installed package. Ran it headless:

- `GazeboSimROS2ControlPlugin` connects, loads the URDF, initializes/configures/activates
  `GazeboSimSystem` hardware for `left_wheel_joint`/`right_wheel_joint` — clean, no errors
  (one cosmetic KDL warning about root-link inertia, harmless).
- `controller_manager` loads and activates both `joint_state_broadcaster` and
  `diff_drive_base_controller` — clean.
- **Confirmed actual motion, not just clean startup logs**: published
  `{linear: {x: 0.5}}` to `/diff_drive_base_controller/cmd_vel_unstamped` for 4 s;
  `/diff_drive_base_controller/odom` moved from `x≈0` to `x≈1.685 m` — consistent with the
  config's `max_acceleration: 1.0 m/s²` ramp-up, not a fluke number.

**Phase 0 done, cleanly, on the last remaining item.** `gz_ros2_control` is confirmed to work
headlessly end-to-end on this machine, matching IMPLEMENTATION_PLAN.md §1.6 exactly. All
background/demo processes killed and confirmed gone afterward.

## SARL checkpoint validation

Pass/fail threshold, pinned before running anything (per your ask): the SARL paper (Chen et
al., ICRA 2019, "Crowd-Robot Interaction") reports **0.99 success rate on 500 test cases in
the invisible-robot setting** for SARL (LM-SARL, a variant, reports 1.00 — we're using plain
SARL). Threshold for treating `tkkim-robot/Gazebo-CrowdNav`'s checkpoint as valid: **success
rate ≥ 0.95** (small tolerance band for run-to-run RL variance). Below 0.90 is a hard fail —
training from scratch starts immediately as a background job if that happens, per your
standing ask, and I'll tell you the moment that's triggered since it changes the timeline.

*(checkpoint result recorded below once the run completes)*

## ONNX Runtime vendor package

Built `crowd_nav_onnxruntime_vendor` (`crowd_nav_ws/src/crowd_nav_onnxruntime_vendor/`):
CMake `file(DOWNLOAD)` + `file(ARCHIVE_EXTRACT)` of the official prebuilt Linux x64 CPU
release, no external vendor-package dependency, exactly as planned in §1.3. `colcon build
--packages-select crowd_nav_onnxruntime_vendor` succeeds. Added a real plumbing test
(`test_onnxruntime_link`, not just a compile check): loads a hand-exported trivial ONNX model
(`Linear(4,1)`, weight=1 bias=0, so output = sum of inputs — a known-answer check) and runs
inference through the vendored runtime. **Result: 10.0 for input [1,2,3,4], as expected. PASS.**

**Real bug caught in the process, not hypothetical**: originally pinned ONNX Runtime 1.17.3
(matches what IMPLEMENTATION_PLAN.md §1.3 says — "mature, well within Humble's window"). The
trivial model, exported with this machine's PyTorch (2.13, current dynamo-based exporter),
carries **ONNX IR version 10**. ONNX Runtime 1.17.3 hard-rejects it at load: *"Unsupported
model IR version: 10, max supported IR version: 9"* — a crash (`Ort::Exception`, SIGABRT), not
a warning. **Bumped the vendored version to 1.20.1** (confirmed via ONNX Runtime's own
compatibility docs: IR version 10 support landed in 1.19). Retested — passes cleanly.

This is exactly the kind of version-skew problem the Phase 0 SARL export spike exists to catch
before it surfaces during Phase 8 with a much more complex model — recorded here since it
already surfaced, on the *first* model exported on this machine. **Action for Phase 8**: when
exporting SARL's real value network, confirm the exporter's emitted IR version against
whatever ONNX Runtime version is pinned at that time — don't assume 1.20.1 stays sufficient if
the pinned PyTorch version changes between now and then.

Also note for later phases: `ament_export_targets`/`install(TARGETS ... EXPORT ...)` does not
work for CMake `IMPORTED` libraries (tried it first, fails: *"install TARGETS given target
'onnxruntime' which does not exist"* — imported targets aren't installable target artifacts).
Downstream packages (`crowd_nav_policy_adapters`, `crowd_nav_controller`, from Phase 6 on)
should consume this vendor package via the classic `ament_export_include_directories()` /
`ament_export_libraries()` + `${crowd_nav_onnxruntime_vendor_INCLUDE_DIRS}` /
`${crowd_nav_onnxruntime_vendor_LIBRARIES}` pattern, which is what's wired up now — not a
modern namespaced `crowd_nav_onnxruntime_vendor::onnxruntime` target (that was the original
design in IMPLEMENTATION_PLAN.md's sketch; doesn't work for a vendored prebuilt .so this way).
