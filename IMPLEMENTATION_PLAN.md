# Safety-Supervised Runtime for Learned Crowd Navigation on ROS 2 Nav2
## Implementation Plan v1.4 (2026-07-21, updated 2026-07-22) — Phase 0 closed

No code has been written yet. This document is the deliverable for this pass. It is meant to
be read once in full, then used as a checklist during the phased build.

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

### 1.2 Pedestrian simulation: HuNav vs custom

`hunav_gazebo_fortress_wrapper` (robotics-upo) is real, Fortress+Humble native, and is the
right choice for the `reactive` pedestrian mode — it wraps HuNavSim's social-force-based
agents behind a Gazebo plugin, with six behavior types (regular / impassive / surprised /
scared / curious / threatening).

None of those is a true non-reactive control condition. "Impassive" is the closest, but it
still routes through HuNav's own behavior engine for the pedestrian's path — it's not
guaranteed to be a pure fixed-waypoint follower with zero awareness of anything. Rather than
fighting HuNav's engine to suppress that, I'm building a second, independent mechanism for
`non_reactive` mode: a small custom node/plugin that drives Gazebo actors along fixed,
pre-scripted waypoint loops with no robot-awareness and no social force model at all. Both
mechanisms publish to the same ground-truth human-state topic/schema, so everything
downstream (`GroundTruthHumanSource`, observation builder, evaluation harness) is blind to
which one is active — pedestrian mode is a launch-time switch, not a code fork.

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

### 1.5 Nav2 KeepoutFilter — no surprises

Confirmed: the mask topic uses latched/transient-local QoS specifically so a new
`OccupancyGrid` can be republished at runtime and picked up without restarting the filter.
Our zone-manager node is a plain publisher with a CRUD service in front of it — no Nav2 patches,
no custom `CostmapFilter` plugin needed. This part of the brief is exactly as easy as it looks.

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
| `crowd_nav_msgs` | `HumanState[]`, `AddZone`/`RemoveZone` srv, `InterventionEvent` msg | No |
| `crowd_nav_onnxruntime_vendor` | Minimal vendored ONNX Runtime CMake package | No |
| `crowd_nav_perception` | `HumanStateSource` interface, `GroundTruthHumanSource` (+ degradation model), `TrackedHumanSource` stub | No |
| `crowd_nav_observation` | Observation builder library/node, canonical `WorldState` struct | No |
| `crowd_nav_policy_adapters` | `PolicyAdapter` interface, `SarlAdapter`, ONNX shape-validation helper | Onnxruntime only |
| `crowd_nav_controller` | `nav2_core::Controller` plugin: rate handling, accel clamping, latency watchdog + failover | Yes (nav2_core) |
| `crowd_nav_safety_supervisor` | Forward-sim, costmap/keepout check, OOD detector, fallback trigger, intervention logging | Yes (costmap_2d) |
| `crowd_nav_costmap_filters` | Zone-manager node (mask gen + republish + CRUD service), uses stock `KeepoutFilter` | Yes (map_server/costmap_filters) |
| `crowd_nav_pedestrians` | HuNav launch/config integration + custom non-reactive scripted-actor node | No (Gazebo/HuNav only) |
| `crowd_nav_evaluation` | Scenario suite, harness runner, metrics collector, CSV + plots | Yes (transitively) |
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

**Phase 1 — Robot digital twin**
`crowd_nav_description` URDF/xacro per §3 of the brief (confirmed geometry + `[PENDING]`
estimates, computed inertia tensors, LiDAR xacro macro with range/rate/resolution/noise as
params), `crowd_nav_control` ros2_control block with the hardware-plugin swap point, spawn
launch in an empty world, keyboard teleop. **Done:** robot drives under teleop, `/scan`
publishes at the configured rate/range/noise, MEASUREMENTS.md is written and matches every
`[PENDING]` xacro property.

**Phase 2 — Baseline Nav2 (MPPI) + AMCL + SLAM toolbox**
Stock Nav2 with `nav2_mppi_controller`, AMCL against a saved map, `slam_toolbox` online
mapping as an alternate launch mode, on whichever world(s) Phase 0 settled on. This *is* the
`baseline_mppi` eval config's runtime. **Done:** robot navigates a goal in both AMCL mode
(saved map) and SLAM mode (online map); SLAM mode is a launch-file demo only — it is not
wired into the evaluation harness (see §6).

**Phase 3 — Dynamic keep-out zones**
Zone-manager node (mask generator + republisher + `AddZone`/`RemoveZone` service),
`costmap_filter_info_server` + `map_server` instance for the mask, stock `KeepoutFilter` in
the local costmap. **Done:** a zone added mid-run blocks a corridor and MPPI visibly replans;
removing it opens the path back up.

**Phase 4 — Pedestrian simulation**
HuNav reactive integration (`regular` behavior) + custom non-reactive scripted-actor node,
both publishing a common ground-truth human-state topic. **Done:** both pedestrian modes are
launch-time switchable; topic echo shows identical schema regardless of which backend is
running.

**Phase 5 — HumanStateSource, perception degradation, observation builder**
`GroundTruthHumanSource` wrapping the Phase 4 topic, degradation model (Gaussian
position/velocity noise, dropout, latency, max-range, costmap-based occlusion check —
occlusion is the one sub-feature I'd cut first under time pressure, see §6),
`TrackedHumanSource` stub, observation builder producing SARL's flat padded vector with a
documented schema (human ordering, padding, frame, units). **Done:** unit tests
(fixed input → known output vector) green; a manual degradation sweep (e.g. dropout=0.3 over
1000 synthetic ticks) produces the expected missing-detection rate within sampling noise.

**Phase 6 — Synthetic adapter + ONNX plumbing validation**
Added after review: starting the policy-integration work directly with SARL bundles two
independent risks together — "does the ONNX/controller-plugin plumbing work" and "did I
correctly reimplement SARL's candidate-action search" — and a failure could be either. So
this phase builds `PolicyAdapter`, the ONNX shape-validation helper, and
`crowd_nav_onnxruntime_vendor`, wired to a **hand-written trivial ONNX model** that declares
the exact tensor shapes `SarlAdapter` will need (a batch of candidate joint-states in, a
scalar per row out) but has trivial internals (e.g. a single linear layer, or a constant
output). `DummyAdapter` implements the full interface against it — `buildInputs` produces a
real batch of one-step-propagated candidates (reusing the kinematics/propagation code SARL
will also need), `selectAction` does something simple and deterministic (e.g. ignore the
network's meaningless output and pick the candidate whose heading is closest to the
goal direction) just to prove data moves correctly end-to-end. **Done:** a standalone C++
harness loads the dummy ONNX model, runs shape validation, performs inference, and decodes an
action, deterministically and reproducibly, with no crashes — this is the first real exercise
of the ONNX vendor package and the adapter interface, with zero SARL-specific risk mixed in.

**Phase 7 — Controller plugin**
`nav2_core::Controller` plugin: hold-last-action + velocity smoothing across the 4 Hz/20 Hz
mismatch, acceleration clamping to configured robot limits, inference-latency watchdog
(covering supervisor-check time too, per §1.7) with failover to MPPI. Built and validated
against `DummyAdapter` from Phase 6, not SARL — this proves the controller plugin's rate
handling and watchdog/failover mechanics independently of whether SARL's search logic is
correct. **Done:** a `policy_raw` config running `DummyAdapter` drives the robot end-to-end in
Nav2 in the open-arena world; artificially stalling inference demonstrably triggers failover
within one watchdog window.

**Phase 8 — SARL ONNX export + SarlAdapter**
Real export of the Phase 0-validated checkpoint (strip the debug `.cpu().numpy()` line, export
the value net, verify bit-close against the original PyTorch model), then `SarlAdapter`
implementing the candidate-action
generation + one-step propagation + batched inference + argmax from §1.4, including the
`policy_radius`/`robot_radius` split from §4.3. Swapped in for `DummyAdapter` via a config
change only (adapter type + model path) — no changes to the Phase 7 controller plugin, which
is itself a live test of the adapter-swap promise the whole `PolicyAdapter` design exists for,
before HEIGHT or ORACLE-Nav ever show up. **Done:** a standalone C++ harness (no Nav2, no
Gazebo) feeds hand-crafted scenarios through `SarlAdapter` and its chosen action matches the
original Python SARL's action on the same inputs; flipping the Phase 7 controller's config
from `dummy` to `sarl` requires zero code changes and produces visibly SARL-like behavior
(yielding, path curvature) instead of the dummy heuristic.

**Phase 9 — Safety supervisor**
Forward-simulation of commanded velocity, costmap + active-keep-out-zone check, OOD detector
(§4.4 defines concrete criteria — see below, this needed sharpening per the brief's own
prompt), fallback to MPPI or controlled stop, per-cause intervention logging. **Done:** an
engineered unsafe command (e.g. one aimed straight at a keep-out zone) is demonstrably
rejected and logged with the correct trigger reason; supervisor geometry and OOD-threshold
unit tests green.

**Phase 10 — Evaluation harness**
Seeded scenario suite (open-arena × structured-depot) × (`baseline_mppi`, `policy_raw`,
`policy_supervised`) × (`reactive`, `non_reactive`), clean episode termination on
collision/timeout, CSV + metrics + plots, perception-noise sweep. **Done:** the full matrix
runs unattended and produces CSV + plots, including the noise-sweep results, with variance/
distributions reported, not just means.

**Phase 11 — CI and docs**
GitHub Actions per-PR gate: build workspace + compile plugins + run unit tests + lint (no
Gazebo, see §6). Separate nightly/manually-triggered workflow: launch the full stack
(Gazebo + Nav2 + one pedestrian) and drive one goal to completion in the open-arena world —
not a correctness check, just a smoke test for launch-file and parameter-schema rot, which
unit tests structurally can't catch and which is exactly what silently breaks in ROS 2
workspaces over time. README "how to add a new policy" walkthrough (documented, using HEIGHT
as the worked example, not implemented). Final MEASUREMENTS.md pass.

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

- Noise: seeded Gaussian draw per human per tick, seed threaded through from the scenario
  seed for reproducibility.
- Dropout: seeded Bernoulli draw per human per tick.
- Latency: implemented as a fixed-length ring buffer of true states per human (not an async
  delay), so replaying the same seed reproduces the same degraded sequence exactly.
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
sets `robot_radius = 0.3` m — roughly 3× our actual robot's physical radius (~0.095 m, half
the 190 mm chassis width). Feeding the network our true radius would itself be a distribution
shift, independent of everything else in §1.1/§1.4. `SarlAdapter` therefore takes two separate
config values: `policy_radius` (default 0.3 m, matching CrowdNav's training distribution —
fed into the network's self-state feature) and `robot_radius` (from the URDF/measured — used
everywhere the actual physical footprint matters: costmap inflation, the safety supervisor's
forward-sim collision check, and the OOD proximity threshold below). This split is a
`SarlAdapter`-specific detail; the generic `PolicyAdapter` interface doesn't need to know
about it.

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

### 4.5 Hardware boundary (ros2_control)

Today: `<hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>` in
`crowd_nav_control`'s `.ros2_control.xacro`, with `diff_drive_controller` talking to velocity
command interfaces and position/velocity state interfaces on `left_wheel_joint` and
`right_wheel_joint`. Documented contract for the future ESP32 plugin: implement
`hardware_interface::SystemInterface` exporting the identical interface names/types, register
via `pluginlib`, and swap only the `<plugin>` tag (plus whatever serial/I2C params the ESP32
interface needs) — `diff_drive_controller`, the URDF joint structure, and everything above the
hardware layer is unchanged. Nothing beyond this contract is built now.

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
6. **HuNav non-reactive gap** — solved by the separate scripted-actor mechanism (§1.2), not a
   residual risk, but flagged because it means slightly more custom Gazebo-actor code than
   "just use HuNav" implied.
7. **Reduced 8 m LiDAR interacting with costmap/MPPI/AMCL sizing in a large depot** — mostly
   dissolves if the world-scale fallback in §1.1 lands on a compact custom world (8 m comfortably
   covers a small floor plan); only a live risk if tugbot_depot ends up used at native scale
   for the actual evaluation, which is precisely what §1.1's decision tree tries to avoid.
8. **No trained SARL checkpoint upstream** (§1.8) — mitigated by using
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
  `robot_radius` (used for actual collision/costmap/supervisor geometry) comes from the URDF —
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
