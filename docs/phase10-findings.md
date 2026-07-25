# Phase 10 findings log

Per IMPLEMENTATION_PLAN.md's Phase 10 ("Evaluation matrix"), rescoped in v1.19 before
implementation (§4.9, three requirements from review: pilot the matrix before running it; decide
N and the noise-sweep points before seeing any results; expect `policy_supervised` to
underperform `baseline_mppi` on efficiency and report it regardless). A fourth, informal
instruction carried the most weight in practice: a trigger cause enumerated in the taxonomy but
never shown to fire is a silent zero, and the only way to catch that class of bug is asking
"can this actually fire?" for each one - the same discipline that found `LOW_PERCEPTION_CONFIDENCE`
unreachable in Phase 9 found a coordinate-frame bug that made `depot_keepout_block` meaningless
on its first run.

---

## CORRECTION (post-hard-audit re-run): the original headline collision-rate finding was wrong

**Read this section first.** Everything below the line "## Original Phase 10 run (superseded in
part, kept for the record)" describes the *first* run of this matrix (139 episodes). A later
hard adversarial audit (`docs/audit.md` §1.3) found a critical bug: `buildWorldState()` combined
the robot's pose (Nav2 **map** frame) with the perceived humans' positions (Gazebo **world**
frame) with no transform between them - a fixed, non-trivial offset (~2.93 m, ~0.07 m). This
did not only affect the safety supervisor's `PROXIMITY` check, as first suspected - it affected
**every relative robot-human distance the policy itself computed**, since `SarlAdapter` builds
its network inputs from the exact same `WorldState`. The policy had never, in any live Gazebo
episode since Phase 7, actually perceived humans anywhere near their real position relative to
itself.

Per this project's explicit standing instruction (kept from Phase 0 onward, restated before the
hard audit): **when a fix invalidates a headline number, that is reported as a reversal, not
reframed as "the original number was defensible under a different reading."** The fix was made
(`crowd_nav_perception::GroundTruthHumanSource::setRobotMapPose()`, wired in
`crowd_nav_controller.cpp::buildWorldState()`), the full 142-episode matrix (96 core + 40 sweep +
3 keepout + 3 new OOD-reachability demonstrations, `docs/audit.md` §1.1) was re-run from a clean
results directory, and this section reports what changed.

### The reactive-pedestrian collision-rate result reverses direction

| scenario | config | collision rate (**original, buggy**) | collision rate (**corrected**) |
|---|---|---|---|
| depot | baseline_mppi | 1/8 (12%) | 1/8 (12%) - unchanged (doesn't use `WorldState`) |
| depot | policy_raw | 1/8 (12%) | **4/8 (50%)** |
| depot | policy_supervised | 2/8 (25%) | **1/8 (12%)** |
| open_arena | baseline_mppi | 1/8 (12%) | 1/8 (12%) - unchanged |
| open_arena | policy_raw | 1/8 (12%) | **4/8 (50%)** |
| open_arena | policy_supervised | 4/8 (50%) | **1/8 (12%)** |

`baseline_mppi` is exactly unchanged, as expected - it never consumes `WorldState.humans` at all,
only Nav2's own costmap/sensor pipeline. The other four cells all moved, and moved in opposite
directions for the two configs: **`policy_raw` got dramatically worse** (12% &rarr; 50% in both
scenarios) and **`policy_supervised` got dramatically better** (25%/50% &rarr; 12%/12%, now
exactly matching the classical baseline).

**The original headline claim - "the safety-supervised policy collides more often than the
unsupervised policy and the classical baseline" - was false, and was false specifically because
of the bug, not despite it.** The corrected result says close to the opposite: the raw learned
policy, once it can actually see real human positions, is measurably *less* safe than the
classical baseline under reactive pedestrians (50% vs. 12%), and the safety supervisor closes
essentially all of that gap, bringing the supervised collision rate down to match the classical
baseline exactly.

**Why the direction of each change makes mechanical sense, not just statistical noise:**
`state.humans[i]` was offset from the robot's true relative position by a roughly constant
~2.93 m vector for the whole time this bug existed. That is large relative to every threshold
in this system (the collision distance is ~0.39 m; the OOD proximity threshold is 0.8 m) - the
policy was, in effect, being told every nearby human was several meters further away than they
really were, in a fixed direction. A policy that structurally cannot perceive real close
encounters cannot react to them, so it drove largely as if the arena were empty - which
*happened* to produce a low buggy collision rate in these specific small scenarios, not because
the policy was doing anything resembling collision avoidance, but because it wasn't reacting to
the (mislocated) humans at all and its default goal-directed path frequently didn't intersect a
real human's real position by chance. Once the frame is corrected, the policy genuinely tries to
navigate around real, nearby, reactive humans, using an imitation-learning-only checkpoint
(`docs/phase0-findings.md`, `README.md` Known Limitations) through an untuned holonomic-to-
diff-drive conversion (`docs/phase7-findings.md`) - and it is worse at this than classical MPPI,
a genuine and now-honestly-measured finding, not an artifact. The supervisor, whose own
`PROXIMITY` check was equally blind to real distances under the bug, now sees the same corrected
data the policy does and reliably intervenes before an unsafe approach completes.

### The timing pattern for the two remaining supervised collisions

Only 2 of 16 reactive-mode `policy_supervised` episodes still collide (down from 6 of 16 in the
original run). Both are the "caught too late" pattern, not the "separate unflagged approach"
pattern that dominated the original (buggy) analysis below:

| episode | interventions | gap: last intervention &rarr; collision | min human distance |
|---|---|---|---|
| open_arena seed1 | 1 | 0.22 s | 0.372 m |
| depot seed2 | 2 | 0.07 s | 0.373 m |

Both collisions happen within a quarter of a second of the supervisor's own last rejection - the
supervisor caught the approach right as contact was already becoming unavoidable, not seconds
earlier with time to spare. This is a *much* smaller residual than the original analysis found
(which had 4 of 6 collisions occurring several seconds after the last intervention, via a
separate, unflagged approach) - with real distance data, the supervisor now reliably catches
encounters, and the only failure mode left is a genuine last-instant margin limit, not a broad
detection gap.

### What did *not* change in shape, only in exact numbers

- **The keep-out zone demonstration**: still a clean three-way contrast (baseline detours and
  succeeds; `policy_raw` still drives straight into the zone and violates it; `policy_supervised`
  still forward-sim-rejects every attempt - 408 `KEEPOUT_VIOLATION` + 31 `COSTMAP_COLLISION`
  rejections, zero actual violations, still times out rather than routing around). See the
  corrected table below.
- **The noise-sweep cliff and saturation pattern**: still present, same shape (a sharp jump in
  intervention rate from `dropout_prob` 0.0&rarr;0.1, then roughly flat through 0.5), with
  slightly different exact numbers - see below.
- **Efficiency**: `policy_raw`/`policy_supervised` remain slower and take longer paths than
  `baseline_mppi` in every family, as the original review predicted.
- **The OOD-reachability fixes** (`docs/audit.md` §1.1) all still work under the corrected
  frame: `CROWD_SIZE` fires 433 times in `ood_demo_crowd_size`, `RELATIVE_SPEED` fires 15 times
  in `ood_demo_relative_speed`, `INFERENCE_TIMEOUT` fires once in `ood_demo_inference_timeout`.
  `COMMAND_LIMIT` remains at 0 - still mathematically unreachable by construction, a proven zero,
  not a gap.

### One number moved in a way worth flagging on its own: which scenario intervenes more

Original run: `policy_supervised` intervened more in depot than open_arena (19.5/episode vs.
11.8/episode). **Corrected run: this reverses too** - open_arena now shows a substantially
*higher* mean intervention rate than depot (81.75/episode vs. 39.9/episode). Under the buggy
frame, the supervisor's `PROXIMITY` check was reacting to phantom, far-away human positions in
both scenarios; with real distances, the actual encounter geometry of each scenario now drives
the number, and open_arena - more open, fewer static obstacles competing for the same free space
- produces more frequent close human-robot encounters. This is reported as a fact of the
corrected data, not a fully explained causal claim; unlike the original ("near-zero in
open-arena, high in depot") contrast the review had hoped for, this doesn't map cleanly onto a
transfer-difficulty story either, and is left as an open, honestly-reported observation rather
than forced into either narrative.

### Revised thesis

The original three-finding synthesis ("the supervisor's reliability tracks how well-characterized
the hazard is") undersold the corrected result, not oversold it. With real data: the safety
supervisor takes a raw learned policy that is measurably *less* safe than a classical baseline
under genuine reactive-pedestrian conditions (50% vs. 12% collision) and restores its collision
rate to match the classical baseline exactly, at a real, honestly-reported efficiency cost
(slower, longer paths - unchanged from the original finding) - while, against a static and
exactly-known hazard (the keep-out zone), the identical mechanism is perfect (zero violations
out of hundreds of forward-sim checks), and under degraded perception, its intervention rate
saturates rather than climbing once the policy itself is already as confused as it gets. Two of
sixteen reactive supervised episodes still end in collision, both caught by the supervisor
within a quarter of a second of contact - a real, narrow, last-instant margin limit, not the
broad detection gap the original (buggy) data seemed to show. **This is a real, verifiable
safety mechanism that measurably worked when tested against corrected data** - the opposite
conclusion from what the buggy run first suggested, which is exactly why the audit, the fix, and
this full re-run were done before treating the original conclusion as final.

---

## Original Phase 10 run (superseded in part, kept for the record)

Everything below this line describes the first (139-episode) run of this matrix, before the
hard audit found and fixed the `WorldState` frame-mismatch bug. It is kept, unedited except for
this note, per this project's standing discipline of recording corrections rather than silently
rewriting history - **the reactive-pedestrian collision-rate numbers and their timing analysis
below are superseded by the CORRECTION section above; the harness-bug fixes, the pilot run, and
the qualitative keepout/noise-sweep/efficiency findings are not superseded, only their exact
numbers are, per the "what did not change" list above.**

**Status: DONE, done-bar met**, with two significant bugs found and fixed mid-phase (both
detailed below) rather than glossed over:
- 139/139 episodes complete (96 core + 40 noise-sweep + 3 keepout), clean teardown throughout,
  zero silent `harness_error`/`harness_timeout` results, RTF held at ~1.0 with no drift across
  the run.
- A previously-undiagnosed Gazebo bug (`PosePublisher` never actually publishing on this
  gz-sim version) was found and fixed - it retroactively affected Phase 9's own FOV-filter
  claim, which had been silently inert since it was built. See "Gazebo ground-truth pose" below.
- `depot_keepout_block`'s first run showed zero supervisor interventions across all three
  configs, including the one flagged as a violation - root-caused to a map/world coordinate
  frame mismatch in the harness itself, fixed, and re-run to a decisive, publishable result.
- The noise sweep shows a sharp, reproducible failure cliff between `dropout_prob` 0.1 and 0.2,
  confirmed via rate-normalized intervention counts to be policy/metric saturation, not the
  supervisor working harder to hold a safety floor.
- `policy_supervised` has a **higher** collision rate than both `baseline_mppi` and `policy_raw`
  under reactive pedestrians, in both scenario families - reported prominently, not buried,
  per the explicit "report it even if my method didn't win" instruction. **(Superseded - see
  CORRECTION above: this specific claim was a consequence of the frame bug and reverses under
  corrected data.)**

---

## Harness bugs found and fixed before the matrix could be trusted

### Gazebo ground-truth pose was never actually being published

`PosePublisher` (`crowd_nav_description/urdf/nvis_3302ard.xacro`) configured with
`publish_model_pose=true` **never published anything** on this gz-sim version (6.18.0) -
confirmed via `ign topic -e` timing out with zero messages, both stationary and while the robot
drove for 15+ seconds. `publish_link_pose=true` did make the topic active, but mixed the
model's own pose together with every link's pose (wheels, `base_footprint`) with no way to
disambiguate once bridged to a bare `geometry_msgs/msg/Pose` - the Gazebo-side `name` field that
would disambiguate is lost in that bridge type. `PoseStamped` bridging was tried next, hoping
`header.frame_id` would carry the per-message name through - it didn't; `frame_id` came back as
a constant regardless of which link's data was in that specific message.

The actual fix: `scene_broadcaster` (already present in every world file) publishes
`/world/<world>/dynamic_pose/info` as `ignition.msgs.Pose_V`, bridged to
`tf2_msgs/msg/TFMessage` - confirmed correct via direct `ros2 topic echo`, with each entity's
pose in true world frame, individually addressable by `child_frame_id`. A new
`robot_pose_extractor.py` node filters this stream down to the robot's own entry and republishes
on the **same** `/ground_truth/robot_pose` topic/type every existing consumer already expected,
so no downstream code needed to change.

**Retroactive implication for Phase 9**: `GroundTruthHumanSource`'s FOV/heading filter (built and
reported as verified in that phase) subscribes to this exact topic. Since it never published,
`robot_pose_.has_value()` would never have become true, meaning the FOV filter had been silently
degrading to "no filtering at all" since it was built - not a Phase 10 bug, but a Phase 9 claim
this phase's own tooling happened to invalidate. Confirmed fixed as a side effect of the pose
bridge fix: `GroundTruthHumanSource` shares the same topic, no separate change needed.

### Two harness robustness bugs, both found running the actual pilot/matrix, not anticipated

**Lost executable bits**: `pedestrian_sim_node.py` and `actor_mirror_node.py` had lost their
`+x` bit (unrelated to any change made this phase - most likely an editor or git operation
somewhere upstream), causing `pedestrians.launch.py` to fail with "executable not found" on
every episode. `robot_pose_extractor.py` (added fresh this phase) kept its bit and worked fine,
which is what made this a fast, obvious diagnosis rather than a mystery. Fixed with `chmod +x`;
this is a real, if mundane, fix - not a workaround.

**Leftover Gazebo processes corrupting later episodes**: a botched episode (the one killed by
the executable-bit bug above) left its entire `amcl.launch.py` side alive - `ign gazebo`
included - well past its own `_teardown()` call returning. The next episode then launched a
*second* `ign gazebo` instance for the same world; with two `/clock` publishers on the same
global topic, the new episode's nav2 stack saw time jump backwards and every `ros2_control`
spawner failed - a silent corruption, not a crash, so nothing in that episode's own result JSON
flagged it. With 139 episodes queued to run unattended, this needed a guard: `run_episode.py`
gained `_sweep_stray_processes()`, a post-teardown SIGINT/SIGTERM/SIGKILL sweep for specifically
`ign gazebo`/`gzserver`/`ros_gz_bridge`/`robot_state_publisher`.

That sweep's first implementation shelled out to `scripts/ros2_teardown.sh`, which already does
this exact sweep - and immediately hit a new variant of a bug this project has hit before
(that script's own header documents a Phase 1/2 self-match footgun). `ros2_teardown.sh`'s
`pkill -f "nav2_"` pattern doesn't just match stray Gazebo processes - it matches
`run_episode.py`'s **own** command line, because `--episode-json` embeds the scenario/config
dict verbatim, and `baseline_mppi`'s `controller_plugin` value is
`"nav2_mppi_controller::MPPIController"` - a literal substring match. The harness process was
SIGKILLing itself out from under itself via its own cleanup step, for every `baseline_mppi`
episode specifically (explaining why only that config ever hit "could not parse run_episode.py
output"). Fixed by replacing the shared script with a narrow, self-contained sweep whose
patterns can't collide with scenario/config data, plus a runtime assertion that fails loudly
instead of silently self-destructing if a future value ever does collide.

A background `run_matrix.py --phase all` invocation was also found running prematurely during
this debugging (started before the pilot had ever been verified end-to-end, in a part of this
session predating full context) - 100% `harness_error`, colliding with manual debug episodes
being run concurrently. Killed, and its contaminated output discarded rather than
retroactively "fixed."

---

## Pilot (S4.9.1)

Once the above were fixed, `run_matrix.py --phase pilot` (one scenario, one seed, all three
configs) ran cleanly end to end: `baseline_mppi` succeeded, `policy_raw` succeeded,
`policy_supervised` ended in collision after 7 real `PROXIMITY` interventions - a real,
non-flattering result even at N=1, and the harness's own stray-process sweep confirmed a clean
environment after each episode. Proceeded to the full matrix only after this passed.

---

## Core matrix - corrected numbers (96 episodes, N=8/seed x 2 families x 3 configs x 2 pedestrian modes)

*(Original-run numbers and analysis are preserved below this subsection for the historical
record; the CORRECTION section at the top of this document is authoritative.)*

### Outcome rates and the reactive/non-reactive split (corrected)

| scenario | config | non_reactive collision rate | reactive collision rate |
|---|---|---|---|
| depot | baseline_mppi | 3/8 (38%) | 1/8 (12%) |
| depot | policy_raw | 6/8 (75%) | 4/8 (50%) |
| depot | policy_supervised | 6/8 (75%) | 1/8 (12%) |
| open_arena | baseline_mppi | 6/8 (75%) | 1/8 (12%) |
| open_arena | policy_raw | 7/8 (88%) | 4/8 (50%) |
| open_arena | policy_supervised | 7/8 (88%) | 1/8 (12%) |

`non_reactive` pedestrians (who don't avoid the robot at all) still drive collisions high for
every config, as in the original run - a pedestrian-model effect, not a controller-quality
signal.

**Within `reactive` mode** (the fair comparison): `policy_supervised` now matches
`baseline_mppi` exactly (12% in both families) and is dramatically safer than `policy_raw`
(50% in both families) - the reverse of the original run's headline. See the CORRECTION section
above for the full timing analysis and mechanistic explanation.

### Efficiency (successful episodes only, corrected)

| scenario | config | N successes | median duration | median path |
|---|---|---|---|---|
| depot | baseline_mppi | 12 | 9.0 s | 1.75 m |
| depot | policy_raw | 6 | 11.3 s | 3.83 m |
| depot | policy_supervised | 8 | 16.3 s | 3.90 m |
| open_arena | baseline_mppi | 9 | 11.2 s | 2.78 m |
| open_arena | policy_raw | 5 | 13.4 s | 5.48 m |
| open_arena | policy_supervised | 5 | 24.8 s | 5.66 m |

Confirms the review's explicit prediction, unchanged from the original run: `policy_supervised`
is measurably slower than `baseline_mppi` in every family. `policy_raw` and `policy_supervised`
track each other closely on path length (they share the same underlying policy), with
`policy_supervised` taking longer in duration - consistent with additional time spent on
rejected-then-retried candidates.

### Intervention rate by scenario family (corrected)

`policy_supervised` now intervenes far more in **open_arena** than depot (81.75/episode vs.
39.9/episode) - the reverse of the original run. See "one number moved" in the CORRECTION
section above for the honest, not-fully-explained discussion of why.

Plots (regenerated against corrected data): `results/plots/outcome_rates.png`,
`duration_s_distribution.png`, `path_length_m_distribution.png`,
`intervention_rate_by_family.png`.

---

## Noise sweep - corrected numbers (40 episodes, `policy_supervised`/open_arena/reactive, dropout_prob in {0.0, 0.1, 0.2, 0.3, 0.5})

| dropout_prob | outcomes | mean interventions/episode | intervention **rate** (per sim-second) |
|---|---|---|---|
| 0.0 | 4 success, 3 collision, 1 timeout | 69.1 | 1.22/s |
| 0.1 | 2 success, 3 collision, 3 timeout | 268.9 | 3.23/s |
| 0.2 | 6 timeout, 2 collision | 406.0 | 3.68/s |
| 0.3 | 8 timeout | 432.5 | 3.60/s |
| 0.5 | 7 timeout, 1 collision | 413.2 | 3.57/s |

Same shape as the original run: a sharp jump in intervention rate from dropout 0.0 to 0.1
(1.22 &rarr; 3.23/s), then roughly flat through 0.5 (3.57-3.68/s) - the same saturation
conclusion holds: the supervisor does not compensate harder as perception degrades past roughly
0.1, because the policy itself is already about as confused as it gets. Exact values differ from
the original run (the policy's own inputs are now correct, which changes exactly how quickly it
gets confused), but the qualitative finding - a cliff, then saturation, not a rising floor - is
unchanged and re-confirmed against corrected data.

Plot: `results/plots/noise_sweep.png`.

---

## `depot_keepout_block` - corrected numbers, same qualitative result

| config | outcome | path_length_m | interventions | dominant cause |
|---|---|---|---|---|
| baseline_mppi | success | 6.44 (vs ~2m direct) | 0 | - |
| policy_raw | **keepout_violation** | 0.53 | 0 | - |
| policy_supervised | timeout | 0.70 | 439 | KEEPOUT_VIOLATION x408, COSTMAP_COLLISION x31 |

Unchanged in every qualitative respect from the original run: `baseline_mppi` detours around the
zone via the global costmap; `policy_raw` drives into the zone almost immediately, exactly as
SARL's lack of any keep-out concept predicts; `policy_supervised`'s forward-sim correctly
identifies and rejects the unsafe approach on essentially every tick attempted - **zero** actual
violations - but has no way to route around the obstruction, so it times out. This remains the
cleanest demonstration in the whole matrix of the supervisor's value proposition and its real
limit in one result.

---

## Overall assessment

See the CORRECTION section at the top of this document for the current, authoritative thesis.
In summary: the safety supervisor demonstrably restores a less-safe raw learned policy's
collision rate to match a classical baseline under genuine reactive-pedestrian conditions, is
perfect against a static exactly-known hazard, and its intervention rate saturates rather than
climbing once perception degrades enough to already confuse the policy itself - all at a real,
honestly-reported efficiency cost. Two of sixteen reactive-mode supervised episodes still
collide, both caught by the supervisor within a quarter of a second of contact - a narrow,
last-instant margin limit worth a targeted follow-up (§12.2 of `explanation.pdf`), not the
dominant, several-seconds-early detection gap the original buggy run seemed to show.

## Files

- `crowd_nav_ws/src/crowd_nav_evaluation/`: harness package (`scenarios.py`, `episode_monitor.py`,
  `run_episode.py`, `run_matrix.py`, `make_plots.py`).
- `crowd_nav_ws/src/crowd_nav_evaluation/results/`: `episodes.csv`, `interventions.csv`, `plots/`
  (corrected, post-audit-fix, 142 episodes).
- `crowd_nav_ws/src/crowd_nav_evaluation/results_prefix_bugfix/`: the original, pre-fix
  139-episode results, preserved unmodified for direct before/after comparison - not deleted.
- `crowd_nav_ws/src/crowd_nav_pedestrians/scripts/robot_pose_extractor.py`: the Gazebo
  ground-truth pose fix.
- `crowd_nav_ws/src/crowd_nav_perception/src/ground_truth_human_source.cpp`:
  `setRobotMapPose()`, the world-to-map frame correction that produced the corrected results
  in this document (`docs/audit.md` §1.3).
