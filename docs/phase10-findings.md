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
  per the explicit "report it even if my method didn't win" instruction.

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

## Core matrix (96 episodes, N=8/seed × 2 families × 3 configs × 2 pedestrian modes)

### Outcome rates and the reactive/non-reactive split

Pooling both pedestrian modes initially suggested `policy_supervised` had the worst collision
rate of the three configs in both scenario families (69% in open_arena, 44% in depot) - but
splitting by pedestrian mode changes the story substantially:

| scenario | config | non_reactive collision rate | reactive collision rate |
|---|---|---|---|
| depot | baseline_mppi | 3/8 (38%) | 1/8 (12%) |
| depot | policy_raw | 6/8 (75%) | 1/8 (12%) |
| depot | policy_supervised | 5/8 (62%) | 2/8 (25%) |
| open_arena | baseline_mppi | 7/8 (88%) | 1/8 (12%) |
| open_arena | policy_raw | 8/8 (100%) | 1/8 (12%) |
| open_arena | policy_supervised | 7/8 (88%) | 4/8 (50%) |

`non_reactive` pedestrians (who don't avoid the robot at all) drive collisions to 62-100% for
**every config**, regardless of scenario or supervisor - this is a pedestrian-model effect, not
a controller-quality signal, and shouldn't be read as differentiating the three configs.

**Within `reactive` mode alone** (the fair, apples-to-apples comparison), a real and
uncomfortable pattern holds: `policy_supervised` collides *more often* than both `baseline_mppi`
and `policy_raw` in both scenario families - 25% vs 12%/12% in depot, and 50% vs 12%/12% in
open_arena. This is reported prominently rather than buried, per the explicit standing
instruction to report the interesting number even when it isn't a win.

Every one of these 6 collisions had at least one real `PROXIMITY` intervention logged
beforehand (1 to 18 per episode) - the supervisor was actively engaged, not inert. Checking the
*timing* between the last logged intervention and the moment of collision (not just presence,
per the explicit ask to distinguish "the supervisor is compensating for worse conditions" from
"the supervisor gave up," the same discipline applied to the noise-sweep plateau below) splits
cleanly into two modes, not one:

| episode | interventions | gap: last intervention → collision | last rejected speed | min human distance |
|---|---|---|---|---|
| depot seed7 | 1 | 0.09s | 1.00 m/s | 0.387 m |
| open_arena seed7 | 1 | 0.07s | 1.00 m/s | 0.387 m |
| depot seed5 | 5 | 8.11s | 0.71 m/s | 0.390 m |
| open_arena seed0 | 7 | 6.02s | 0.71 m/s | 0.372 m |
| open_arena seed2 | 15 | 6.56s | 1.00 m/s | 0.376 m |
| open_arena seed1 | 18 | 1.65s | 0.71 m/s | 0.343 m |

The single-intervention episodes collide within a tenth of a second of the rejection - the
supervisor caught the approach right as contact was already imminent (rejected speed a
meaningful 1.0 m/s, not near-zero), too late for a stop to change the outcome. But the
higher-intervention episodes (5-18 logged rejections) show a **large** gap - several seconds
where the supervisor considered every candidate safe - between the last handled encounter and
an independent, later collision with no rejection immediately before it. That's the opposite of
"the stop itself creates the new risk": if a full stop were confusing reactive avoidance, the
collision should cluster right at/after the intervention that caused it, the way the two
single-intervention episodes do. Instead, the dominant pattern (4 of 6) is the supervisor
correctly handling one encounter, then a **separate, unflagged** close approach - one
`PROXIMITY` didn't catch in time - producing the collision on its own. `min_human_distance_m` is
essentially identical across all six (0.34-0.39 m, all just under `COLLISION_DISTANCE_M`) - these
are narrow misses-that-weren't, not high-speed impacts, consistent with a detection-timing gap
rather than a violent failure. The evidence points to `PROXIMITY`'s threshold/reaction-time
margin against the harness's own (tighter) ground-truth collision distance, not the supervisor's
stop response, as the more likely source of this gap - worth a dedicated follow-up (logging the
pedestrian's own velocity around these specific windows) before treating it as confirmed, but
better-supported by this data than the full-stop hypothesis considered first.

This is a derivable limitation, not a surprising one, and it traces to the same root as the
FOV/radius issues found in Phases 8-9: both the OOD `PROXIMITY` threshold and the forward-sim's
1.0s lookahead (4 steps × `time_step_s=0.25s`) are values pulled from the reference SARL
implementation's own training configuration - the proximity threshold explicitly derived from
CrowdNav's `env.config` (`discomfort_dist=0.2`, `radii=0.3`, IMPLEMENTATION_PLAN.md v1.1), and
`time_step_s` pinned against the checkpoint's own training config in `policy_adapter.yaml`,
neither re-tuned against this robot's actual operating conditions. A margin sized for the
training distribution's encounter geometry isn't guaranteed to cover a depot's reactive
pedestrians closing distance faster than that distribution ever produced - the same pattern as
inheriting a reference implementation's parameters into an environment whose real conditions
weren't what those parameters were chosen for.

### Efficiency (successful episodes only)

Confirms the review's explicit prediction: `policy_supervised` is measurably slower than
`baseline_mppi` in every family (median duration ~15-18s vs ~9-12s; median path length also
longer). `policy_raw` and `policy_supervised` track each other closely on efficiency, which
makes sense since they share the same underlying policy - the efficiency cost is inherent to
the SARL policy itself, not the supervisor layer on top of it.

### Intervention rate by scenario family (the headline metric, S4.9.4)

`policy_supervised` intervenes more in depot than open_arena (19.5/episode vs 11.8/episode,
~65% higher) - directionally consistent with a transfer-difficulty story, though not the
"near-zero in open-arena, high in depot" clean contrast the review offered as the best-case
shape of evidence. Both scenarios show substantial supervisor engagement; depot is harder, not
qualitatively different.

Plots: `results/plots/outcome_rates.png`, `duration_s_distribution.png`,
`path_length_m_distribution.png`, `intervention_rate_by_family.png`.

---

## Noise sweep (40 episodes, `policy_supervised`/open_arena/reactive, dropout_prob ∈ {0.0, 0.1, 0.2, 0.3, 0.5})

| dropout_prob | outcomes | mean interventions/episode | intervention **rate** (per sim-second) |
|---|---|---|---|
| 0.0 | 4 success, 4 collision | 17.0 | 0.81/s |
| 0.1 | 7 success, 1 collision | 212.2 | 3.01/s |
| 0.2 | 7 timeout, 1 nav2_aborted | 398.1 | 3.58/s |
| 0.3 | 7 timeout, 1 collision | 439.6 | 3.66/s |
| 0.5 | 7 timeout, 1 collision | 408.5 | 3.60/s |

A sharp, reproducible cliff between dropout 0.1 and 0.2: success collapses from 7/8 to 0/8, and
outcomes flip almost entirely to `timeout` - not gradual degradation, a discrete failure mode.

The raw per-episode intervention count keeps climbing through dropout 0.3 before leveling off,
which could be read either as "the supervisor works harder as perception gets worse" (a safety
floor) or "the policy is equally lost regardless of severity" (saturation) - these imply
different conclusions, so the raw count alone doesn't settle it. Normalizing to a rate (per
second of sim time, removing the confound that 0.2+ episodes are mostly timeout-bound at a
fixed 120s ceiling) answers it directly: the rate jumps sharply from 0.0→0.1 (0.81→3.01/s) and
then goes essentially **flat** from 0.1 through 0.5 (3.01-3.66/s, no monotonic trend). If the
supervisor were compensating for worsening perception, the rate should keep climbing with
`dropout_prob` - the underlying per-tick probability of losing track of at least one of the 4
tracked humans keeps rising steeply across this range (~34% at 0.1 to ~94% at 0.5). It doesn't
climb. **This is metric/policy saturation, not a supervisor floor**: the policy is already
about as confused as it gets once perception degrades past roughly 0.1, and further
degradation doesn't meaningfully change how often the supervisor has to intervene, because the
policy's own behavior has already collapsed into the same stuck-rejected-retry loop.

Plot: `results/plots/noise_sweep.png`.

---

## `depot_keepout_block`: a coordinate-frame bug that made the first run meaningless

### The symptom

The scenario's first run showed `intervention_count_total = 0` for **all three configs**,
including `policy_supervised`, whose episode the harness itself reported as `keepout_violation`.
This directly contradicted the plan's own stated expectation (`policy_raw` was expected to
violate the zone, "SARL has no concept of a static keep-out region at all") - `policy_raw`
instead reported `success`, and `policy_supervised` was the one flagged as violating. Per this
project's established discipline (the same one that found `LOW_PERCEPTION_CONFIDENCE`
unreachable in Phase 9), this was investigated to a root cause rather than reported at face
value.

### Elimination, in order of cost

1. **Mask geometry**: decoded the actual `mask.pgm`/`mask.yaml` written during the episode -
   correctly placed at the intended coordinates.
2. **Timing**: the costmap filter mask was confirmed live in both local and global costmaps
   ~0.7s before navigation even began - not a race.
3. **Reachability**: instrumented `checkForwardSim` directly (a single log line, not inference)
   and re-ran the failing episode - the function executes every control tick, over 100 times in
   a single 9.6s episode. Not a guard condition or an unreached code path.
4. **The actual bug**: `state.robot.px/py` (what `checkForwardSim` forward-simulates from) is
   the standard nav2_core controller-plugin pose argument - **map frame**, matching the costmap
   and the keepout mask exactly (`zone_manager_node.py`'s own code comment: zones are stored
   "all in the 'map' frame"). But `episode_monitor.py`'s ground-truth zone-violation check
   compared `/ground_truth/robot_pose` - Gazebo **world** frame (via this phase's own
   `robot_pose_extractor.py`) - against that same map-frame-intended zone spec. Given this
   scenario's spawn/map correspondence (world `(-3,0)` localizes to map `(0,0)`), the map-frame
   goal `(2,0)` corresponds to world `(-1,0)` - almost exactly the boundary of the zone as the
   harness (mis)interpreted it in world frame. The "violation" was a numerical coincidence
   between two different frames, not a real zone entry; the supervisor's own logic (map frame,
   internally consistent) never rejected anything because the robot's real map-frame path -
   from `(0,0)` toward `(2,0)` - never went anywhere near the zone at map `x∈[-1,0]`, which sits
   *behind* the start, not on the route to the goal.

Worth stating plainly: steps 1-3 above all came up clean, and that's not wasted effort - it's
the evidence that the supervisor's own design was never the problem. Mask geometry was correct,
the mask was live in time, and `checkForwardSim` executed on every tick exactly as designed.
Nothing in the supervisor changed once the actual bug was found, because nothing in the
supervisor was wrong; the harness was wrong about ground truth while claiming to report it, a
failure mode indistinguishable from a genuine supervisor defect until checked this specifically.

### The fix

`episode_monitor.py` now checks the zone against `/amcl_pose` (already subscribed to for the
covariance-trace metric, already map-frame) instead of the world-frame ground-truth topic.
`scenarios.py` repositions the zone from `center_x=-0.5` (chosen with world-frame intuition,
landing behind the map-frame start) to `center_x=1.0` (actually on the map-frame path between
spawn and goal).

### The corrected result

Re-running just the 3 episodes with both fixes in place produced a clean, decisive,
publishable three-way contrast - and matches the plan's original hypothesis:

| config | outcome | path_length_m | interventions | dominant cause |
|---|---|---|---|---|
| baseline_mppi | success | 6.37 (vs ~2m direct) | 0 | - |
| policy_raw | **keepout_violation** | 0.64 | 0 | - |
| policy_supervised | timeout | 0.38 | 428 | KEEPOUT_VIOLATION ×425, COSTMAP_COLLISION ×3 |

`baseline_mppi` routes a large detour around the zone via Nav2's global-costmap keepout layer -
a planning-level effect, present regardless of which local controller is active.
`policy_raw` drives essentially straight into the zone after half a meter, exactly as SARL's
lack of any keep-out concept predicts. `policy_supervised`'s forward-sim correctly identifies
and rejects the same unsafe approach 425 times - **zero** actual violations - but has no
mechanism to route around the obstruction the way the global planner does, so it gets stuck in
a reject-retry loop and times out rather than either violating or succeeding. This is a clean
demonstration of the supervisor's actual value proposition and its real limit in the same
result: it can reliably prevent the unsafe outcome, but it cannot make the underlying policy
smarter about finding an alternative path.

---

## Overall assessment

The headline comparison isn't "policy_supervised wins" - it doesn't, on collision rate or
efficiency, against either alternative. Read together rather than as three separate bullets,
the keepout result, the collision-rate finding, and the noise-sweep cliff describe one
underlying pattern: **the supervisor's reliability tracks how well-characterized the hazard is,
not just whether it's dangerous.**

`depot_keepout_block` is the cleanest case because the hazard is static, exactly known, and
geometrically simple - the forward-sim check has a fixed region to test against and gets it
right on every single tick, 425/425, zero violations. The reactive-pedestrian collision-rate
result is the same mechanism under harder conditions: humans are dynamic, and the timing
breakdown above shows the supervisor isn't failing the way a "full stop confuses reactive
avoidance" story would predict (that would cluster collisions right after an intervention,
which only 2 of 6 cases show) - it's that `PROXIMITY`'s detection margin doesn't always keep
pace with how fast a reactive human can close distance, so 4 of 6 collisions happen seconds
after the supervisor last found anything to reject, via a separate, unflagged approach. The
noise sweep fits the same shape from a third angle: once perception degrades enough that the
policy can no longer characterize the humans around it at all, the supervisor doesn't get
better at compensating (the rate-normalized intervention count is flat, not rising) - it just
keeps rejecting the same confused output forever. In all three: the mechanism itself never
misfires or gets confused; what varies is how completely the *input to* that mechanism
describes the actual hazard - perfectly for a static zone, imperfectly for a fast dynamic
human, and not at all once perception saturates.

That's a real, verifiable safety mechanism, not a feature that always helps - a supervisor that
is precise against hard geometric constraints and structurally has no fallback smarter than
refusal once its own checks run out of margin. Both the capability cost (gets stuck rather than
succeeding) and the detection-margin gap were plausible outcomes going into this phase; both
are reported here, at the same level of prominence as the keepout result, because they
happened - not because either flatters the project's own thesis.

## Files

- `crowd_nav_ws/src/crowd_nav_evaluation/`: harness package (`scenarios.py`, `episode_monitor.py`,
  `run_episode.py`, `run_matrix.py`, `make_plots.py`).
- `crowd_nav_ws/src/crowd_nav_evaluation/results/`: `episodes.csv`, `interventions.csv`, `plots/`.
- `crowd_nav_ws/src/crowd_nav_pedestrians/scripts/robot_pose_extractor.py`: the Gazebo
  ground-truth pose fix.
