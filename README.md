# Safety-Supervised Runtime for Learned Crowd Navigation on ROS 2 Nav2

A deployment runtime that lets a learned crowd-navigation RL policy drive a Nav2 robot safely
in simulation — policy inference, observation construction, hard safety constraints,
out-of-distribution detection, and graceful fallback to a classical controller. The
contribution is the software layer around the policy, not the policy itself. See
`IMPLEMENTATION_PLAN.md` for the full design and phased build order, and `docs/` for the
detailed findings log kept as each phase landed.

## Development setup

This project uses **CycloneDDS**, not the ROS 2 default FastRTPS, after a real reliability
issue in Phase 2 (`docs/phase2-findings.md`): FastRTPS's shared-memory transport left stale
`/dev/shm` segments after ungraceful process kills, which degraded into Nav2 lifecycle
hangs during heavy iterative testing. Before running anything in this workspace:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

(already appended to `~/.bashrc` on the dev machine this was built on — new interactive
shells pick it up automatically; only needed manually in non-interactive/scripted contexts).
`scripts/check_dds_health.sh` and `scripts/ros2_teardown.sh` provide an automated dirty-state
check and safe cleanup regardless of which RMW is active - see Phase 2 findings for both.

**Status**: Phase 10 of 12 complete. Phase 2: baseline Nav2 + AMCL + SLAM, 5/5 goals both modes,
verified against ground-truth pose (`docs/phase2-findings.md`). Phase 3: dynamic keep-out
zones, verified with a real mid-navigation block-and-detour test (`docs/phase3-findings.md`).
Phase 4: deterministic pedestrian simulation (HuNav dropped - see `IMPLEMENTATION_PLAN.md`
§1.2), byte-identical seeded reproduction and sim-time stepping proven by direct measurement,
not assumed (`docs/phase4-findings.md`). Phase 5: `GroundTruthHumanSource` (RNG-substream-
isolated degradation model, sim-time-keyed latency) and the observation builder, schema
verified with a round-trip test against the real `tkkim-robot/Gazebo-CrowdNav` reference
implementation's own `CADRL.rotate()`, not a hand-computed expectation (`docs/phase5-findings.md`).
Phase 6: `PolicyAdapter` interface, candidate action-space generation/one-step propagation
(re-verified against the pinned checkpoint's actual config/source, not memory), ONNX shape
validation proven to reject 5 different real mismatches (not just accept the correct case), and
`DummyAdapter` - a permanent zero-checkpoint smoke test for the whole inference path, not a
Phase 6 throwaway (`docs/phase6-findings.md`). Phase 7: `CrowdNavController`, a real
`nav2_core::Controller` embedding `nav2_mppi_controller::MPPIController` as a genuine pluginlib
fallback - end-to-end driving in Gazebo confirmed, and the failover transition verified
quantitatively (a real single-tick command discontinuity at the switch, bounded to the exact
configured accel/decel limits by the existing velocity smoother), not just the trigger
(`docs/phase7-findings.md`). Phase 8: `SarlAdapter` - the real SARL candidate-action search,
ONNX export verified bit-close against the original PyTorch checkpoint before any C++ code was
written, action-match tested against 10 adversarial cases mined from the reference's own value
array (top-two gaps as tight as 0.0018%). Found and documented a real architectural gap along
the way: the checkpoint's own source repo silently FOV-filters its human list before SARL ever
sees it, which this project did not yet replicate at the time (`docs/phase8-findings.md`).
Phase 9: the FOV/range filter and dummy-injection fix landed first, before any supervisor code,
per explicit review sequencing; `crowd_nav_safety_supervisor` - a forward-sim/costmap collision
check sharing the controller's own costmap instance structurally, a 5-criteria OOD detector, and
per-cause intervention logging - live-verified in Gazebo with three of its eight trigger causes
firing for real, non-engineered reasons (a sustained collision rejection held for 90+
consecutive ticks with zero misses) and a clean baseline run showing zero false positives
(`docs/phase9-findings.md`). Phase 10: the full 139-episode evaluation matrix (96 core + 40
perception-noise sweep + 3 keep-out-zone episodes), piloted first per explicit review
requirement. Found and fixed two significant bugs mid-phase rather than reporting through them:
a Gazebo `PosePublisher` bug that had left ground-truth robot pose silently dead since Phase 4
(retroactively meaning Phase 9's FOV filter had been inert the entire time it was "verified"),
and a map/world coordinate-frame mismatch that made the keep-out-zone scenario's first run
meaningless. Once fixed, that scenario produced a clean, decisive three-way result matching the
original hypothesis (`docs/phase10-findings.md`), and the noise sweep found a sharp,
reproducible failure cliff between perception-dropout 0.1 and 0.2, confirmed via rate-normalized
intervention counts to be policy saturation rather than the supervisor working harder. The
matrix also found a real, unflattering result: under reactive pedestrians, the supervised
policy has a **higher** collision rate than both the raw policy and the stock MPPI baseline, in
both scenario families - reported in full, not just the predicted efficiency cost. See
`IMPLEMENTATION_PLAN.md` §3 for the full phase list.

## Known limitations (stated deliberately, not discovered as accidents)

These are real constraints on what this project's results mean, recorded here so they're
visible up front rather than buried in a findings doc no one reads before citing a number.

- **The simulated LiDAR is 180° field of view, not 360°.** The robot's `gpu_lidar` sensor
  hit a confirmed rendering bug in this project's gz-sim 6.18.0 (Fortress) install: a
  ~176°-wide self-detection artifact in the sensor's own rear hemisphere (proven
  sensor-frame-relative by rotating the sensor and watching the artifact move with it — not
  robot geometry, not fixable by mount height or sensor type). Masked at the source rather
  than filtered downstream, which is the right call for avoiding a two-topics-diverge
  localization bug, but the honest cost is a real ~184°→180° field-of-view reduction. Combined
  with the project's already-deliberate 8 m range cap (a conservative budget-sensor spec, see
  `IMPLEMENTATION_PLAN.md` §3), the simulated robot runs a sensor closer to a real cheap 180°
  LiDAR than the original 360°/8 m spec. Full diagnostic trail: `docs/phase1-findings.md`.
  Related upstream issue (not an exact match, but the same problem family, still open):
  [gazebosim/gz-sim#2743](https://github.com/gazebosim/gz-sim/issues/2743).
- **The SARL policy checkpoint is imitation-learning-only, not RL-refined.** The RL-trained
  checkpoint from the fork this project sources weights from failed validation (0.21 success
  rate vs. the paper's reported 0.99); root-caused via differential testing to that specific
  file, not a reproduction bug. The IL-only checkpoint from the same repo validates cleanly
  (0.96 success / 0.02 collision). Chosen over an alternative, genuinely RL-trained checkpoint
  from a different fork (0.72/0.18/0.10) because an 18%-collision-rate base policy undermines
  this project's core evaluation question — distinguishing "the safety supervisor caught
  genuinely unsafe behavior" from "the supervisor is compensating for a mediocre policy." Real
  cost: an IL-only policy won't show RL-refined crowd-interaction timing distinct from the
  classical ORCA baseline it's evaluated against. Full reasoning: `docs/phase0-findings.md`,
  `IMPLEMENTATION_PLAN.md` §1.8. A second, genuinely RL-trained policy (HEIGHT) is a scoped
  future phase (§3, Phase 12) specifically to close this gap.
- **Ground-truth human perception by default**, though the degradation model is now exercised
  directly: Phase 10's noise sweep (`docs/phase10-findings.md`) swept `dropout_prob` from 0 to
  0.5 and found a sharp failure cliff between 0.1 and 0.2 (success collapses from 7/8 to 0/8),
  confirmed to be policy/metric saturation rather than a gradual decline or the supervisor
  compensating harder for worse perception.
- **The safety supervisor's OOD detector characterizes world-state novelty, not input-pipeline
  correctness.** Its five criteria (crowd size, proximity, relative speed, command magnitude,
  perception confidence) flag a scene unlike the training distribution; none of them can detect
  that the observation reaching the policy was already wrong before any threshold looked at it
  (Phase 8's FOV-filter finding, resolved in Phase 9, was exactly that class of bug - see
  `docs/phase9-findings.md` §"Design notes carried into Phase 10+"). A clean OOD-trigger rate is
  not, by itself, evidence the input pipeline is correct; that needs differential testing
  against a reference implementation, which is how the Phase 8 bug was actually found.
- **The safety supervisor's reliability tracks how well-characterized the hazard is, not just
  whether it's dangerous.** Phase 10's full matrix (`docs/phase10-findings.md`) found that under
  reactive pedestrians - the fairest, best-case comparison - `policy_supervised` has a *higher*
  collision rate than both the raw (unsupervised) policy and the stock MPPI baseline, in both
  scenario families (25%/50% vs 12%/12%). Every one of those collisions had at least one real
  supervisor intervention logged beforehand - the mechanism was engaged, not inert - and timing
  analysis rules out the simplest explanation (a full stop confusing a reactive pedestrian's own
  avoidance): only 2 of 6 collisions happen right after an intervention, while 4 of 6 happen
  several seconds after the supervisor last found anything to reject, via a separate, unflagged
  approach. The more likely story: `PROXIMITY`'s detection margin doesn't always keep pace with
  a fast-closing dynamic human. The keep-out-zone scenario, by contrast, is a static, exactly
  known hazard, and the same mechanism gets it right on every tick (425/425 correct rejections,
  zero violations, `baseline_mppi` and `policy_raw` behaving exactly as hypothesized). The
  mechanism itself doesn't misfire in either case; what varies is how completely its input
  characterizes the actual hazard - perfectly for a fixed zone, imperfectly for a fast dynamic
  human.

## Known upstream API/doc discrepancies (verified against real behavior, not assumed)

Nav2's own documentation isn't always right either — worth a standing reminder to verify
against the actual running service/behavior, not just the docstring, the same discipline this
project applies to its own claims.

- **`nav2_msgs/srv/LoadMap`'s `map_url` field doc comment describes a `file:///path/to/map.yaml`
  form as valid.** On this project's Nav2 Humble build, a `file://`-prefixed URL makes
  `map_server` return `RESULT_INVALID_MAP_METADATA` for any map, including a trivially valid
  one used to isolate the cause — confirmed via direct `ros2 service call` testing. A plain
  absolute path (no scheme prefix) works correctly. Used by `crowd_nav_zones`' zone-manager
  node (Phase 3, `docs/phase3-findings.md`) to reload the keep-out zone mask at runtime.

More will be added here as later phases land. See `IMPLEMENTATION_PLAN.md` for anything not
yet reflected here — this file summarizes it, not the other way around.
