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

**Status**: Phase 8 of 12 complete. Phase 2: baseline Nav2 + AMCL + SLAM, 5/5 goals both modes,
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
sees it, which this project does not yet replicate (`docs/phase8-findings.md`). See
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
- **Ground-truth human perception by default.** Acceptable per the project's own design (see
  `IMPLEMENTATION_PLAN.md` §1.4 of the original brief), but a real limitation until the
  perception-degradation model (noise, dropout, latency, occlusion) is exercised in the
  evaluation harness.
- **SarlAdapter feeds the network every perceived human, with no field-of-view or range
  restriction.** The checkpoint's own source repo (`tkkim-robot/Gazebo-CrowdNav`) applies a
  simulated depth-camera FOV filter (Intel D435I spec: 85.2°/12 m) inside its own `JointState`
  construction before SARL ever sees the human list - found while building Phase 8's
  action-match test fixture (`docs/phase8-findings.md`). This project's sensor is a different,
  narrower device (~180°/8 m, per the LiDAR limitation above), so copying that exact filter
  would itself be wrong - but feeding the network humans the robot's real sensor couldn't
  actually see is a genuine, unaddressed distribution-shift risk, not validated against how the
  checkpoint's source repo evaluated it. Flagged for resolution before Phase 10's numbers are
  treated as meaningful crowd-interaction results.

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
