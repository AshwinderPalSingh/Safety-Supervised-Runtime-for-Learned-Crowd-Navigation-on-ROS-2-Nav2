# Safety-Supervised Runtime for Learned Crowd Navigation on ROS 2 Nav2

A deployment runtime that lets a learned crowd-navigation RL policy (SARL) drive a Nav2 robot
in simulation, wrapped in a hard, verifiable safety layer: a forward-simulated collision/keep-out
check and a 5-criteria out-of-distribution detector, with graceful fallback to a classical MPPI
controller. The contribution is the software layer around the policy, not the policy itself —
and the 139-episode evaluation matrix this project ran against it (Phase 10,
`docs/phase10-findings.md`) is written up below because it's the most interesting part of the
project, not because the method wins outright. It doesn't, on every metric. That's the point.

## Results: what the safety supervisor actually does, and where its limits are

**These results were corrected after a hard adversarial audit found a critical bug** (a
coordinate-frame mismatch that silently fed the policy — not just the supervisor — human
positions offset by several meters from their real location, for the entire time this project
ran any live Gazebo episode; `docs/audit.md` §1.3). The full 142-episode matrix was re-run
against the fix, and the collision-rate result below is the corrected one — it is **not** the
result an earlier version of this project reported, and that reversal is stated plainly, not
smoothed over: see `docs/phase10-findings.md`'s "CORRECTION" section for the full before/after
comparison and mechanism.

Three results, run together, describe one underlying pattern rather than three separate ones:
**the safety supervisor takes a raw learned policy that is measurably less safe than a
classical baseline under real conditions, and restores it to match the classical baseline —
while remaining perfect against a static hazard and honest about where its own margin runs out.**

- **Against a static, exactly-known hazard, the mechanism is perfect.** A permanent
  keep-out-zone scenario (`depot_keepout_block`) shows the three configs behaving exactly as
  hypothesized: the stock MPPI baseline routes a 6.4 m detour around the zone via Nav2's own
  costmap planner; the unsupervised policy (`policy_raw`) drives straight into it after half a
  meter, exactly as expected from a policy with no concept of a static keep-out region; and the
  supervised policy's forward-sim check correctly rejects the identical unsafe approach **439
  times out of 439** — zero violations — but has no way to route around the obstruction the way
  the planner does, so it gets stuck and times out rather than either violating or succeeding.
  That's the cleanest possible demonstration of what a bounded safety check actually buys you:
  a deterministic, correct refusal, with no fallback smarter than refusal itself.
- **Against fast, dynamic hazards, the supervisor closes a real safety gap the raw policy has.**
  Under the fairest comparison in the matrix (reactive pedestrians, who actively avoid the
  robot), the *raw*, unsupervised policy has a **higher** collision rate than the classical MPPI
  baseline (50% vs 12%, in both scenario families) — an honest, unflattering measurement of an
  imitation-learning-only checkpoint driven through an untuned holonomic-to-diff-drive
  conversion. The supervised policy closes essentially all of that gap: 12% collision rate in
  both families, matching the classical baseline exactly. Two of sixteen reactive-mode
  supervised episodes still collide; both are caught by the supervisor within a quarter of a
  second of contact — a narrow, last-instant margin limit, not a broad detection gap. The
  residual points at the same underlying cause as the FOV/radius mismatches found in Phases
  8–9: the OOD proximity threshold and the forward-sim's lookahead window are both values
  inherited from the reference policy's own training configuration, never re-tuned against this
  robot's actual depot dynamics.
- **Under degraded perception, the effect saturates rather than compensating.** A 5-point
  perception-dropout sweep found a sharp, reproducible cliff between `dropout_prob` 0.0 and 0.1
  (intervention rate roughly triples), then goes essentially flat through 0.5, even though the
  underlying probability of losing track of a human keeps climbing steeply across that range.
  If the supervisor were compensating harder for worse perception, that rate should climb with
  it. It doesn't: the policy is already as confused as it gets past roughly 0.1, and the
  supervisor doesn't get better at catching a policy that's already lost — it just keeps
  rejecting the same confused output.

Full evidence chain, including the two significant harness bugs found and fixed mid-phase (a
Gazebo plugin that had silently killed ground-truth pose since Phase 4; a coordinate-frame bug
that made the keep-out scenario's first run look like a supervisor defect when the supervisor
was never the problem) and the later hard-audit correction above: **`docs/phase10-findings.md`**
(evaluation detail) and **`docs/audit.md`** (the audit that found the frame bug and the
OOD-reachability gap). A complete, interview-grade account of the whole project, including this
correction, is in **`explanation.pdf`**.

## `PolicyAdapter`: one real implementation, a deliberately defended seam

`PolicyAdapter` (`crowd_nav_policy_adapters/include/crowd_nav_policy_adapters/policy_adapter.hpp`)
has exactly one production implementation, `SarlAdapter`, plus `DummyAdapter` (a
zero-checkpoint smoke test for the inference path, not a second policy family). That's a
deliberate choice, not an unfinished one: building a second real adapter (HEIGHT, see Future
Work) means first validating an officially-unverified checkpoint — its own, separate
investigation, the same class of risk that turned out badly for a different checkpoint in
Phase 0 (§ below) — and folding that open-ended risk into this phase would have traded a clean,
finished result for an unbounded one. The interface is designed to be extended without the
rest of the runtime changing; the concrete extension path below states exactly what that would
require, so the seam is checkable without a second implementation existing yet.

**The interface** is four methods: `expectedShape()` (declared ONNX input/output tensor
names/shapes), `buildInputs(WorldState)` (canonical world state → whatever tensor batch this
policy family's network needs), `selectAction(TensorBundle, WorldState)` (network output →
a holonomic `Velocity2D`), `name()`. `WorldState` (`crowd_nav_observation/world_state.hpp`) is
a plain struct — a robot self-state plus a flat list of `HumanObservation`s — with no assumed
tensor layout or graph structure baked in; it's the common currency every adapter consumes,
regardless of what internal representation it builds from that.

**What `SarlAdapter` does with it**, concretely, so the contrast with HEIGHT is specific rather
than abstract: SARL is a *value network*, not a direct policy net (see `docs/phase0-findings.md`
for why that distinction mattered for the ONNX export). `buildInputs()` enumerates a discretized
candidate action space, one-step-propagates the world state under each candidate, and builds
SARL's flat 9-self-field + 5-per-human-field vector (`ObservationBuilder`) for every propagated
candidate, batched as rows — shape `(candidates, humans, 13)`. `selectAction()` runs argmax over
the batch's value-network outputs plus an immediate-reward term to pick the best candidate.
None of that candidate-enumeration/one-step-lookahead/argmax machinery is part of the
`PolicyAdapter` interface itself — it's `SarlAdapter`'s own internal strategy, private to that
implementation.

**What a `HeightAdapter` would need to implement**, concretely (per `docs/phase0-findings.md`'s
own research into HEIGHT's architecture):
- `buildInputs()` would construct a **heterogeneous spatio-temporal graph**, not a flat vector -
  node features split by type (robot, human), edges encoding spatial relationships, and a
  temporal stack of recent frames rather than a single-timestep snapshot. This is genuinely
  harder than `SarlAdapter`'s version: a graph-structured `TensorBundle` (multiple named
  tensors - node features, edge indices, temporal buffers) instead of one flat batch, and
  `WorldState` doesn't need to change to support it, since it was never SARL-shaped to begin
  with - it's already just a robot state and a flat human list, which is exactly what a graph
  builder would consume as its node source.
- `selectAction()` would actually be **simpler** than SARL's: HEIGHT outputs an action directly
  via PPO, with no candidate enumeration, one-step propagation, or argmax search needed at all -
  it's a direct read of the network's own output.
- `expectedShape()` and `name()` are adapter-local bookkeeping either way, no new design needed.

**What would not change**, and is exactly what makes the seam a real reusability proof rather
than an assertion: `crowd_nav_controller`'s decision-core (watchdog, rate handling, MPPI
fallback), `crowd_nav_safety_supervisor` (operates on the adapter's output `Velocity2D` and the
canonical `WorldState`, never on adapter internals), the evaluation harness
(`crowd_nav_evaluation`), and `WorldState`/`HumanObservation` themselves. The only integration
point in `crowd_nav_controller.cpp` is a one-line addition to a plain string switch already
built for exactly this (`adapter_type == "sarl"` / `"dummy"` today; `"height"` would be a third
branch constructing `HeightAdapter` the same way) - not a refactor, a config-driven case
addition. If a real `HeightAdapter` build ever required touching any of those four unchanged
things, that itself would be the important finding (the abstraction wasn't as clean as
designed) - which is exactly the kind of result this project has consistently chosen to report
rather than hide throughout every phase above.

## Future work

**HEIGHT integration (a genuinely RL-trained second policy family) is explicitly scoped as
follow-on work, not part of this project's current scope, and treated as its own
investigation rather than folded in here.** Two reasons, stated plainly rather than left
implicit:
- It starts by validating `Shuijing725/CrowdNav_HEIGHT`'s official checkpoint (a Google Drive
  link, currently **unverified** - `IMPLEMENTATION_PLAN.md` §1.8), with the same skepticism
  the RL-trained SARL checkpoint deserved before differential testing found it only achieved a
  0.21 success rate against the paper's reported 0.99 (`docs/phase0-findings.md`). If this
  checkpoint turns out the same way, validating it is its own multi-day investigation before
  any adapter code gets written - a real, open-ended risk, not a formality.
- `HeightAdapter` itself (see above) is real new work against a fundamentally different
  observation shape - not a config swap, and not reusable from `SarlAdapter`'s
  candidate-action-search pattern.

Decided explicitly rather than left to drift: this project closes at Phase 11 with a complete,
honestly-evidenced result on its own terms (a real safety-supervised runtime, validated end to
end, with the tradeoffs above reported in full). `PolicyAdapter` as a one-implementation
interface with a concretely-stated extension path is a deliberate, defended stopping point, not
an unfinished one - see the section above for exactly what a second implementation would
require and exactly what would stay unchanged. If HEIGHT integration happens, it's the start of
a separately-scoped effort, given the open-ended checkpoint risk above, not a phase folded into
this one's remaining backlog.

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
  `IMPLEMENTATION_PLAN.md` §1.8. A second, genuinely RL-trained policy (HEIGHT) is scoped
  follow-on work, not part of this project - see Future Work above.
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
  whether it's dangerous** - see Results above for the full three-finding synthesis
  (`docs/phase10-findings.md`).

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

CI (`.github/workflows/`): a per-PR gate builds the workspace, compiles every plugin, and runs
the full unit-test suite (no Gazebo - see `IMPLEMENTATION_PLAN.md` §6 for why sim-in-CI stays a
separate, non-blocking tier). A nightly/manually-triggered workflow launches the full stack
(Gazebo + Nav2 + one pedestrian), drives one goal to completion, and - per a requirement pinned
after Phase 10's own experience - asserts that the specific topics the stack depends on are
actually live (starting with `/ground_truth/robot_pose`, the exact topic that silently stopped
publishing for three phases before anything noticed). It's a smoke test for launch-file and
parameter-schema rot and exactly this class of silent-topic-death, not a behavioral correctness
gate, and it doesn't block PRs.

## Build history

12 phases planned, 11 complete - the project closes here (Phase 12, HEIGHT integration, is
scoped follow-on work, see Future Work above, decided deliberately rather than left ambiguous).
Each phase's detailed findings, including every bug found and how it was actually verified
rather than assumed fixed, are logged as it landed: `docs/phase0-findings.md` through
`docs/phase11-findings.md`. `IMPLEMENTATION_PLAN.md` §3 has the full phase-by-phase plan and
final status; this file summarizes the result, not the other way around.
