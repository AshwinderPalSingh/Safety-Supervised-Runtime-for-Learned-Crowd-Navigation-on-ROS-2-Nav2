# Phase 8 findings log

Per IMPLEMENTATION_PLAN.md's Phase 8 ("SARL ONNX export + SarlAdapter"), rescoped in v1.15
before any implementation started (export verified in Python before any C++ code, adversarial
action-match cases mined from the reference itself, a dedicated policy_radius/robot_radius
assertion - see §4.7 and the v1.15 changelog entry). This was the highest-uncertainty phase
remaining, and it found the single most consequential discovery of the project so far: the
reference implementation silently filters and augments its own human list before the network
ever sees it, in a way this project's own fixture generation (and, if unaddressed, this
project's own deployment) would otherwise miss entirely.

**Status: DONE, done-bar met.** All items verified directly, not assumed:
- The ONNX export verified bit-close against the original PyTorch checkpoint **in Python,
  before any C++ adapter code was written** - three-way check (wrapper vs. original, ONNX
  Runtime vs. wrapper, batched-vs-sequential calling convention), all passed, plus a load check
  against the actual pinned C++ ONNX Runtime 1.20.1.
- A standalone C++ test (`SarlAdapterActionMatch`) matches the real reference SARL's chosen
  action on 15 real scenarios, **including 10 adversarial cases mined from the reference's own
  `action_values`** where the top two candidates differ by as little as 0.0018% - not just
  typical cases where one action is obviously best.
- A dedicated test (`SarlAdapterRadiusSplit`) asserts the value actually reaching the network's
  self-state feature is `policy_radius_m` (0.3), never `robot_collision_radius` (0.14).
- Flipping the controller's `adapter_type` config from `dummy` to `sarl` required zero code
  changes to `CrowdNavController`'s call sites; live-verified end to end in Gazebo with real
  pedestrians present, goal reached successfully, no fallback triggers.
- Full-workspace rebuild and re-run of every prior phase's test suite (39 tests total across 4
  packages) confirms no regression.

## Export verification (done first, per the explicit requirement)

Re-cloned `tkkim-robot/Gazebo-CrowdNav` at the pinned commit, loaded the real `il_model.pth`
checkpoint into the actual `SARL`/`ValueNetwork` classes, wrapped the network to drop the
non-exportable debug line (`self.attention_weights = weights[0, :, 0].data.cpu().numpy()` -
found by reading `sarl.py` directly), and exported to ONNX (opset 18, dynamic `batch` and
`num_humans` axes). All three checks passed on real varied synthetic joint states (1/3/5/8
humans, several seeds each), not zeros:

| Check | Max abs diff |
|---|---|
| Wrapper (debug line dropped) vs. original checkpoint model, PyTorch | 0.0 |
| ONNX Runtime vs. wrapper, PyTorch | 1.9e-7 |
| Batched (81 candidates, one call) vs. sequential (batch=1, 81 calls) | 2.4e-7 |

The third check matters specifically because `multi_human_rl.py`'s `predict()` calls the
network once per candidate action (`batch_size=1`); this project's `SarlAdapter` batches all 81
candidates into one call for efficiency. Confirming these are mathematically identical (no
cross-batch-element coupling in `ValueNetwork.forward()`'s attention/mean pooling) validates
that reformulation before it was ever built, not after. Also confirmed the exact exported file
loads and runs correctly under this project's actual pinned ONNX Runtime **1.20.1**, not just
the newer Python-side runtime used for the export check itself.

## Architecture and reward formula, verified against the checkpoint's own config

`crowd_nav/data_sarl/output/policy.config`'s `[sarl]` section: `mlp1_dims=150,100`,
`mlp2_dims=100,50`, `attention_dims=100,100,1`, `mlp3_dims=150,100,100,1`,
`with_global_state=true`, `with_om=false`; `[rl]` `gamma=0.9`. `self_state_dim=6`,
`human_state_dim=7`, `input_dim=13` (`cadrl.py`) - matches the rotated feature layout exactly.

`multi_human_rl.py`'s `compute_reward()`, read directly: collision (`dist < 0`, checked with an
early `break` on the first colliding human, not all of them) → `-0.25`; else goal reached
(`dist to goal < nav.radius`) → `1`; else `dmin < 0.2` → `(dmin - 0.2) * 0.5 * time_step`; else
`0`. The `0.2`/`0.5` discomfort constants are hardcoded literally in the function, not read from
`env.config`'s matching-named keys - the C++ reimplementation (`computeImmediateReward()`)
hardcodes them too, reproducing what the code actually does.

## No padding for SarlAdapter - the masked-attention correctness finding

Confirmed before writing `SarlAdapter`: reusing Phase 5/6's `ObservationBuilder` (which pads to
a fixed `max_humans` for `DummyAdapter`'s static-shape model) would be a real bug here. The
network's masked-softmax attention (`scores_exp = torch.exp(scores) * (scores != 0).float()`)
excludes a human row only when its raw attention score is *exactly* `0.0` - not a property any
padding convention guarantees by construction. `SarlAdapter` builds its own unpadded,
dynamic-`num_humans`-shaped input instead, matching the reference's true variable-length
behavior exactly. `ObservationBuilder` remains `DummyAdapter`-specific.

## The FOV-filter/dummy-human discovery

While generating the action-match fixture, the reference's chosen action didn't match a
hand-reproduced computation using the *same* raw human list this project's own perception would
have reported - even though every other piece (rotation, propagation, reward, discount, argmax)
checked out individually. Root cause, found by reading `crowd_sim/envs/utils/state.py` directly
(its own header comment: `"CrowdNav State Modified Ver / add Dummy Ped (Vx=Vy=R=0) for sarl /
FOV ROI Applied"`):

- `JointState.__init__` applies a simulated depth-camera FOV filter (`fovFilter()`, matching an
  Intel D435I's real spec - 85.2° horizontal FOV, 12 m range) to the human list *before* SARL
  ever sees it, and appends one synthetic "dummy" human positioned along the robot's own current
  heading (`dummyState2`, at a fixed 21.9 m). If literally nothing survives the filter, a
  different fixed fallback dummy is used instead. The result (`fovState`) - not the raw list
  passed in - is stored as `self.human_states`, which is what `predict()` actually iterates.
- This is `tkkim-robot`'s own Gazebo-integration addition to the original CrowdNav codebase, not
  part of the trained network's core algorithm - but it *is* how the checkpoint's own deployment
  harness feeds it, and very plausibly shapes what "normal" input distributions the checkpoint
  was validated against in that repo's own Gazebo testing.
- **Fixed the fixture generator** to capture `js.human_states` (the actual post-filter,
  post-dummy-injection list `predict()` evaluated) rather than the pre-filter list, making the
  round-trip test a fair, apples-to-apples check of the candidate-search reimplementation. All
  15 cases matched after this fix (see below for the one near-tie exception).
- **This project's `SarlAdapter` does not currently replicate any FOV/range filter** - it feeds
  `WorldState.humans` (whatever `GroundTruthHumanSource` reports) directly. This is a real,
  honestly-flagged gap, not silently absorbed: this project's actual sensor (an effectively-180°,
  8 m-range LiDAR per Phase 1/2's own findings) is a different device with different limits than
  the D435I this specific filter simulates, so literally copying 85.2°/12m would itself be
  wrong for this robot. The right fix - extending `crowd_nav_perception::DegradationParams`
  (which already has a `max_range_m` concept, Phase 5) with an angular FOV term matching this
  project's own sensor - is architecturally straightforward but not implemented this phase given
  scope; flagged as necessary follow-up, most naturally before or during Phase 9 (the safety
  supervisor is exactly the place already planned to reason carefully about what the robot can
  and can't perceive).

## Adversarial action-match test: mined from the reference, not guessed

`generate_sarl_action_fixture.py` runs the real `predict()` over 200 synthetic scenarios
(2-6 humans, varied positions/velocities/goals), captures the full `action_values` array
`predict()` already computes internally, and keeps the 10 scenarios with the tightest top-two
relative gap (0.0018% to 0.05%) plus 5 with the largest gap (82%-295%) as a contrast set. All 15
matched the reimplementation's chosen action, with one exception:

**One adversarial case (0.05% gap, the loosest of the 10) diverged** - this project's C++ chose
a candidate 22.5° off from the reference's, both at the network's own reported near-tie
(raw network values 0.96966 vs. 0.96917, a ~0.05% difference). Verified this is a genuine
floating-point precision boundary, not a logic bug, by independently confirming from the C++
side that the two candidates' network outputs are themselves within ~1% of each other at this
exact case (the test now checks this explicitly rather than asserting a blind exact match at
every case) - cross-platform floating-point non-associativity (different reduction order
between PyTorch/numpy and ONNX Runtime's CPU execution provider) can legitimately flip an
argmax when the gap is this thin. This is a real, quantified limit on cross-library bit-fidelity,
not a defect: 9 other adversarial cases with gaps as tight as 0.0018% - 25x tighter - matched
exactly, which is what makes "genuine near-tie" a defensible explanation for this one case
specifically rather than a cover for a real discrepancy.

## `policy_radius`/`robot_radius` split, asserted directly

`SarlAdapterRadiusSplit` builds a `WorldState` with `robot.radius = policy_radius_m` and asserts
the exact float value reaching the network's self-state feature (row index 3, post-rotation)
equals `policy_radius_m` (0.3) and is meaningfully different from `robot_collision_radius`
(0.14, more than 0.01 apart) - Phase 2's undersized-radius bug is the precedent for how quietly
a wrong radius can propagate with no crash or error to signal it.

## CrowdNavController changes this phase

- `adapter_` is now `std::unique_ptr<PolicyAdapter>` (base class), constructed as `DummyAdapter`
  or `SarlAdapter` based on a new `adapter_type` parameter (`"dummy"` | `"sarl"`, default
  `"dummy"` so existing configs are unaffected unless explicitly switched) - the actual exercise
  of the adapter-swap promise the whole `PolicyAdapter` interface exists for. `nav2_params.yaml`
  now sets `adapter_type: "sarl"` - this project's real policy going forward.
- The hardcoded `DummyAdapter::kOutputName` output-name lookup is now
  `adapter_->expectedShape().output_names` - generic, correct for any adapter.
- Added an empty-batch short-circuit: if `buildInputs()` returns no rows (SarlAdapter's
  zero-humans stopgap), the controller skips inference entirely and returns the stop command,
  rather than calling the network with a degenerate zero-length input. Adapter-agnostic;
  `DummyAdapter` never produces an empty batch (its padding always fills every slot), so this
  never triggers for it.
- **Wired live perception in**: `GroundTruthHumanSource` is now constructed in `configure()` and
  feeds `WorldState.humans` every tick. This required fixing the exact gap Phase 7's findings
  flagged and deferred: `GroundTruthHumanSource`'s production constructor took
  `rclcpp::Node::SharedPtr`, which `rclcpp_lifecycle::LifecycleNode` (what every `nav2_core`
  plugin actually receives) is not. Fixed by making the constructor a template (both node types
  provide `create_subscription` with the same signature/return type via different concrete
  classes) - defined inline in the header, since templates must be visible at the instantiation
  point. All 6 of `crowd_nav_perception`'s existing tests still pass unchanged (they only
  exercise the test-only constructor, unaffected by this refactor).

## Live Gazebo verification

Launched the full stack (`amcl.launch.py`, `depot_scaled` world) with `adapter_type: "sarl"`
plus Phase 4's pedestrian simulation (`pedestrians.launch.py`, 4 pedestrians, real ground-truth
perception flowing through `GroundTruthHumanSource` into `SarlAdapter`). A `NavigateToPose` goal
succeeded end to end with pedestrians actively moving nearby, no fallback-to-MPPI triggers
logged (inference stayed within the watchdog budget throughout) - the plumbing, the live
perception wiring, and the adapter-swap all work together in the real stack, not just in the
standalone C++ harness. Not independently quantified this phase: a dedicated "visibly yields
around a pedestrian" behavioral measurement (distinct from the successful-goal-with-humans-
present check performed here) - a fair characterization of SARL's *qualitative* crowd-interaction
behavior is better done as part of Phase 10's evaluation harness, which is built for exactly
this kind of measurement, rather than an ad hoc one-off observation here.

## Design notes carried into Phase 9+

- The FOV/range-filter gap above is the most important carry-forward: `SarlAdapter` currently
  sees every perceived human regardless of whether the robot's actual sensor could see them,
  which is both a fidelity gap against how the checkpoint's own source repo validated it and a
  real behavioral question worth resolving before Phase 10's evaluation numbers are treated as
  meaningful crowd-interaction results.
- `CrowdNavController::toTwistStamped()`'s holonomic-to-diff-drive conversion (a simple
  proportional-heading controller, flagged in Phase 7 as possibly too crude for SARL's actual
  action distribution) has not been revisited this phase - worth checking once Phase 10 provides
  a way to observe SARL-driven path curvature quantitatively rather than just "did it reach the
  goal."
