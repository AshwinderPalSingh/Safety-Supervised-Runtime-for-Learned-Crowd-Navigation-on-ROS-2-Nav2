# Phase 6 findings log

Per IMPLEMENTATION_PLAN.md's Phase 6 ("Synthetic adapter + ONNX plumbing validation"), rescoped
in v1.11 before any implementation started (candidate action space pinned to a single config,
verified against the real reference source; `DummyAdapter` scoped as a permanent fixture, not a
Phase 6 throwaway - see §4.3.1 and the v1.11 changelog entry). Same discipline as prior phases:
findings recorded as they land.

**Status: DONE, done-bar met.** All items verified directly, not assumed:
- A standalone C++ harness (gtest) loads the checked-in `dummy_policy.onnx`, runs shape
  validation against it, performs real inference, and decodes an action - deterministically
  and reproducibly, no crashes.
- **Shape validation proven to actually reject a mismatch**, not just accept the one correct
  case: 5 deliberately-wrong `ShapeSpec`s (wrong feature dim, wrong candidate count, wrong
  name, wrong rank, an extra input) each throw `ShapeValidationError`; a dynamic-batch marker
  (`-1`) is correctly accepted against the model's fixed batch dim.
- Inference output matches an independently-computed known-correct answer (sum of each input
  row, since the trivial model is `Linear(feature_dim, 1)` with weight=1/bias=0) - proves data
  moved through correctly, not just "didn't crash."
- Candidate action-space generation and one-step propagation match independently-computed
  (Python, not this project's own C++ formula) reference values.
- Two fresh `DummyAdapter` instances given the same input produce byte-identical output.

16/16 new tests pass; full-workspace rebuild and re-run of all prior test suites (25 tests
total across `crowd_nav_perception`, `crowd_nav_observation`, `crowd_nav_policy_adapters`)
confirms no regression.

## What this phase is actually for

Stated plainly in the v1.11 rescope: this phase is what guarantees a Phase 8 failure means "the
SARL search reimplementation is wrong" and nothing else. Everything it builds is real and
permanent, reused unchanged by Phase 8 - the vendored ONNX package (already built and proven in
Phase 0), the shape-validation helper, the `PolicyAdapter` interface, and the candidate-action
generation/one-step propagation code. `DummyAdapter` is the one throwaway *policy*, and even it
stays in the tree afterward as a permanent zero-checkpoint regression fixture (§3 Phase 8) - not
deleted once Phase 8 lands real weights.

## What was built

New package `crowd_nav_policy_adapters`:
- `policy_adapter.hpp`: `ShapeSpec`, `TensorBundle`, `Velocity2D`, and the abstract
  `PolicyAdapter` interface, per §4.3.
- `candidate_action_space.hpp/.cpp`: `CandidateActionSpaceConfig` (single source of truth for
  candidate-batch shape, loaded from `config/policy_adapter.yaml`) and
  `buildCandidateActionSpace()`, reproducing `cadrl.py`'s `build_action_space()` exactly
  (exponential speed sampling, `linspace` rotations, stop action at index 0).
- `candidate_propagation.hpp/.cpp`: `propagateCandidate()`, reproducing `cadrl.py`'s
  `propagate()` exactly - self advances under the candidate action, humans advance under their
  own current velocity (constant-velocity assumption).
- `shape_validation.hpp/.cpp`: `validateSessionShapes()` - queries an `Ort::Session`'s actual
  input/output names and shapes and compares against a `ShapeSpec`, throwing
  `ShapeValidationError` with the exact mismatch. `-1` in the expected shape matches any extent
  at that position (dimension count must still match).
- `onnx_inference.hpp/.cpp`: `runInference()` - thin wrapper turning a `TensorBundle` into
  `Ort::Value`s, running `Session::Run()`, and turning the outputs back into a `TensorBundle`.
- `dummy_adapter.hpp/.cpp`: `DummyAdapter`. `buildInputs()` generates the real 81-candidate
  batch, propagates each one step, and runs each propagated state through
  `crowd_nav_observation::ObservationBuilder` (Phase 5, unchanged) to get its raw feature row.
  `selectAction()` ignores the network's (meaningless) output and picks whichever stashed
  candidate's heading is closest to the goal direction.
- `scripts/generate_dummy_model.py`: reads the same `policy_adapter.yaml`, builds a
  `Linear(feature_dim, 1)` with weight=1/bias=0 (so output = sum(input), a checkable answer),
  and exports it to `models/dummy_policy.onnx` with a **static** declared shape (no
  `dynamic_axes`) - deliberately, so shape validation has a concrete batch dimension to check.
- `config/policy_adapter.yaml`: the single config file (`speed_samples: 5`,
  `rotation_samples: 16`, `time_step_s: 0.25`, `max_humans: 5`, `human_radius_m: 0.3`) both the
  Python generator and the C++ adapters read - no hand-typed `81` on either side.

## Candidate action space - verified against the actual reference source (§4.3.1)

Re-fetched `crowd_nav/configs/policy.config` and `env.config`, and `cadrl.py`'s
`build_action_space()`/`propagate()`, directly from the pinned commit rather than recalling the
plan's own §7 assumption from memory:
- `[action_space]`: `kinematics = holonomic`, `speed_samples = 5`, `rotation_samples = 16`,
  `sampling = exponential`, `query_env = true`.
- `[env]`: `time_step = 0.25`.
- Speeds are **exponentially** spaced: `speeds[i] = (exp((i+1)/speed_samples) - 1) / (e - 1) *
  v_pref`. Rotations are `linspace(0, 2*pi, rotation_samples, endpoint=False)`. Candidate 0 is
  always the stop action.
- `propagate()`'s `FullState` branch (self) only ever applies the candidate action to itself;
  its `ObservableState` branch (humans) always uses the human's own current velocity - the
  candidate action never touches human propagation. Verified with a test built specifically to
  catch a mix-up here (`HumansAdvanceUnderTheirOwnConstantVelocityNotTheCandidateAction` feeds
  a candidate action wildly different from the human's velocity and asserts the human moved
  under its own velocity, not the candidate's).
- `predict()` has a `query_env=True` branch (and the pinned `policy.config` does set it) that
  asks the *training-time simulator* for the true next human states instead of assuming
  constant velocity. Not usable at deployment - there's no simulator to query once this runs
  against Gazebo/real sensors - so the deployed adapter necessarily takes the `else` branch's
  constant-velocity propagation. Not a new decision, just confirming the plan's existing choice
  is the only one actually available outside training.

## Shape validation - proven to reject, not just accept

The risk flagged in review: a shape validator that only gets tested against the one shape it
was built to expect can't prove it would catch a *real* mismatch - it might rubber-stamp
anything. `test_shape_validation.cpp` includes 5 deliberately-wrong `ShapeSpec`s against the
real generated model (wrong feature dim, wrong candidate count, wrong input name, wrong output
rank, an unexpected extra input) - each throws `ShapeValidationError` with a message identifying
the exact mismatch. A `-1` dynamic-batch marker is separately confirmed to be accepted against
the model's fixed 81, proving the dynamic-dimension carve-out works without disabling the check
entirely.

## Environment note: onnxscript wasn't installed

`generate_dummy_model.py`'s `torch.onnx.export()` uses this machine's default dynamo-based
exporter (torch 2.13, per §1.3's Phase 0 finding), which imports `onnxscript` - not installed
by default in this environment. Installed via `pip3 install --user onnxscript onnx` (also pulled
in a `numpy` upgrade to 2.2.6 in user site-packages, already the effective version per the
`gym`-vs-NumPy-2.0 warning seen during Phase 5's fixture generation, so not a new drift this
introduced). Documented here since anyone re-running the generator script on a fresh machine
will hit the same missing-dependency error.

## Verification: the exported model's shape really is static

Loaded the generated `dummy_policy.onnx` directly with the `onnx` package (independent of both
this project's C++ shape-validation code and the export script's own print statement) and
confirmed the declared shapes are `candidates: [81, 34]` / `value: [81, 1]` - concrete integers,
not `dynamic_axes` placeholders - before ever wiring it into the C++ test.
