# Phase 11 findings log

Per IMPLEMENTATION_PLAN.md's Phase 11 ("CI and docs"), the project's final phase: a per-PR CI
gate, a nightly Gazebo smoke test, and a README rewrite - closing the project at 11 of the
originally-planned 12 phases (Phase 12/HEIGHT integration scoped out explicitly, see the
README's Future Work section and IMPLEMENTATION_PLAN.md's Phase 12 entry).

**Status: DONE, done-bar met.**
- `.github/workflows/ci.yml`: per-PR build + full 60-test unit suite + lint, via
  `ros-tooling/setup-ros`/`action-ros-ci`, no Gazebo.
- `.github/workflows/nightly-smoke-test.yml` + `.github/scripts/run_smoke_test_episode.py`:
  launches the full stack, drives one goal, asserts specific topics actually carried messages.
- README rewritten to lead with results/thesis; `PolicyAdapter` extension path stated
  concretely; Future Work names HEIGHT explicitly with its risk stated plainly.
- `MEASUREMENTS.md` final pass.

## Lint: a real gate, not informational

`ament_lint`'s tools (`ament_cpplint`, `ament_flake8`, `ament_xmllint`, `ament_lint_cmake`) were
never run against this codebase before this phase - none of the 13 packages declare
`ament_lint_auto` test dependencies, and no prior phase had reason to check. Running them
before deciding how to wire lint into CI (rather than assuming either "it'll be clean" or
"it'll need to be informational-only") found 19 cpplint violations across 5 C++ packages and 22
flake8 violations across 8 Python files - all mechanical, none indicating a real logic problem:
- Missing explicit `#include <string>`/`<vector>`/`<memory>` for types used but only available
  via transitive inclusion from another header - a real, if minor, fragility (would break under
  a different compiler/stdlib that doesn't happen to transitively pull the same header).
- Three false-positive `build/include_subdir` warnings, all on `#include "onnxruntime_cxx_api.h"`
  - a genuinely flat third-party header, not a project file cpplint's naming-convention
    heuristic should apply to. Suppressed via `// NOLINT(build/include_subdir)` rather than
    "fixed" by restructuring a vendor header's own layout.
- Line-length (100 char cpplint / 99 char flake8) violations, all straightforward wraps.
- One genuine dead import (`numpy` in `make_plots.py`, imported, never used) - removed rather
  than suppressed.
- A handful of flake8 continuation-indent (E127) and blank-line (E306) style nits.

`legal/copyright` was excluded from the cpplint gate deliberately, not overlooked: this project
doesn't add per-file copyright-header boilerplate anywhere, a stylistic choice consistent with
its own no-boilerplate-comments discipline throughout every phase, not a gap to close.

All 19 + 22 violations fixed before the lint job was wired into CI, so the gate starts at zero
known debt and can be genuinely blocking rather than informational-with-known-exceptions.
Verified with a full-workspace rebuild and the complete 60-test suite afterward - the fixes are
all non-behavioral (import additions, line wraps, one dead-import removal), and nothing broke.

## The nightly smoke test's actual requirement: topic liveness, not just goal-reached

The plan pinned this requirement directly from Phase 10's own experience: a stack that launches
cleanly and reaches its goal can still be running on a dead topic underneath, exactly what
happened to `/ground_truth/robot_pose` for three phases (`docs/phase10-findings.md`). A
"did the goal succeed" smoke test would never have caught that; a topic-liveness assertion
would have, immediately.

Implemented by extending `episode_monitor.py`'s **existing** subscriptions
(`/ground_truth/robot_pose`, `/pedestrians`, `/amcl_pose`) with simple message counters, plus one
new subscription (`/scan`, not previously tracked) - deliberately not a parallel topic-checking
mechanism, so the same counts are available to any future harness user, not just CI. Returned as
`topic_message_counts` in the episode result dict alongside the existing metrics.

Verified in both directions before being wired into the workflow, not just written and trusted:
- **Positive**: a real single episode (`baseline_mppi`/open_arena) showed all four topics
  populated correctly (1000+ ground-truth-pose messages, 100+ pedestrian messages, 20+ AMCL
  poses, 30+ scans, for a ~10-20s episode) - confirms the counters work end to end against the
  real stack, not just against a mock.
- **Negative**: a synthetic zero-count case (`ground_truth_robot_pose: 0`) was fed directly to
  the check logic and confirmed to actually fail with the expected message, rather than the
  assertion being a tautology that would pass regardless of the data.

The GitHub Actions-specific parts of both workflow files (Gazebo/Xvfb provisioning on a fresh
runner, `actions/cache` for the vendored ONNX Runtime download) are the one piece **not** yet
exercised against live infrastructure - this repo has no remote configured, so neither workflow
has actually run on GitHub's own runners. Stated plainly in both files' own header comments
rather than claimed as verified, per this project's standing "verify against real behavior, not
assumption" discipline - the episode-invocation and topic-assertion logic they call was
verified locally first, the runner provisioning is a best-effort standard configuration.

## A third instance of the lost-executable-bit bug

Found incidentally while manually verifying the smoke test locally (not something the smoke
test itself was designed to catch, though it would have failed loudly on this exact bug):
`zone_manager_node.py`, `export_sarl_model.py`, `generate_dummy_model.py`, and two test-fixture
generator scripts had lost their executable bit - and critically, `git ls-files -s` showed
`100644` in the **tracked index itself**, not just the working tree, meaning this wasn't a
transient runtime fluke (unlike the two scripts fixed the same way in Phase 10) - it had been
committed non-executable at some point and never caught, because nothing before this phase
happened to invoke `zone_manager_node.py` fresh enough to notice the missing bit. `zone_manager_node.py`
specifically is loaded by every single episode in the evaluation matrix (the keepout costmap
layer is always active, not just for the named keepout scenario) - had this shipped into the
nightly workflow unfixed, the very first scheduled run would have failed on it. Fixed with
`chmod +x` + a normal commit (mode changes `100644 → 100755` are visible in the commit diff, not
just implied).

## Files

- `.github/workflows/ci.yml`, `.github/workflows/nightly-smoke-test.yml`,
  `.github/scripts/run_smoke_test_episode.py` (new).
- `crowd_nav_ws/src/crowd_nav_evaluation/scripts/episode_monitor.py`: `topic_message_counts`.
- Lint fixes across `crowd_nav_controller`, `crowd_nav_safety_supervisor` (already clean),
  `crowd_nav_observation`, `crowd_nav_perception`, `crowd_nav_policy_adapters`, and several
  Python scripts workspace-wide.
- `README.md`: full restructure (results/thesis lead, `PolicyAdapter` extension path, Future
  Work, limitations/upstream-discrepancies retained, dev setup moved down, phase history
  condensed to a pointer at `docs/`).
- `crowd_nav_ws/src/crowd_nav_description/MEASUREMENTS.md`: one clarifying addition
  (`robot_collision_radius` is spec-derived, not measurement-pending).
