# Phase 5 findings log

Per IMPLEMENTATION_PLAN.md's Phase 5 ("HumanStateSource, perception degradation, observation
builder"), rescoped in v1.9 before any implementation started (RNG substream separation,
sim-time-keyed latency buffer, schema pinned to the reference implementation - see §4.1.1,
§4.1.2, §4.2, and the v1.9 changelog entry). Same discipline as prior phases: findings recorded
as they land.

**Status: DONE, done-bar met.** All four done-bar items verified directly, not assumed from the
design:
- Unit tests (fixed input -> known output vector) green.
- **Round-trip verified against the actual reference implementation**, not hand-computed
  expectations: a synthetic robot+human state fed through this project's observation builder,
  then through a transcribed `CADRL.rotate()`, matches the same state fed through the real
  cloned `tkkim-robot/Gazebo-CrowdNav` repo's own `rotate()`.
- Degradation sweep (dropout=0.3, 1000 synthetic ticks) matches the expected missing-detection
  rate within 5-sigma binomial sampling tolerance, with the RNG-substream separation confirmed
  independently (not just assumed from "we used two seed_seqs").
- Latency ring buffer verified keyed to sim time, not tick count, with a test constructed
  specifically to fail under a "N-ticks-back" implementation.

Two new packages: `crowd_nav_perception` (`GroundTruthHumanSource`, `TrackedHumanSource` stub)
and `crowd_nav_observation` (`WorldState`, `ObservationBuilder`).

## What was built

**`crowd_nav_perception`:**
- `human_state_source.hpp`: `HumanObservation` (`id`, `x,y,vx,vy`, `cov_xx,cov_yy,cov_xy`,
  `rclcpp::Time stamp`) and the abstract `HumanStateSource` interface
  (`getHumans(query_time)`) - the seam `TrackedHumanSource` will implement later without
  touching the observation builder.
- `degradation_params.hpp`: `sigma_pos_m`, `sigma_vel_mps`, `dropout_prob`, `latency_s`,
  `std::optional<double> max_range_m`, `bool occlusion_check` (declared, not implemented this
  phase - see "Cut" below), `degradation_seed`, `tick_period_s`.
- `ground_truth_human_source.hpp/.cpp`: `GroundTruthHumanSource`. Two constructors - a
  production one wiring real ROS subscriptions, and a test-only one
  (`GroundTruthHumanSource(const DegradationParams&)`) with no ROS I/O at all. This split was a
  deliberate refactor so the degradation/latency logic is unit-testable without standing up
  real pub/sub; `ingestPedestrian()`/`setRobotPose()` are the pure-logic entry points both the
  production subscriptions and the tests call.
- `tracked_human_source.hpp`: stub throwing `NotImplementedYet`, per the plan's Phase 5 scope
  (real tracking is out of scope here).

**`crowd_nav_observation`:**
- `world_state.hpp`: `RobotSelfState` (9 fields, schema per §4.1.1) and `WorldState` (self +
  `vector<HumanObservation>`).
- `observation_builder.hpp/.cpp`: `ObservationBuilder::build()` produces SARL's raw
  (pre-rotation) flat vector - 9 self-state fields, then `max_humans()` blocks of 5 human
  fields each, closest-to-robot first, padded with far-away/stationary/zero-radius dummies past
  the real human count. Rotation into egocentric features is explicitly NOT done here - that's
  `SarlAdapter`'s job (Phase 8); this package has zero Nav2/policy dependency by design (§2).

## RNG substream separation (v1.9 requirement #1)

`GroundTruthHumanSource::deriveSubstreamSeed(scenario_seed, subsystem_tag)` builds a
`std::mt19937_64` seed via `std::seed_seq{low32(scenario_seed), high32(scenario_seed),
subsystem_tag}`, generating two `uint32_t` and combining them into the 64-bit seed. Different
`subsystem_tag` values (e.g. 0 for Phase 4's pedestrian-motion RNG, 1 for this phase's
degradation RNG) from the *same* `scenario_seed` produce independent, non-correlated streams -
confirmed, not assumed, by `DerivedSubstreamSeedsAreIndependentAndDeterministic`: two instances
built with the same `(seed, tag)` produce byte-identical output sequences; an instance built
with a different tag diverges immediately. This is what makes it safe to sweep a degradation
parameter (e.g. `sigma_pos_m`) without perturbing where the pedestrians actually walked - the
entire point of the requirement.

One subtlety worth recording: `degrade()` always draws from `pos_noise_dist_`, `vel_noise_dist_`,
and `dropout_dist_` on every call, regardless of whether the configured params are zero. If the
draws were conditionally skipped when a param is zero, the RNG's consumption pattern (how many
values it pulls per call) would change across sweep points, which would silently reintroduce
the same class of bug the substream split was meant to prevent, just one level down. Drawing
unconditionally costs nothing and keeps the sequence shape identical at every sweep point.

## Latency ring buffer keyed to sim time (v1.9 requirement #2)

Rejected a fixed-tick-count ring buffer up front (the v1.9 rescope already flagged this - see
§4.2). Implemented instead as `history_`: a `map<id, deque<(rclcpp::Time, HumanObservation)>>`.
`ingestPedestrian()` pushes each degraded sample with its real timestamp and prunes entries
older than `max(latency_s, 0) + 1.0` seconds; `delayedLookup(id, query_time)` computes
`target_time = query_time - Duration(latency_s)` and linear-scans the (small, bounded) deque
for the freshest sample at or before `target_time`.

`LatencyIsKeyedToSimTimeNotTickCount` is the test built specifically to fail under a
tick-counting implementation: ingests four samples at *uneven* sim-time intervals (t = 0.0,
0.3, 1.7, 2.0, with x = 0, 1, 2, 3), then queries at t = 2.0 with `latency_s = 1.0`. The
correct answer is x = 1.0 (the t = 0.3 sample, the freshest one at or before t = 1.0) - not
x = 2.0 ("3 ticks back" under a naive tick-counter) and not x = 3.0 ("latest" if latency were
ignored entirely). Passes.

## Round-trip verification against the reference implementation (v1.9 requirement #3)

This was the requirement flagged as worth the extra time "because the observation vector is
the single highest-leverage place for a silent bug in this project." Full trail:

1. Re-cloned `tkkim-robot/Gazebo-CrowdNav` at the pinned commit
   (`9cad128d124f86bafe48d2cd11b5eee74bec77d9` - the previous session's clone had been in a
   scratchpad directory that didn't survive to this session; re-cloned and re-verified the
   commit hash matched before regenerating anything) and re-read `cadrl.py`'s `rotate()`
   directly rather than trusting a memorized transcription of it.
2. `test/generate_reference_fixture.py` imports the real `CADRL` class from the clone, sets
   `policy.kinematics = 'holonomic'` (matching the checkpoint, §1.9), and calls the actual
   `policy.rotate(...)` on five synthetic `(self_state, human_state)` rows spanning a spread of
   headings/distances/velocities. Output: `test/fixtures/sarl_rotate_reference.txt`, one line
   per case (27 space-separated values: 9 self_state + 5 human_state + 13 rotated), checked into
   the repo so the C++ test never needs network access or a reference-repo checkout.
3. `test_observation_builder.cpp` reconstructs a `WorldState` from each fixture line, runs it
   through this project's own `ObservationBuilder::build()`, then applies a **test-only**
   `referenceRotate()` helper - a direct transcription of `cadrl.py`'s `rotate()` (verified
   against the freshly re-read source, not memory) - to the builder's raw output, and asserts
   the result matches the fixture's pre-computed `rotated` column to `1e-6`. Deliberately not
   part of the public library: production rotation belongs to Phase 8's `SarlAdapter`, per the
   plan's own phase boundaries; this exists only so the round-trip test operates in the same
   space the fixture was computed in.
4. All 5 fixture cases pass.

**A real gap this caught before the code ever compiled**: `ObservationBuilder::build()`
originally referenced a `radius` field on `HumanObservation` that doesn't exist -
`HumanObservation` (crowd_nav_perception) has no radius field, because perception never
measures it. Checked `pedestrian_sim_node.py` (Phase 4) to see how the project actually handles
human radius today: it's a single `ped_radius` config parameter (0.25 m) applied uniformly to
every pedestrian, never per-individual. So per-human radius is architecturally a config
constant in this project already, not a sensed quantity - the correct fix was adding a
`human_radius` constructor parameter to `ObservationBuilder` (same category as
`RobotSelfState::v_pref`: matched to the training/simulation distribution, not observed), not
retrofitting a fake "sensed" field onto `HumanObservation`. This is exactly the kind of
schema-mismatch bug the round-trip test was added to catch - it surfaced at first build attempt
rather than at Phase 10.

## JSON dependency avoided

The fixture was originally generated as JSON, with `crowd_nav_observation`'s `package.xml`
declaring a `nlohmann-json-dev` test dependency. `rosdep resolve nlohmann-json-dev` wasn't
available in this environment to confirm that rosdep key actually maps to the installed apt
package (`nlohmann-json3-dev`, confirmed present via `dpkg -l`) - rather than ship an unverified
dependency, switched the fixture to plain whitespace-delimited text (comment lines prefixed
`#`, one case per data line). Removes the need for any JSON library in the C++ test to parse
five lines of floats, and removes a dependency this project couldn't verify would resolve
correctly in CI or on a fresh machine.

## Design notes / cuts

- **Occlusion check declared, not implemented** (`DegradationParams::occlusion_check`): per
  §4.2's own framing, this is the first piece to cut under time pressure. Left as a documented,
  inert flag rather than a half-implemented raycast, so a later phase can fill it in without a
  schema change.
- **`TrackedHumanSource` remains a stub** throwing `NotImplementedYet` - real tracking is
  explicitly out of this phase's scope per the plan.
