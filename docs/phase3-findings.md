# Phase 3 findings log

Per IMPLEMENTATION_PLAN.md's Phase 3 ("Dynamic keep-out zones"), same discipline as prior
phases: findings recorded as they land.

**Status: DONE, done-bar met** — verified with real evidence (goal outcomes, ground-truth
pose, controller logs showing the actual replan/recovery mechanism), not just "the zone
appeared in the topic list."

## What was built

New package `crowd_nav_zones`:
- `srv/AddZone.srv` / `srv/RemoveZone.srv`: `zone_id` + axis-aligned rectangle (`center_x`,
  `center_y`, `size_x`, `size_y`, map frame) in, `success`/`message` out.
- `scripts/zone_manager_node.py`: the only genuinely new logic in this feature. Tracks active
  zones, renders them into a trinary-mode mask PGM+YAML pair (0/black = keepout, 255/white =
  clear — the standard Nav2 keepout-mask convention, not invented here), and asks a real
  `map_server` instance to reload it via the stock `LoadMap` service whenever a zone changes.
  Deliberately does **not** publish an `OccupancyGrid` or do any costmap math itself — see
  "don't reimplement what the Nav2 stack already does better"
  (`docs/phase2-findings.md`), applied here on the very next phase after it was written down.
- `launch/zones.launch.py`: the zone_manager node, a dedicated `map_server` instance (renamed
  `mask_server`) serving the mask on its own topic, `costmap_filter_info_server` describing
  how to interpret it, and a lifecycle manager for the two lifecycle nodes. Included from both
  `amcl.launch.py` and `slam.launch.py`.
- `nav2_params.yaml`: stock `nav2_costmap_2d::KeepoutFilter` added to **both** the local and
  global costmap plugin lists (see "why both," below — the plan's original text said local
  only, and that turned out to be insufficient for the phase's own done-bar).

Mask files are written to `/tmp/crowd_nav_zone_masks/`, deliberately outside this workspace's
path — the directory name contains spaces, which has broken independent tools three times
already in Phases 1-2 (`docs/phase1-findings.md`, `docs/phase2-findings.md`). Not risking a
fourth occurrence for a file this same node both writes and reads back.

## Real bugs found and fixed

1. **`LoadMap.srv`'s own doc comment is wrong for this Nav2 build.** The message documents
   `map_url` as accepting `file:///path/to/maps/floor1.yaml` syntax. Empirically, on this
   install, a `file://`-prefixed URL makes `map_server` return `RESULT_INVALID_MAP_METADATA`
   (result=3) for *any* map file, including a trivially valid 4x4 test map used to isolate the
   cause from anything specific to the zone mask's content. A plain absolute path (no scheme
   prefix) works correctly. Confirmed via direct `ros2 service call` testing, not assumed from
   the docstring — the same "verify against the real behavior, not the comment" discipline
   Phase 2 established, now generalized past just this project's own code to a stock message
   definition too.
2. **Relative mask topic name silently resolved inside the costmap's own namespace, not
   root.** `costmap_filter_info_server`'s `mask_topic` param was set to `keepout_filter_mask`
   (relative). Nothing about that looked wrong until `KeepoutFilter` (running inside
   `local_costmap`'s own costmap-wrapper node) subscribed to it: `ros2 topic info` showed the
   subscription actually landed on `/local_costmap/keepout_filter_mask` — a topic nothing
   publishes on — while `mask_server` published on plain `/keepout_filter_mask` (0 subscription
   count). Fixed by making both `topic_name` (on `mask_server`) and `mask_topic`/
   `filter_info_topic` (on `costmap_filter_info_server`) fully absolute (leading `/`).
3. **Calling a service from inside a service callback needs a `MultiThreadedExecutor` +
   `ReentrantCallbackGroup`, or it deadlocks.** `zone_manager`'s `add_zone`/`remove_zone`
   handlers need to call `mask_server`'s `load_map` service and wait for the result before
   replying. The naive approach (`rclpy.spin_until_future_complete(self, future)` from inside
   the callback) would nest a second spin call on a node already being spun by its own
   executor — unsafe on a `SingleThreadedExecutor` (rclpy's default) and still wrong with a
   `MultiThreadedExecutor`. Fixed: a `ReentrantCallbackGroup` on both the services and the
   client, `MultiThreadedExecutor` in `main()`, and a plain `threading.Event` + done-callback
   inside `_reload_mask_server` instead of a nested spin call — letting a different executor
   thread process the client's response while the callback's own thread blocks on the event.
4. **The phase's own done-bar can't be met with `KeepoutFilter` in the local costmap only,
   despite that being the plan's original text.** First real test (send a goal, add a zone
   directly in the robot's path, watch for a detour): the zone *did* apply and *did* correctly
   block local motion into it (confirmed - `AddZone` succeeded, the robot's real trajectory,
   via ground-truth pose logging, stopped right at the zone's edge and never crossed it), but
   the robot then got stuck spinning in place, not detouring, with `controller_server` logging
   repeated `Failed to make progress` errors - the same NavFn-recovery-loop signature from
   Phase 2's goal-overshoot bug. Root cause: the *global* costmap (feeding NavFn's path search)
   had no idea the zone existed, so NavFn kept handing MPPI the exact same path straight
   through it; MPPI correctly refused to violate the keepout, the progress checker fired, a
   recovery (spin) ran, and the identical broken global plan got computed again - an infinite
   loop, not a replan. Fixed by adding `KeepoutFilter` to the global costmap too. This is a
   deviation from the plan's literal "local costmap" text, made because testing the plan's own
   stated done-bar (not just "the mask exists") showed it was necessary, not because of scope
   creep for its own sake.

## Done-bar verification, with real numbers

Test: send a `NavigateToPose` goal along a ~7m corridor the robot had already successfully
driven in Phase 2's reliability gate. ~3s after the goal is accepted (robot moving, confirmed
via `/amcl_pose`, not raw odom - see the note below), call `AddZone` to place a 1.0x1.5m zone
directly ahead of the robot's *current map-frame position*.

- **With the zone active**: the robot's real trajectory (ground-truth pose, continuously
  logged) stopped at the zone's boundary and stayed there for ~65 seconds while
  `controller_server` logged repeated `Failed to make progress` / `Optimizer fail to compute
  path` errors - then began moving again along a different route (visible directly in the
  logged x/y trajectory, not inferred) and reached the original goal: `controller_server`
  logged `Reached the goal!`, `bt_navigator` logged `Goal succeeded`. Total elapsed: ~150s+
  including the stuck period - slow, but a genuine detour and success, not a stall.
- **After `RemoveZone`**: sent a fresh goal back through the same corridor (the former zone
  location). `SUCCEEDED in 20.7s` - a direct trip, no detour, no stuck period. This is the
  actual "removing it opens the path back up" contrast the plan's done-bar asks for, not just
  the zone's absence from a topic echo.
- Ground-truth pose logging throughout (both legs): zero tip events, confirming this is a
  navigation/planning story, not a rerun of Phase 2's physics bugs.

**Note on the test script's own bug, fixed along the way**: the first version of this test used
raw `/diff_drive_base_controller/odom` to decide where to place the zone ("directly ahead of
the robot"), rather than `/amcl_pose` (map frame). Odom and map frame coincide only at the
instant AMCL's correction happens to be zero, so this was a real frame-mismatch risk in the
*test*, not the zone infrastructure - caught by comparing the intended zone location against
where the robot's real trajectory actually stopped, which lined up correctly only once the
test switched to `/amcl_pose`. Recorded here rather than silently fixed, per this project's
own established discipline.

## Why the recovery took ~65 seconds, not immediately

Not deeply root-caused - the zone was reliably discovered and correctly forced a real detour,
which is the actual done-bar - but worth naming as an open observation rather than pretending
it was instant: `navigate_to_pose_w_replanning_and_recovery.xml`'s default recovery ladder
tries less drastic actions (spin, wait) before a full replan increasingly asserts itself, and
each `Failed to make progress` cycle has its own timeout budget. If Phase 9's safety supervisor
or later phases need *fast* reaction to a newly-placed zone (e.g. a zone appearing directly in
front of a moving robot in a tighter space), this latency is worth revisiting - candidate
levers include the BT's replanning period/triggers and `nav2_controller`'s own progress-checker
timeouts, not touched in this phase since the done-bar didn't require fast reaction, just
correct eventual reaction.
