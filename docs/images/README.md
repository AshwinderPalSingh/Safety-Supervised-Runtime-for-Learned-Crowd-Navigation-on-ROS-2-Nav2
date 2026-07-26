# Visual assets checklist

The evaluation plots and architecture diagrams in this directory are real, generated data —
already wired into the main [`README.md`](../../README.md). Everything below is a live GUI
capture I can't take myself (no way to drive Gazebo/RViz's windows from here), so it's a
checklist for a manual capture pass. Drop each file at the exact path listed and its `<img>`
block in the README's [Demo & Screenshots](../../README.md#demo--screenshots) section — already
written, just commented out — starts rendering with no further edits.

Screen recording → GIF: `peek` (simplest, GUI) or `ffmpeg -i in.mp4 -vf "fps=12,scale=800:-1"
out.gif`. Keep GIFs short (5–10 s) and under ~8 MB so GitHub renders them inline without a
"large file" warning.

## Essential (cover the architecture + the core safety claim)

**1. `demo-gazebo.png`** — Gazebo, depot world, robot visible among several pedestrians (not an
empty world). Angle so at least 2–3 pedestrians and the robot are in frame together.

```bash
ros2 launch crowd_nav_bringup amcl.launch.py
# separate terminal:
ros2 launch crowd_nav_pedestrians pedestrians.launch.py num_pedestrians:=6
```
Wait for pedestrians to spawn and start moving, then screenshot the Gazebo window.

**2. `demo-rviz.png`** — RViz, same run, showing the local/global costmap, the robot model, the
live human position markers (from whichever `human_source_type` is active), and an active
planned path. Send a goal first so a path exists:

```bash
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```
(or use RViz's own "Nav2 Goal" toolbar button and click a point in the map).

**3. `demo-crowd-navigation.gif`** — 5–10 s of the robot actively navigating through the moving
crowd from capture 1/2 above — the single most important clip, since it's the one that shows
the whole system actually working, not just a static frame. RViz or Gazebo view, either works;
RViz is more informative (shows the plan re-routing around people).

## Recommended (show the safety mechanism actually firing)

**4. `demo-intervention.gif`** — a real `PROXIMITY` (or other OOD) intervention firing: terminal
running `amcl.launch.py` (showing the `[SafetySupervisor]` rejection log line) next to RViz at
the same moment, ideally windowed side by side before recording. Easiest way to force one on
demand rather than waiting for a random encounter: temporarily start pedestrians close to the
robot's path —

```bash
ros2 launch crowd_nav_pedestrians pedestrians.launch.py num_pedestrians:=8 mode:=reactive seed:=7
```
and send a goal that crosses the crowd's path. Watch the `amcl.launch.py` terminal for a line
containing `PROXIMITY` or `InterventionEvent`; capture the few seconds around it.

**5. `demo-keepout.gif`** — the cleanest, most quotable result in the whole project (439/439
rejections, zero violations): the supervised policy repeatedly refusing to cross a keepout zone
boundary. Reproduce it directly through the evaluation harness's own `keepout` phase — this runs
all three configs (`baseline_mppi`, `policy_raw`, `policy_supervised`) back to back against the
`depot_keepout_block` scenario, so watch the terminal output for `config=policy_supervised` and
capture that segment (Gazebo or RViz, either shows the zone-boundary rejections):

```bash
cd crowd_nav_ws/src/crowd_nav_evaluation/scripts
python3 run_matrix.py --results-dir /tmp/keepout_demo --phase keepout
```
`run_episode.py` (which this drives per-episode) takes a JSON-encoded episode dict, not simple
flags — going through `run_matrix.py --phase keepout` is the realistic way to reproduce this
scenario by hand rather than hand-constructing that JSON.

## Optional (polish, not essential)

**6. `demo-terminal-launch.gif`** — a clean terminal recording of the two launch commands from
capture 1 coming up end to end (useful for a README reader deciding whether the project actually
runs before they clone it). Any terminal recorder (`asciinema`, `peek` on a terminal window)
works; convert `asciinema` output with `agg` if a GIF specifically is wanted over a cast file.
