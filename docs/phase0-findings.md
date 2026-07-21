# Phase 0 findings log

Live log, per finding, per IMPLEMENTATION_PLAN.md's Phase 0. Each entry below was committed
as it landed, not written up after the fact. See IMPLEMENTATION_PLAN.md for what each check
is for and what "done" means.

## Environment survey (2026-07-21)

- ROS 2 Humble installed at `/opt/ros/humble`.
- Gazebo: `gz sim` reports **Gazebo Sim 6.18.0 = Fortress**. Confirmed correct distro, not
  Garden/Harmonic.
- Python 3.10.12, no PyTorch installed system-wide.
- No GPU (`nvidia-smi` absent) — confirms the CPU-only ONNX Runtime call in
  IMPLEMENTATION_PLAN.md §1.3 is correct for this machine, not just in general.
- `colcon` present. `cmake`/`g++`/`libboost-all-dev` present (needed to build Python-RVO2 for
  the CrowdNav checkpoint validation).
- `ros-humble-nav2-*` (mppi-controller, amcl, costmap-2d, etc.) and `ros-humble-slam-toolbox`
  already installed. `ros-humble-gz-ros2-control` is **available via apt but not installed**.
- **Blocked, needs you**: installing `gz_ros2_control` requires `sudo apt install
  ros-humble-gz-ros2-control`, and this machine has no passwordless sudo. I can't run this
  myself — please run it (or grant a way for me to), and I'll pick the diff-drive-demo check
  back up. Nothing else in Phase 0 depends on it, so it's not blocking the rest of this phase.

## SARL checkpoint validation

Pass/fail threshold, pinned before running anything (per your ask): the SARL paper (Chen et
al., ICRA 2019, "Crowd-Robot Interaction") reports **0.99 success rate on 500 test cases in
the invisible-robot setting** for SARL (LM-SARL, a variant, reports 1.00 — we're using plain
SARL). Threshold for treating `tkkim-robot/Gazebo-CrowdNav`'s checkpoint as valid: **success
rate ≥ 0.95** (small tolerance band for run-to-run RL variance). Below 0.90 is a hard fail —
training from scratch starts immediately as a background job if that happens, per your
standing ask, and I'll tell you the moment that's triggered since it changes the timeline.

*(checkpoint result recorded below once the run completes)*
