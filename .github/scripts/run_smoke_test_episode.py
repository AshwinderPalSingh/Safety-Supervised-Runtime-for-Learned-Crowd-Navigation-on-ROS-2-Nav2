#!/usr/bin/env python3
"""Nightly smoke test body (nightly-smoke-test.yml): runs exactly one episode via the
evaluation harness's own run_episode.py (subprocess, matching how run_matrix.py itself invokes
it - no parallel launch mechanism), then asserts two things a plain "did it reach the goal"
check would miss: a real terminal outcome, and that the topics this project's stack depends on
actually carried messages. That second assertion exists because of a real bug: Phase 10 found
/ground_truth/robot_pose had been silently dead since Phase 4, with the stack launching fine
and Nav2 reaching its goal the whole time (docs/phase10-findings.md). A goal-reached check alone
would not have caught that; a topic-liveness check would have, immediately.
"""
import json
import subprocess
import sys

# baseline_mppi/dummy: the fastest, most reliable config in the Phase 10 matrix (~20s wall,
# zero supervisor/inference dependency) - this smoke test is about launch-file/topic health,
# not policy behavior, so the cheapest config that still exercises the full stack (Gazebo, Nav2,
# AMCL, the keepout costmap layer, the pedestrian simulator, ground-truth pose) is the right one.
EPISODE = {
    "episode_id": "nightly_smoke_test",
    "scenario": {
        "world_file": "open_arena.sdf", "world_name": "crowd_nav_open_arena",
        "map": "open_arena.yaml", "spawn": [-3.0, 0.0, 0.0], "goal": [3.0, 0.0],
        "num_pedestrians": 4,
    },
    "config": {
        "controller_plugin": "nav2_mppi_controller::MPPIController",
        "adapter_type": "dummy", "supervisor_enabled": "false",
    },
    "pedestrian_mode": "reactive", "seed": 0, "dropout_prob": 0.0, "zone": None,
}

# A message every ~5 real seconds is a generous floor for an ~10-20s episode at each of these
# topics' own real publish rates (AMCL and /ground_truth/robot_pose both well above 10Hz,
# /pedestrians and /scan lower but still multiple Hz) - not tuned to the exact rate, just loose
# enough to not be sensitive to CI-runner slowdown while still catching "zero, this topic is
# dead" outright.
MIN_EXPECTED_MESSAGES = {
    "ground_truth_robot_pose": 5,
    "pedestrians": 5,
    "amcl_pose": 1,
    "scan": 5,
}

ACCEPTABLE_OUTCOMES = {"success", "collision", "timeout", "nav2_aborted", "keepout_violation"}


def main():
    result = subprocess.run(
        [
            "ros2", "run", "crowd_nav_evaluation", "run_episode.py",
            "--episode-json", json.dumps(EPISODE),
            "--log-dir", "/tmp/nightly_smoke_test_logs",
        ],
        capture_output=True, text=True, timeout=180,
    )
    last_line = result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ""
    try:
        episode_result = json.loads(last_line)
    except (json.JSONDecodeError, IndexError):
        print("SMOKE TEST FAILED: could not parse run_episode.py output", file=sys.stderr)
        print(f"--- stdout ---\n{result.stdout}", file=sys.stderr)
        print(f"--- stderr ---\n{result.stderr}", file=sys.stderr)
        sys.exit(1)

    print(f"Episode result: {json.dumps(episode_result, indent=2)}")

    outcome = episode_result.get("outcome")
    if outcome not in ACCEPTABLE_OUTCOMES:
        print(
            f"SMOKE TEST FAILED: outcome '{outcome}' is not a real terminal outcome "
            f"(harness_error/harness_timeout means the stack itself never came up correctly)",
            file=sys.stderr)
        sys.exit(1)

    counts = episode_result.get("topic_message_counts")
    if counts is None:
        print(
            "SMOKE TEST FAILED: no topic_message_counts in the result - "
            "episode_monitor.py's own liveness tracking didn't run",
            file=sys.stderr)
        sys.exit(1)

    failures = []
    for topic, min_count in MIN_EXPECTED_MESSAGES.items():
        actual = counts.get(topic, 0)
        if actual < min_count:
            failures.append(f"  {topic}: {actual} messages (expected >= {min_count})")

    if failures:
        print(
            "SMOKE TEST FAILED: at least one topic the stack depends on looks dead "
            "(launched fine, but not actually publishing):",
            file=sys.stderr)
        print("\n".join(failures), file=sys.stderr)
        sys.exit(1)

    print("Smoke test passed: real outcome, all tracked topics live.")


if __name__ == "__main__":
    main()
