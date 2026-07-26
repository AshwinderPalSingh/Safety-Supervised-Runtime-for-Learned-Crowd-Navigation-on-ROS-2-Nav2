#!/usr/bin/env python3
"""Scenario/config/matrix definitions for the Phase 10 evaluation harness.
Plain data, imported directly by run_episode.py/run_matrix.py
(installed side by side in lib/crowd_nav_evaluation/, no package __init__ needed - a script's
own directory is always on sys.path when run directly).

N and the noise-sweep points are committed here, in one place, matching S4.9.2's explicit
"decided now, not after seeing any results" requirement - nothing downstream should introduce
a second, different number.
"""

CORE_SEEDS_N = 8
NOISE_SWEEP_DROPOUT_PROBS = (0.0, 0.1, 0.2, 0.3, 0.5)
NOISE_SWEEP_SEEDS_N = 8

MAX_EPISODE_DURATION_S = 120.0
# Ground-truth collision definition for the harness's OWN outcome determination - deliberately
# this robot's real physical radius (matching crowd_nav_control/diff_drive_controller.yaml's
# world, not policy_radius_m's training-distribution value, S4.3) plus the pedestrian sim's own
# real radius, since the harness measures actual physical safety, not training-distribution
# matching.
ROBOT_PHYSICAL_RADIUS_M = 0.14
PED_RADIUS_M = 0.25
COLLISION_DISTANCE_M = ROBOT_PHYSICAL_RADIUS_M + PED_RADIUS_M


# human_source_type pinned to "ground_truth" explicitly on every config below, not left to
# CrowdNavController's own default (which is "lidar_tracked" as of docs/lidar_perception-
# findings.md - a bare launch should reflect real, sensor-based perception). Every episode this
# harness has ever run, and every headline number in docs/phase10-findings.md/docs/audit.md, was
# produced against the ground-truth oracle - pinning it here keeps the entire already-reported
# 142-episode matrix exactly reproducible regardless of what the controller's own default becomes
# later. A real perception-in-the-loop evaluation (running these same scenarios against
# LidarHumanTrackerSource instead) is a deliberate, separate follow-on measurement, not something
# that should happen silently as a side effect of changing a default elsewhere.
CONFIGS = {
    "baseline_mppi": {
        "controller_plugin": "nav2_mppi_controller::MPPIController",
        "adapter_type": "dummy",  # unused - MPPIController never reads FollowPath.adapter_type
        "supervisor_enabled": "false",
        "human_source_type": "ground_truth",
    },
    "policy_raw": {
        "controller_plugin": "crowd_nav_controller::CrowdNavController",
        "adapter_type": "sarl",
        "supervisor_enabled": "false",
        "human_source_type": "ground_truth",
    },
    "policy_supervised": {
        "controller_plugin": "crowd_nav_controller::CrowdNavController",
        "adapter_type": "sarl",
        "supervisor_enabled": "true",
        "human_source_type": "ground_truth",
    },
}

# Reused across every scenario in a family - kept here once rather than repeated per scenario
# dict, so the two families stay directly comparable in the one dimension that matters (size),
# per S4.9's "same footprint as depot, no clutter" design.
_OPEN_ARENA_WORLD = "open_arena.sdf"
_OPEN_ARENA_MAP = "open_arena.yaml"
_DEPOT_WORLD = "depot_scaled.sdf"
_DEPOT_MAP = "depot_scaled.yaml"
_SPAWN = (-3.0, 0.0, 0.0)

# world_name is the SDF <world name="..."> value, NOT the world_file filename above - Gazebo
# topics (scene_broadcaster's dynamic_pose/info, used for ground-truth pose - see
# robot_pose_extractor.py) are namespaced by this, and there's no mechanical way to derive one
# from the other, so both are carried explicitly end to end (run_episode.py passes both to their
# respective launch files).
_OPEN_ARENA_WORLD_NAME = "crowd_nav_open_arena"
_DEPOT_WORLD_NAME = "crowd_nav_depot_scaled"

# Core matrix: 2 families x 2 pedestrian modes, N=8 seeds each (seeds assigned outside this
# dict, see iter_core_episodes() below) - num_pedestrians=4 is deliberately below
# SafetySupervisorConfig::max_train_humans (5 default) so CROWD_SIZE isn't a structural given
# for every episode (phase9-findings.md's own CROWD_SIZE finding).
CORE_SCENARIOS = {
    "open_arena": {
        "world_file": _OPEN_ARENA_WORLD,
        "world_name": _OPEN_ARENA_WORLD_NAME,
        "map": _OPEN_ARENA_MAP,
        "spawn": _SPAWN,
        "goal": (3.0, 0.0),
        "num_pedestrians": 4,
    },
    "depot": {
        "world_file": _DEPOT_WORLD,
        "world_name": _DEPOT_WORLD_NAME,
        "map": _DEPOT_MAP,
        "spawn": _SPAWN,
        "goal": (2.0, 0.0),
        "num_pedestrians": 4,
    },
}

# Named, permanent scenario - replaces Phase 9's inconclusive
# ad hoc zone placements. Run once per config, not part of the N=8 statistical matrix. Expected
# outcome per config is documented in the plan, not just implied by the harness's own behavior.
#
# zone center_x=1.0 (not -0.5, its original value): zones are authored and enforced in the 'map'
# frame (zone_manager_node.py's own storage comment), where this scenario's spawn (-3.0, 0.0 in
# GAZEBO WORLD frame) localizes to map (0, 0) and the goal (2.0, 0.0) IS map frame directly - so
# the map-frame path runs (0,0)->(2,0), and a zone meant to sit on that path needs a map-frame
# x between 0 and 2, not -0.5 (which is behind the start, in a direction the robot never travels
# toward). The original value was chosen with WORLD-frame intuition (roughly midway between
# world x=-3 and world x=2) without accounting for the world/map offset - found only after the
# first real run showed zero supervisor interventions across all three configs, including the
# one the harness itself flagged as a violation (see docs/phase10-findings.md for the full
# root-cause trace, including the matching fix to episode_monitor.py's own zone check, which had
# the identical frame confusion on the harness's ground-truth side).
KEEPOUT_BLOCK_SCENARIO = {
    "name": "depot_keepout_block",
    "world_file": _DEPOT_WORLD,
    "world_name": _DEPOT_WORLD_NAME,
    "map": _DEPOT_MAP,
    "spawn": _SPAWN,
    "goal": (2.0, 0.0),
    "num_pedestrians": 0,  # isolate the keep-out mechanism, not perception/crowd effects
    "zone": {"zone_id": "keepout_block", "center_x": 1.0, "center_y": 0.0,
             "size_x": 1.0, "size_y": 1.5},
}


def iter_core_episodes():
    """Yields one dict per core-matrix episode: scenario name, scenario def, config name,
    config def, pedestrian mode, seed. 2 families x 3 configs x 2 modes x N=8 = 96 episodes."""
    for scenario_name, scenario in CORE_SCENARIOS.items():
        for config_name, config in CONFIGS.items():
            for mode in ("reactive", "non_reactive"):
                for seed in range(CORE_SEEDS_N):
                    yield {
                        "episode_id": f"{scenario_name}_{config_name}_{mode}_seed{seed}",
                        "scenario_name": scenario_name,
                        "scenario": scenario,
                        "config_name": config_name,
                        "config": config,
                        "pedestrian_mode": mode,
                        "seed": seed,
                        "dropout_prob": 0.0,
                        "zone": None,
                    }


def iter_noise_sweep_episodes():
    """policy_supervised / open_arena / reactive only (S4.9.2) - isolates the perception
    effect from depot's own navigability difficulty. 5 dropout points x N=8 = 40 episodes."""
    scenario = CORE_SCENARIOS["open_arena"]
    config = CONFIGS["policy_supervised"]
    for dropout_prob in NOISE_SWEEP_DROPOUT_PROBS:
        for seed in range(NOISE_SWEEP_SEEDS_N):
            yield {
                "episode_id": f"noise_sweep_dropout{dropout_prob}_seed{seed}",
                "scenario_name": "open_arena",
                "scenario": scenario,
                "config_name": "policy_supervised",
                "config": config,
                "pedestrian_mode": "reactive",
                "seed": seed,
                "dropout_prob": dropout_prob,
                "zone": None,
            }


def iter_keepout_block_episodes():
    """One run per config (S4.9.3) - a decisive demonstration plus a permanent regression
    test, not a statistical distribution, so it isn't seeded/repeated like the core matrix."""
    scenario = KEEPOUT_BLOCK_SCENARIO
    for config_name, config in CONFIGS.items():
        yield {
            "episode_id": f"depot_keepout_block_{config_name}",
            "scenario_name": "depot_keepout_block",
            "scenario": scenario,
            "config_name": config_name,
            "config": config,
            "pedestrian_mode": "reactive",
            "seed": 0,
            "dropout_prob": 0.0,
            "zone": scenario["zone"],
        }


def iter_ood_reachability_episodes():
    """docs/audit.md S1.1: CROWD_SIZE, RELATIVE_SPEED, and INFERENCE_TIMEOUT never fired across
    the entire 139-episode core/sweep/keepout matrix - three of the eight enumerated OOD trigger
    causes were silent zeros, and for the first two, structurally so: no scenario in this suite
    ever configures more than 4 pedestrians (max_train_humans=5) or a pedestrian speed above
    1.0 m/s (max_train_speed_mps=1.5). Rather than retune those thresholds to match what this
    project's scenarios happen to do (which would be tuning the measuring stick to the
    measurement), these are three named, permanent, single-run demonstrations - the same
    pattern KEEPOUT_BLOCK_SCENARIO already established for exactly this purpose (S4.9.3): not
    part of the N=8 statistical matrix, but a decisive proof-of-reachability and a permanent
    regression test in one artifact. policy_supervised only - baseline_mppi/policy_raw run with
    supervisor_enabled=false, so no OOD criterion could ever fire for them regardless of scene.
    COMMAND_LIMIT is not demonstrated here: docs/audit.md S1.1 already proves it mathematically
    unreachable by construction (max candidate speed exactly equals, never exceeds, the
    threshold) - firing it would require overriding the real production config values, which
    would demonstrate a different, artificial system, not this one."""
    base = CORE_SCENARIOS["open_arena"]

    crowd_size_scenario = dict(base, num_pedestrians=6)  # exceeds max_train_humans=5
    yield {
        "episode_id": "ood_demo_crowd_size",
        "scenario_name": "ood_demo_crowd_size",
        "scenario": crowd_size_scenario,
        "config_name": "policy_supervised",
        "config": CONFIGS["policy_supervised"],
        "pedestrian_mode": "reactive",
        "seed": 0,
        "dropout_prob": 0.0,
        "zone": None,
    }

    yield {
        "episode_id": "ood_demo_relative_speed",
        "scenario_name": "ood_demo_relative_speed",
        "scenario": base,
        "config_name": "policy_supervised",
        "config": CONFIGS["policy_supervised"],
        "pedestrian_mode": "reactive",
        "seed": 0,
        "dropout_prob": 0.0,
        "zone": None,
        "ped_max_speed": 2.0,  # exceeds max_train_speed_mps=1.5
    }

    yield {
        "episode_id": "ood_demo_inference_timeout",
        "scenario_name": "ood_demo_inference_timeout",
        "scenario": base,
        "config_name": "policy_supervised",
        "config": CONFIGS["policy_supervised"],
        "pedestrian_mode": "reactive",
        "seed": 0,
        "dropout_prob": 0.0,
        "zone": None,
        "debug_inject_decision_delay_s": 0.5,  # exceeds watchdog_window_s=0.03
    }
