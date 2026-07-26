#!/usr/bin/env python3
"""Top-level driver for the Phase 10 evaluation matrix. Each
episode runs as its own subprocess (run_episode.py) - a fresh ROS graph/process tree per
episode, so one hung or crashed episode can't wedge the rest of the matrix. Writes episodes.csv
and interventions.csv incrementally (flushed after every episode), so a partial run is never a
lost run.

--pilot runs exactly S4.9.1's pilot: one scenario, one seed, all three configs. Anything else
(--phase core|sweep|keepout|all) is the real matrix, only ever run after the pilot has been
inspected by hand.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
import time

import scenarios as scn

THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def pilot_episodes():
    """S4.9.1: one scenario, one seed, all three configs - not part of any counted N."""
    scenario_name = 'open_arena'
    scenario = scn.CORE_SCENARIOS[scenario_name]
    for config_name, config in scn.CONFIGS.items():
        yield {
            'episode_id': f'pilot_{scenario_name}_{config_name}',
            'scenario_name': scenario_name,
            'scenario': scenario,
            'config_name': config_name,
            'config': config,
            'pedestrian_mode': 'reactive',
            'seed': 0,
            'dropout_prob': 0.0,
            'zone': None,
        }


def run_one(episode, results_dir, timeout_s):
    log_dir = os.path.join(results_dir, 'logs', episode['episode_id'])
    cmd = [
        sys.executable, os.path.join(THIS_DIR, 'run_episode.py'),
        '--episode-json', json.dumps(episode),
        '--log-dir', log_dir,
    ]
    start = time.monotonic()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return {
            'episode_id': episode['episode_id'], 'outcome': 'harness_timeout',
            'wall_s': time.monotonic() - start,
        }
    wall_s = time.monotonic() - start
    # run_episode.py's stdout is exactly one JSON line (its own print(json.dumps(...))) -
    # anything else on stdout/stderr is launch/ROS noise, kept in the per-episode log dir, not
    # parsed here.
    last_line = proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else ''
    try:
        result = json.loads(last_line)
    except (json.JSONDecodeError, IndexError):
        result = {
            'episode_id': episode['episode_id'], 'outcome': 'harness_error',
            'error': 'could not parse run_episode.py output',
            'stdout_tail': proc.stdout[-2000:], 'stderr_tail': proc.stderr[-2000:],
        }
    result['wall_s'] = wall_s
    return result


EPISODES_FIELDS = [
    'episode_id', 'scenario_name', 'config_name', 'pedestrian_mode', 'seed', 'dropout_prob',
    'outcome', 'duration_s', 'wall_s', 'path_length_m', 'min_human_distance_m',
    'intervention_count_total',
] + [f'intervention_count_cause_{c}' for c in range(8)]

INTERVENTIONS_FIELDS = [
    'episode_id', 'sim_stamp_sec', 'sim_stamp_nanosec', 'cause', 'rejected_vx', 'rejected_vy',
    'sent_vx', 'sent_vy', 'amcl_cov_trace',
]


def write_episode_row(writer, episode, result):
    row = {
        'episode_id': episode['episode_id'],
        'scenario_name': episode['scenario_name'],
        'config_name': episode['config_name'],
        'pedestrian_mode': episode['pedestrian_mode'],
        'seed': episode['seed'],
        'dropout_prob': episode['dropout_prob'],
        'outcome': result.get('outcome'),
        'duration_s': result.get('duration_s'),
        'wall_s': result.get('wall_s'),
        'path_length_m': result.get('path_length_m'),
        'min_human_distance_m': result.get('min_human_distance_m'),
        'intervention_count_total': result.get('intervention_count_total', 0),
    }
    by_cause = result.get('intervention_count_by_cause', {}) or {}
    for c in range(8):
        row[f'intervention_count_cause_{c}'] = by_cause.get(str(c), by_cause.get(c, 0))
    writer.writerow(row)


def write_intervention_rows(writer, episode, result):
    for r in result.get('intervention_rows', []) or []:
        row = {'episode_id': episode['episode_id']}
        row.update(r)
        row.pop('timestamp_s', None)
        writer.writerow(row)


def run_batch(episodes, results_dir, timeout_s, episodes_writer, interventions_writer,
              episodes_file, interventions_file):
    total = len(episodes)
    for i, episode in enumerate(episodes):
        print(f"[{i + 1}/{total}] running {episode['episode_id']}...", file=sys.stderr, flush=True)
        result = run_one(episode, results_dir, timeout_s)
        write_episode_row(episodes_writer, episode, result)
        write_intervention_rows(interventions_writer, episode, result)
        episodes_file.flush()
        interventions_file.flush()
        duration = result.get('duration_s') or 0.0
        print(
            f"  -> {result.get('outcome')} in {result.get('wall_s', 0):.1f}s wall "
            f"({duration:.1f}s sim)", file=sys.stderr, flush=True)
        if result.get('error'):
            print(f"     error: {result['error']}", file=sys.stderr, flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-dir', required=True)
    parser.add_argument(
        '--phase', choices=['pilot', 'core', 'sweep', 'keepout', 'ood_reachability', 'all'],
        default='pilot')
    parser.add_argument('--timeout-s', type=float, default=180.0)
    args = parser.parse_args()

    os.makedirs(args.results_dir, exist_ok=True)
    episodes_path = os.path.join(args.results_dir, 'episodes.csv')
    interventions_path = os.path.join(args.results_dir, 'interventions.csv')
    episodes_is_new = not os.path.exists(episodes_path)
    interventions_is_new = not os.path.exists(interventions_path)

    with open(episodes_path, 'a', newline='') as ef, \
         open(interventions_path, 'a', newline='') as intf:
        ewriter = csv.DictWriter(ef, fieldnames=EPISODES_FIELDS)
        iwriter = csv.DictWriter(intf, fieldnames=INTERVENTIONS_FIELDS)
        if episodes_is_new:
            ewriter.writeheader()
        if interventions_is_new:
            iwriter.writeheader()

        if args.phase == 'pilot':
            episodes = list(pilot_episodes())
        elif args.phase == 'core':
            episodes = list(scn.iter_core_episodes())
        elif args.phase == 'sweep':
            episodes = list(scn.iter_noise_sweep_episodes())
        elif args.phase == 'keepout':
            episodes = list(scn.iter_keepout_block_episodes())
        elif args.phase == 'ood_reachability':
            episodes = list(scn.iter_ood_reachability_episodes())
        else:
            episodes = (
                list(scn.iter_core_episodes()) + list(scn.iter_noise_sweep_episodes()) +
                list(scn.iter_keepout_block_episodes()) +
                list(scn.iter_ood_reachability_episodes()))

        print(f"Running {len(episodes)} episodes (phase={args.phase})...", file=sys.stderr)
        run_batch(episodes, args.results_dir, args.timeout_s, ewriter, iwriter, ef, intf)

    print(f"Done. Wrote {episodes_path} and {interventions_path}", file=sys.stderr)


if __name__ == '__main__':
    main()
