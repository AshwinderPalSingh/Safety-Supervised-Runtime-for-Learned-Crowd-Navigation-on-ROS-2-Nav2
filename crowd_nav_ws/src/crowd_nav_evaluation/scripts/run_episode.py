#!/usr/bin/env python3
"""Runs exactly ONE evaluation episode (IMPLEMENTATION_PLAN.md S4.9): launches the ROS2/Gazebo
stack, waits for it to become ready, optionally adds a keep-out zone, runs the in-episode
monitor to a terminal outcome, tears the stack down, and prints one JSON line with the result.
Designed to be invoked as a subprocess per episode (run_matrix.py does this) so each episode
gets a genuinely fresh ROS graph/process tree - no node-name or DDS-participant reuse risk
across episodes, and one hung episode can't wedge the whole matrix run.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory

RMW = 'rmw_cyclonedds_cpp'


def _env():
    env = os.environ.copy()
    env['RMW_IMPLEMENTATION'] = RMW
    return env


def _start(cmd, log_path):
    log_file = open(log_path, 'w')
    proc = subprocess.Popen(
        cmd, env=_env(), stdout=log_file, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return proc, log_file


def _wait_for_marker(log_path, required_substrings, timeout_s,
                      error_markers=('Traceback', 'RLException')):
    """Waits for a single LOG LINE containing every one of required_substrings (not just the
    file as a whole) - amcl.launch.py runs two lifecycle managers (navigation and zones), and a
    plain whole-file substring search for "Managed nodes are active" matches whichever one
    happens to log it first, which can be lifecycle_manager_zones (a small, separate node set)
    well before the real Nav2 stack (map_server/amcl/controller_server/...) has finished
    activating - found via the pilot run (S4.9.1) exactly as designed to catch, not assumed."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if os.path.exists(log_path):
            with open(log_path, 'r', errors='replace') as f:
                for line in f:
                    if all(s in line for s in required_substrings):
                        return True
                    for em in error_markers:
                        if em in line:
                            return False
        time.sleep(0.5)
    return False


def _teardown(proc, log_file, grace_s=5.0):
    if proc.poll() is not None:
        log_file.close()
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        log_file.close()
        return
    deadline = time.monotonic() + grace_s
    while proc.poll() is None and time.monotonic() < deadline:
        time.sleep(0.2)
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + grace_s
        while proc.poll() is None and time.monotonic() < deadline:
            time.sleep(0.2)
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=10)
    log_file.close()


def run(episode, log_dir):
    os.makedirs(log_dir, exist_ok=True)
    scenario = episode['scenario']
    config = episode['config']
    spawn = scenario['spawn']

    maps_dir = os.path.join(get_package_share_directory('crowd_nav_bringup'), 'maps')
    map_path = os.path.join(maps_dir, scenario['map'])

    amcl_cmd = [
        'ros2', 'launch', 'crowd_nav_bringup', 'amcl.launch.py',
        f"map:={map_path}",
        f"world_file:={scenario['world_file']}",
        f"spawn_x:={spawn[0]}", f"spawn_y:={spawn[1]}", f"spawn_yaw:={spawn[2]}",
        f"controller_plugin:={config['controller_plugin']}",
        f"adapter_type:={config['adapter_type']}",
        f"supervisor_enabled:={config['supervisor_enabled']}",
        f"perception_dropout_prob:={episode['dropout_prob']}",
        f"perception_degradation_seed:={episode['seed']}",
    ]
    amcl_log_path = os.path.join(log_dir, 'amcl.log')
    amcl_proc, amcl_log = _start(amcl_cmd, amcl_log_path)

    ped_cmd = [
        'ros2', 'launch', 'crowd_nav_pedestrians', 'pedestrians.launch.py',
        f"num_pedestrians:={scenario['num_pedestrians']}",
        f"mode:={episode['pedestrian_mode']}",
        f"seed:={episode['seed']}",
        f"world_name:={scenario['world_name']}",
    ]
    ped_log_path = os.path.join(log_dir, 'pedestrians.log')
    ped_proc, ped_log = _start(ped_cmd, ped_log_path)

    result = {'episode_id': episode['episode_id'], 'outcome': 'harness_error'}
    try:
        ready = _wait_for_marker(
            amcl_log_path, ('lifecycle_manager_navigation', 'Managed nodes are active'),
            timeout_s=60.0)
        if not ready:
            result['error'] = 'stack did not become ready within 60s'
            return result

        if episode['zone'] is not None:
            zone = episode['zone']
            add_zone_cmd = [
                'ros2', 'service', 'call', '/add_zone', 'crowd_nav_zones/srv/AddZone',
                json.dumps(zone),
            ]
            subprocess.run(
                add_zone_cmd, env=_env(), capture_output=True, text=True, timeout=15)

        # rclpy import deferred until here - importing before the stack's own DDS
        # participants exist is harmless, but keeping it local makes the dependency ordering
        # (stack up, then monitor) explicit in the code, not just in execution timing.
        import rclpy
        from episode_monitor import EpisodeMonitor

        rclpy.init(args=None)
        try:
            monitor = EpisodeMonitor(scenario['goal'], episode['zone'])
            metrics = monitor.run()
            monitor.destroy_node()
        finally:
            rclpy.shutdown()

        result.update(metrics)
        result['outcome'] = metrics.get('outcome', 'harness_error')
    finally:
        _teardown(ped_proc, ped_log)
        _teardown(amcl_proc, amcl_log)

    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--episode-json', required=True, help='episode dict, JSON-encoded')
    parser.add_argument('--log-dir', required=True)
    args = parser.parse_args()
    episode = json.loads(args.episode_json)
    result = run(episode, args.log_dir)
    print(json.dumps(result))


if __name__ == '__main__':
    main()
