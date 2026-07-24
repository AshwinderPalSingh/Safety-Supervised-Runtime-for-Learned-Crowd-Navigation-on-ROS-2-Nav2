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


def _wait_for_marker(
        log_path, required_substrings, timeout_s, error_markers=('Traceback', 'RLException')):
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


SWEEP_PATTERNS = ('ign gazebo', 'gzserver', 'ros_gz_bridge', 'robot_state_publisher')


def _sweep_stray_processes(log_dir):
    """Belt-and-suspenders cleanup, run after every episode's own graceful teardown.

    Found needed while debugging the Phase 10 pilot (not anticipated going in): a botched
    episode (one whose pedestrians.launch.py failed at start) still left its amcl.launch.py
    side - including 'ign gazebo' itself - alive well past that episode's own _teardown() call
    returning. The next episode then launched a SECOND 'ign gazebo' for the same world, and
    with two /clock publishers on the same global topic name, its nav2 stack saw time jump
    backwards and every ros2_control spawner failed - a silent corruption, not a crash, so nothing
    in that episode's own result JSON flagged it. With 139 episodes queued to run unattended,
    that failure mode needed a guard, not just a note for next time.

    NOT implemented by shelling out to scripts/ros2_teardown.sh, despite that script already
    doing this exact SIGINT/grace/SIGTERM/grace/SIGKILL sweep - found out the hard way (first
    real pilot run, baseline_mppi episode only): that script's broader pattern list includes
    "nav2_", and this process's OWN command line contains the episode's JSON-encoded config,
    which for baseline_mppi is controller_plugin: "nav2_mppi_controller::MPPIController" -
    a literal substring match. `pkill -f "nav2_"` doesn't just catch stray Gazebo processes, it
    catches THIS process too, and SIGINTs/SIGKILLs run_episode.py out from under itself before
    it can print its own result. This is the self-match footgun that script's own header already
    warns about (see its Phase 1/2 history), in a new guise the header didn't anticipate: not the
    invoking shell's text, but a sibling process's argv containing scenario/config data. The
    patterns below are deliberately narrower than that script's - just the specific processes
    actually observed leaking - and every value ever placed in an episode dict (world/map
    filenames, controller_plugin strings, adapter_type, numeric fields) is checked against them
    to keep it that way.
    """
    own_cmdline = ' '.join(sys.argv)
    for pattern in SWEEP_PATTERNS:
        # The exact bug being guarded against: if a future scenario/config value ever makes
        # this process's own argv match one of these patterns, a naive sweep would sigkill
        # itself the same way it did here for "nav2_". Fail loudly instead of silently.
        assert pattern not in own_cmdline, (
            f"sweep pattern {pattern!r} matches this process's own command line - "
            "would self-match, see _sweep_stray_processes docstring")
    own_pid = os.getpid()
    sweep_log_path = os.path.join(log_dir, 'stray_process_sweep.log')
    with open(sweep_log_path, 'w') as f:
        for sig, grace_s in ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
            found = set()
            for pattern in SWEEP_PATTERNS:
                out = subprocess.run(
                    ['pgrep', '-f', pattern], capture_output=True, text=True).stdout
                found.update(int(p) for p in out.split())
            found.discard(own_pid)
            if not found:
                f.write(f"no stray processes matching {SWEEP_PATTERNS} - clean\n")
                return
            f.write(f"sending {sig.name} to stray pids {sorted(found)}\n")
            for pid in found:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            time.sleep(grace_s)
        # Re-check after the SIGKILL grace period instead of assuming survival - found on the
        # first real matrix run (not anticipated going in): the unconditional warning below fired
        # for a pid that pgrep just hadn't been re-polled for since SIGKILL is unmaskable and had,
        # in fact, already succeeded - a false alarm that would have wasted debugging time on
        # every occurrence for the rest of the 139-episode run.
        still_alive = set()
        for pattern in SWEEP_PATTERNS:
            out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout
            still_alive.update(int(p) for p in out.split())
        still_alive.discard(own_pid)
        if still_alive:
            f.write(f"WARNING: pids {sorted(still_alive)} survived SIGKILL - check manually\n")
        else:
            f.write("all stray pids confirmed dead after SIGKILL\n")


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
        try:
            _sweep_stray_processes(log_dir)
        except Exception as exc:  # never let the sweep itself hide the episode's own result
            with open(os.path.join(log_dir, 'stray_process_sweep.log'), 'a') as f:
                f.write(f"\n[run_episode] sweep raised: {exc!r}\n")

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
