#!/usr/bin/env bash
# Graceful teardown for a ros2/Gazebo simulation session, added after Phase 2
# (docs/phase2-findings.md) found that repeated `kill -9` across a long session leaves
# /dev/shm littered with stale FastRTPS shared-memory segments that don't get cleaned up
# because a SIGKILLed DDS participant never runs its own cleanup path. That degraded DDS
# reliability badly enough to hang Nav2's lifecycle manager outright.
#
# Order matters: SIGINT/SIGTERM first (let each process run its own shutdown/cleanup path),
# a grace period, THEN SIGKILL only for anything still alive, and /dev/shm cleanup only after
# confirming nothing ROS-related is still running (never delete shared memory a live process
# might still hold).
#
# Safe to run as a saved script (unlike an inline eval'd one-liner): the patterns below don't
# appear in this script's own invocation command, so pgrep/pkill -f here can't self-match the
# way it did during interactive Phase 1/2 debugging (see docs/phase1-findings.md) - that
# footgun is specifically about a pattern appearing in the text of the command that's running
# the search, which isn't the case for `bash ros2_teardown.sh` invoked on its own.
#
# CONFIRMED IN PRACTICE (Phase 2, second occurrence): that safety breaks if this script is
# invoked in the SAME shell command as something else that itself contains one of these
# patterns as literal text (e.g. a manual `pkill`/`grep` one-liner run just before it) - the
# self-match risk comes back because the *combined* command line now contains the pattern
# again. Always invoke this script by itself, not chained after other pattern-matching
# commands in the same call.
set -uo pipefail

PATTERNS=(
  "ign gazebo"
  "gz sim"
  "ros2 launch"
  "nav2_"
  "slam_toolbox"
  "robot_state_publisher"
  "ros_gz_bridge"
  "controller_manager"
)

echo "[ros2_teardown] Sending SIGINT to matching processes..."
for pattern in "${PATTERNS[@]}"; do
  pkill -INT -f "$pattern" 2>/dev/null || true
done

sleep 5

echo "[ros2_teardown] Sending SIGTERM to anything still alive..."
for pattern in "${PATTERNS[@]}"; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done

sleep 3

echo "[ros2_teardown] Checking for stragglers..."
remaining=""
for pattern in "${PATTERNS[@]}"; do
  found=$(pgrep -f "$pattern" 2>/dev/null || true)
  if [ -n "$found" ]; then
    remaining="$remaining $found"
  fi
done

if [ -n "$remaining" ]; then
  echo "[ros2_teardown] Forcing SIGKILL on stragglers: $remaining"
  # shellcheck disable=SC2086
  kill -9 $remaining 2>/dev/null || true
  sleep 2
fi

still_running=0
for pattern in "${PATTERNS[@]}"; do
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    still_running=1
  fi
done

if [ "$still_running" -eq 1 ]; then
  echo "[ros2_teardown] WARNING: something is still running after SIGKILL - not touching /dev/shm."
  echo "[ros2_teardown] Run 'ps aux | grep -E \"ign gazebo|nav2_|ros2 launch\"' and investigate manually."
  exit 1
fi

echo "[ros2_teardown] Nothing ROS/Gazebo-related running. Cleaning stale FastRTPS shared memory..."
stale_count=$(ls /dev/shm/ 2>/dev/null | grep -c fastrtps || true)
if [ "$stale_count" -gt 0 ]; then
  rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
  echo "[ros2_teardown] Removed $stale_count stale fastrtps_* segment(s) from /dev/shm."
else
  echo "[ros2_teardown] /dev/shm already clean, nothing to remove."
fi

echo "[ros2_teardown] Restarting the ros2 daemon (picks up a clean discovery state)..."
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

echo "[ros2_teardown] Done."
