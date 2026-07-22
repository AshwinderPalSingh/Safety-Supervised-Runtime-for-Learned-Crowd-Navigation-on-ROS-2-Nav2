#!/usr/bin/env bash
# Pre-flight check: fail loudly if the DDS shared-memory state looks dirty, rather than let a
# launch silently degrade into the lifecycle-activation hangs found in Phase 2
# (docs/phase2-findings.md). Meant to be run before any `ros2 launch` invocation, and
# especially before Phase 10's evaluation harness runs the full stack unattended, hundreds of
# times - by then this must be an automatic gate, not something to remember to check by hand.
#
# Threshold is a judgement call, not a hard fact: a handful of segments from the ros2 daemon
# and other legitimate long-lived participants is normal. Tens of them is the pattern Phase 2
# actually observed (87) right before the lifecycle manager started hanging.
set -uo pipefail

THRESHOLD="${DDS_HEALTH_STALE_THRESHOLD:-20}"

stale_count=$(ls /dev/shm/ 2>/dev/null | grep -c fastrtps || true)

if [ "$stale_count" -gt "$THRESHOLD" ]; then
  echo "[check_dds_health] FAIL: $stale_count fastrtps_* segments in /dev/shm (threshold: $THRESHOLD)." >&2
  echo "[check_dds_health] This is the exact pattern that preceded Nav2 lifecycle-activation hangs in Phase 2." >&2
  echo "[check_dds_health] Run scripts/ros2_teardown.sh first (only if no ROS/Gazebo processes are currently running)." >&2
  exit 1
fi

echo "[check_dds_health] OK: $stale_count fastrtps_* segments in /dev/shm (threshold: $THRESHOLD)."
exit 0
