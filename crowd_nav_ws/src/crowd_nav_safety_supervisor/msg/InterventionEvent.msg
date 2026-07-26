# Published on /intervention_events every time SafetySupervisor
# rejects a candidate command - Phase 10's evaluation harness is the eventual consumer for the
# "intervention rate broken down by cause" headline metric. Also logged via RCLCPP_WARN at the
# point of rejection (edge-triggered per cause) for interactive visibility.

builtin_interfaces/Time stamp

# Full trigger taxonomy (S4.4's closing paragraph) - the 5 OOD criteria, the 2 forward-sim
# causes, and the watchdog's own INFERENCE_TIMEOUT (published by CrowdNavController directly,
# not SafetySupervisor, since it fires on a different path entirely, S4.8.4).
uint8 CROWD_SIZE=0
uint8 PROXIMITY=1
uint8 RELATIVE_SPEED=2
uint8 COMMAND_LIMIT=3
uint8 LOW_PERCEPTION_CONFIDENCE=4
uint8 COSTMAP_COLLISION=5
uint8 KEEPOUT_VIOLATION=6
uint8 INFERENCE_TIMEOUT=7
uint8 cause

# The policy's candidate command that was rejected, and what was actually sent instead (S4.8.4:
# a controlled stop for every cause except INFERENCE_TIMEOUT, which defers to the embedded MPPI).
float64 rejected_vx
float64 rejected_vy
float64 sent_vx
float64 sent_vy
