# Measurements needed on the physical Nvis 3302ARD

Every value below is an estimate marked `[PENDING]` at its point of use in the xacro/config
files. Replace the estimate with a measured value in **every file listed** for that row (some
values are duplicated between the URDF and the controller config and must be kept in sync
manually — they are not derived from each other).

| # | Value | Current estimate | Tool | Update in |
|---|---|---|---|---|
| 1 | `wheel_radius` | 0.0325 m | Calipers, measured across the wheel's rolling diameter, halved | `crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro` (`wheel_radius` property) **and** `crowd_nav_ws/src/crowd_nav_control/config/diff_drive_controller.yaml` (`wheel_radius`) |
| 2 | `wheel_width` | 0.026 m | Calipers, measured across the wheel's tread width | `crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro` (`wheel_width` property) |
| 3 | `wheel_separation` | 0.180 m | Measure center-to-center distance between the two drive wheels along the axle | `crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro` (`wheel_separation` property) **and** `crowd_nav_ws/src/crowd_nav_control/config/diff_drive_controller.yaml` (`wheel_separation`) |
| 4 | `caster_offset_x` | -0.070 m | Ruler/calipers from the drive-wheel axle centerline to the caster contact point, along the robot's forward axis (negative = behind the axle) | `crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro` (`caster_offset_x` property) |
| 5 | `caster_radius` | 0.012 m | Calipers on the caster wheel/ball | `crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro` (`caster_radius` property) |
| 6 | `chassis_ground_clearance` | 0.005 m | Ruler, gap between the chassis underside and the ground with the robot resting on its wheels | `crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro` (`chassis_ground_clearance` property) |
| 7 | `wheel_mass` (each) | 0.05 kg | Kitchen/postal scale, one wheel removed and weighed individually | `crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro` (`wheel_mass` property) |
| 8 | `caster_mass` | 0.02 kg | Same scale, caster assembly weighed individually | `crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro` (`caster_mass` property) |
| 9 | `lidar_standoff` | 0.005 m | Ruler, height of the LiDAR's mounting bracket above the chassis top surface (depends on whichever LiDAR unit and bracket end up used — see note below) | `crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro` (`lidar_standoff` property) |

Also worth re-checking once the physical robot is in hand (not a missing number, but an
assumption baked into the model): `chassis_mass` is currently computed as
`total_mass - 2*wheel_mass - caster_mass` (see the xacro) — if you weigh the bare chassis
directly, prefer that measurement over the subtraction and adjust `total_mass` or the split if
they don't reconcile.

## Not measurements — deliberate design parameters, not physical unknowns

These are **not** in the table above because they aren't properties of the physical robot to
measure; they're parameters this project chooses:

- **LiDAR range/rate/resolution/noise** (`crowd_nav_description/urdf/lidar.xacro` macro
  args, defaults: 8 m range, 5 Hz, 360 samples/1°, 0.02 m noise stddev): a deliberately
  conservative *spec* per the project brief, not a measurement of a specific sensor. If a real
  LiDAR is chosen later (e.g. an LD14-class unit), update these to match its actual datasheet
  — at that point it becomes a real measurement/lookup, but right now it's a design choice.
- **`max_vel_x` / velocity limits** (`crowd_nav_control/config/diff_drive_controller.yaml`):
  set to 1.0 m/s, the *policy-training-matched* regime (IMPLEMENTATION_PLAN.md §7), not the
  physical robot's real top speed. The physical Nvis on its 8.4 V/3000 mAh pack under-driving
  the 12 V/150 RPM motors will manage roughly **0.3 m/s** — that's the hardware-deviation
  value to switch to if/when physical deployment happens (already set as `min_velocity` in the
  same file as a placeholder for the reverse direction; the forward hardware-matched value
  should replace `max_velocity` specifically for hardware runs, not simulation runs).

## How to update these efficiently

All nine measured values live in xacro `<xacro:property>` tags with an inline `[PENDING]`
comment (`crowd_nav_ws/src/crowd_nav_description/urdf/nvis_3302ard.xacro`) — search that file
for `[PENDING]` to find every one in place, plus the two properties that are also duplicated
in `crowd_nav_control/config/diff_drive_controller.yaml`. Nothing else in the codebase should
hardcode these numbers; if you find a second hardcoded copy anywhere else, that's a bug to fix
(the whole point of the xacro properties is a single source of truth per value).
