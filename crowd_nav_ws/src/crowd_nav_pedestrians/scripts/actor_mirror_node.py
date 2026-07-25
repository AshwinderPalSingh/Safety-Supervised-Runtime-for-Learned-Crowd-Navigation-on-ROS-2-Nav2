#!/usr/bin/env python3
"""Gazebo physical body for Phase 4's pedestrian simulation (IMPLEMENTATION_PLAN.md S1.2/S3;
role expanded post-Phase-11 so LiDAR-based perception has something real to detect - see
docs/lidar_perception-findings.md). Reads the authoritative PedestrianArray topic published by
pedestrian_sim_node.py and teleports one collidable Gazebo model per pedestrian to match.

Position/velocity are still never authoritative here and never read directly by anything else:
GroundTruthHumanSource, LidarHumanTrackerSource, the observation builder, and the evaluation
harness all consume pedestrian_sim_node's topic (or, for LidarHumanTrackerSource, the robot's
own /scan) directly - a lagging/drifting body position here is a simulation-fidelity issue, not
a correctness bug in anything that reads state from elsewhere (see docs/phase4-findings.md's
original mirror-node-drift risk, S5 risk #6 of the plan). What DID change: this model now has
real collision geometry, so it physically exists in the simulation - the robot's LiDAR can
raycast against it, and the robot can physically contact it. Launched by default (not opt-in)
for exactly this reason: without a physical body, a LiDAR-based HumanStateSource has nothing to
perceive in sim, and the whole point of building one is so the robot never gets pedestrian
positions handed to it directly (see docs/lidar_perception-findings.md's audit).

Calls to Gazebo's spawn/set_pose services are fire-and-forget (call_async, no blocking wait on
the result): correctness doesn't depend on any single pose update landing, so there's no reason
to pay for a synchronous round-trip (or the reentrancy complexity that would come with blocking
on it from inside a subscription callback - see crowd_nav_zones' zone_manager_node.py for where
that complexity was actually necessary, because zone changes ARE correctness-critical)."""
import math

import rclpy
from crowd_nav_pedestrians.msg import PedestrianArray
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity

MARKER_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <!-- static=true: no inertial block, and correctness would break either way - a dynamic
    body would fall under gravity between SetEntityPose corrections (falls under gravity,
    looks like blinking, eventually drops out of view - found live, see git history), and
    would fight pedestrian_sim_node's own social-force integration for authority over the
    position. static=true exempts it from Gazebo's own physics entirely, so it sits exactly
    where the last pose update left it - but static bodies still participate in raycasting
    (the robot's LiDAR sees them) and in contact response against the dynamic robot body,
    which is exactly the two properties this needs. -->
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>{radius}</radius><length>1.6</length></cylinder></geometry>
        <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>{radius}</radius><length>1.6</length></cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>"""


class ActorMirrorNode(Node):
    def __init__(self):
        super().__init__('actor_mirror_node')
        self.declare_parameter('num_pedestrians', 6)
        self.declare_parameter('marker_z', 0.8)
        # Matches pedestrian_sim_node's own 'ped_radius' default (0.25 m) - the same collision
        # radius that node's social-force model already uses for every pedestrian-robot and
        # pedestrian-pedestrian repulsion term, so the physical body a real sensor would detect
        # has the same footprint the simulation's own dynamics already assume, not a second,
        # independently-chosen number that could drift out of sync with it.
        self.declare_parameter('body_radius', 0.25)
        # pedestrian_sim_node publishes at 20 Hz (dt=0.05); sending a SetEntityPose call per
        # pedestrian on every single message floods Gazebo's set_pose service with far more
        # fire-and-forget requests than it (or the render thread it shares work with) can drain
        # while also running physics/rendering/the full Nav2 stack - observed live as stutter/
        # blinking and, once the backlog grows enough, bodies appearing to freeze or vanish.
        # Only affects sim-side rendering/pose fidelity, not GroundTruthHumanSource or
        # LidarHumanTrackerSource (neither reads this node's own state), but cheap to fix: skip
        # most messages and only actually move the bodies every Nth one. Default 4 => ~5 Hz
        # updates, still fast enough for both visual smoothness and LiDAR tracking to work.
        self.declare_parameter('mirror_update_divisor', 4)
        num_pedestrians = self.get_parameter('num_pedestrians').value
        self.marker_z = self.get_parameter('marker_z').value
        self.body_radius = self.get_parameter('body_radius').value
        self.update_divisor = max(1, self.get_parameter('mirror_update_divisor').value)

        self.spawn_client = self.create_client(SpawnEntity, '/world/crowd_nav_depot_scaled/create')
        self.set_pose_client = self.create_client(
            SetEntityPose, '/world/crowd_nav_depot_scaled/set_pose')

        self.spawned = False
        self.num_pedestrians = num_pedestrians
        self._tick_count = 0
        self.create_subscription(PedestrianArray, 'pedestrians', self._on_pedestrians, 10)

    def _spawn_markers(self):
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'spawn service not available, pedestrian bodies node has nothing to move')
            return
        for i in range(self.num_pedestrians):
            req = SpawnEntity.Request()
            req.entity_factory.name = f'pedestrian_marker_{i}'
            req.entity_factory.sdf = MARKER_SDF_TEMPLATE.format(
                name=f'pedestrian_marker_{i}', radius=self.body_radius)
            req.entity_factory.pose = Pose()
            req.entity_factory.pose.position.z = self.marker_z
            self.spawn_client.call_async(req)
        self.spawned = True
        self.get_logger().info(
            f'Requested spawn of {self.num_pedestrians} collidable pedestrian bodies '
            f'(radius={self.body_radius}m)')

    def _on_pedestrians(self, msg):
        if not self.spawned:
            self._spawn_markers()
        self._tick_count += 1
        if self._tick_count % self.update_divisor != 0:
            return
        for ped in msg.pedestrians:
            req = SetEntityPose.Request()
            req.entity.name = f'pedestrian_marker_{ped.id}'
            req.entity.type = 2  # ros_gz_interfaces/msg/Entity MODEL
            req.pose.position.x = ped.x
            req.pose.position.y = ped.y
            req.pose.position.z = self.marker_z
            yaw = math.atan2(ped.vy, ped.vx) if (ped.vx or ped.vy) else 0.0
            req.pose.orientation.z = math.sin(yaw / 2.0)
            req.pose.orientation.w = math.cos(yaw / 2.0)
            self.set_pose_client.call_async(req)


def main():
    rclpy.init()
    node = ActorMirrorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
