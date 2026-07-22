#!/usr/bin/env python3
"""Visual-only Gazebo mirror for Phase 4's pedestrian simulation (IMPLEMENTATION_PLAN.md S1.2/
S3). Launch-toggleable, off by default. Reads the authoritative PedestrianArray topic published
by pedestrian_sim_node.py and moves simple marker models in Gazebo to match - purely for
visualization/demos. Never authoritative and never read by anything else: GroundTruthHumanSource,
the observation builder, and the evaluation harness all consume pedestrian_sim_node's topic
directly (see docs/phase4-findings.md's mirror-node-drift risk, S5 risk #6 of the plan - a
lagging/drifting mirror is a rendering bug, not a correctness bug, by construction).

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
    <static>false</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>0.25</radius><length>1.6</length></cylinder></geometry>
        <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class ActorMirrorNode(Node):
    def __init__(self):
        super().__init__('actor_mirror_node')
        self.declare_parameter('num_pedestrians', 6)
        self.declare_parameter('marker_z', 0.8)
        num_pedestrians = self.get_parameter('num_pedestrians').value
        self.marker_z = self.get_parameter('marker_z').value

        self.spawn_client = self.create_client(SpawnEntity, '/world/crowd_nav_depot_scaled/create')
        self.set_pose_client = self.create_client(
            SetEntityPose, '/world/crowd_nav_depot_scaled/set_pose')

        self.spawned = False
        self.num_pedestrians = num_pedestrians
        self.create_subscription(PedestrianArray, 'pedestrians', self._on_pedestrians, 10)

    def _spawn_markers(self):
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('spawn service not available, mirror node has nothing to move')
            return
        for i in range(self.num_pedestrians):
            req = SpawnEntity.Request()
            req.entity_factory.name = f'pedestrian_marker_{i}'
            req.entity_factory.sdf = MARKER_SDF_TEMPLATE.format(name=f'pedestrian_marker_{i}')
            req.entity_factory.pose = Pose()
            req.entity_factory.pose.position.z = self.marker_z
            self.spawn_client.call_async(req)
        self.spawned = True
        self.get_logger().info(f'Requested spawn of {self.num_pedestrians} visual markers')

    def _on_pedestrians(self, msg):
        if not self.spawned:
            self._spawn_markers()
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
