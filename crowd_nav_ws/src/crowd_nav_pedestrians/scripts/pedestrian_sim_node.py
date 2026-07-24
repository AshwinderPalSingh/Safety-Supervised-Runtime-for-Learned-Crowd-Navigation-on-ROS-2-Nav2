#!/usr/bin/env python3
"""Phase 4 pedestrian simulator (IMPLEMENTATION_PLAN.md S1.2/S3, rescoped in v1.7 - HuNav
dropped). A single deterministic social-force model, not two separate mechanisms:
reactive/non_reactive is a config flag (params.mode) that only toggles whether the robot's
ground-truth position contributes a repulsion force, not a different code path.

Determinism (the entire justification for dropping HuNav - Phase 10's evaluation matrix needs
byte-identical trajectories across same-seed runs, not "similar"):
  - All randomness (initial positions/goals, new-goal selection when a goal is reached) draws
    from a single `random.Random(seed)` instance owned by this node - never the global `random`
    module or numpy's global RNG, both of which could pick up state from unrelated code.
  - The physics step itself is pure, order-fixed floating point (fixed iteration order over a
    list, not a set/dict whose iteration order could vary) - no additional randomness.
  - The simulation steps in FIXED dt increments (params.dt), not "wall-clock time since the
    last callback" - see _on_clock below. Given the same sequence of sim-time values from
    /clock, this produces the exact same sequence of fixed-dt steps regardless of real-world
    scheduling jitter between runs.

Sim-time stepping (also load-bearing for determinism, not just "nice to have"): this node
tracks simulation time from the /clock topic, not wall-clock timers. With this project's
measured RTF sometimes below 1.0 under heavier physics configurations (docs/phase2-findings.md),
a wall-clock-driven pedestrian simulator would silently desynchronize from Gazebo, and seeded
reproducibility would be meaningless regardless of how careful the RNG seeding is.

Ground-truth robot pose (not /odom): subscribes to a bridged Gazebo PosePublisher topic
(geometry_msgs/msg/Pose, see nvis_3302ard.xacro's PosePublisher plugin and
pedestrians.launch.py's ros_gz_bridge invocation) - odometry drift has no business leaking
into how simulated pedestrians react to the robot's actual position.
"""
import math
import random

import rclpy
from crowd_nav_pedestrians.msg import Pedestrian, PedestrianArray
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class SimPedestrian:
    __slots__ = ('id', 'x', 'y', 'vx', 'vy', 'goal_x', 'goal_y')

    def __init__(self, id_, x, y, goal_x, goal_y):
        self.id = id_
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.goal_x = goal_x
        self.goal_y = goal_y


class PedestrianSimNode(Node):
    def __init__(self):
        super().__init__('pedestrian_sim_node')

        self.declare_parameter('seed', 42)
        self.declare_parameter('num_pedestrians', 6)
        self.declare_parameter('mode', 'reactive')  # 'reactive' or 'non_reactive'
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('min_x', -3.5)
        self.declare_parameter('max_x', 3.4)
        self.declare_parameter('min_y', -1.7)
        self.declare_parameter('max_y', 1.5)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('goal_radius', 0.3)
        self.declare_parameter('ped_radius', 0.25)
        self.declare_parameter('robot_radius', 0.14)
        self.declare_parameter('relaxation_time', 0.5)
        self.declare_parameter('ped_repulsion_strength', 2.0)
        self.declare_parameter('ped_repulsion_range', 0.3)
        self.declare_parameter('robot_repulsion_strength', 3.0)
        self.declare_parameter('robot_repulsion_range', 0.4)
        self.declare_parameter('robot_pose_topic', '/ground_truth/robot_pose')

        self.mode = self.get_parameter('mode').value
        self.dt = self.get_parameter('dt').value
        self.min_x = self.get_parameter('min_x').value
        self.max_x = self.get_parameter('max_x').value
        self.min_y = self.get_parameter('min_y').value
        self.max_y = self.get_parameter('max_y').value
        self.max_speed = self.get_parameter('max_speed').value
        self.goal_radius = self.get_parameter('goal_radius').value
        self.ped_radius = self.get_parameter('ped_radius').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.tau = self.get_parameter('relaxation_time').value
        self.ped_a = self.get_parameter('ped_repulsion_strength').value
        self.ped_b = self.get_parameter('ped_repulsion_range').value
        self.robot_a = self.get_parameter('robot_repulsion_strength').value
        self.robot_b = self.get_parameter('robot_repulsion_range').value

        seed = self.get_parameter('seed').value
        num_pedestrians = self.get_parameter('num_pedestrians').value
        self.rng = random.Random(seed)
        self.pedestrians = [
            self._spawn_pedestrian(i) for i in range(num_pedestrians)
        ]

        self.sim_time = None
        self.next_step_time = 0.0
        self.robot_pose = None

        self.pub = self.create_publisher(PedestrianArray, 'pedestrians', 10)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(
            Pose, self.get_parameter('robot_pose_topic').value, self._on_robot_pose, 10)

        self.get_logger().info(
            f'pedestrian_sim_node: seed={seed} num_pedestrians={num_pedestrians} '
            f'mode={self.mode} dt={self.dt}')

    def _spawn_pedestrian(self, id_):
        x = self.rng.uniform(self.get_parameter('min_x').value, self.get_parameter('max_x').value)
        y = self.rng.uniform(self.get_parameter('min_y').value, self.get_parameter('max_y').value)
        goal_x, goal_y = self._pick_goal()
        return SimPedestrian(id_, x, y, goal_x, goal_y)

    def _pick_goal(self):
        return (self.rng.uniform(self.min_x, self.max_x), self.rng.uniform(self.min_y, self.max_y))

    def _on_robot_pose(self, msg):
        self.robot_pose = (msg.position.x, msg.position.y)

    def _on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.sim_time = t
        # Fixed-dt catch-up loop: however many /clock messages arrive, and whatever real-world
        # jitter separates them, the pedestrians advance in exact, fixed dt increments keyed
        # only to sim time - see the module docstring's determinism note.
        while self.next_step_time <= self.sim_time:
            self._step()
            self.next_step_time += self.dt

    def _step(self):
        forces = [self._total_force(i) for i in range(len(self.pedestrians))]
        for ped, (fx, fy) in zip(self.pedestrians, forces):
            ped.vx += fx * self.dt
            ped.vy += fy * self.dt
            speed = math.hypot(ped.vx, ped.vy)
            if speed > self.max_speed:
                scale = self.max_speed / speed
                ped.vx *= scale
                ped.vy *= scale
            ped.x += ped.vx * self.dt
            ped.y += ped.vy * self.dt

            if ped.x < self.min_x:
                ped.x = self.min_x
                ped.vx = 0.0
            elif ped.x > self.max_x:
                ped.x = self.max_x
                ped.vx = 0.0
            if ped.y < self.min_y:
                ped.y = self.min_y
                ped.vy = 0.0
            elif ped.y > self.max_y:
                ped.y = self.max_y
                ped.vy = 0.0

            if math.hypot(ped.x - ped.goal_x, ped.y - ped.goal_y) < self.goal_radius:
                ped.goal_x, ped.goal_y = self._pick_goal()

        self._publish()

    def _total_force(self, i):
        ped = self.pedestrians[i]
        fx, fy = self._goal_force(ped)

        for j, other in enumerate(self.pedestrians):
            if j == i:
                continue
            rx, ry = self._repulsion(
                ped.x, ped.y, other.x, other.y,
                2.0 * self.ped_radius, self.ped_a, self.ped_b)
            fx += rx
            fy += ry

        if self.mode == 'reactive' and self.robot_pose is not None:
            rx, ry = self._repulsion(
                ped.x, ped.y, self.robot_pose[0], self.robot_pose[1],
                self.ped_radius + self.robot_radius, self.robot_a, self.robot_b)
            fx += rx
            fy += ry

        return (fx, fy)

    def _goal_force(self, ped):
        dx, dy = ped.goal_x - ped.x, ped.goal_y - ped.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return (0.0, 0.0)
        desired_vx = self.max_speed * dx / dist
        desired_vy = self.max_speed * dy / dist
        return ((desired_vx - ped.vx) / self.tau, (desired_vy - ped.vy) / self.tau)

    @staticmethod
    def _repulsion(x, y, ox, oy, combined_radius, a, b):
        dx, dy = x - ox, y - oy
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            dist = 1e-6
        magnitude = a * math.exp((combined_radius - dist) / b)
        return (magnitude * dx / dist, magnitude * dy / dist)

    def _publish(self):
        msg = PedestrianArray()
        msg.header.stamp.sec = int(self.sim_time)
        msg.header.stamp.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
        msg.header.frame_id = 'map'
        for ped in self.pedestrians:
            p = Pedestrian()
            p.id = ped.id
            p.x = ped.x
            p.y = ped.y
            p.vx = ped.vx
            p.vy = ped.vy
            msg.pedestrians.append(p)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PedestrianSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
