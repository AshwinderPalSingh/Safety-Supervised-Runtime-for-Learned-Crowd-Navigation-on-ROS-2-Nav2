#!/usr/bin/env python3
"""In-episode monitor: an rclpy node that sends the NavigateToPose
goal, watches ground truth for the harness's own outcome determination (never the robot's own
perception - matching this project's established "verify against ground truth" discipline,
docs/phase2-findings.md), and returns one episode's metrics. One process, one episode, then
exits - run_episode.py is the outer orchestrator that launches/tears down the ROS2 stack around
this.
"""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from crowd_nav_pedestrians.msg import PedestrianArray
from crowd_nav_safety_supervisor.msg import InterventionEvent
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan

from scenarios import COLLISION_DISTANCE_M, MAX_EPISODE_DURATION_S


class EpisodeMonitor(Node):
    def __init__(self, goal_xy, zone):
        # use_sim_time is auto-declared by Node.__init__ itself (every rclpy node gets it) -
        # an explicit declare_parameter() call for it throws ParameterAlreadyDeclaredException
        # (found running the pilot, not assumed). parameter_overrides is the correct way to set
        # it at construction instead - every other node in this project gets it from a launch
        # file's 'use_sim_time': True entry; this script has no launch file, so it sets the
        # same override directly.
        super().__init__(
            'episode_monitor',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.goal_xy = goal_xy
        self.zone = zone  # dict with center_x/center_y/size_x/size_y, or None

        self.outcome = None  # set once, first terminal condition wins
        self.start_time = None
        self.latest_robot_xy = None
        self.prev_robot_xy = None
        self.path_length_m = 0.0
        self.min_human_distance_m = math.inf
        self.intervention_count_total = 0
        self.intervention_count_by_cause = {}
        self.intervention_rows = []  # for interventions.csv
        self.latest_amcl_cov_trace = None
        self._goal_handle = None
        self._nav2_result_code = None

        # Message counts for the topics this project's own stack depends on, not just outcome/
        # metrics - added for the Phase 11 nightly smoke test's own explicit requirement:
        # assert these topics are actually live, not just that the
        # launch succeeds and a goal is reached. Reuses this class's existing subscriptions
        # rather than a parallel topic-checking mechanism - /ground_truth/robot_pose is the
        # exact topic that silently stopped publishing for three phases (docs/phase10-findings.md,
        # the PosePublisher bug) while every other signal, including a reached goal, looked fine.
        self.topic_message_counts = {
            'ground_truth_robot_pose': 0, 'pedestrians': 0, 'amcl_pose': 0, 'scan': 0,
        }

        self.create_subscription(Pose, '/ground_truth/robot_pose', self._on_robot_pose, 10)
        self.create_subscription(PedestrianArray, '/pedestrians', self._on_pedestrians, 10)
        self.create_subscription(
            InterventionEvent, '/intervention_events', self._on_intervention, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._timer = self.create_timer(0.2, self._tick)

    def _on_robot_pose(self, msg):
        self.topic_message_counts['ground_truth_robot_pose'] += 1
        xy = (msg.position.x, msg.position.y)
        if self.latest_robot_xy is not None:
            dx = xy[0] - self.latest_robot_xy[0]
            dy = xy[1] - self.latest_robot_xy[1]
            self.path_length_m += math.hypot(dx, dy)
        self.latest_robot_xy = xy

    def _on_scan(self, msg):
        self.topic_message_counts['scan'] += 1

    def _on_pedestrians(self, msg):
        self.topic_message_counts['pedestrians'] += 1
        if self.latest_robot_xy is None:
            return
        rx, ry = self.latest_robot_xy
        for p in msg.pedestrians:
            d = math.hypot(p.x - rx, p.y - ry)
            if d < self.min_human_distance_m:
                self.min_human_distance_m = d
            if d < COLLISION_DISTANCE_M and self.outcome is None:
                self.outcome = 'collision'

    def _on_intervention(self, msg):
        self.intervention_count_total += 1
        self.intervention_count_by_cause[msg.cause] = (
            self.intervention_count_by_cause.get(msg.cause, 0) + 1)
        self.intervention_rows.append({
            # wall-clock capture order, sim-time cause is msg.stamp
            'timestamp_s': time.monotonic(),
            'sim_stamp_sec': msg.stamp.sec,
            'sim_stamp_nanosec': msg.stamp.nanosec,
            'cause': msg.cause,
            'rejected_vx': msg.rejected_vx,
            'rejected_vy': msg.rejected_vy,
            'sent_vx': msg.sent_vx,
            'sent_vy': msg.sent_vy,
            'amcl_cov_trace': self.latest_amcl_cov_trace,
        })

    def _on_amcl_pose(self, msg):
        self.topic_message_counts['amcl_pose'] += 1
        cov = msg.pose.covariance
        self.latest_amcl_cov_trace = cov[0] + cov[7]

        # Keepout zones are authored and enforced entirely in the 'map' frame
        # (zone_manager_node.py's own zone-storage comment, matching the KeepoutFilter costmap
        # layer and the safety supervisor's forward-sim, both of which consume the same
        # map-frame robot pose Nav2 supplies to the controller). /ground_truth/robot_pose (used
        # above, in _on_robot_pose) is Gazebo WORLD frame - comparing it against this map-frame
        # zone spec was a genuine bug (found investigating why depot_keepout_block showed zero
        # interventions even in the one episode the harness itself called a violation): the two
        # numbers only looked comparable by the coincidence of this scenario's world-to-map
        # offset, not because either config actually entered the real, costmap-enforced zone.
        # AMCL's pose is already map-frame, matching every real consumer of zone geometry, so
        # it's the correct source for this check.
        if self.zone is not None and self.outcome is None:
            xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            cx, cy = self.zone['center_x'], self.zone['center_y']
            hx, hy = self.zone['size_x'] / 2.0, self.zone['size_y'] / 2.0
            if (cx - hx) <= xy[0] <= (cx + hx) and (cy - hy) <= xy[1] <= (cy + hy):
                self.outcome = 'keepout_violation'

    def _tick(self):
        if self.outcome is not None:
            return
        if self.start_time is None:
            return
        if (self.get_clock().now().nanoseconds / 1e9) - self.start_time > MAX_EPISODE_DURATION_S:
            self.outcome = 'timeout'
            return
        if self._nav2_result_code is not None:
            if self._nav2_result_code == GoalStatus.STATUS_SUCCEEDED:
                self.outcome = 'success'
            else:
                self.outcome = 'nav2_aborted'

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.outcome = 'nav2_aborted'
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        self._nav2_result_code = future.result().status

    def send_goal(self):
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(self.goal_xy[0])
        goal.pose.pose.position.y = float(self.goal_xy[1])
        goal.pose.pose.orientation.w = 1.0
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def run(self, action_server_wait_s=30.0):
        """Blocks until the action server is up, sends the goal, spins until a terminal
        outcome is reached, returns the episode's metrics dict."""
        if not self._nav_client.wait_for_server(timeout_sec=action_server_wait_s):
            return {
                'outcome': 'nav2_aborted',
                'error': 'navigate_to_pose action server never came up',
            }
        self.send_goal()
        while rclpy.ok() and self.outcome is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._goal_handle is not None and self.outcome != 'success':
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        duration_s = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        return {
            'outcome': self.outcome,
            'duration_s': duration_s,
            'path_length_m': self.path_length_m,
            'min_human_distance_m': (
                None if self.min_human_distance_m == math.inf else self.min_human_distance_m),
            'intervention_count_total': self.intervention_count_total,
            'intervention_count_by_cause': self.intervention_count_by_cause,
            'intervention_rows': self.intervention_rows,
            'topic_message_counts': self.topic_message_counts,
        }
