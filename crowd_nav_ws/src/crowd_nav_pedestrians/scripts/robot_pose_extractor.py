#!/usr/bin/env python3
"""Republishes the robot's ground-truth world-frame pose as a plain geometry_msgs/msg/Pose on
/ground_truth/robot_pose, extracted from the bridged tf2_msgs/msg/TFMessage stream.

Found while building Phase 10 ( addendum), not assumed: the
PosePublisher Gazebo plugin's <publish_model_pose>true</publish_model_pose> (with
publish_link_pose left false, crowd_nav_description/urdf/nvis_3302ard.xacro) never actually
published anything on this gz-sim version (6.18.0) - confirmed with `ign topic -e`, which timed
out with zero messages even while the robot was actively driving. Enabling
publish_link_pose=true DID make the topic active, but mixed the model's own pose together with
every individual link's pose (wheels, base_footprint) on the SAME topic with no way to tell
them apart once bridged down to a bare geometry_msgs/msg/Pose (the richer per-message Gazebo
"name" field that would disambiguate them is lost in that specific bridge type).

The scene_broadcaster system plugin (already present in every world file, no extra
configuration needed) publishes /world/<world>/dynamic_pose/info as an ignition.msgs.Pose_V,
bridged here to tf2_msgs/msg/TFMessage - confirmed correct via direct testing: the
"nvis_3302ard" entry's translation reflects real world-frame position (matching the actual
spawn offset), each entity individually addressable by child_frame_id, unlike the
PosePublisher's link-relative, undisambiguated stream. This node is the one place that knows
about that filtering, so every existing consumer (pedestrian_sim_node.py's own subscription,
GroundTruthHumanSource, the Phase 10 harness's EpisodeMonitor) keeps subscribing to the exact
same /ground_truth/robot_pose topic/type it always has - zero downstream changes required.
"""
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class RobotPoseExtractor(Node):
    def __init__(self):
        super().__init__('robot_pose_extractor')
        self.declare_parameter('robot_model_name', 'nvis_3302ard')
        self.declare_parameter('input_topic', '/world/crowd_nav_depot_scaled/dynamic_pose/info')
        self.robot_model_name = self.get_parameter('robot_model_name').value
        input_topic = self.get_parameter('input_topic').value

        self.pub = self.create_publisher(Pose, '/ground_truth/robot_pose', 10)
        self.create_subscription(TFMessage, input_topic, self._on_tf, 10)

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == self.robot_model_name:
                out = Pose()
                out.position.x = t.transform.translation.x
                out.position.y = t.transform.translation.y
                out.position.z = t.transform.translation.z
                out.orientation = t.transform.rotation
                self.pub.publish(out)
                return


def main():
    rclpy.init()
    node = RobotPoseExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
