"""Phase 4 pedestrian simulation bringup (IMPLEMENTATION_PLAN.md S1.2/S3). Always launches the
deterministic pedestrian_sim_node.py plus a bridge for the robot's ground-truth pose (needed by
'reactive' mode regardless of mirroring). The visual-only Gazebo actor mirror is
launch-toggleable via mirror_enabled, off by default - see docs/phase4-findings.md and S5 risk
#6 of the plan for why a disabled-by-default mirror is the right default (it's cosmetic, not
correctness-critical, and every RTF/headless-correctness measurement in this phase's done-bar
needs to be taken with it off)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    seed = LaunchConfiguration('seed')
    num_pedestrians = LaunchConfiguration('num_pedestrians')
    max_speed = LaunchConfiguration('max_speed')
    mirror_enabled = LaunchConfiguration('mirror_enabled')
    world_name = 'crowd_nav_depot_scaled'

    robot_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_pose_bridge',
        output='screen',
        arguments=[
            '/model/nvis_3302ard/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose',
        ],
        remappings=[('/model/nvis_3302ard/pose', '/ground_truth/robot_pose')],
    )

    pedestrian_sim = Node(
        package='crowd_nav_pedestrians',
        executable='pedestrian_sim_node.py',
        name='pedestrian_sim_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': mode,
            'seed': seed,
            'num_pedestrians': num_pedestrians,
            'max_speed': max_speed,
            'robot_pose_topic': '/ground_truth/robot_pose',
        }],
    )

    # Mirror-only: service bridges for spawning/moving visual markers, and the mirror node
    # itself. Both conditional on mirror_enabled - never active by default.
    mirror_service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='mirror_service_bridge',
        output='screen',
        arguments=[
            f'/world/{world_name}/create@ros_gz_interfaces/srv/SpawnEntity',
            f'/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        condition=IfCondition(mirror_enabled),
    )

    actor_mirror = Node(
        package='crowd_nav_pedestrians',
        executable='actor_mirror_node.py',
        name='actor_mirror_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'num_pedestrians': num_pedestrians,
        }],
        condition=IfCondition(mirror_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='reactive'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('num_pedestrians', default_value='6'),
        # Exposed (Phase 9 reachability audit, docs/phase9-findings.md follow-up): previously
        # only settable via a raw ros2 param override, not through this launch file's own
        # argument surface like seed/num_pedestrians already are - needed to build a Phase 10
        # scenario deliberately above SafetySupervisorConfig::max_train_speed_mps (1.5 m/s
        # default), since the node's own 1.0 m/s default can never exceed that threshold.
        DeclareLaunchArgument('max_speed', default_value='1.0'),
        DeclareLaunchArgument('mirror_enabled', default_value='false'),
        robot_pose_bridge,
        pedestrian_sim,
        mirror_service_bridge,
        actor_mirror,
    ])
