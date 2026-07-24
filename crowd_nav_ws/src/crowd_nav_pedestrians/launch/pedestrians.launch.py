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
    world_name = LaunchConfiguration('world_name')

    # PosePublisher's publish_model_pose (crowd_nav_description/urdf/nvis_3302ard.xacro) never
    # actually publishes on this gz-sim version (6.18.0) - confirmed with `ign topic -e`, zero
    # messages even while the robot was actively driving (found building Phase 10, see
    # robot_pose_extractor.py's docstring for the full investigation). scene_broadcaster (present
    # in every world file already) is the working alternative: bridge its Pose_V stream to
    # TFMessage, then filter to just the robot entity in a dedicated node so every existing
    # /ground_truth/robot_pose consumer needs zero changes.
    robot_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ground_truth_pose_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            [
                '/world/', world_name, '/dynamic_pose/info',
                '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
        ],
    )

    robot_pose_extractor = Node(
        package='crowd_nav_pedestrians',
        executable='robot_pose_extractor.py',
        name='robot_pose_extractor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_model_name': 'nvis_3302ard',
            'input_topic': ['/world/', world_name, '/dynamic_pose/info'],
        }],
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
            ['/world/', world_name, '/create@ros_gz_interfaces/srv/SpawnEntity'],
            ['/world/', world_name, '/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
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
        # Gazebo world name (the SDF <world name="..."> value), NOT the world_file .sdf filename
        # used by spawn_robot.launch.py - the two aren't the same string and there's no existing
        # mapping between them, so callers (run_episode.py) pass both explicitly.
        DeclareLaunchArgument('world_name', default_value='crowd_nav_depot_scaled'),
        robot_pose_bridge,
        robot_pose_extractor,
        pedestrian_sim,
        mirror_service_bridge,
        actor_mirror,
    ])
