"""Phase 4 pedestrian simulation bringup. Always launches the
deterministic pedestrian_sim_node.py plus a bridge for the robot's ground-truth pose (needed by
'reactive' mode regardless of mirroring).

mirror_enabled now defaults to true (was false through Phase 11) - see
docs/lidar_perception-findings.md. It no longer just spawns a cosmetic visual marker; it spawns
a real collidable Gazebo body per pedestrian, which is what makes LidarHumanTrackerSource (the
default HumanStateSource as of this change, crowd_nav_controller's human_source_type) able to
perceive anything at all in simulation - without a physical body, the robot's own /scan has
nothing to detect and the whole point of not cheating via ground truth is moot. Still
launch-toggleable to false for anyone who specifically wants the old cosmetic-free/ground-truth
comparison configuration (e.g. re-running a Phase 10-era measurement for direct comparison)."""
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
    mirror_update_divisor = LaunchConfiguration('mirror_update_divisor')
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

    # Service bridges for spawning/moving pedestrian bodies, and the node that owns them - both
    # conditional on mirror_enabled, which now defaults to true (see module docstring).
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
            'mirror_update_divisor': mirror_update_divisor,
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
        DeclareLaunchArgument('mirror_enabled', default_value='true'),
        # Marker pose-update rate divisor (actor_mirror_node.py) - the sim publishes pedestrian
        # positions at 20 Hz; sending a SetEntityPose call per pedestrian on every single message
        # floods Gazebo's set_pose service well past what it can drain alongside physics/
        # rendering/the full Nav2 stack, observed live as marker blinking/freezing. Default 4 =>
        # ~5 Hz marker updates, still visually smooth, a quarter of the request rate.
        DeclareLaunchArgument('mirror_update_divisor', default_value='4'),
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
