from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('crowd_nav_bringup'), 'maps', 'depot_scaled.yaml',
        ]),
    )
    # World/spawn and controller-config launch args (IMPLEMENTATION_PLAN.md S4.9) - defaults
    # match this launch file's own pre-Phase-10 hardcoded values exactly, so every existing
    # manual invocation is unaffected; the evaluation harness overrides them per scenario/config.
    world_file_arg = DeclareLaunchArgument('world_file', default_value='depot_scaled.sdf')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-3.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')
    controller_plugin_arg = DeclareLaunchArgument(
        'controller_plugin', default_value='crowd_nav_controller::CrowdNavController')
    adapter_type_arg = DeclareLaunchArgument('adapter_type', default_value='sarl')
    supervisor_enabled_arg = DeclareLaunchArgument('supervisor_enabled', default_value='true')
    # Phase 10 noise sweep (S4.9.2) - off by default, matching DegradationParams' own
    # convention; found while wiring the sweep that CrowdNavController never exposed these at
    # all before now (docs/phase9-findings.md addendum's own reachability-audit lesson, applied
    # here before running anything, not after).
    perception_dropout_prob_arg = DeclareLaunchArgument(
        'perception_dropout_prob', default_value='0.0')
    perception_degradation_seed_arg = DeclareLaunchArgument(
        'perception_degradation_seed', default_value='0')
    # OOD-reachability audit (docs/audit.md S1.1): exposes CrowdNavController's existing
    # debug_inject_decision_delay_s test hook (already implemented, already a real ROS param -
    # only never previously reachable from this launch file) so the harness can demonstrate
    # INFERENCE_TIMEOUT firing for real rather than leave it a permanently-untested mechanism.
    debug_inject_decision_delay_s_arg = DeclareLaunchArgument(
        'debug_inject_decision_delay_s', default_value='0.0')

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('crowd_nav_gazebo'), 'launch', 'spawn_robot.launch.py',
            ])
        ),
        launch_arguments=[
            ('world_file', LaunchConfiguration('world_file')),
            ('spawn_x', LaunchConfiguration('spawn_x')),
            ('spawn_y', LaunchConfiguration('spawn_y')),
            ('spawn_yaw', LaunchConfiguration('spawn_yaw')),
        ],
    )

    zones = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('crowd_nav_zones'), 'launch', 'zones.launch.py',
            ])
        ),
    )

    nav2_params = PathJoinSubstitution([
        FindPackageShare('crowd_nav_bringup'), 'config', 'nav2_params.yaml',
    ])
    bt_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'), 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    ])

    map_server = Node(
        package='nav2_map_server', executable='map_server', output='screen',
        parameters=[nav2_params, {'yaml_filename': LaunchConfiguration('map')}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', output='screen',
        parameters=[nav2_params],
    )
    # FollowPath overrides (IMPLEMENTATION_PLAN.md S4.9) - a plain dict entry AFTER the base
    # nav2_params.yaml in the parameters list overrides just these three keys; every other
    # FollowPath/costmap/AMCL parameter still comes from the one shared YAML file, no
    # per-config file duplication. supervisor_enabled needs an explicit ParameterValue(...,
    # value_type=bool) - a LaunchConfiguration substitution resolves to a plain string, and
    # without this it would be passed as a string parameter instead of the bool
    # supervisor_enabled_ (CrowdNavController) actually declares and reads with .as_bool().
    controller_overrides = {
        'FollowPath.plugin': LaunchConfiguration('controller_plugin'),
        'FollowPath.adapter_type': LaunchConfiguration('adapter_type'),
        'FollowPath.supervisor_enabled': ParameterValue(
            LaunchConfiguration('supervisor_enabled'), value_type=bool),
        'FollowPath.perception_dropout_prob': ParameterValue(
            LaunchConfiguration('perception_dropout_prob'), value_type=float),
        'FollowPath.perception_degradation_seed': ParameterValue(
            LaunchConfiguration('perception_degradation_seed'), value_type=int),
        'FollowPath.debug_inject_decision_delay_s': ParameterValue(
            LaunchConfiguration('debug_inject_decision_delay_s'), value_type=float),
    }
    controller_server = Node(
        package='nav2_controller', executable='controller_server', output='screen',
        parameters=[nav2_params, controller_overrides],
        remappings=[('odom', '/diff_drive_base_controller/odom')],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', output='screen',
        parameters=[nav2_params],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', output='screen',
        parameters=[nav2_params],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', output='screen',
        parameters=[nav2_params, {
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': bt_xml,
        }],
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel_smoothed', '/diff_drive_base_controller/cmd_vel_unstamped')],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', output='screen',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server', 'amcl', 'controller_server', 'planner_server',
                'behavior_server', 'bt_navigator', 'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        map_arg,
        world_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        controller_plugin_arg,
        adapter_type_arg,
        supervisor_enabled_arg,
        perception_dropout_prob_arg,
        perception_degradation_seed_arg,
        debug_inject_decision_delay_s_arg,
        spawn,
        zones,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
    ])
