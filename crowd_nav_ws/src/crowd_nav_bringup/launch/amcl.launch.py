from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('crowd_nav_bringup'), 'maps', 'depot_scaled.yaml',
        ]),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('crowd_nav_gazebo'), 'launch', 'spawn_robot.launch.py',
            ])
        ),
        launch_arguments=[
            ('world_file', 'depot_scaled.sdf'),
            ('spawn_x', '-3.0'),
            ('spawn_y', '0.0'),
            ('spawn_yaw', '0.0'),
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
    controller_server = Node(
        package='nav2_controller', executable='controller_server', output='screen',
        parameters=[nav2_params],
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
