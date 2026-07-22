from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    nav2_params = PathJoinSubstitution([
        FindPackageShare('crowd_nav_bringup'), 'config', 'nav2_params.yaml',
    ])
    slam_params = PathJoinSubstitution([
        FindPackageShare('crowd_nav_bringup'), 'config', 'slam_toolbox.yaml',
    ])
    bt_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'), 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    ])

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        parameters=[slam_params],
    )

    # nav2_controller subscribes to a hardcoded 'odom' topic name internally (OdomSmoother) -
    # remapped to our actual controller's odom topic.
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
    # controller_server publishes unsmoothed velocity on the default 'cmd_vel' topic;
    # velocity_smoother subscribes to that same default name, and its smoothed OUTPUT is what
    # actually needs to reach the robot's real command topic.
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel_smoothed', '/diff_drive_base_controller/cmd_vel_unstamped')],
    )
    local_costmap_container_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', output='screen',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server', 'planner_server', 'behavior_server',
                'bt_navigator', 'velocity_smoother',
            ],
        }],
    )

    return LaunchDescription([
        spawn,
        slam_toolbox,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        local_costmap_container_lifecycle,
    ])
