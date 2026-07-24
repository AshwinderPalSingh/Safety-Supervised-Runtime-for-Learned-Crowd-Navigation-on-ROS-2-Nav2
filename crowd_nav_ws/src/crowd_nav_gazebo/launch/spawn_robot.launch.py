import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _check_dds_health():
    # Fails the launch immediately, before spawning anything, rather than let a dirty DDS
    # state silently degrade into the Nav2 lifecycle-activation hangs found in Phase 2
    # (docs/phase2-findings.md). This is the launch-time half of that phase's standing ask;
    # scripts/ros2_teardown.sh is the corresponding cleanup half.
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
    check_script = os.path.join(repo_root, 'scripts', 'check_dds_health.sh')
    if not os.path.isfile(check_script):
        # Workspace layout differs from a source checkout (e.g. a bare install-space-only
        # deployment) - don't block launches over a missing convenience script.
        return
    result = subprocess.run(['bash', check_script], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"DDS health check failed, refusing to launch:\n{result.stdout}{result.stderr}\n"
            "Run scripts/ros2_teardown.sh (only if no ROS/Gazebo processes are currently "
            "running), then retry."
        )


def generate_launch_description():
    _check_dds_health()

    world_file_arg = DeclareLaunchArgument('world_file', default_value='empty.sdf')
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    # Quoted explicitly: this workspace's path contains spaces (the project directory name),
    # and Command() shlex-splits its resolved string, which would otherwise fragment the path
    # into multiple bogus arguments.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        "'", PathJoinSubstitution([
            FindPackageShare('crowd_nav_description'), 'urdf', 'nvis_3302ard.xacro',
        ]), "'",
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Invoked directly via ExecuteProcess rather than including ros_gz_sim's gz_sim.launch.py:
    # that launch file internally re-splits its gz_args string, which fragments this
    # workspace's space-containing path into multiple bogus arguments (confirmed: "ign gazebo"
    # loads this exact world fine when invoked directly with the same path). ExecuteProcess's
    # cmd list elements map 1:1 to argv with no further splitting, sidestepping the issue.
    # gz_sim.launch.py normally forwards LD_LIBRARY_PATH into IGN_GAZEBO_SYSTEM_PLUGIN_PATH
    # so gz-sim's system-plugin loader (a separate search path from the dynamic linker) can
    # find things like gz_ros2_control-system.so. Bypassing that launch file (see above)
    # means doing this ourselves.
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    gz_sim = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r', '-s', '-v', '1',
            PathJoinSubstitution([
                FindPackageShare('crowd_nav_gazebo'), 'worlds', LaunchConfiguration('world_file'),
            ]),
        ],
        additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ld_library_path},
        output='screen',
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', '-name', 'nvis_3302ard', '-allow_renaming', 'true',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            PathJoinSubstitution([
                FindPackageShare('crowd_nav_control'), 'config', 'diff_drive_controller.yaml',
            ]),
        ],
    )

    return LaunchDescription([
        world_file_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        gz_sim,
        robot_state_publisher,
        clock_bridge,
        scan_bridge,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster_spawner]),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner, on_exit=[diff_drive_spawner]),
        ),
    ])
