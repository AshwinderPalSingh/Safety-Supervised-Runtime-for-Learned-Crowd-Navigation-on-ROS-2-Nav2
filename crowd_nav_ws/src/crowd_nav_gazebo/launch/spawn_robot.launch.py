import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            PathJoinSubstitution([FindPackageShare('crowd_nav_gazebo'), 'worlds', 'empty.sdf']),
        ],
        additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ld_library_path},
        output='screen',
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'nvis_3302ard', '-allow_renaming', 'true'],
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
        gz_sim,
        robot_state_publisher,
        clock_bridge,
        scan_bridge,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster_spawner]),
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[diff_drive_spawner]),
        ),
    ])
