"""Brings up Phase 3's dynamic keep-out zone infrastructure: the zone_manager node (owns
AddZone/RemoveZone and the mask file), a dedicated map_server instance serving that mask on
its own topic, and costmap_filter_info_server describing how to interpret it. The KeepoutFilter
plugin itself is wired into the local costmap in nav2_params.yaml, not here - this launch file
only stands up the mask pipeline feeding it.

zone_manager writes its initial (all-clear) mask file synchronously at construction time, but
still needs a moment to actually start before mask_server's on_configure reads it - the
TimerAction delay below is that ordering safeguard, not a workaround for anything flaky."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    mask_dir = LaunchConfiguration('mask_dir')
    resolution = LaunchConfiguration('resolution')
    origin_x = LaunchConfiguration('origin_x')
    origin_y = LaunchConfiguration('origin_y')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')

    zone_manager_node = Node(
        package='crowd_nav_zones',
        executable='zone_manager_node.py',
        name='zone_manager',
        output='screen',
        parameters=[{
            'mask_dir': mask_dir,
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'width': width,
            'height': height,
        }],
    )

    mask_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='mask_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': [mask_dir, '/mask.yaml'],
            # Absolute (leading /), not relative: KeepoutFilter runs inside the local_costmap
            # node's own namespace, so a relative mask_topic here would make it subscribe to
            # /local_costmap/keepout_filter_mask instead of where this node actually publishes -
            # confirmed via ros2 topic info showing 0 publishers on the relative-resolved name.
            'topic_name': '/keepout_filter_mask',
            'frame_id': 'map',
        }],
    )

    costmap_filter_info_node = LifecycleNode(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        namespace='',
        output='screen',
        parameters=[{
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_filter_mask',
            'type': 0,
            'base': 0.0,
            'multiplier': 1.0,
        }],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_zones',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['mask_server', 'costmap_filter_info_server'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('mask_dir', default_value='/tmp/crowd_nav_zone_masks'),
        DeclareLaunchArgument('resolution', default_value='0.03'),
        DeclareLaunchArgument('origin_x', default_value='-1.1'),
        DeclareLaunchArgument('origin_y', default_value='-2.19'),
        DeclareLaunchArgument('width', default_value='276'),
        DeclareLaunchArgument('height', default_value='143'),
        zone_manager_node,
        TimerAction(
            period=2.0,
            actions=[mask_server_node, costmap_filter_info_node, lifecycle_manager_node],
        ),
    ])
