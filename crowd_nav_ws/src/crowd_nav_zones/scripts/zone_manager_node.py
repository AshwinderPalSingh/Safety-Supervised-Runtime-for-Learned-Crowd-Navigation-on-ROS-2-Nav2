#!/usr/bin/env python3
"""Zone manager for Phase 3's dynamic keep-out zones.

Owns the only genuinely new logic in this feature: tracking a set of named rectangular
zones and rendering them into a costmap-filter mask. Everything that turns that mask into an
actual enforced no-go region - the KeepoutFilter costmap plugin, costmap_filter_info_server,
the mask's own map_server instance - is stock Nav2 (see docs/phase2-findings.md, "don't
reimplement what the Nav2 stack already does better": this node deliberately does NOT publish
an OccupancyGrid itself or do any costmap math - it writes a map file and asks a real
map_server to (re)load it, the same mechanism a human would use to swap maps at runtime).

Mask encoding matches Nav2's documented keepout-filter convention exactly: 0 (black) = keepout,
255 (white) = clear, loaded as a `mode: trinary` map with default thresholds so map_server's
own occupancy-probability conversion turns black into occupancy 100 and white into 0 - paired
with a CostmapFilterInfo of type=0 (keepout), base=0.0, multiplier=1.0 (this node's launch
file), which is the standard configuration for this filter type, not something invented here.

Mask files are written to a directory OUTSIDE this workspace's path (default /tmp/...): this
project's directory name contains spaces, which has broken three independent tools already
(see docs/phase1-findings.md, docs/phase2-findings.md) via subprocess/path-quoting bugs. Not
risking a fourth occurrence for a file this node itself both writes and reads back.
"""
import os
import threading

import rclpy
from crowd_nav_zones.srv import AddZone, RemoveZone
from nav2_msgs.srv import LoadMap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class ZoneManager(Node):
    def __init__(self):
        super().__init__('zone_manager')

        self.declare_parameter('mask_dir', '/tmp/crowd_nav_zone_masks')
        self.declare_parameter('resolution', 0.03)
        self.declare_parameter('origin_x', -1.1)
        self.declare_parameter('origin_y', -2.19)
        self.declare_parameter('width', 276)
        self.declare_parameter('height', 143)

        self.mask_dir = self.get_parameter('mask_dir').value
        self.resolution = self.get_parameter('resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        os.makedirs(self.mask_dir, exist_ok=True)
        self.pgm_path = os.path.join(self.mask_dir, 'mask.pgm')
        self.yaml_path = os.path.join(self.mask_dir, 'mask.yaml')

        # Zones this node currently knows about. zone_id -> (center_x, center_y, size_x, size_y),
        # all in the 'map' frame, matching the grid parameters above.
        self.zones = {}

        # Write an all-clear mask synchronously at construction time, before spinning - the
        # mask map_server needs a valid file to load at its own on_configure, which can happen
        # before this node ever services a request.
        self._write_mask()
        self.get_logger().info(f'Initial all-clear mask written to {self.yaml_path}')

        # A ReentrantCallbackGroup + MultiThreadedExecutor (see main()) is required here: the
        # add/remove service callbacks below block on load_map_client's response, which can
        # only arrive if the executor is free to process it concurrently. On a
        # SingleThreadedExecutor (rclpy's default) this would deadlock - the executor would be
        # stuck inside the outer service callback, unable to spin the client's own response.
        cb_group = ReentrantCallbackGroup()
        self.load_map_client = self.create_client(LoadMap, '/mask_server/load_map', callback_group=cb_group)

        self.create_service(AddZone, 'add_zone', self._on_add_zone, callback_group=cb_group)
        self.create_service(RemoveZone, 'remove_zone', self._on_remove_zone, callback_group=cb_group)

    def _write_mask(self):
        """Render self.zones into the mask PGM+YAML pair, overwriting in place."""
        row = bytearray([255]) * self.width
        rows = [bytearray(row) for _ in range(self.height)]

        for (center_x, center_y, size_x, size_y) in self.zones.values():
            min_x = center_x - size_x / 2.0
            max_x = center_x + size_x / 2.0
            min_y = center_y - size_y / 2.0
            max_y = center_y + size_y / 2.0

            col_lo = max(0, int((min_x - self.origin_x) / self.resolution))
            col_hi = min(self.width, int((max_x - self.origin_x) / self.resolution) + 1)
            row_lo = max(0, int((min_y - self.origin_y) / self.resolution))
            row_hi = min(self.height, int((max_y - self.origin_y) / self.resolution) + 1)

            for r in range(row_lo, row_hi):
                for c in range(col_lo, col_hi):
                    rows[r][c] = 0

        # PGM stores row 0 as the TOP of the image, but map_server's origin convention treats
        # increasing row index as increasing world y (like the rest of this project's map
        # handling, see docs/phase2-findings.md's ground-truth pose logging) - so row 0 in the
        # file must be the row with the HIGHEST y, i.e. rows need to be written bottom-to-top.
        with open(self.pgm_path, 'wb') as f:
            f.write(f'P5\n{self.width} {self.height}\n255\n'.encode('ascii'))
            for r in reversed(range(self.height)):
                f.write(bytes(rows[r]))

        with open(self.yaml_path, 'w') as f:
            f.write(
                f'image: {os.path.basename(self.pgm_path)}\n'
                f'mode: trinary\n'
                f'resolution: {self.resolution}\n'
                f'origin: [{self.origin_x}, {self.origin_y}, 0]\n'
                f'negate: 0\n'
                f'occupied_thresh: 0.65\n'
                f'free_thresh: 0.25\n'
            )

    def _reload_mask_server(self):
        if not self.load_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('mask_server/load_map service not available, zone change not applied')
            return False
        req = LoadMap.Request()
        # Plain absolute path, NOT a "file://" URI: LoadMap.srv's own doc comment says
        # "file:///path/to/maps/floor1.yaml" is valid syntax, but empirically, on this Nav2
        # build, a file:// prefix makes map_server return RESULT_INVALID_MAP_METADATA even for
        # a trivially valid map - confirmed directly via `ros2 service call` on a known-good
        # file, isolating it to the URI scheme rather than anything about the mask content. A
        # plain path works. Verify against the real service before trusting a message's own
        # doc comment, not just this one - see docs/phase2-findings.md for the broader pattern.
        req.map_url = self.yaml_path

        # Deliberately not rclpy.spin_until_future_complete(self, ...) here: this method runs
        # INSIDE an add_zone/remove_zone service callback, which is itself already being
        # executed by the node's own executor. Nesting a second spin call on the same node
        # from within a callback deadlocks on a SingleThreadedExecutor and is unsafe even with
        # a MultiThreadedExecutor. Instead, block this callback's own worker thread on a plain
        # threading.Event and let a DIFFERENT executor thread (this node uses
        # MultiThreadedExecutor, see main()) invoke the done-callback when the response arrives.
        done_event = threading.Event()
        result_holder = {}

        def _on_done(fut):
            result_holder['response'] = fut.result()
            done_event.set()

        future = self.load_map_client.call_async(req)
        future.add_done_callback(_on_done)
        if not done_event.wait(timeout=10.0):
            self.get_logger().error('mask_server/load_map call timed out')
            return False
        result = result_holder.get('response')
        if result is None:
            self.get_logger().error('mask_server/load_map call failed')
            return False
        if result.result != LoadMap.Response.RESULT_SUCCESS:
            self.get_logger().error(f'mask_server rejected the reloaded mask, result={result.result}')
            return False
        return True

    def _on_add_zone(self, request, response):
        self.zones[request.zone_id] = (
            request.center_x, request.center_y, request.size_x, request.size_y,
        )
        self._write_mask()
        if self._reload_mask_server():
            response.success = True
            response.message = f'zone {request.zone_id} active, {len(self.zones)} zone(s) total'
            self.get_logger().info(f'Added zone {request.zone_id}: {self.zones[request.zone_id]}')
        else:
            del self.zones[request.zone_id]
            response.success = False
            response.message = 'failed to reload mask_server, zone not applied'
        return response

    def _on_remove_zone(self, request, response):
        if request.zone_id not in self.zones:
            response.success = False
            response.message = f'no active zone with id {request.zone_id}'
            return response
        removed = self.zones.pop(request.zone_id)
        self._write_mask()
        if self._reload_mask_server():
            response.success = True
            response.message = f'zone {request.zone_id} removed, {len(self.zones)} zone(s) remaining'
            self.get_logger().info(f'Removed zone {request.zone_id}: {removed}')
        else:
            self.zones[request.zone_id] = removed
            response.success = False
            response.message = 'failed to reload mask_server, removal not applied'
        return response


def main():
    rclpy.init()
    node = ZoneManager()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
