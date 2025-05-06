import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from builtin_interfaces.msg import Duration

import numpy as np
from scipy.ndimage import binary_dilation, label, find_objects
from tf_transformations import quaternion_from_euler
import math

class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        self.map_received = False

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.marker_pub = self.create_publisher(Marker, '/navigable_mask_marker', qos)
        self.path_pub = self.create_publisher(Path, '/coverage_path', qos)

        self.get_logger().info("CoveragePlanner tile sweep v1 started. Waiting for map...")

    def map_callback(self, msg):
        if self.map_received:
            return
        self.map_received = True
        self.get_logger().info("Map received!")

        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        height, width = grid.shape
        self.get_logger().info(f"Map size: {width}x{height}, resolution: {resolution:.3f}")

        free_mask = grid == 0
        obstacle_mask = grid == 100

        dilated_obstacles = binary_dilation(obstacle_mask, iterations=10)
        nav_mask = np.logical_and(free_mask, ~dilated_obstacles)

        labeled, _ = label(nav_mask)
        start_x_px = int(-origin_y / resolution)
        start_y_px = int(-origin_x / resolution)
        start_x_px = np.clip(start_x_px, 0, height - 1)
        start_y_px = np.clip(start_y_px, 0, width - 1)
        robot_region = labeled[start_x_px, start_y_px]

        if robot_region == 0:
            self.get_logger().warn("Robot is not in a navigable region.")
            return

        nav_mask = labeled == robot_region
        self.publish_navigable_marker(nav_mask, resolution, origin_x, origin_y)

        visited = np.zeros_like(nav_mask, dtype=bool)
        all_waypoints = []
        max_waypoints = 361  # Cap on total waypoints

        while len(all_waypoints) < max_waypoints:
            waypoints = self.sweep_snake(nav_mask, visited, resolution, origin_x, origin_y, tile_size_m=0.25)
            if not waypoints:
                break

            remaining = max_waypoints - len(all_waypoints)
            all_waypoints.extend(waypoints[:remaining])

            if len(all_waypoints) >= max_waypoints:
                self.get_logger().warn(f"Reached max waypoint cap ({max_waypoints}). Stopping early.")
                break

            unvisited = np.logical_and(nav_mask, ~visited)
            labeled_unvisited, _ = label(unvisited)
            slices = find_objects(labeled_unvisited)

            resumed = False
            for i, slc in enumerate(slices):
                if slc is None:
                    continue
                region = labeled_unvisited[slc] == (i + 1)
                h, w = region.shape
                if h >= 40 and w >= 40:
                    cy, cx = np.array(np.argwhere(region).mean(axis=0), dtype=int)
                    center_row = slc[0].start + cy
                    center_col = slc[1].start + cx
                    self.get_logger().info(f"Detected unvisited cluster at ({center_row}, {center_col}), resuming sweep...")
                    visited[center_row, center_col] = False
                    resumed = True
                    break
            if not resumed:
                break

        self.get_logger().info(f"Planned {len(all_waypoints)} total sweep waypoints.")
        self.publish_path(all_waypoints)

    def sweep_snake(self, mask, visited, resolution, origin_x, origin_y, tile_size_m=0.25):
        tile_size_px = max(1, int(tile_size_m / resolution))
        height, width = mask.shape
        waypoints = []
        row_dirs = [(0, 1), (0, -1)]  # right, left

        for start_row in range(height):
            for start_col in range(width):
                if mask[start_row, start_col] and not visited[start_row, start_col]:
                    row, col = start_row, start_col
                    break
            else:
                continue
            break
        else:
            self.get_logger().warn("No navigable start point found.")
            return []

        dir_idx = 0
        backtrack_stack = []

        while True:
            if not mask[row, col] or visited[row, col]:
                break

            visited[row, col] = True
            backtrack_stack.append((row, col))
            x = origin_x + col * resolution
            y = origin_y + row * resolution
            waypoints.append((x, y))

            dr, dc = row_dirs[dir_idx]
            next_r = row + dr * tile_size_px
            next_c = col + dc * tile_size_px

            if (
                0 <= next_r < height and 0 <= next_c < width
                and mask[next_r, next_c]
                and not visited[next_r, next_c]
            ):
                row, col = next_r, next_c
                continue

            found = False
            for delta_r in range(tile_size_px, height - row, tile_size_px):
                down_r = row + delta_r
                if down_r >= height:
                    break
                if mask[down_r, col] and not visited[down_r, col]:
                    row = down_r
                    dir_idx = 1 - dir_idx
                    found = True
                    break

            if found:
                continue

            self.get_logger().info("Falling back to backtracking...")
            dir_idx = 1 - dir_idx
            while backtrack_stack:
                b_row, b_col = backtrack_stack.pop()
                down_r = b_row + tile_size_px
                if (
                    0 <= down_r < height
                    and mask[down_r, b_col]
                    and not visited[down_r, b_col]
                ):
                    x = origin_x + b_col * resolution
                    y = origin_y + b_row * resolution
                    waypoints.append((x, y))
                    row, col = down_r, b_col
                    self.get_logger().info("Backtracked and dropped down. Resuming sweep.")
                    break
            else:
                self.get_logger().info("Backtracking failed. Ending sweep.")
                break

        return waypoints

    def prune_waypoints(self, waypoints, tolerance=1e-3):
        if len(waypoints) <= 2:
            return waypoints

        pruned = [waypoints[0]]

        for i in range(1, len(waypoints) - 1):
            x0, y0 = pruned[-1]
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i + 1]

            v1 = (x1 - x0, y1 - y0)
            v2 = (x2 - x1, y2 - y1)

            cross = abs(v1[0]*v2[1] - v1[1]*v2[0])
            dot = v1[0]*v2[0] + v1[1]*v2[1]
            norm1 = math.hypot(*v1)
            norm2 = math.hypot(*v2)

            if norm1 * norm2 == 0:
                pruned.append(waypoints[i])
                continue

            sin_angle = cross / (norm1 * norm2)

            if abs(sin_angle) > tolerance:
                pruned.append(waypoints[i])

        pruned.append(waypoints[-1])
        self.get_logger().info(f"Pruned from {len(waypoints)} to {len(pruned)} waypoints.")
        return pruned

    def publish_path(self, waypoints):
        if not waypoints:
            return

        pruned_waypoints = self.prune_waypoints(waypoints)

        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = now

        for i in range(len(pruned_waypoints)):
            x, y = pruned_waypoints[i]
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            if i < len(pruned_waypoints) - 1:
                dx = pruned_waypoints[i + 1][0] - x
                dy = pruned_waypoints[i + 1][1] - y
                yaw = math.atan2(dy, dx)
                q = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            else:
                pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published Path with {len(path_msg.poses)} poses.")

    def publish_navigable_marker(self, mask, resolution, origin_x, origin_y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "coverage_debug"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = resolution
        marker.scale.y = resolution
        marker.scale.z = resolution
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime = Duration(sec=0)

        height, width = mask.shape
        for row in range(height):
            for col in range(width):
                if mask[row, col]:
                    pt = Point()
                    pt.x = origin_x + col * resolution
                    pt.y = origin_y + row * resolution
                    pt.z = 0.0
                    marker.points.append(pt)

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published navigable area marker with {len(marker.points)} points.")


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
