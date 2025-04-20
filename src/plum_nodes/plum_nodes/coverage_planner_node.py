import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.ndimage import binary_dilation, binary_fill_holes, label
import math
from tf_transformations import quaternion_from_euler


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        self.map_received = False
        self.waypoints = []

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.marker_pub = self.create_publisher(Marker, '/coverage_path_marker', qos)
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)
        self.publish_timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("CoveragePlanner node started. Waiting for map...")

    def map_callback(self, msg):
        if self.map_received:
            return
        self.map_received = True
        self.get_logger().info("Map received!")

        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        free_mask = grid == 0
        obstacle_mask = grid == 100

        filled_obstacles = binary_fill_holes(obstacle_mask)
        dilated = binary_dilation(filled_obstacles, iterations=5)
        safe_mask = np.logical_and(free_mask, ~dilated)

        labeled, num_regions = label(safe_mask)

        origin_map_x = int(-origin_y / resolution)
        origin_map_y = int(-origin_x / resolution)
        origin_map_x = np.clip(origin_map_x, 0, grid.shape[0] - 1)
        origin_map_y = np.clip(origin_map_y, 0, grid.shape[1] - 1)

        origin_region = labeled[origin_map_x, origin_map_y]
        self.get_logger().info(f"Map origin @ grid ({origin_map_x}, {origin_map_y}) in region {origin_region}")

        if origin_region != 0:
            room_mask = labeled == origin_region
            self.get_logger().info("Flood fill from robot start pose succeeded.")
        else:
            self.get_logger().warn("Could not find safe region connected to robot start pose. Falling back to largest safe region.")
            region_sizes = [(np.sum(labeled == i), i) for i in range(1, num_regions + 1)]
            region_sizes.sort(reverse=True)
            if region_sizes:
                largest_region = region_sizes[0][1]
                room_mask = labeled == largest_region
            else:
                self.get_logger().error("No safe regions found in map.")
                return

        safe_coords = np.argwhere(room_mask)
        self.get_logger().info(f"Found {len(safe_coords)} safe pixels in final region")

        # Tile coverage logic
        tile_size_m = 0.2
        tile_size_px = int(tile_size_m / resolution)
        visited_tiles = set()
        tile_points = {}
        max_tiles = 500

        for y, x in safe_coords:
            tile_x = x // tile_size_px
            tile_y = y // tile_size_px
            key = (tile_y, tile_x)
            if key not in visited_tiles:
                visited_tiles.add(key)
                tile_points[key] = (x, y)
                if len(tile_points) >= max_tiles:
                    break

        self.get_logger().info(f"Generated {len(tile_points)} unique tile points")

        # Convert to world coordinates
        world_points = []
        for key in tile_points:
            x, y = tile_points[key]
            wx = origin_x + x * resolution
            wy = origin_y + y * resolution
            world_points.append((wx, wy))

        world_points = world_points[:500]

        # Nearest-neighbor ordering
        raw_path = []
        if world_points:
            visited = set()
            current = world_points[0]
            while len(visited) < len(world_points):
                raw_path.append(current)
                visited.add(current)
                next_point = min(
                    (pt for pt in world_points if pt not in visited),
                    key=lambda p: math.hypot(p[0] - current[0], p[1] - current[1]),
                    default=None
                )
                current = next_point if next_point else current

        # Pruning
        pruned_path = []
        dist_threshold = 4
        angle_threshold_rad = math.radians(5)

        if len(raw_path) >= 2:
            pruned_path.append(raw_path[0])
            for i in range(1, len(raw_path) - 1):
                x0, y0 = pruned_path[-1]
                x1, y1 = raw_path[i]
                x2, y2 = raw_path[i + 1]

                dist = math.hypot(x1 - x0, y1 - y0)
                v1 = (x1 - x0, y1 - y0)
                v2 = (x2 - x1, y2 - y1)

                dot = v1[0] * v2[0] + v1[1] * v2[1]
                norm1 = math.hypot(*v1)
                norm2 = math.hypot(*v2)
                angle = math.acos(dot / (norm1 * norm2 + 1e-6)) if norm1 > 0 and norm2 > 0 else 0.0

                if dist > dist_threshold or abs(angle) > angle_threshold_rad:
                    pruned_path.append(raw_path[i])

            pruned_path.append(raw_path[-1])
        else:
            pruned_path = raw_path

        self.waypoints = pruned_path
        self.get_logger().info(f"Planned {len(self.waypoints)} safe waypoints.")

    def timer_callback(self):
        if not self.waypoints:
            return

        limited_waypoints = self.waypoints[:20000]
        now = self.get_clock().now().to_msg()

        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.header.stamp = now
        sphere_marker.ns = "coverage_path"
        sphere_marker.id = 0
        sphere_marker.type = Marker.SPHERE_LIST
        sphere_marker.action = Marker.ADD
        sphere_marker.scale.x = 0.05
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05
        sphere_marker.color.r = 0.5
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 1.0
        sphere_marker.color.a = 1.0
        sphere_marker.lifetime = Duration(sec=0)

        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = now
        line_marker.ns = "coverage_path"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02
        line_marker.color.r = 0.5
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        line_marker.lifetime = Duration(sec=0)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = now

        for i in range(len(limited_waypoints)):
            x, y = limited_waypoints[i]
            pt = Point(x=x, y=y, z=0.0)
            sphere_marker.points.append(pt)
            line_marker.points.append(pt)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position = pt

            if i < len(limited_waypoints) - 1:
                dx = limited_waypoints[i + 1][0] - x
                dy = limited_waypoints[i + 1][1] - y
                yaw = math.atan2(dy, dx)
                q = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            else:
                pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.marker_pub.publish(sphere_marker)
        self.marker_pub.publish(line_marker)
        self.path_pub.publish(path_msg)

        self.get_logger().info(f"Published {len(limited_waypoints)} waypoints.")


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
