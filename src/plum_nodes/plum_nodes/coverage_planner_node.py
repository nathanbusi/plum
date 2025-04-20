import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.ndimage import binary_dilation, label
import math
from tf_transformations import quaternion_from_euler


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        self.map_received = False
        self.waypoints = []

        # Subscribe to the map with transient QoS
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        # Publish markers with transient QoS
        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(Marker, '/coverage_path_marker', marker_qos)

        # Publish path as nav_msgs/Path
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)

        # Timer to re-publish markers every second
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

        # Obstacle dilation to avoid planning too close
        free_mask = grid == 0
        obstacle_mask = grid == 100
        dilated = binary_dilation(obstacle_mask, iterations=25)
        safe_mask = np.logical_and(free_mask, ~dilated)

        # Flood-fill filtering: keep only region connected to origin
        labeled, num_regions = label(safe_mask)

        origin_map_x = int(-origin_y / resolution)
        origin_map_y = int(-origin_x / resolution)
        origin_map_x = np.clip(origin_map_x, 0, grid.shape[0] - 1)
        origin_map_y = np.clip(origin_map_y, 0, grid.shape[1] - 1)
        origin_region = labeled[origin_map_x, origin_map_y]

        safe_mask = labeled == origin_region
        safe_coords = np.argwhere(safe_mask)

        # Simple tile-based planner
        tile_size_m = 0.3
        tile_size_px = int(tile_size_m / resolution)
        visited_tiles = set()
        tile_points = {}

        for y, x in safe_coords:
            tile_x = x // tile_size_px
            tile_y = y // tile_size_px
            key = (tile_y, tile_x)
            if key not in visited_tiles:
                visited_tiles.add(key)
                tile_points[key] = (x, y)

        # Convert to world-frame coordinates
        world_points = []
        for key in tile_points:
            x, y = tile_points[key]
            wx = origin_x + x * resolution
            wy = origin_y + y * resolution
            world_points.append((wx, wy))

        # Nearest-neighbor reordering
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

        # Combined distance + angle-based pruning
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
        self.get_logger().info(f"Planned {len(self.waypoints)} safe, reachable waypoints.")

    def timer_callback(self):
        if not self.waypoints:
            return

        limited_waypoints = self.waypoints[:20000]
        now = self.get_clock().now().to_msg()

        # Spheres
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

        # Path line
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

        # Path message
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

            # Calculate orientation toward next waypoint
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

        self.get_logger().info(f"Published {len(limited_waypoints)} points as markers + path.")


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
