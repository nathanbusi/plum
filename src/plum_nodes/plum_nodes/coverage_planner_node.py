import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.ndimage import binary_dilation, label


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
        dilated = binary_dilation(obstacle_mask, iterations=3)
        safe_mask = np.logical_and(free_mask, ~dilated)

        # Flood-fill filtering: keep only region connected to origin
        labeled, num_regions = label(safe_mask)

        # Compute origin pixel
        origin_map_x = int(-origin_y / resolution)
        origin_map_y = int(-origin_x / resolution)
        origin_map_x = np.clip(origin_map_x, 0, grid.shape[0] - 1)
        origin_map_y = np.clip(origin_map_y, 0, grid.shape[1] - 1)
        origin_region = labeled[origin_map_x, origin_map_y]

        # Only keep safe cells in the origin-connected region
        safe_mask = labeled == origin_region
        safe_coords = np.argwhere(safe_mask)

        # Simple tile-based planner
        tile_size_m = 0.25
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

        sorted_keys = sorted(tile_points.keys(), key=lambda k: (k[0], k[1] if k[0] % 2 == 0 else -k[1]))

        self.waypoints = []
        for key in sorted_keys:
            x, y = tile_points[key]
            wx = origin_x + x * resolution
            wy = origin_y + y * resolution
            self.waypoints.append((wx, wy))

        self.get_logger().info(f"Planned {len(self.waypoints)} safe, reachable waypoints.")

    def timer_callback(self):
        if not self.waypoints:
            return

        # Spheres
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "map"
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = "coverage_path"
        sphere_marker.id = 0
        sphere_marker.type = Marker.SPHERE_LIST
        sphere_marker.action = Marker.ADD
        sphere_marker.scale.x = 0.05
        sphere_marker.scale.y = 0.05
        sphere_marker.scale.z = 0.05
        sphere_marker.color.r = 0.5
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 1.0  # Purple
        sphere_marker.color.a = 1.0
        sphere_marker.lifetime = Duration(sec=0)

        # Path line
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "coverage_path"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.02
        line_marker.color.r = 0.5
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0  # Purple
        line_marker.color.a = 1.0
        line_marker.lifetime = Duration(sec=0)

        for x, y in self.waypoints:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = 0.0
            sphere_marker.points.append(pt)
            line_marker.points.append(pt)

        self.marker_pub.publish(sphere_marker)
        self.marker_pub.publish(line_marker)
        self.get_logger().info(f"Published {len(self.waypoints)} points as spheres + path line.")


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
