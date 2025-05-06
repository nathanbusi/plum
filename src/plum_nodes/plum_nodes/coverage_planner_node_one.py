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
#v1

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
        dilated = binary_dilation(filled_obstacles, iterations=6)
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

        fill_spacing_m = 0.3
        fill_spacing_px = max(1, int(fill_spacing_m / resolution))

        height, width = room_mask.shape
        dense_path = []

        reverse = False
        for row in range(0, height, fill_spacing_px):
            line = room_mask[row, :]
            segments = []
            current_start = None

            for col in range(width):
                if line[col]:
                    if current_start is None:
                        current_start = col
                else:
                    if current_start is not None:
                        if col - current_start >= 2:
                            segments.append((current_start, col - 1))
                        current_start = None
            if current_start is not None and width - current_start >= 2:
                segments.append((current_start, width - 1))

            for start, end in (segments if not reverse else reversed(segments)):
                col_range = range(start, end + 1) if not reverse else reversed(range(start, end + 1))
                for col in col_range:
                    if 1 <= row < height - 1 and 1 <= col < width - 1:
                        window = room_mask[row - 1:row + 2, col - 1:col + 2]
                        if np.all(window):
                            wx = origin_x + col * resolution
                            wy = origin_y + row * resolution
                            dense_path.append((wx, wy))
            reverse = not reverse

        def direction(p1, p2):
            dx = round(p2[0] - p1[0], 3)
            dy = round(p2[1] - p1[1], 3)
            norm = math.hypot(dx, dy)
            return (round(dx / norm, 2), round(dy / norm, 2)) if norm > 0 else (0, 0)

        simplified = []
        if dense_path:
            simplified.append(dense_path[0])
            for i in range(1, len(dense_path) - 1):
                d1 = direction(dense_path[i - 1], dense_path[i])
                d2 = direction(dense_path[i], dense_path[i + 1])
                if d1 != d2:
                    simplified.append(dense_path[i])
            simplified.append(dense_path[-1])

        self.waypoints = simplified
        self.get_logger().info(f"Reduced {len(dense_path)} dense points to {len(simplified)} simplified waypoints.")

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

        self.get_logger().info(f"Published {len(limited_waypoints)} final waypoints.")


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
