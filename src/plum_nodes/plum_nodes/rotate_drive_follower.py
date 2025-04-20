import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import math


class SimpleWaypointFollower(Node):
    def __init__(self):
        super().__init__('simple_waypoint_follower')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/coverage_path', self.path_callback, 10)

        self.pose = None
        self.waypoints = []
        self.current_index = 0
        self.state = 'rotate'
        self.timer = self.create_timer(0.1, self.control_loop)

        self.move_duration = 0.0
        self.move_start_time = None

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def path_callback(self, msg):
        if len(msg.poses) > 0 and not self.waypoints:
            self.waypoints = msg.poses
            self.current_index = 0
            self.state = 'rotate'
            self.get_logger().info(f"📍 Received {len(msg.poses)} waypoints.")

    def control_loop(self):
        if not self.pose or self.current_index >= len(self.waypoints):
            return

        current = self.pose.position
        orientation = self.pose.orientation
        yaw = self.get_yaw_from_quaternion(orientation)

        target_pose = self.waypoints[self.current_index].pose
        target = target_pose.position
        target_yaw = self.get_yaw_from_quaternion(target_pose.orientation)

        dx = target.x - current.x
        dy = target.y - current.y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - yaw)
        final_angle_diff = self.normalize_angle(target_yaw - yaw)

        cmd = Twist()

        if self.state == 'rotate':
            if abs(angle_diff) > 0.1:
                cmd.angular.z = 1.0 * angle_diff
                self.get_logger().info(f"🔁 Rotating to face waypoint: angle_diff={angle_diff:.2f}")
            else:
                self.get_logger().info("✅ Facing waypoint. Driving...")
                self.move_duration = distance / 0.2  # estimate time to reach
                self.move_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.state = 'drive'

        elif self.state == 'drive':
            elapsed = self.get_clock().now().seconds_nanoseconds()[0] - self.move_start_time
            if elapsed < self.move_duration:
                cmd.linear.x = 0.2
                self.get_logger().info(f"🚗 Driving for {elapsed:.1f}/{self.move_duration:.1f} seconds")
            else:
                self.get_logger().info("🛑 Finished drive duration. Rotating to final orientation...")
                self.state = 'final_rotate'

        elif self.state == 'final_rotate':
            if abs(final_angle_diff) > 0.1:
                cmd.angular.z = 1.0 * final_angle_diff
                self.get_logger().info(f"🧭 Rotating to goal orientation: diff={final_angle_diff:.2f}")
            else:
                self.get_logger().info(f"✅ Waypoint {self.current_index + 1}/{len(self.waypoints)} complete.")
                self.current_index += 1
                self.state = 'rotate'

        self.cmd_pub.publish(cmd)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SimpleWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
