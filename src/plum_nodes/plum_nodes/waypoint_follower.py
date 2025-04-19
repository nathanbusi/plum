import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.get_logger().info("Initializing Waypoint Follower...")

        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.waypoints = []
        self.current_index = 0
        self.goal_handle = None
        self.path_received = False  # <-- added flag

        self.path_sub = self.create_subscription(
            Path,
            '/coverage_path',
            self.path_callback,
            10
        )

        self.timer = self.create_timer(2.0, self.try_send_next_goal)

    def path_callback(self, msg):
        if self.path_received:
            return  # <-- ignore updates after first one

        if len(msg.poses) < 2:
            self.get_logger().warn("Received too few waypoints to follow.")
            return

        self.get_logger().info(f"📡 Received path with {len(msg.poses)} poses.")
        self.waypoints = msg.poses
        self.current_index = 0
        self.path_received = True  # <-- prevent future updates

    def try_send_next_goal(self):
        if not self.waypoints:
            self.get_logger().warn("No waypoints received yet.")
            return

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available.')
            return

        self.timer.cancel()
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("✅ All waypoints completed.")
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.waypoints[self.current_index]

        self.get_logger().info(f"📍 Sending waypoint {self.current_index + 1}/{len(self.waypoints)}...")
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"❌ Goal {self.current_index} was rejected.")
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info(f"🚀 Goal {self.current_index} accepted.")
        self.goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        status = self.goal_handle.status

        if status == 4:
            self.get_logger().warn(f"⚠️ Goal {self.current_index} aborted. Skipping.")
        elif status == 5:
            self.get_logger().warn(f"⚠️ Goal {self.current_index} rejected. Skipping.")
        else:
            self.get_logger().info(f"✅ Reached waypoint {self.current_index}.")

        self.current_index += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
