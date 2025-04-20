import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path


class Breadcrumbs(Node):
    def __init__(self):
        super().__init__('breadcrumb_publisher')
        self.path_pub = self.create_publisher(Path, '/breadcrumb_path', 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.path = Path()
        self.path.header.frame_id = 'map'

    def pose_callback(self, msg):
        stamped_pose = PoseStamped()
        stamped_pose.header = msg.header
        stamped_pose.pose = msg.pose.pose  # strip off the covariance

        self.path.poses.append(stamped_pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = Breadcrumbs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
