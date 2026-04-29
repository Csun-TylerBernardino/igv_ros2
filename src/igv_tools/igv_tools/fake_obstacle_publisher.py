import rclpy
from rclpy.node import Node

from igv_interfaces.msg import ObstacleSummary


class FakeObstaclePublisher(Node):
    def __init__(self):
        super().__init__('fake_obstacle_publisher')

        self.pub = self.create_publisher(ObstacleSummary, '/obstacle_summary', 10)
        self.timer = self.create_timer(0.5, self.publish_obstacle)

        self.get_logger().info('Fake obstacle publisher started')

    def publish_obstacle(self):
        msg = ObstacleSummary()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.obstacle_ahead = False
        msg.nearest_obstacle_distance_m = 999.0
        msg.nearest_obstacle_bearing_rad = 0.0
        msg.lane_blocked = False
        msg.left_lane_open = True
        msg.right_lane_open = True
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
