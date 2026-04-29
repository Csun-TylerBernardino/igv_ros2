import rclpy
from rclpy.node import Node

from igv_interfaces.msg import LaneEstimate


class FakeLanePublisher(Node):
    def __init__(self):
        super().__init__('fake_lane_publisher')

        self.pub = self.create_publisher(LaneEstimate, '/lane_estimate', 10)
        self.timer = self.create_timer(0.5, self.publish_lane)

        self.get_logger().info('Fake lane publisher started')

    def publish_lane(self):
        msg = LaneEstimate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.valid = True
        msg.confidence = 0.95
        msg.lateral_error_m = 0.0
        msg.heading_error_rad = 0.0
        msg.lane_width_m = 3.0
        msg.stop_bar_detected = False
        msg.stop_bar_distance_m = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLanePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
