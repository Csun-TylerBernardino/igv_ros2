import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from igv_interfaces.msg import BehaviorState


class CmdVelManager(Node):
    def __init__(self):
        super().__init__('cmd_vel_manager')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.behavior_sub = self.create_subscription(
            BehaviorState,
            '/behavior_state',
            self.behavior_callback,
            10
        )

        self.latest_speed = 0.0
        self.latest_steering = 0.0
        self.timer = self.create_timer(0.1, self.publish_cmd)

        self.get_logger().info('cmd_vel manager started')

    def behavior_callback(self, msg: BehaviorState):
        self.latest_speed = msg.target_speed_mps
        self.latest_steering = msg.target_steering_rad

    def publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.latest_speed
        cmd.angular.z = self.latest_steering
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
