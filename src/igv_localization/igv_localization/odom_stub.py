import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class OdomStub(Node):
    def __init__(self):
        super().__init__('odom_stub')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('rate_hz', 20.0)

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)

        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.declare_parameter('linear_x', 0.0)
        self.declare_parameter('linear_y', 0.0)
        self.declare_parameter('linear_z', 0.0)

        self.declare_parameter('angular_x', 0.0)
        self.declare_parameter('angular_y', 0.0)
        self.declare_parameter('angular_z', 0.0)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.z = float(self.get_parameter('z').value)

        self.roll = float(self.get_parameter('roll').value)
        self.pitch = float(self.get_parameter('pitch').value)
        self.yaw = float(self.get_parameter('yaw').value)

        self.linear_x = float(self.get_parameter('linear_x').value)
        self.linear_y = float(self.get_parameter('linear_y').value)
        self.linear_z = float(self.get_parameter('linear_z').value)

        self.angular_x = float(self.get_parameter('angular_x').value)
        self.angular_y = float(self.get_parameter('angular_y').value)
        self.angular_z = float(self.get_parameter('angular_z').value)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.publish_odom)

        self.get_logger().info(
            f'Odom stub started: {self.odom_frame} -> {self.base_frame}, '
            f'publish_tf={self.publish_tf}, rate_hz={self.rate_hz}'
        )

    def publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_euler(self.roll, self.pitch, self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.linear.y = self.linear_y
        odom.twist.twist.linear.z = self.linear_z

        odom.twist.twist.angular.x = self.angular_x
        odom.twist.twist.angular.y = self.angular_y
        odom.twist.twist.angular.z = self.angular_z

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = self.z

            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomStub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
