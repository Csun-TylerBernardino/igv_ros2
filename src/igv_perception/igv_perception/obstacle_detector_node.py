import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException

from igv_interfaces.msg import ObstacleSummary


def rotate_point_by_quaternion(x, y, z, qx, qy, qz, qw):
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    rx = (1 - 2 * (yy + zz)) * x + 2 * (xy - wz) * y + 2 * (xz + wy) * z
    ry = 2 * (xy + wz) * x + (1 - 2 * (xx + zz)) * y + 2 * (yz - wx) * z
    rz = 2 * (xz - wy) * x + 2 * (yz + wx) * y + (1 - 2 * (xx + yy)) * z
    return rx, ry, rz


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.obstacle_pub = self.create_publisher(ObstacleSummary, '/obstacle_summary', 10)
        self.box_marker_pub = self.create_publisher(Marker, '/obstacles/box', 10)
        self.obstacle_points_pub = self.create_publisher(PointCloud2, '/obstacles/points', 10)

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',
            self.cloud_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('box_frame', 'base_link')
        self.declare_parameter('min_x', 0.6)
        self.declare_parameter('max_x', 1.5)
        self.declare_parameter('min_y', -0.35)
        self.declare_parameter('max_y', 0.35)
        self.declare_parameter('min_z', 0.10)
        self.declare_parameter('max_z', 0.80)
        self.declare_parameter('min_points', 80)

        self.box_frame = str(self.get_parameter('box_frame').value)
        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_y = float(self.get_parameter('max_y').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.min_points = int(self.get_parameter('min_points').value)

        self.get_logger().info('Obstacle detector node started')
        self.get_logger().info(
            f'Obstacle box in {self.box_frame}: '
            f'x=[{self.min_x}, {self.max_x}] '
            f'y=[{self.min_y}, {self.max_y}] '
            f'z=[{self.min_z}, {self.max_z}] '
            f'min_points={self.min_points}'
        )

    def publish_box_marker(self, obstacle_active: bool):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.box_frame
        marker.ns = 'obstacle_detector'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = (self.min_x + self.max_x) / 2.0
        marker.pose.position.y = (self.min_y + self.max_y) / 2.0
        marker.pose.position.z = (self.min_z + self.max_z) / 2.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.max_x - self.min_x
        marker.scale.y = self.max_y - self.min_y
        marker.scale.z = self.max_z - self.min_z

        if obstacle_active:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.35
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.20

        self.box_marker_pub.publish(marker)

    def publish_obstacle_points(self, points_in_box):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.box_frame

        cloud_msg = point_cloud2.create_cloud_xyz32(header, points_in_box)
        self.obstacle_points_pub.publish(cloud_msg)

    def transform_point_to_box_frame(self, x, y, z, transform):
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        tz = float(transform.transform.translation.z)

        qx = float(transform.transform.rotation.x)
        qy = float(transform.transform.rotation.y)
        qz = float(transform.transform.rotation.z)
        qw = float(transform.transform.rotation.w)

        rx, ry, rz = rotate_point_by_quaternion(x, y, z, qx, qy, qz, qw)

        return rx + tx, ry + ty, rz + tz

    def cloud_callback(self, msg):
        count = 0
        nearest_dist = float('inf')
        nearest_bearing = 0.0
        points_in_box = []

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.box_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f'No transform {msg.header.frame_id} -> {self.box_frame}: {e}')
            return

        try:
            for pt in point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            ):
                x_src = float(pt[0])
                y_src = float(pt[1])
                z_src = float(pt[2])

                x, y, z = self.transform_point_to_box_frame(x_src, y_src, z_src, tf_msg)

                if not (self.min_x <= x <= self.max_x):
                    continue
                if not (self.min_y <= y <= self.max_y):
                    continue
                if not (self.min_z <= z <= self.max_z):
                    continue

                points_in_box.append((float(x), float(y), float(z)))
                count += 1

                dist = math.sqrt(x * x + y * y + z * z)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_bearing = math.atan2(y, x)

        except Exception as e:
            self.get_logger().error(f'Obstacle processing error: {e}')
            return

        obstacle_active = count >= self.min_points

        self.publish_box_marker(obstacle_active)
        self.publish_obstacle_points(points_in_box)

        msg_out = ObstacleSummary()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = self.box_frame

        if obstacle_active:
            msg_out.obstacle_ahead = True
            msg_out.nearest_obstacle_distance_m = float(nearest_dist)
            msg_out.nearest_obstacle_bearing_rad = float(nearest_bearing)
            msg_out.lane_blocked = True
            msg_out.left_lane_open = False
            msg_out.right_lane_open = False
        else:
            msg_out.obstacle_ahead = False
            msg_out.nearest_obstacle_distance_m = 999.0
            msg_out.nearest_obstacle_bearing_rad = 0.0
            msg_out.lane_blocked = False
            msg_out.left_lane_open = True
            msg_out.right_lane_open = True

        self.obstacle_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
