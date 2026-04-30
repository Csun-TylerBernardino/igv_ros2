import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException


def rotate_vector_by_quaternion(x, y, z, qx, qy, qz, qw):
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


class CloudDebugFilterNode(Node):
    def __init__(self):
        super().__init__('cloud_debug_filter_node')

        self.filtered_cloud_pub = self.create_publisher(PointCloud2, '/debug/filtered_cloud', 10)
        self.fov_marker_pub = self.create_publisher(Marker, '/debug/camera_fov_ground', 10)

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',
            self.cloud_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/color/rect/image/camera_info',
            self.camera_info_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('min_z', 0.0)
        self.declare_parameter('max_z', 1.5)
        self.declare_parameter('max_range_m', 6.096)
        self.declare_parameter('sample_step', 3)
        self.declare_parameter('fov_ground_z', 0.02)
        self.declare_parameter('fov_max_range_m', 3.35)
        self.declare_parameter('fov_top_crop_ratio', 0.45)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.sample_step = int(self.get_parameter('sample_step').value)
        self.fov_ground_z = float(self.get_parameter('fov_ground_z').value)
        self.fov_max_range_m = float(self.get_parameter('fov_max_range_m').value)
        self.fov_top_crop_ratio = float(self.get_parameter('fov_top_crop_ratio').value)

        self.have_camera_info = False
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.img_w = None
        self.img_h = None
        self.camera_frame = None

        self.create_timer(0.5, self.publish_fov_marker)

        self.get_logger().info('Cloud debug filter node started')

    def camera_info_callback(self, msg):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.img_w = int(msg.width)
        self.img_h = int(msg.height)
        self.camera_frame = msg.header.frame_id
        self.have_camera_info = True

    def transform_point(self, x, y, z, transform):
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        tz = float(transform.transform.translation.z)

        qx = float(transform.transform.rotation.x)
        qy = float(transform.transform.rotation.y)
        qz = float(transform.transform.rotation.z)
        qw = float(transform.transform.rotation.w)

        rx, ry, rz = rotate_vector_by_quaternion(x, y, z, qx, qy, qz, qw)
        return rx + tx, ry + ty, rz + tz

    def cloud_callback(self, msg):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f'No transform {msg.header.frame_id} -> {self.target_frame}: {e}')
            return

        filtered_points = []

        try:
            for i, pt in enumerate(point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True
            )):
                if self.sample_step > 1 and (i % self.sample_step) != 0:
                    continue

                x_src = float(pt[0])
                y_src = float(pt[1])
                z_src = float(pt[2])

                range_src = math.sqrt(x_src * x_src + y_src * y_src + z_src * z_src)
                if range_src > self.max_range_m:
                    continue

                x, y, z = self.transform_point(x_src, y_src, z_src, tf_msg)

                if z < self.min_z:
                    continue
                if z > self.max_z:
                    continue

                filtered_points.append((float(x), float(y), float(z)))

        except Exception as e:
            self.get_logger().error(f'Filtered cloud processing error: {e}')
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame

        cloud_msg = point_cloud2.create_cloud_xyz32(header, filtered_points)
        self.filtered_cloud_pub.publish(cloud_msg)

    def publish_fov_marker(self):
        if not self.have_camera_info:
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time()
            )
        except TransformException:
            return

        origin_x = float(tf_msg.transform.translation.x)
        origin_y = float(tf_msg.transform.translation.y)
        origin_z = float(tf_msg.transform.translation.z)

        qx = float(tf_msg.transform.rotation.x)
        qy = float(tf_msg.transform.rotation.y)
        qz = float(tf_msg.transform.rotation.z)
        qw = float(tf_msg.transform.rotation.w)

        top_v = float(self.img_h) * self.fov_top_crop_ratio
        bottom_v = float(self.img_h - 1)

        corners = [
            (0.0, top_v),
            (float(self.img_w - 1), top_v),
            (float(self.img_w - 1), bottom_v),
            (0.0, bottom_v),
        ]

        ground_pts = []

        for u, v in corners:
            x_opt = (u - self.cx) / self.fx
            y_opt = (v - self.cy) / self.fy
            z_opt = 1.0

            dx, dy, dz = rotate_vector_by_quaternion(x_opt, y_opt, z_opt, qx, qy, qz, qw)

            if abs(dz) < 1e-6:
                return

            t = -origin_z / dz
            if t <= 0:
                return

            px = origin_x + t * dx
            py = origin_y + t * dy
            pz = self.fov_ground_z

            planar_dist = math.sqrt((px - origin_x) ** 2 + (py - origin_y) ** 2)
            if planar_dist > self.fov_max_range_m:
                scale = self.fov_max_range_m / planar_dist
                px = origin_x + (px - origin_x) * scale
                py = origin_y + (py - origin_y) * scale

            ground_pts.append((px, py, pz))

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.target_frame
        marker.ns = 'camera_fov_ground'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        ordered = [ground_pts[0], ground_pts[1], ground_pts[2], ground_pts[3], ground_pts[0]]
        for x, y, z in ordered:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            marker.points.append(p)

        self.fov_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CloudDebugFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
