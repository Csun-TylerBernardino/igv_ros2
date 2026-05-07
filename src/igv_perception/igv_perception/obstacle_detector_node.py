import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker

from igv_interfaces.msg import ObstacleSummary


def get_xyz_offsets(cloud_msg):
    x_offset = None
    y_offset = None
    z_offset = None

    for field in cloud_msg.fields:
        if field.name == 'x':
            x_offset = int(field.offset)
        elif field.name == 'y':
            y_offset = int(field.offset)
        elif field.name == 'z':
            z_offset = int(field.offset)

    if x_offset is None or y_offset is None or z_offset is None:
        raise ValueError('PointCloud2 missing x/y/z fields')

    return x_offset, y_offset, z_offset


def cloud_to_xyz_arrays(cloud_msg):
    x_offset, y_offset, z_offset = get_xyz_offsets(cloud_msg)

    endian = '>' if cloud_msg.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + 'f4', endian + 'f4', endian + 'f4'],
        'offsets': [x_offset, y_offset, z_offset],
        'itemsize': int(cloud_msg.point_step),
    })

    arr = np.frombuffer(cloud_msg.data, dtype=dtype, count=int(cloud_msg.width) * int(cloud_msg.height))
    arr = arr.reshape((int(cloud_msg.height), int(cloud_msg.width)))

    return arr['x'].astype(np.float32), arr['y'].astype(np.float32), arr['z'].astype(np.float32)


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.bridge = CvBridge()

        # Current stack outputs we want to preserve
        self.summary_pub = self.create_publisher(ObstacleSummary, '/obstacle_summary', 10)
        self.box_marker_pub = self.create_publisher(Marker, '/obstacles/box', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/obstacles/points', 10)

        # New debug image outputs
        self.debug_boxed_pub = self.create_publisher(RosImage, '/debug/obstacle_boxed', 10)
        self.debug_mask_pub = self.create_publisher(RosImage, '/debug/obstacle_mask', 10)
        self.debug_depth_pub = self.create_publisher(RosImage, '/debug/obstacle_depth_preview', 10)

        # Keep launch compatibility with old params
        self.declare_parameter('box_frame', 'base_link')
        self.declare_parameter('min_x', 0.6)
        self.declare_parameter('max_x', 1.5)
        self.declare_parameter('min_y', -0.35)
        self.declare_parameter('max_y', 0.35)
        self.declare_parameter('min_z', 0.10)
        self.declare_parameter('max_z', 0.80)
        self.declare_parameter('min_points', 80)

        # New custom detector params
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/color/rect/image')
        self.declare_parameter('cloud_topic', '/zed/zed_node/point_cloud/cloud_registered')

        self.declare_parameter('min_distance_m', 0.2)
        self.declare_parameter('max_distance_m', 1.524)   # 5 ft
        self.declare_parameter('min_box_area', 1200)
        self.declare_parameter('box_padding', 8)

        self.declare_parameter('use_roi', True)
        self.declare_parameter('roi_top_percent', 0.50)

        self.declare_parameter('point_sample_step', 4)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.cloud_topic = str(self.get_parameter('cloud_topic').value)

        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        self.max_distance_m = float(self.get_parameter('max_distance_m').value)
        self.min_box_area = int(self.get_parameter('min_box_area').value)
        self.box_padding = int(self.get_parameter('box_padding').value)

        self.use_roi = bool(self.get_parameter('use_roi').value)
        self.roi_top_percent = float(self.get_parameter('roi_top_percent').value)
        self.point_sample_step = int(self.get_parameter('point_sample_step').value)

        self.latest_cloud_msg = None
        self.latest_x = None
        self.latest_y = None
        self.latest_z = None

        self.image_sub = self.create_subscription(
            RosImage,
            self.image_topic,
            self.image_callback,
            10
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            10
        )

        self.get_logger().info('Custom obstacle detector node started')
        self.get_logger().info(
            f'image_topic={self.image_topic}, cloud_topic={self.cloud_topic}, '
            f'min_distance_m={self.min_distance_m}, max_distance_m={self.max_distance_m}'
        )

    def cloud_callback(self, msg):
        self.latest_cloud_msg = msg
        try:
            self.latest_x, self.latest_y, self.latest_z = cloud_to_xyz_arrays(msg)
        except Exception as e:
            self.latest_cloud_msg = None
            self.latest_x = None
            self.latest_y = None
            self.latest_z = None
            self.get_logger().error(f'Cloud parse error: {e}')

    def publish_empty_marker(self, header):
        marker = Marker()
        marker.header = header
        marker.ns = 'obstacle_detector'
        marker.id = 0
        marker.action = Marker.DELETE
        self.box_marker_pub.publish(marker)

    def publish_box_marker(self, header, points_xyz):
        if points_xyz.shape[0] == 0:
            self.publish_empty_marker(header)
            return

        mins = np.min(points_xyz, axis=0)
        maxs = np.max(points_xyz, axis=0)
        center = (mins + maxs) / 2.0
        scale = np.maximum(maxs - mins, np.array([0.05, 0.05, 0.05], dtype=np.float32))

        marker = Marker()
        marker.header = header
        marker.ns = 'obstacle_detector'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.35

        self.box_marker_pub.publish(marker)

    def publish_points(self, header, points_xyz):
        if points_xyz.shape[0] == 0:
            cloud_msg = point_cloud2.create_cloud_xyz32(header, [])
        else:
            cloud_msg = point_cloud2.create_cloud_xyz32(header, points_xyz.tolist())
        self.points_pub.publish(cloud_msg)

    def publish_debug_images(self, header, boxed_image, obstacle_mask, depth_preview):
        boxed_msg = self.bridge.cv2_to_imgmsg(boxed_image, encoding='bgr8')
        boxed_msg.header = header
        self.debug_boxed_pub.publish(boxed_msg)

        mask_msg = self.bridge.cv2_to_imgmsg(obstacle_mask, encoding='mono8')
        mask_msg.header = header
        self.debug_mask_pub.publish(mask_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_preview, encoding='bgr8')
        depth_msg.header = header
        self.debug_depth_pub.publish(depth_msg)

    def image_callback(self, msg):
        if self.latest_cloud_msg is None or self.latest_z is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image convert error: {e}')
            return

        boxed_image = image.copy()
        img_h, img_w = image.shape[:2]
        cloud_h, cloud_w = self.latest_z.shape[:2]

        # resize Z to image size for 2D mask logic if needed
        if (cloud_w, cloud_h) != (img_w, img_h):
            z_for_mask = cv2.resize(self.latest_z, (img_w, img_h), interpolation=cv2.INTER_NEAREST)
        else:
            z_for_mask = self.latest_z

        valid_depth = np.isfinite(z_for_mask)

        obstacle_mask = (
            valid_depth &
            (z_for_mask >= self.min_distance_m) &
            (z_for_mask <= self.max_distance_m)
        ).astype(np.uint8) * 255

        if self.use_roi:
            roi_start_y = int(img_h * self.roi_top_percent)
            obstacle_mask[:roi_start_y, :] = 0

        kernel = np.ones((5, 5), np.uint8)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        obstacle_mask = cv2.dilate(obstacle_mask, kernel, iterations=2)

        contours, _ = cv2.findContours(
            obstacle_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        all_points_xyz = []
        nearest_points_xyz = np.empty((0, 3), dtype=np.float32)
        nearest_distance_m = float('inf')
        nearest_bearing_rad = 0.0
        found_obstacle = False

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_box_area:
                continue

            x, y, bw, bh = cv2.boundingRect(contour)

            x1 = max(x - self.box_padding, 0)
            y1 = max(y - self.box_padding, 0)
            x2 = min(x + bw + self.box_padding, img_w - 1)
            y2 = min(y + bh + self.box_padding, img_h - 1)

            # map image ROI -> cloud ROI
            cx1 = int(round(x1 * (cloud_w - 1) / max(img_w - 1, 1)))
            cy1 = int(round(y1 * (cloud_h - 1) / max(img_h - 1, 1)))
            cx2 = int(round(x2 * (cloud_w - 1) / max(img_w - 1, 1)))
            cy2 = int(round(y2 * (cloud_h - 1) / max(img_h - 1, 1)))

            cx1 = np.clip(cx1, 0, cloud_w - 1)
            cy1 = np.clip(cy1, 0, cloud_h - 1)
            cx2 = np.clip(cx2, 0, cloud_w - 1)
            cy2 = np.clip(cy2, 0, cloud_h - 1)

            if cx2 <= cx1 or cy2 <= cy1:
                continue

            box_x = self.latest_x[cy1:cy2 + 1, cx1:cx2 + 1]
            box_y = self.latest_y[cy1:cy2 + 1, cx1:cx2 + 1]
            box_z = self.latest_z[cy1:cy2 + 1, cx1:cx2 + 1]

            box_valid_mask = (
                np.isfinite(box_x) &
                np.isfinite(box_y) &
                np.isfinite(box_z) &
                (box_z >= self.min_distance_m) &
                (box_z <= self.max_distance_m)
            )

            if not np.any(box_valid_mask):
                continue

            box_valid_depth = box_z[box_valid_mask]
            distance_m = float(np.median(box_valid_depth))
            distance_ft = distance_m * 3.28084

            label = f"Obstacle {distance_ft:.1f} ft"

            cv2.rectangle(boxed_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.rectangle(boxed_image, (x1, max(y1 - 25, 0)), (min(x1 + 190, img_w - 1), y1), (0, 255, 0), -1)
            cv2.putText(
                boxed_image,
                label,
                (x1 + 5, max(y1 - 7, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                2
            )

            sampled_x = box_x[::self.point_sample_step, ::self.point_sample_step]
            sampled_y = box_y[::self.point_sample_step, ::self.point_sample_step]
            sampled_z = box_z[::self.point_sample_step, ::self.point_sample_step]
            sampled_valid = box_valid_mask[::self.point_sample_step, ::self.point_sample_step]

            sampled_points = np.stack(
                [sampled_x[sampled_valid], sampled_y[sampled_valid], sampled_z[sampled_valid]],
                axis=1
            ).astype(np.float32)

            if sampled_points.shape[0] == 0:
                continue

            found_obstacle = True
            all_points_xyz.append(sampled_points)

            if distance_m < nearest_distance_m:
                nearest_distance_m = distance_m
                nearest_points_xyz = sampled_points
                med_x = float(np.median(sampled_points[:, 0]))
                med_z = float(np.median(sampled_points[:, 2]))
                nearest_bearing_rad = float(math.atan2(med_x, med_z))

        if all_points_xyz:
            all_points_xyz = np.concatenate(all_points_xyz, axis=0)
        else:
            all_points_xyz = np.empty((0, 3), dtype=np.float32)

        depth_near = np.where(
            valid_depth &
            (z_for_mask >= self.min_distance_m) &
            (z_for_mask <= self.max_distance_m),
            z_for_mask,
            0
        )

        if np.any(depth_near > 0):
            depth_preview = cv2.normalize(depth_near, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        else:
            depth_preview = np.zeros_like(obstacle_mask, dtype=np.uint8)

        depth_preview = cv2.applyColorMap(depth_preview, cv2.COLORMAP_JET)

        points_header = Header()
        points_header.stamp = msg.header.stamp
        points_header.frame_id = self.latest_cloud_msg.header.frame_id

        self.publish_points(points_header, all_points_xyz)

        if found_obstacle:
            self.publish_box_marker(points_header, nearest_points_xyz)
        else:
            self.publish_empty_marker(points_header)

        summary = ObstacleSummary()
        summary.header.stamp = msg.header.stamp
        summary.header.frame_id = self.latest_cloud_msg.header.frame_id

        if found_obstacle:
            summary.obstacle_ahead = True
            summary.nearest_obstacle_distance_m = float(nearest_distance_m)
            summary.nearest_obstacle_bearing_rad = float(nearest_bearing_rad)
            summary.lane_blocked = True
            summary.left_lane_open = False
            summary.right_lane_open = False
        else:
            summary.obstacle_ahead = False
            summary.nearest_obstacle_distance_m = 999.0
            summary.nearest_obstacle_bearing_rad = 0.0
            summary.lane_blocked = False
            summary.left_lane_open = True
            summary.right_lane_open = True

        self.summary_pub.publish(summary)
        self.publish_debug_images(msg.header, boxed_image, obstacle_mask, depth_preview)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

