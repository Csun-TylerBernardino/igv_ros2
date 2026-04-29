import os
import cv2
import numpy as np
from PIL import Image

import torch
import torch.nn.functional as F
from torchvision import transforms as TF
from transformers import SegformerConfig, SegformerForSemanticSegmentation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import PointCloud2, CameraInfo
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from igv_interfaces.msg import LaneEstimate


DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
INPUT_SIZE = (360, 640)
LANE_THRESHOLD = 0.015
ROI_TOP_RATIO = 0.35

CAMERA_HEIGHT_M = 0.9144
CAMERA_PITCH_DEG = 30.0
CAMERA_X_OFFSET_M = 0.2718
CAMERA_Y_OFFSET_M = 0.0718
GROUND_Z_OFFSET_M = -0.04


def build_model(model_path):
    config = SegformerConfig(
        num_labels=2,
        id2label={0: "background", 1: "lane"},
        label2id={"background": 0, "lane": 1},
        depths=[3, 4, 6, 3],
        hidden_sizes=[64, 128, 320, 512],
        decoder_hidden_size=768,
        num_attention_heads=[1, 2, 5, 8],
        patch_sizes=[7, 3, 3, 3],
        strides=[4, 2, 2, 2],
        sr_ratios=[8, 4, 2, 1],
        mlp_ratios=[4, 4, 4, 4],
        reshape_last_stage=True,
        drop_path_rate=0.1,
    )

    model = SegformerForSemanticSegmentation(config)

    checkpoint = torch.load(model_path, map_location=DEVICE)

    if "model_state_dict" in checkpoint:
        state_dict = checkpoint["model_state_dict"]
    else:
        state_dict = checkpoint

    model.load_state_dict(state_dict, strict=True)
    model.to(DEVICE)
    model.eval()
    return model


def get_transform():
    return TF.Compose([
        TF.Resize(INPUT_SIZE),
        TF.ToTensor(),
        TF.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        ),
    ])


def apply_clahe_gray(gray, clip_limit=2.0, tile_grid_size=(8, 8)):
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    return clahe.apply(gray)


def preprocess_frame(frame_bgr, transform):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    clahe_gray = apply_clahe_gray(gray, clip_limit=2.0, tile_grid_size=(8, 8))
    gray_3ch = cv2.cvtColor(clahe_gray, cv2.COLOR_GRAY2RGB)

    pil_img = Image.fromarray(gray_3ch)
    x = transform(pil_img).unsqueeze(0).to(DEVICE)

    return x, gray, clahe_gray


def predict_mask(model, x, out_h, out_w):
    with torch.no_grad():
        outputs = model(pixel_values=x, return_dict=True)
        logits = F.interpolate(
            outputs["logits"],
            size=(out_h, out_w),
            mode="bilinear",
            align_corners=False,
        )

        probs = torch.softmax(logits, dim=1)
        lane_prob = probs[:, 1, :, :].squeeze(0).cpu().numpy()

    mask = (lane_prob > LANE_THRESHOLD).astype(np.uint8)
    mask[:int(out_h * ROI_TOP_RATIO), :] = 0

    kernel_close = np.ones((19, 19), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

    kernel_open = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

    kernel_dilate = np.ones((7, 7), np.uint8)
    mask = cv2.dilate(mask, kernel_dilate, iterations=2)

    return mask, lane_prob


def overlay_mask(frame_bgr, mask):
    color_mask = np.zeros_like(frame_bgr)
    color_mask[mask > 0] = [0, 0, 255]
    return cv2.addWeighted(frame_bgr, 0.35, color_mask, 1.0, 0)


def compute_lane_estimate(mask, lane_prob):
    h, w = mask.shape[:2]

    y0 = int(h * 0.60)
    roi_mask = mask[y0:, :]
    roi_prob = lane_prob[y0:, :]

    ys, xs = np.where(roi_mask > 0)

    if len(xs) < 200:
        return False, 0.0, 0.0, 0.0, 3.0

    lane_center_x = float(np.mean(xs))
    image_center_x = w / 2.0
    lateral_error_px = image_center_x - lane_center_x

    lateral_error_m = (lateral_error_px / (w / 2.0)) * 1.5

    heading_error_rad = 0.0
    if len(xs) > 50:
        ys_full = ys.astype(np.float32)
        xs_full = xs.astype(np.float32)
        fit = np.polyfit(ys_full, xs_full, 1)
        slope_dx_dy = fit[0]
        heading_error_rad = float(np.arctan(slope_dx_dy))

    lane_pixels = roi_prob[roi_mask > 0]
    confidence = float(np.clip(np.mean(lane_pixels), 0.0, 1.0)) if lane_pixels.size > 0 else 0.0

    return True, confidence, float(lateral_error_m), heading_error_rad, 3.0


class SegformerLaneNode(Node):
    def __init__(self):
        super().__init__('segformer_lane_node')

        self.bridge = CvBridge()
        self.lane_pub = self.create_publisher(LaneEstimate, '/lane_estimate', 10)
        self.lane_cloud_pub = self.create_publisher(PointCloud2, '/lanes/point_cloud', 10)

        package_share = get_package_share_directory('igv_perception')
        model_path = os.path.join(package_share, 'models', 'best_modelV.2.pth')

        self.get_logger().info(f'Using device: {DEVICE}')
        self.get_logger().info(f'Loading local model from: {model_path}')

        self.model = build_model(model_path)
        self.transform = get_transform()

        self.processing = False
        self.frame_count = 0

        self.have_camera_info = False
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.image_sub = self.create_subscription(
            RosImage,
            '/zed/zed_node/rgb/color/rect/image',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/color/rect/image/camera_info',
            self.camera_info_callback,
            10
        )

        cv2.namedWindow("Original Input", cv2.WINDOW_NORMAL)
        cv2.namedWindow("CLAHE Gray Input", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Lane Detection", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)

        cv2.resizeWindow("Original Input", 480, 270)
        cv2.resizeWindow("CLAHE Gray Input", 480, 270)
        cv2.resizeWindow("Lane Detection", 480, 270)
        cv2.resizeWindow("Mask", 480, 270)

        self.get_logger().info('Segformer lane node started')

    def camera_info_callback(self, msg):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.have_camera_info = True

    def publish_lane_cloud(self, mask, image_header):
        if not self.have_camera_info:
            return

        h, w = mask.shape[:2]
        ys, xs = np.where(mask > 0)

        header = Header()
        header.stamp = image_header.stamp
        header.frame_id = 'base_link'

        if len(xs) == 0:
            empty_cloud = point_cloud2.create_cloud_xyz32(header, [])
            self.lane_cloud_pub.publish(empty_cloud)
            return

        sample_step = 10
        xs = xs[::sample_step]
        ys = ys[::sample_step]

        theta = np.deg2rad(CAMERA_PITCH_DEG)
        c = np.cos(theta)
        s = np.sin(theta)

        points = []

        for u, v in zip(xs.tolist(), ys.tolist()):
            x_opt = (float(u) - self.cx) / self.fx
            y_opt = (float(v) - self.cy) / self.fy
            z_opt = 1.0

            dx0 = z_opt
            dy0 = -x_opt
            dz0 = -y_opt

            dx = c * dx0 + s * dz0
            dy = dy0
            dz = -s * dx0 + c * dz0

            if dz >= -1e-6:
                continue

            t = CAMERA_HEIGHT_M / (-dz)

            px = CAMERA_X_OFFSET_M + t * dx
            py = CAMERA_Y_OFFSET_M + t * dy
            pz = GROUND_Z_OFFSET_M

            if 0.0 < px < 15.0 and abs(py) < 5.0:
                points.append((float(px), float(py), float(pz)))

        lane_cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.lane_cloud_pub.publish(lane_cloud_msg)

    def image_callback(self, msg):
        if self.processing:
            return

        self.processing = True
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame_bgr.shape[:2]

            x, gray, clahe_gray = preprocess_frame(frame_bgr, self.transform)
            mask, lane_prob = predict_mask(self.model, x, h, w)
            vis = overlay_mask(frame_bgr, mask)

            valid, confidence, lateral_error_m, heading_error_rad, lane_width_m = compute_lane_estimate(mask, lane_prob)

            lane_msg = LaneEstimate()
            lane_msg.header.stamp = msg.header.stamp
            lane_msg.header.frame_id = 'base_link'
            lane_msg.valid = valid
            lane_msg.confidence = confidence
            lane_msg.lateral_error_m = lateral_error_m
            lane_msg.heading_error_rad = heading_error_rad
            lane_msg.lane_width_m = lane_width_m
            lane_msg.stop_bar_detected = False
            lane_msg.stop_bar_distance_m = 0.0
            self.lane_pub.publish(lane_msg)

            self.publish_lane_cloud(mask, msg.header)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                lane_pixels = int(mask.sum())
                self.get_logger().info(
                    f'lane_pixels={lane_pixels} valid={valid} conf={confidence:.3f} '
                    f'lat_err={lateral_error_m:.3f} head_err={heading_error_rad:.3f}'
                )

            cv2.imshow("Original Input", frame_bgr)
            cv2.imshow("CLAHE Gray Input", clahe_gray)
            cv2.imshow("Lane Detection", vis)
            cv2.imshow("Mask", mask * 255)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Lane processing error: {e}')

        self.processing = False


def main(args=None):
    rclpy.init(args=args)
    node = SegformerLaneNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
