import math
import json
import time

import cv2
import numpy as np
from PIL import Image
from urllib.request import urlopen
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from avoidance_rerouting.vision_detection_core import (
    load_model,
    detect_frame,
    INFER_WIDTH,
    KNOWN_OBJECT_HEIGHTS,
    DEFAULT_HEIGHT,
)


def fetch_jpeg(url):
    """Fetch a single JPEG frame from an HTTP endpoint."""
    resp = urlopen(url, timeout=5)
    data = resp.read()
    arr = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return frame


class VisionDetectionNode(Node):
    def __init__(self):
        super().__init__('vision_detection_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_url', '')
        self.declare_parameter('detection_rate', 5.0)
        self.declare_parameter('text_prompt', 'iphone.')

        camera_index = self.get_parameter('camera_index').value
        self.camera_url = self.get_parameter('camera_url').value
        detection_rate = self.get_parameter('detection_rate').value
        self.text_prompt = self.get_parameter('text_prompt').value

        self.publisher = self.create_publisher(String, '/vision_detections', 10)

        self.get_logger().info('Loading GroundingDINO model...')
        self.processor, self.model, self.device = load_model()
        self.get_logger().info(f'Model loaded on {self.device}')

        # Use HTTP snapshot mode if camera_url points to /frame endpoint
        self.use_http = bool(self.camera_url)
        self.cap = None

        if self.use_http:
            # Ensure URL ends with /frame for snapshot mode
            self.snapshot_url = self.camera_url.rstrip('/') + '/frame'
            self.get_logger().info(f'Using HTTP snapshot mode: {self.snapshot_url}')
            # Fetch one frame to get dimensions
            frame = fetch_jpeg(self.snapshot_url)
            if frame is None:
                self.get_logger().error(f'Cannot fetch frame from {self.snapshot_url}')
                return
            self.frame_h, self.frame_w = frame.shape[:2]
        else:
            self.cap = cv2.VideoCapture(camera_index)
            if not self.cap.isOpened():
                self.get_logger().error(f'Cannot open camera {camera_index}')
                return
            self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.get_logger().info(f'Camera opened: {self.frame_w}x{self.frame_h}')

        self.frame_count = 0
        self.timer = self.create_timer(1.0 / detection_rate, self.detect_callback)

    def detect_callback(self):
        t0 = time.monotonic()

        if self.use_http:
            try:
                frame = fetch_jpeg(self.snapshot_url)
            except Exception as e:
                self.get_logger().warn(f'HTTP fetch failed: {e}')
                return
            if frame is None:
                self.get_logger().warn('Failed to decode HTTP frame')
                return
        else:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn('Failed to read frame')
                return

        t_read = time.monotonic() - t0

        if INFER_WIDTH and self.frame_w > INFER_WIDTH:
            scale = INFER_WIDTH / self.frame_w
            small = cv2.resize(frame, (INFER_WIDTH, int(self.frame_h * scale)))
            pil_img = Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
        else:
            pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        # Save frame to disk for debugging
        cv2.imwrite('/workspace/debug_frame.jpg', frame)

        dets = detect_frame(
            pil_img,
            (self.frame_h, self.frame_w),
            self.processor,
            self.model,
            self.device,
            self.text_prompt,
        )

        t_infer = time.monotonic() - t0 - t_read
        self.frame_count += 1
        self.get_logger().info(
            f'Frame {self.frame_count}: read={t_read:.2f}s infer={t_infer:.2f}s total={time.monotonic()-t0:.2f}s dets={len(dets)}'
        )

        if dets:
            for d in dets:
                self.get_logger().info(
                    f'  -> {d["class_name"]} conf={d["confidence"]:.3f} '
                    f'dist={d["distance_m"]} angle={d.get("angle_rad")}'
                )

        payload = []
        for d in dets:
            if d["distance_m"] is None:
                continue
            class_name = d["class_name"].strip().lower()
            height = KNOWN_OBJECT_HEIGHTS.get(class_name, DEFAULT_HEIGHT)
            width = d["width_m"] if d["width_m"] is not None else 0.3
            payload.append({
                "class_name": class_name,
                "distance_m": d["distance_m"],
                "angle_deg": math.degrees(d["angle_rad"]),
                "width_m": width,
                "height_m": height,
                "confidence": d["confidence"],
            })

        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)

        if payload:
            self.get_logger().info(f'Published {len(payload)} detections')

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
