import math
import json

import cv2
from PIL import Image
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


class VisionDetectionNode(Node):
    def __init__(self):
        super().__init__('vision_detection_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('detection_rate', 5.0)
        self.declare_parameter('text_prompt', 'iphone.')

        camera_index = self.get_parameter('camera_index').value
        detection_rate = self.get_parameter('detection_rate').value
        self.text_prompt = self.get_parameter('text_prompt').value

        self.publisher = self.create_publisher(String, '/vision_detections', 10)

        self.get_logger().info('Loading GroundingDINO model...')
        self.processor, self.model, self.device = load_model()
        self.get_logger().info(f'Model loaded on {self.device}')

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera {camera_index}')
            return

        self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'Camera opened: {self.frame_w}x{self.frame_h}')

        self.timer = self.create_timer(1.0 / detection_rate, self.detect_callback)

    def detect_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        if INFER_WIDTH and self.frame_w > INFER_WIDTH:
            scale = INFER_WIDTH / self.frame_w
            small = cv2.resize(frame, (INFER_WIDTH, int(self.frame_h * scale)))
            pil_img = Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))
        else:
            pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        dets = detect_frame(
            pil_img,
            (self.frame_h, self.frame_w),
            self.processor,
            self.model,
            self.device,
            self.text_prompt,
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
