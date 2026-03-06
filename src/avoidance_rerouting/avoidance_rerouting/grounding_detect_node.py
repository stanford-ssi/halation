"""
Vision Detection Node
=====================
Wraps GroundingDINO into a ROS2 node.
Publishes detected obstacles to /obstacle_detections.

Published Topics:
  /obstacle_detections  (std_msgs/String)  JSON obstacle list

Run:
    ros2 run avoidance_rerouting vision_detection
"""

import json
import math
import threading
from typing import List, Dict, Any, Optional

import cv2
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── Detection config ──────────────────────────────────────────────────────────
CAMERA_INDEX       = 0          # webcam index, or change to video path
DETECT_EVERY_N     = 2          # run inference every N frames
INFER_WIDTH        = 480        # resize width for faster inference
CONFIDENCE_THRESHOLD = 0.35     # ignore detections below this
CAMERA_HFOV_DEG    = 60.0       # horizontal field of view of your camera

# Text prompt — comma-separated object classes to detect
DETECT_CLASSES = [
    "person", "rock", "boulder", "cone", "box",
    "barrier", "wall", "vehicle", "obstacle",
]

# Known real-world widths (metres) — used for distance estimation
# Add more objects here as you identify what the rover encounters
KNOWN_OBJECT_WIDTHS = {
    "person":   0.5,
    "rock":     0.4,
    "boulder":  0.8,
    "cone":     0.3,
    "box":      0.4,
    "barrier":  0.8,
    "wall":     1.0,
    "vehicle":  1.8,
    "obstacle": 0.5,
    "iphone":   0.08,
}


def compute_focal_length_px(image_width: int, hfov_deg: float) -> float:
    return (image_width / 2.0) / math.tan(math.radians(hfov_deg / 2.0))


def estimate_spatial(det: Dict, image_width: int, focal_length_px: float) -> Dict:
    """
    Given a detection bounding box, compute:
      - angle_rad: horizontal angle from camera centre (negative=left, positive=right)
      - distance_m: estimated distance if object width is known, else None
    """
    x1, _, x2, _ = det["bbox_xyxy"]
    bbox_pixel_width = x2 - x1
    bbox_center_x    = (x1 + x2) / 2.0
    image_center_x   = image_width / 2.0

    angle_rad  = math.atan2(bbox_center_x - image_center_x, focal_length_px)
    class_name = det["class_name"].strip().lower()

    # Try exact match first, then partial match
    known_width = KNOWN_OBJECT_WIDTHS.get(class_name)
    if known_width is None:
        for key, w in KNOWN_OBJECT_WIDTHS.items():
            if key in class_name or class_name in key:
                known_width = w
                break

    distance_m = None
    if known_width is not None and bbox_pixel_width > 0:
        distance_m = (known_width * focal_length_px) / bbox_pixel_width

    return {
        "angle_rad":  angle_rad,
        "distance_m": distance_m,
        "width_m":    known_width,
    }


class VisionDetectionNode(Node):
    """ROS2 node that runs GroundingDINO and publishes obstacle detections."""

    def __init__(self):
        super().__init__("vision_detection")

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(String, "/obstacle_detections", 10)

        # ── Load model ────────────────────────────────────────────────────────
        if torch.cuda.is_available():
            self.device = "cuda"
        elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
            self.device = "mps"
        else:
            self.device = "cpu"
        self.get_logger().info(f"Loading GroundingDINO on {self.device} ...")

        model_id = "IDEA-Research/grounding-dino-tiny"
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
            model_id
        ).to(self.device)
        self.model.eval()

        self.text_prompt = ". ".join(DETECT_CLASSES) + "."
        self.get_logger().info(f"Text prompt: {self.text_prompt}")

        # ── Camera ────────────────────────────────────────────────────────────
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera index {CAMERA_INDEX}")
            raise RuntimeError("Camera not available")

        self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.focal_length_px = compute_focal_length_px(self.frame_w, CAMERA_HFOV_DEG)
        self.get_logger().info(
            f"Camera {self.frame_w}x{self.frame_h}, "
            f"focal_px={self.focal_length_px:.1f}"
        )

        # ── Detection state ───────────────────────────────────────────────────
        self._detect_lock     = threading.Lock()
        self._latest_dets: List[Dict] = []
        self._detect_busy     = False
        self._frame_count     = 0

        # ── Timer — capture + publish at ~15 Hz ───────────────────────────────
        self.create_timer(1.0 / 15.0, self._timer_cb)
        self.get_logger().info("Vision detection node ready.")

    # ── Main timer ────────────────────────────────────────────────────────────

    def _timer_cb(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Camera read failed.", throttle_duration_sec=5.0)
            return

        self._frame_count += 1

        # Kick off background inference every N frames
        if not self._detect_busy and self._frame_count % DETECT_EVERY_N == 0:
            self._detect_busy = True
            t = threading.Thread(
                target=self._infer_worker,
                args=(frame.copy(),),
                daemon=True,
            )
            t.start()

        # Always publish the latest detections (may be from a previous frame)
        with self._detect_lock:
            dets = list(self._latest_dets)
        self._publish(dets)

    # ── Inference (runs in background thread) ────────────────────────────────

    def _infer_worker(self, frame_bgr):
        try:
            # Resize for faster inference
            if INFER_WIDTH and self.frame_w > INFER_WIDTH:
                scale = INFER_WIDTH / self.frame_w
                small = cv2.resize(
                    frame_bgr,
                    (INFER_WIDTH, int(self.frame_h * scale))
                )
            else:
                small = frame_bgr

            pil_img = Image.fromarray(cv2.cvtColor(small, cv2.COLOR_BGR2RGB))

            inputs = self.processor(
                images=pil_img,
                text=self.text_prompt,
                return_tensors="pt",
            ).to(self.device)

            with torch.no_grad():
                outputs = self.model(**inputs)

            results = self.processor.post_process_grounded_object_detection(
                outputs,
                inputs["input_ids"],
                target_sizes=[(self.frame_h, self.frame_w)],
            )[0]

            boxes  = results["boxes"].cpu().float().numpy().tolist()
            scores = results["scores"].cpu().float().numpy().tolist()
            labels = results["labels"]

            dets = []
            for b, s, lab in zip(boxes, scores, labels):
                if s < CONFIDENCE_THRESHOLD:
                    continue
                x1, y1, x2, y2 = map(float, b)
                det = {
                    "class_name": str(lab),
                    "confidence": float(s),
                    "bbox_xyxy":  [x1, y1, x2, y2],
                }
                spatial = estimate_spatial(det, self.frame_w, self.focal_length_px)
                det.update(spatial)
                dets.append(det)

            with self._detect_lock:
                self._latest_dets = dets

        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
        finally:
            self._detect_busy = False

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish(self, dets: List[Dict]):
        """
        Publish detections that have a valid distance and angle.
        Format matches what routing_simulation.py and local_avoidance expect.
        """
        obstacles = []
        for d in dets:
            if d.get("distance_m") is None:
                continue
            if d.get("angle_rad") is None:
                continue
            obstacles.append({
                "class_name":  d["class_name"],
                "confidence":  round(d["confidence"], 3),
                "angle_rad":   round(d["angle_rad"], 4),
                "distance_m":  round(d["distance_m"], 3),
                "width_m":     d.get("width_m"),
            })

        msg = String()
        msg.data = json.dumps({"obstacles": obstacles})
        self.pub.publish(msg)

        if obstacles:
            self.get_logger().info(
                f"{len(obstacles)} obstacle(s): " +
                ", ".join(
                    f"{o['class_name']} {o['distance_m']:.1f}m "
                    f"{math.degrees(o['angle_rad']):.0f}deg"
                    for o in obstacles
                ),
                throttle_duration_sec=1.0,
            )

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()