import math
import torch
import cv2
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

MODEL_ID = "IDEA-Research/grounding-dino-tiny"
INFER_WIDTH = 480
CONFIDENCE_THRESHOLD = 0.5
CAMERA_HFOV_DEG = 60.0

KNOWN_OBJECT_WIDTHS = {
    "iphone": 0.08,
    "phone": 0.08,
    "cell phone": 0.08,
    "smartphone": 0.08,
}

KNOWN_OBJECT_HEIGHTS = {
    "iphone": 0.15,
    "phone": 0.15,
    "cell phone": 0.15,
    "smartphone": 0.15,
}

DEFAULT_HEIGHT = 0.5


def load_model(device=None):
    if device is None:
        if torch.cuda.is_available():
            device = "cuda"
        elif torch.backends.mps.is_available():
            device = "mps"
        else:
            device = "cpu"
    processor = AutoProcessor.from_pretrained(MODEL_ID)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(MODEL_ID).to(device)
    model.eval()
    return processor, model, device


def compute_focal_length_px(image_width, hfov_deg):
    return (image_width / 2.0) / math.tan(math.radians(hfov_deg / 2.0))


def estimate_spatial(det, image_width, focal_length_px):
    x1, _, x2, _ = det["bbox_xyxy"]
    bbox_pixel_width = x2 - x1
    bbox_center_x = (x1 + x2) / 2.0
    image_center_x = image_width / 2.0

    angle_rad = math.atan2(bbox_center_x - image_center_x, focal_length_px)

    class_name = det["class_name"].strip().lower()
    known_width = KNOWN_OBJECT_WIDTHS.get(class_name)

    distance_m = None
    if known_width is not None and bbox_pixel_width > 0:
        distance_m = (known_width * focal_length_px) / bbox_pixel_width

    return {
        "distance_m": distance_m,
        "angle_rad": angle_rad,
        "width_m": known_width,
    }


def detect_frame(pil_img, original_size, processor, model, device, text_prompt):
    inputs = processor(images=pil_img, text=text_prompt, return_tensors="pt").to(device)

    with torch.no_grad():
        outputs = model(**inputs)

    results = processor.post_process_grounded_object_detection(
        outputs,
        inputs["input_ids"],
        target_sizes=[original_size],
    )[0]

    boxes = results["boxes"].cpu().float().numpy().tolist()
    scores = results["scores"].cpu().float().numpy().tolist()
    labels = results["labels"]

    img_width = original_size[1]
    focal_length_px = compute_focal_length_px(img_width, CAMERA_HFOV_DEG)

    detections = []
    for i, (b, s, lab) in enumerate(zip(boxes, scores, labels)):
        if s < CONFIDENCE_THRESHOLD:
            continue
        x1, y1, x2, y2 = map(float, b)
        u = (x1 + x2) / 2.0
        v = float(y2)
        det = {
            "id": f"det_{i}",
            "class_name": lab,
            "confidence": float(s),
            "bbox_xyxy": [x1, y1, x2, y2],
            "pixel_uv": [u, v],
        }
        spatial = estimate_spatial(det, img_width, focal_length_px)
        det["distance_m"] = spatial["distance_m"]
        det["angle_rad"] = spatial["angle_rad"]
        det["width_m"] = spatial["width_m"]
        detections.append(det)
    return detections
