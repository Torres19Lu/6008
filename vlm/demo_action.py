import json
import sys

import cv2
from vlfm.vlm.server_wrapper import send_request
from vlfm.vlm.yolov7 import YOLOv7Client


if len(sys.argv) < 3:
    print("用法: python demo_action.py 图片路径 目标名称")
    sys.exit(1)

image_path = sys.argv[1]
target = sys.argv[2].lower()

image = cv2.imread(image_path)
if image is None:
    print("错误：无法读取图片：", image_path)
    sys.exit(1)

# 第一步：YOLO 找到目标的位置
detector = YOLOv7Client(port=12184)
detections = detector.predict(image)
detections.filter_by_class([target])
detections.filter_by_conf(0.25)

result = {
    "target": target,
    "detected": False,
    "action": "SEARCH",
    "yolo_confidence": 0.0,
    "clip_similarity": None,
    "distance_m": None,
    "target_reached": False,
}

if detections.num_detections > 0:
    # 选择 YOLO 置信度最高的目标
    best = int(detections.logits.argmax())

    x1, y1, x2, y2 = detections.boxes[best].tolist()
    yolo_confidence = float(detections.logits[best])

    height, width = image.shape[:2]

    px1 = max(0, min(width - 1, int(x1 * width)))
    py1 = max(0, min(height - 1, int(y1 * height)))
    px2 = max(px1 + 1, min(width, int(x2 * width)))
    py2 = max(py1 + 1, min(height, int(y2 * height)))

    # 裁剪出 YOLO 找到的目标，交给 OpenCLIP 和文字比较
    crop_bgr = image[py1:py2, px1:px2]
    crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)

    clip_response = send_request(
        "http://localhost:12182/blip2itm",
        image=crop_rgb,
        txt=f"a photo of a {target}",
    )
    clip_similarity = float(clip_response["response"])

    # 根据检测框中心位置决定方向
    center_x = (x1 + x2) / 2

    if center_x < 0.4:
        action = "TURN_LEFT"
    elif center_x > 0.6:
        action = "TURN_RIGHT"
    else:
        action = "FORWARD"

    result = {
        "target": target,
        "detected": True,
        "action": action,
        "yolo_confidence": round(yolo_confidence, 3),
        "clip_similarity": round(clip_similarity, 3),
        "box": [x1, y1, x2, y2],
        "distance_m": None,
        "target_reached": False,
    }

print(json.dumps(result, ensure_ascii=False, indent=2))
