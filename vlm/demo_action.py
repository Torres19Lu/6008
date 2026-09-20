import argparse
import json
import sys
from pathlib import Path

import cv2

from vlfm.vlm.server_wrapper import send_request
from vlfm.vlm.yolov7 import YOLOv7Client


def parse_args():
    parser = argparse.ArgumentParser(
        description="使用 YOLOv7 和 OpenCLIP 检测语言指定的目标"
    )
    parser.add_argument("image", help="待检测图片路径")
    parser.add_argument(
        "target_text",
        help='完整语言目标，例如 "red chair"',
    )
    parser.add_argument(
        "--base-class",
        default=None,
        help='YOLO 基础类别，例如 "chair"。省略时取目标文字的最后一个单词',
    )
    parser.add_argument(
        "--yolo-threshold",
        type=float,
        default=0.25,
        help="YOLO 置信度阈值",
    )
    parser.add_argument(
        "--clip-threshold",
        type=float,
        default=0.20,
        help="OpenCLIP 相似度临时阈值，后续可根据实测调整",
    )
    return parser.parse_args()


def print_result(result):
    print(json.dumps(result, ensure_ascii=False, indent=2))


def main():
    args = parse_args()

    image_path = Path(args.image)
    target_text = args.target_text.strip().lower()

    if not target_text:
        print("错误：目标文字不能为空", file=sys.stderr)
        sys.exit(1)

    # 例如目标为 red chair 时，默认基础类别为 chair。
    base_class = (
        args.base_class.strip().lower()
        if args.base_class
        else target_text.split()[-1]
    )

    image_bgr = cv2.imread(str(image_path))
    if image_bgr is None:
        print(f"错误：无法读取图片：{image_path}", file=sys.stderr)
        sys.exit(1)

    detector = YOLOv7Client(port=12184)
    detections = detector.predict(image_bgr)
    detections.filter_by_class([base_class])
    detections.filter_by_conf(args.yolo_threshold)

    empty_result = {
        "target_text": target_text,
        "base_class": base_class,
        "detected": False,
        "frame_id": None,
        "object_position": None,
        "depth_m": None,
        "yolo_confidence": 0.0,
        "clip_similarity": None,
        "bbox_normalized": None,
    }

    if detections.num_detections == 0:
        print_result(empty_result)
        return

    height, width = image_bgr.shape[:2]
    candidates = []

    # 对每个 YOLO 检测框都计算 OpenCLIP 相似度。
    for index in range(detections.num_detections):
        x1, y1, x2, y2 = detections.boxes[index].tolist()
        yolo_confidence = float(detections.logits[index])

        px1 = max(0, min(width - 1, int(x1 * width)))
        py1 = max(0, min(height - 1, int(y1 * height)))
        px2 = max(px1 + 1, min(width, int(x2 * width)))
        py2 = max(py1 + 1, min(height, int(y2 * height)))

        crop_bgr = image_bgr[py1:py2, px1:px2]
        if crop_bgr.size == 0:
            continue

        crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)

        clip_response = send_request(
            "http://localhost:12182/blip2itm",
            image=crop_rgb,
            txt=f"a photo of a {target_text}",
        )
        clip_similarity = float(clip_response["response"])

        candidates.append(
            {
                "yolo_confidence": yolo_confidence,
                "clip_similarity": clip_similarity,
                "bbox_normalized": [x1, y1, x2, y2],
                "center_normalized": [
                    (x1 + x2) / 2.0,
                    (y1 + y2) / 2.0,
                ],
            }
        )

    if not candidates:
        print_result(empty_result)
        return

    # 如果画面中存在多把椅子，选择最符合语言目标的一把。
    best = max(candidates, key=lambda item: item["clip_similarity"])
    semantic_match = best["clip_similarity"] >= args.clip_threshold

    result = {
        "target_text": target_text,
        "base_class": base_class,
        "detected": semantic_match,
        "frame_id": None,
        "object_position": None,
        "depth_m": None,
        "yolo_confidence": round(best["yolo_confidence"], 3),
        "clip_similarity": round(best["clip_similarity"], 3),
        "bbox_normalized": [
            round(value, 6) for value in best["bbox_normalized"]
        ],
        "center_normalized": [
            round(value, 6) for value in best["center_normalized"]
        ],
    }

    print_result(result)


if __name__ == "__main__":
    main()
