import argparse
import json
import sys
from pathlib import Path

import cv2

from vlfm.vlm.server_wrapper import send_request


DEFAULT_LABELS = [
    "red chair",
    "blue chair",
    "black chair",
    "white chair",
    "wooden chair",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="测试 OpenCLIP 图片与多个文字描述的相似度"
    )
    parser.add_argument("image", help="待测试图片路径")
    parser.add_argument(
        "labels",
        nargs="*",
        help="需要比较的文字；省略时比较多种颜色的椅子",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    image_path = Path(args.image)
    image_bgr = cv2.imread(str(image_path))

    if image_bgr is None:
        print(f"错误：无法读取图片：{image_path}", file=sys.stderr)
        sys.exit(1)

    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    labels = args.labels or DEFAULT_LABELS

    scores = []

    for label in labels:
        clean_label = label.strip().lower()
        if not clean_label:
            continue

        response = send_request(
            "http://localhost:12182/blip2itm",
            image=image_rgb,
            txt=f"a photo of a {clean_label}",
        )

        scores.append(
            {
                "label": clean_label,
                "similarity": round(float(response["response"]), 6),
            }
        )

    scores.sort(
        key=lambda item: item["similarity"],
        reverse=True,
    )

    result = {
        "image": str(image_path),
        "best_match": scores[0]["label"] if scores else None,
        "scores": scores,
    }

    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
