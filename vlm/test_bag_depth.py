import argparse
import json
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


RGB_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"


def parse_args():
    parser = argparse.ArgumentParser(
        description="检查 ROS bag 中的 32FC1 深度数据"
    )
    parser.add_argument("bag", help="ROS1 bag 路径")
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="检查多少张深度图",
    )
    return parser.parse_args()


def image_bytes(message):
    data = message.data

    if hasattr(data, "tobytes"):
        return data.tobytes()

    return bytes(data)


def decode_depth(message):
    encoding = str(message.encoding).upper()

    if encoding != "32FC1":
        raise ValueError(
            f"暂不支持深度编码：{message.encoding}"
        )

    byte_order = ">" if int(message.is_bigendian) else "<"
    dtype = np.dtype(f"{byte_order}f4")

    raw = np.frombuffer(
        image_bytes(message),
        dtype=dtype,
    )

    row_width = int(message.step) // 4
    depth = raw.reshape(
        int(message.height),
        row_width,
    )

    return depth[:, : int(message.width)].copy()


def main():
    args = parse_args()
    bag_path = Path(args.bag)

    if not bag_path.exists():
        raise FileNotFoundError(f"找不到 bag：{bag_path}")

    results = []

    with AnyReader([bag_path]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic == DEPTH_TOPIC
        ]

        for connection, timestamp, rawdata in reader.messages(
            connections=connections
        ):
            message = reader.deserialize(
                rawdata,
                connection.msgtype,
            )
            depth = decode_depth(message)

            valid = depth[
                np.isfinite(depth)
                & (depth > 0.1)
                & (depth < 20.0)
            ]

            if valid.size == 0:
                sample = {
                    "timestamp_s": timestamp / 1e9,
                    "valid_pixels": 0,
                    "valid_ratio": 0.0,
                    "minimum_m": None,
                    "median_m": None,
                    "maximum_m": None,
                    "center_depth_m": None,
                }
            else:
                center_y = depth.shape[0] // 2
                center_x = depth.shape[1] // 2
                center_depth = float(
                    depth[center_y, center_x]
                )

                if (
                    not np.isfinite(center_depth)
                    or center_depth <= 0.1
                    or center_depth >= 20.0
                ):
                    center_depth = None

                sample = {
                    "timestamp_s": timestamp / 1e9,
                    "valid_pixels": int(valid.size),
                    "valid_ratio": round(
                        float(valid.size / depth.size),
                        4,
                    ),
                    "minimum_m": round(
                        float(np.min(valid)),
                        4,
                    ),
                    "median_m": round(
                        float(np.median(valid)),
                        4,
                    ),
                    "maximum_m": round(
                        float(np.max(valid)),
                        4,
                    ),
                    "center_depth_m": (
                        round(center_depth, 4)
                        if center_depth is not None
                        else None
                    ),
                }

            results.append(sample)

            if len(results) >= args.samples:
                break

    output = {
        "bag": str(bag_path),
        "depth_topic": DEPTH_TOPIC,
        "encoding": "32FC1",
        "unit": "metres",
        "samples": results,
    }

    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
