import argparse
import bisect
import json
import os
import sys
from collections import defaultdict, deque
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader


# 允许从团队仓库运行，同时使用原版 VLFM Python 包。
VLFM_REPO = Path(
    os.environ.get(
        "VLFM_REPO",
        Path.home() / "projects" / "vlfm",
    )
)

if VLFM_REPO.exists():
    sys.path.insert(0, str(VLFM_REPO))

try:
    from vlfm.vlm.server_wrapper import send_request
    from vlfm.vlm.yolov7 import YOLOv7Client
except ModuleNotFoundError as error:
    raise RuntimeError(
        "找不到原版 VLFM Python 包。请设置：\n"
        "export VLFM_REPO=$HOME/projects/vlfm\n"
        "export PYTHONPATH=$VLFM_REPO:$PYTHONPATH"
    ) from error


RGB_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"

CAMERA_FRAME = "camera_color_optical_frame"
MAP_FRAME = "map"


def parse_args():
    parser = argparse.ArgumentParser(
        description="从 ROS bag 定位语言指定目标并输出 map 坐标"
    )
    parser.add_argument("bag", help="ROS1 bag 文件路径")
    parser.add_argument(
        "--target",
        default="red chair",
        help='完整语言目标，例如 "red chair"',
    )
    parser.add_argument(
        "--base-class",
        default="chair",
        help='YOLO 基础类别，例如 "chair"',
    )
    parser.add_argument(
        "--output",
        default="semantic_memory.json",
        help="语义记忆输出文件",
    )
    parser.add_argument(
        "--frame-stride",
        type=int,
        default=5,
        help="每隔多少张 RGB 图进行一次识别",
    )
    parser.add_argument(
        "--max-pairs",
        type=int,
        default=0,
        help="最多处理多少组 RGB-D；0 表示不限制",
    )
    parser.add_argument(
        "--sync-tolerance",
        type=float,
        default=0.08,
        help="RGB 与深度最大时间差，单位秒",
    )
    parser.add_argument(
        "--yolo-threshold",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--clip-threshold",
        type=float,
        default=0.20,
    )
    return parser.parse_args()


def message_bytes(message):
    data = message.data
    if hasattr(data, "tobytes"):
        return data.tobytes()
    return bytes(data)


def stamp_to_ns(stamp, fallback_ns):
    if stamp is None:
        return fallback_ns

    seconds = getattr(
        stamp,
        "sec",
        getattr(stamp, "secs", 0),
    )
    nanoseconds = getattr(
        stamp,
        "nanosec",
        getattr(
            stamp,
            "nsec",
            getattr(stamp, "nsecs", 0),
        ),
    )

    value = int(seconds) * 1_000_000_000 + int(nanoseconds)
    return value if value > 0 else fallback_ns


def message_stamp_ns(message, fallback_ns):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    return stamp_to_ns(stamp, fallback_ns)


def decode_rgb(message):
    encoding = str(message.encoding).lower()
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)

    raw = np.frombuffer(
        message_bytes(message),
        dtype=np.uint8,
    ).reshape(height, step)

    pixels = raw[:, : width * 3].reshape(
        height,
        width,
        3,
    )

    if encoding == "rgb8":
        return pixels.copy()

    if encoding == "bgr8":
        return cv2.cvtColor(
            pixels,
            cv2.COLOR_BGR2RGB,
        )

    raise ValueError(
        f"暂不支持 RGB 编码：{message.encoding}"
    )


def decode_depth(message):
    encoding = str(message.encoding).upper()

    if encoding != "32FC1":
        raise ValueError(
            f"暂不支持深度编码：{message.encoding}"
        )

    byte_order = ">" if int(message.is_bigendian) else "<"
    dtype = np.dtype(f"{byte_order}f4")

    raw = np.frombuffer(
        message_bytes(message),
        dtype=dtype,
    )

    row_width = int(message.step) // 4
    depth = raw.reshape(
        int(message.height),
        row_width,
    )

    return depth[:, : int(message.width)].copy()


def quaternion_matrix(x, y, z, w):
    norm = np.sqrt(x * x + y * y + z * z + w * w)

    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def transform_matrix(transform):
    translation = transform.translation
    rotation = transform.rotation

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_matrix(
        float(rotation.x),
        float(rotation.y),
        float(rotation.z),
        float(rotation.w),
    )
    matrix[:3, 3] = [
        float(translation.x),
        float(translation.y),
        float(translation.z),
    ]
    return matrix


class TransformStore:
    def __init__(self):
        self.static = {}
        self.dynamic = defaultdict(list)
        self.dynamic_times = {}

    def add(
        self,
        parent,
        child,
        timestamp_ns,
        matrix,
        is_static,
    ):
        key = (parent, child)

        if is_static:
            self.static[key] = matrix
        else:
            self.dynamic[key].append(
                (timestamp_ns, matrix)
            )

    def finalize(self):
        for key, records in self.dynamic.items():
            records.sort(key=lambda item: item[0])
            self.dynamic_times[key] = [
                item[0] for item in records
            ]

    def matrix_at(self, key, timestamp_ns):
        if key in self.static:
            return self.static[key]

        records = self.dynamic.get(key)
        if not records:
            return None

        times = self.dynamic_times[key]
        index = bisect.bisect_right(
            times,
            timestamp_ns,
        ) - 1

        if index < 0:
            index = 0

        return records[index][1]

    def lookup(self, source, target, timestamp_ns):
        if source == target:
            return np.eye(4, dtype=np.float64)

        all_keys = set(self.static) | set(self.dynamic)
        graph = defaultdict(list)

        for parent, child in all_keys:
            matrix_parent_child = self.matrix_at(
                (parent, child),
                timestamp_ns,
            )

            if matrix_parent_child is None:
                continue

            # child 坐标转换到 parent 坐标。
            graph[child].append(
                (parent, matrix_parent_child)
            )

            # parent 坐标转换到 child 坐标。
            graph[parent].append(
                (
                    child,
                    np.linalg.inv(matrix_parent_child),
                )
            )

        queue = deque(
            [
                (
                    source,
                    np.eye(4, dtype=np.float64),
                )
            ]
        )
        visited = {source}

        while queue:
            current_frame, current_from_source = queue.popleft()

            for next_frame, next_from_current in graph[current_frame]:
                if next_frame in visited:
                    continue

                next_from_source = (
                    next_from_current
                    @ current_from_source
                )

                if next_frame == target:
                    return next_from_source

                visited.add(next_frame)
                queue.append(
                    (
                        next_frame,
                        next_from_source,
                    )
                )

        raise RuntimeError(
            f"找不到坐标转换链：{source} -> {target}"
        )


def load_camera_and_tf(bag_path):
    transform_store = TransformStore()
    camera_intrinsics = None

    selected_topics = {
        CAMERA_INFO_TOPIC,
        TF_TOPIC,
        TF_STATIC_TOPIC,
    }

    with AnyReader([bag_path]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in selected_topics
        ]

        for connection, bag_timestamp, rawdata in reader.messages(
            connections=connections
        ):
            message = reader.deserialize(
                rawdata,
                connection.msgtype,
            )

            if (
                connection.topic == CAMERA_INFO_TOPIC
                and camera_intrinsics is None
            ):
                matrix = message.K
                camera_intrinsics = {
                    "fx": float(matrix[0]),
                    "fy": float(matrix[4]),
                    "cx": float(matrix[2]),
                    "cy": float(matrix[5]),
                    "frame_id": str(message.header.frame_id),
                }
                continue

            if connection.topic not in {
                TF_TOPIC,
                TF_STATIC_TOPIC,
            }:
                continue

            is_static = (
                connection.topic == TF_STATIC_TOPIC
            )

            for stamped_transform in message.transforms:
                parent = str(
                    stamped_transform.header.frame_id
                )
                child = str(
                    stamped_transform.child_frame_id
                )

                timestamp_ns = stamp_to_ns(
                    stamped_transform.header.stamp,
                    bag_timestamp,
                )

                transform_store.add(
                    parent=parent,
                    child=child,
                    timestamp_ns=timestamp_ns,
                    matrix=transform_matrix(
                        stamped_transform.transform
                    ),
                    is_static=is_static,
                )

    if camera_intrinsics is None:
        raise RuntimeError("bag 中没有找到 CameraInfo")

    transform_store.finalize()
    return camera_intrinsics, transform_store


def normalized_box_to_pixels(
    box,
    width,
    height,
):
    x1, y1, x2, y2 = box

    px1 = max(
        0,
        min(width - 1, int(x1 * width)),
    )
    py1 = max(
        0,
        min(height - 1, int(y1 * height)),
    )
    px2 = max(
        px1 + 1,
        min(width, int(x2 * width)),
    )
    py2 = max(
        py1 + 1,
        min(height, int(y2 * height)),
    )

    return px1, py1, px2, py2


def robust_depth_in_box(depth, pixel_box):
    px1, py1, px2, py2 = pixel_box

    width = px2 - px1
    height = py2 - py1

    # 使用检测框内部区域，减少背景和地面干扰。
    inner_x1 = px1 + int(width * 0.25)
    inner_x2 = px2 - int(width * 0.25)
    inner_y1 = py1 + int(height * 0.25)
    inner_y2 = py2 - int(height * 0.25)

    if inner_x2 <= inner_x1 or inner_y2 <= inner_y1:
        return None

    region = depth[
        inner_y1:inner_y2,
        inner_x1:inner_x2,
    ]

    valid = region[
        np.isfinite(region)
        & (region > 0.1)
        & (region < 20.0)
    ]

    if valid.size < 20:
        return None

    return float(np.median(valid))


def localize_frame(
    rgb,
    depth,
    timestamp_ns,
    detector,
    transform_store,
    intrinsics,
    target_text,
    base_class,
    yolo_threshold,
    clip_threshold,
):
    image_bgr = cv2.cvtColor(
        rgb,
        cv2.COLOR_RGB2BGR,
    )

    detections = detector.predict(image_bgr)
    detections.filter_by_class([base_class])
    detections.filter_by_conf(yolo_threshold)

    if detections.num_detections == 0:
        return None, "YOLO 未检测到 chair"

    height, width = rgb.shape[:2]
    candidates = []

    for index in range(detections.num_detections):
        box = detections.boxes[index].tolist()
        yolo_confidence = float(
            detections.logits[index]
        )

        pixel_box = normalized_box_to_pixels(
            box,
            width,
            height,
        )
        px1, py1, px2, py2 = pixel_box

        crop_rgb = rgb[py1:py2, px1:px2]
        if crop_rgb.size == 0:
            continue

        response = send_request(
            "http://localhost:12182/blip2itm",
            image=crop_rgb,
            txt=f"a photo of a {target_text}",
        )
        clip_similarity = float(response["response"])

        depth_m = robust_depth_in_box(
            depth,
            pixel_box,
        )
        if depth_m is None:
            continue

        center_u = (px1 + px2) / 2.0
        center_v = (py1 + py2) / 2.0

        # 相机光学坐标：X 向右，Y 向下，Z 向前。
        camera_x = (
            (center_u - intrinsics["cx"])
            * depth_m
            / intrinsics["fx"]
        )
        camera_y = (
            (center_v - intrinsics["cy"])
            * depth_m
            / intrinsics["fy"]
        )
        camera_z = depth_m

        map_from_camera = transform_store.lookup(
            CAMERA_FRAME,
            MAP_FRAME,
            timestamp_ns,
        )

        point_camera = np.array(
            [
                camera_x,
                camera_y,
                camera_z,
                1.0,
            ],
            dtype=np.float64,
        )
        point_map = map_from_camera @ point_camera

        candidates.append(
            {
                "timestamp_s": timestamp_ns / 1e9,
                "map_position": [
                    float(point_map[0]),
                    float(point_map[1]),
                    float(point_map[2]),
                ],
                "camera_position": [
                    camera_x,
                    camera_y,
                    camera_z,
                ],
                "depth_m": depth_m,
                "yolo_confidence": yolo_confidence,
                "clip_similarity": clip_similarity,
                "bbox_normalized": box,
            }
        )

    if not candidates:
        return None, "检测到 chair，但没有有效深度"

    best = max(
        candidates,
        key=lambda item: item["clip_similarity"],
    )

    if best["clip_similarity"] < clip_threshold:
        return (
            None,
            "OpenCLIP 相似度低于阈值："
            f"{best['clip_similarity']:.3f}",
        )

    return best, "成功"


def load_existing_memory(path):
    if not path.exists():
        return {}

    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


def main():
    args = parse_args()

    bag_path = Path(args.bag)
    output_path = Path(args.output)

    if not bag_path.exists():
        raise FileNotFoundError(f"找不到 bag：{bag_path}")

    if args.frame_stride < 1:
        raise ValueError("--frame-stride 必须大于等于 1")

    target_text = args.target.strip().lower()
    base_class = args.base_class.strip().lower()

    print("读取 CameraInfo 和 TF...")
    intrinsics, transform_store = load_camera_and_tf(
        bag_path
    )

    print(
        "相机内参："
        f"fx={intrinsics['fx']:.3f}, "
        f"fy={intrinsics['fy']:.3f}, "
        f"cx={intrinsics['cx']:.3f}, "
        f"cy={intrinsics['cy']:.3f}"
    )

    print("连接 YOLOv7 服务...")
    detector = YOLOv7Client(port=12184)

    pending_rgb = None
    pending_depth = None
    rgb_count = 0
    processed_pairs = 0
    observations = []

    with AnyReader([bag_path]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in {
                RGB_TOPIC,
                DEPTH_TOPIC,
            }
        ]

        for connection, bag_timestamp, rawdata in reader.messages(
            connections=connections
        ):
            message = reader.deserialize(
                rawdata,
                connection.msgtype,
            )
            timestamp_ns = message_stamp_ns(
                message,
                bag_timestamp,
            )

            if connection.topic == RGB_TOPIC:
                rgb_count += 1

                if (
                    (rgb_count - 1)
                    % args.frame_stride
                    != 0
                ):
                    continue

                pending_rgb = (
                    timestamp_ns,
                    decode_rgb(message),
                )

            elif connection.topic == DEPTH_TOPIC:
                pending_depth = (
                    timestamp_ns,
                    decode_depth(message),
                )

            if pending_rgb is None or pending_depth is None:
                continue

            rgb_timestamp, rgb = pending_rgb
            depth_timestamp, depth = pending_depth

            difference_s = abs(
                rgb_timestamp - depth_timestamp
            ) / 1e9

            if difference_s > args.sync_tolerance:
                if rgb_timestamp < depth_timestamp:
                    pending_rgb = None
                else:
                    pending_depth = None
                continue

            processed_pairs += 1
            pair_timestamp = rgb_timestamp

            try:
                observation, status = localize_frame(
                    rgb=rgb,
                    depth=depth,
                    timestamp_ns=pair_timestamp,
                    detector=detector,
                    transform_store=transform_store,
                    intrinsics=intrinsics,
                    target_text=target_text,
                    base_class=base_class,
                    yolo_threshold=args.yolo_threshold,
                    clip_threshold=args.clip_threshold,
                )
            except Exception as error:
                observation = None
                status = f"处理失败：{error}"

            print(
                f"[{processed_pairs}] "
                f"t={pair_timestamp / 1e9:.3f} "
                f"RGB-D差={difference_s:.3f}s："
                f"{status}"
            )

            if observation is not None:
                observations.append(observation)

            pending_rgb = None
            pending_depth = None

            if (
                args.max_pairs > 0
                and processed_pairs >= args.max_pairs
            ):
                break

    if not observations:
        print()
        print("没有得到有效的红色椅子 map 坐标。")
        print("请检查模型服务、图像视角和阈值。")
        sys.exit(2)

    map_positions = np.array(
        [
            observation["map_position"]
            for observation in observations
        ],
        dtype=np.float64,
    )

    semantic_object = {
        "target_text": target_text,
        "base_class": base_class,
        "detected": True,
        "frame_id": MAP_FRAME,
        "object_position": {
            "x": round(
                float(np.median(map_positions[:, 0])),
                3,
            ),
            "y": round(
                float(np.median(map_positions[:, 1])),
                3,
            ),
            "z": round(
                float(np.median(map_positions[:, 2])),
                3,
            ),
        },
        "depth_m": round(
            float(
                np.median(
                    [
                        item["depth_m"]
                        for item in observations
                    ]
                )
            ),
            3,
        ),
        "yolo_confidence": round(
            float(
                np.median(
                    [
                        item["yolo_confidence"]
                        for item in observations
                    ]
                )
            ),
            3,
        ),
        "clip_similarity": round(
            float(
                np.median(
                    [
                        item["clip_similarity"]
                        for item in observations
                    ]
                )
            ),
            3,
        ),
        "observation_count": len(observations),
        "source": str(bag_path),
    }

    memory = load_existing_memory(output_path)
    memory[target_text] = semantic_object

    with open(output_path, "w", encoding="utf-8") as file:
        json.dump(
            memory,
            file,
            ensure_ascii=False,
            indent=2,
        )

    print()
    print("最终语义目标：")
    print(
        json.dumps(
            semantic_object,
            ensure_ascii=False,
            indent=2,
        )
    )
    print()
    print(f"已经保存到：{output_path}")


if __name__ == "__main__":
    main()
