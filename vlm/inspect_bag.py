import argparse
import json
from pathlib import Path

from rosbags.highlevel import AnyReader


TOPICS = {
    "/camera/color/image_raw",
    "/camera/aligned_depth_to_color/image_raw",
    "/camera/color/camera_info",
    "/camera/aligned_depth_to_color/camera_info",
    "/slam/frontend/odom",
    "/tf",
    "/tf_static",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="检查 ROS bag 中相机、里程计和 TF 的真实信息"
    )
    parser.add_argument("bag", help="ROS1 bag 文件路径")
    return parser.parse_args()


def frame_id(header):
    return str(header.frame_id)


def main():
    args = parse_args()
    bag_path = Path(args.bag)

    if not bag_path.exists():
        raise FileNotFoundError(f"找不到 bag：{bag_path}")

    result = {
        "bag": str(bag_path),
        "topics": {},
        "rgb_image": None,
        "depth_image": None,
        "rgb_camera_info": None,
        "depth_camera_info": None,
        "odometry": None,
        "tf_pairs": [],
    }

    tf_pairs = set()
    recorded_topics = set()

    with AnyReader([bag_path]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in TOPICS
        ]

        for connection in connections:
            result["topics"][connection.topic] = {
                "msgtype": connection.msgtype,
                "message_count": connection.msgcount,
            }

        for connection, timestamp, rawdata in reader.messages(
            connections=connections
        ):
            topic = connection.topic
            message = reader.deserialize(
                rawdata,
                connection.msgtype,
            )

            if (
                topic == "/camera/color/image_raw"
                and topic not in recorded_topics
            ):
                result["rgb_image"] = {
                    "frame_id": frame_id(message.header),
                    "timestamp_s": timestamp / 1e9,
                    "width": int(message.width),
                    "height": int(message.height),
                    "encoding": str(message.encoding),
                    "step": int(message.step),
                    "is_bigendian": int(message.is_bigendian),
                }
                recorded_topics.add(topic)

            elif (
                topic
                == "/camera/aligned_depth_to_color/image_raw"
                and topic not in recorded_topics
            ):
                result["depth_image"] = {
                    "frame_id": frame_id(message.header),
                    "timestamp_s": timestamp / 1e9,
                    "width": int(message.width),
                    "height": int(message.height),
                    "encoding": str(message.encoding),
                    "step": int(message.step),
                    "is_bigendian": int(message.is_bigendian),
                }
                recorded_topics.add(topic)

            elif (
                topic == "/camera/color/camera_info"
                and topic not in recorded_topics
            ):
                result["rgb_camera_info"] = {
                    "frame_id": frame_id(message.header),
                    "width": int(message.width),
                    "height": int(message.height),
                    "distortion_model": str(
                        message.distortion_model
                    ),
                    "K": [
                        float(value)
                        for value in message.K
                    ],
                }
                recorded_topics.add(topic)

            elif (
                topic
                == "/camera/aligned_depth_to_color/camera_info"
                and topic not in recorded_topics
            ):
                result["depth_camera_info"] = {
                    "frame_id": frame_id(message.header),
                    "width": int(message.width),
                    "height": int(message.height),
                    "distortion_model": str(
                        message.distortion_model
                    ),
                    "K": [
                        float(value)
                        for value in message.K
                    ],
                }
                recorded_topics.add(topic)

            elif (
                topic == "/slam/frontend/odom"
                and topic not in recorded_topics
            ):
                result["odometry"] = {
                    "frame_id": frame_id(message.header),
                    "child_frame_id": str(
                        message.child_frame_id
                    ),
                }
                recorded_topics.add(topic)

            elif topic in {"/tf", "/tf_static"}:
                for transform in message.transforms:
                    parent = frame_id(transform.header)
                    child = str(transform.child_frame_id)
                    tf_pairs.add((parent, child, topic))

    result["tf_pairs"] = [
        {
            "parent": parent,
            "child": child,
            "topic": topic,
        }
        for parent, child, topic in sorted(tf_pairs)
    ]

    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
