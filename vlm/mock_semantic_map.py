import argparse
import json
import math
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="使用模拟深度和位姿验证语义目标坐标输出结构"
    )
    parser.add_argument("input_json", help="模拟输入 JSON")
    parser.add_argument(
        "--output",
        default="semantic_memory.json",
        help="语义记忆输出文件",
    )
    return parser.parse_args()


def load_json(path):
    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


def save_json(path, data):
    with open(path, "w", encoding="utf-8") as file:
        json.dump(data, file, ensure_ascii=False, indent=2)


def main():
    args = parse_args()

    input_path = Path(args.input_json)
    output_path = Path(args.output)
    data = load_json(input_path)

    target_text = data["target_text"].strip().lower()
    base_class = data.get(
        "base_class",
        target_text.split()[-1],
    ).strip().lower()

    depth_m = float(data["depth_m"])

    robot_x = float(data["robot_pose"]["x"])
    robot_y = float(data["robot_pose"]["y"])
    robot_yaw_deg = float(data["robot_pose"]["yaw_deg"])

    center_x = float(
        data["detection"]["center_x_normalized"]
    )
    horizontal_fov_deg = float(
        data["camera"]["horizontal_fov_deg"]
    )

    # 这是模拟计算，不是真实相机内参和 TF 转换。
    camera_angle_deg = (
        center_x - 0.5
    ) * horizontal_fov_deg

    target_angle_deg = (
        robot_yaw_deg + camera_angle_deg
    )
    target_angle_rad = math.radians(target_angle_deg)

    target_x = robot_x + depth_m * math.cos(target_angle_rad)
    target_y = robot_y + depth_m * math.sin(target_angle_rad)

    semantic_object = {
        "target_text": target_text,
        "base_class": base_class,
        "detected": True,
        "frame_id": data["frame_id"],
        "object_position": {
            "x": round(target_x, 3),
            "y": round(target_y, 3),
            "z": 0.0,
        },
        "depth_m": depth_m,
        "yolo_confidence": data["detection"]["yolo_confidence"],
        "clip_similarity": data["detection"]["clip_similarity"],
        "source": "mock_input_only",
    }

    memory = {}
    if output_path.exists():
        memory = load_json(output_path)

    memory[target_text] = semantic_object
    save_json(output_path, memory)

    query_result = {
        "target_text": target_text,
        "found_in_memory": target_text in memory,
        "semantic_object": memory[target_text],
    }

    print("模拟检测并保存的语义目标：")
    print(json.dumps(semantic_object, ensure_ascii=False, indent=2))

    print("\n查询语义记忆的结果：")
    print(json.dumps(query_result, ensure_ascii=False, indent=2))

    print("\n模拟测试成功")
    print("注意：这里不是导航目标，也没有进行避障或路径规划。")


if __name__ == "__main__":
    main()
