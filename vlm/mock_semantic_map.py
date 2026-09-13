import json
import math
import sys
from pathlib import Path


def load_json(path):
    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


if len(sys.argv) != 2:
    print("用法: python mock_semantic_map.py mock_slam_input.json")
    sys.exit(1)

input_path = Path(sys.argv[1])
data = load_json(input_path)

target = data["target_text"]
depth_m = float(data["depth_m"])

robot_x = float(data["robot_pose"]["x"])
robot_y = float(data["robot_pose"]["y"])
robot_yaw_deg = float(data["robot_pose"]["yaw_deg"])

center_x = float(data["detection"]["center_x_normalized"])
horizontal_fov_deg = float(data["camera"]["horizontal_fov_deg"])

# 目标偏离画面中心的角度。
# center_x=0.5 表示目标位于画面中央。
camera_angle_deg = (center_x - 0.5) * horizontal_fov_deg
target_angle_deg = robot_yaw_deg + camera_angle_deg
target_angle_rad = math.radians(target_angle_deg)

# 简化的二维坐标投影。
target_x = robot_x + depth_m * math.cos(target_angle_rad)
target_y = robot_y + depth_m * math.sin(target_angle_rad)

semantic_object = {
    "label": target,
    "frame_id": data["frame_id"],
    "position": {
        "x": round(target_x, 3),
        "y": round(target_y, 3),
        "z": 0.0
    },
    "depth_m": depth_m,
    "yolo_confidence": data["detection"]["yolo_confidence"],
    "clip_similarity": data["detection"]["clip_similarity"]
}

memory_path = Path("semantic_memory.json")
memory = {}

if memory_path.exists():
    memory = load_json(memory_path)

memory[target] = semantic_object

with open(memory_path, "w", encoding="utf-8") as file:
    json.dump(memory, file, ensure_ascii=False, indent=2)

navigation_goal = {
    "target": target,
    "found_in_memory": target in memory,
    "frame_id": memory[target]["frame_id"],
    "goal_x": memory[target]["position"]["x"],
    "goal_y": memory[target]["position"]["y"]
}

print("检测并记录的语义目标：")
print(json.dumps(semantic_object, ensure_ascii=False, indent=2))

print("\n查询目标后返回的导航坐标：")
print(json.dumps(navigation_goal, ensure_ascii=False, indent=2))

print("\n模拟测试成功")
