from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader


bag_path = Path(
    "/mnt/d/VLFM-data/slam_handoff/g1_red_chair_demo.bag"
)
output_dir = Path(
    "/mnt/d/VLFM-data/slam_handoff/rgb_samples_dense"
)
output_dir.mkdir(parents=True, exist_ok=True)


with AnyReader([bag_path]) as reader:
    connections = [
        connection
        for connection in reader.connections
        if connection.topic == "/camera/color/image_raw"
    ]

    saved = 0

    for index, (connection, timestamp, rawdata) in enumerate(
        reader.messages(connections=connections)
    ):
        # 每5帧保存一张，避免导出太多图片
        if index % 5 != 0:
            continue

        message = reader.deserialize(rawdata, connection.msgtype)

        data = np.asarray(message.data, dtype=np.uint8)
        rows = data.reshape(message.height, message.step)

        rgb = rows[:, : message.width * 3].reshape(
            message.height,
            message.width,
            3,
        )
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        output_path = output_dir / f"rgb_{index:04d}.jpg"
        cv2.imwrite(str(output_path), bgr)

        print("保存：", output_path)
        saved += 1

    print(f"完成，共保存 {saved} 张图片")
