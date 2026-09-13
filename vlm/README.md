# VLM：红色椅子语义定位演示

## 1. 模块简介

本目录是 6008 项目中的轻量 VLM（Vision-Language Model）模块。

该模块基于开源项目 [VLFM](https://github.com/rai-opensource/vlfm) 的模型服务结构进行简化和改造，使用：

- YOLOv7：检测图片中的常见物体并提供检测框；
- OpenCLIP：比较检测到的物体图片与用户目标文字的相似度；
- RGB-D 与 SLAM 位姿：后续用于将二维检测结果转换到地图坐标。

由于设备显存和 WSL 图形环境限制，当前不运行 Habitat，也不复现完整的 VLFM Frontier 探索与 PointNav。该模块的目标是完成一个可上机器人演示的简化语义目标定位流程。

## 2. 最终演示场景

最终计划在教室或其他较简单、空旷的环境中完成以下演示：

1. SLAM 模块首先建立环境地图。
2. 机器人位于起始位置，原地缓慢旋转一至两圈，观察周围环境。
3. 相机持续提供 RGB 和深度数据。
4. YOLOv7 检测画面中的 `chair`。
5. OpenCLIP 根据用户目标文字 `red chair`，判断检测到的椅子是否与“红色椅子”匹配。
6. 结合深度、相机参数和 SLAM TF，将红色椅子转换到 `map` 坐标系。
7. 在语义记忆中保存：
   ```json
   {
     "label": "red chair",
     "frame_id": "map",
     "position": {
       "x": 2.732,
       "y": 3.0,
       "z": 0.0
     }
   }
   ```
8. 机器人扫描结束后回到起始位置。
9. 用户在终端输入目标 `red chair`。
10. VLM 从语义记忆中查询目标坐标，并将该坐标发送给导航模块。
11. 导航和控制模块带领机器人接近目标。
12. 机器人到达设定的安全距离后停止，并显示：
    ```text
    Target red chair reached
    ```

停止距离将根据教室空间、机器人安全要求和底层控制效果进行调整。

## 3. 当前完成情况

目前已经完成：

- YOLOv7 模型加载和图片检测测试；
- OpenCLIP 模型加载和图文相似度测试；
- YOLOv7 与 OpenCLIP 的轻量化运行环境；
- 根据检测框位置输出简单方向；
- 使用模拟深度和模拟机器人位姿计算地图坐标；
- 将语义目标保存到本地记忆；
- 根据目标文字查询已保存的导航坐标。

目前尚未完成：

- 实时 ROS RGB 和深度数据接入；
- 相机内参与真实 TF 坐标转换；
- `g1_msgs` 消息封装；
- 与真实 SLAM、导航、MuJoCo 和 G1 的联调。

## 4. 目录说明

```text
vlm/
├── README.md
├── demo_action.py
├── test_clip.py
├── mock_semantic_map.py
├── mock_slam_input.json
├── scripts/
│   └── launch_vlm_servers_lite.sh
└── overlay/
    └── vlfm/
        └── vlm/
            ├── clipitm.py
            ├── yolov7.py
            └── server_wrapper.py
```

文件说明：

- `demo_action.py`：YOLOv7 与 OpenCLIP 合并测试；
- `test_clip.py`：单独测试图片和不同文字的相似度；
- `mock_semantic_map.py`：使用假深度和假位姿计算目标地图坐标；
- `mock_slam_input.json`：模拟 SLAM、相机和检测输入；
- `launch_vlm_servers_lite.sh`：轻量模型服务启动脚本；
- `overlay/`：需要覆盖到原版 VLFM 对应目录的修改文件。

## 5. 模拟 SLAM 测试

模拟测试不依赖 ROS、相机或模型服务。

运行：

```bash
cd vlm
python mock_semantic_map.py mock_slam_input.json
```

预期输出包含：

```json
{
  "target": "red chair",
  "found_in_memory": true,
  "frame_id": "map",
  "goal_x": 2.732,
  "goal_y": 3.0
}
```

并显示：

```text
模拟测试成功
```

此结果只用于验证程序结构和坐标计算流程，不代表真实相机标定精度。

## 6. 图片与文字测试

模型权重和 Conda 环境不包含在本仓库中，需要按照原版 VLFM 的说明单独安装。

当前测试环境名称为：

```bash
conda activate vlm
```

每个新终端中执行：

```bash
cd ~/projects/vlfm
export LD_PRELOAD="$CONDA_PREFIX/lib/libstdc++.so.6"
```

启动 OpenCLIP：

```bash
python -m vlfm.vlm.clipitm --port 12182
```

启动 YOLOv7：

```bash
python -m vlfm.vlm.yolov7 --port 12184
```

在另一个终端测试：

```bash
python demo_action.py /path/to/chair.jpg chair
```

示例输出：

```json
{
  "target": "chair",
  "detected": true,
  "action": "FORWARD",
  "yolo_confidence": 0.966,
  "clip_similarity": 0.254,
  "distance_m": null,
  "target_reached": false
}
```

`distance_m` 当前为空，因为尚未接入真实深度数据。

## 7. 计划接收的真实输入

后续计划从相机和 SLAM 接收：

- RGB：`sensor_msgs/Image`
- 深度：`sensor_msgs/Image`，需要确认单位为米或毫米
- 相机参数：`sensor_msgs/CameraInfo`
- 机器人位姿：`nav_msgs/Odometry`
- 坐标关系：`/tf` 和 `/tf_static`
- 可选注册点云：`sensor_msgs/PointCloud2`
- 用户目标文字，例如 `red chair`

实际 ROS topic 名称需要在联调时确认。

## 8. 计划输出

检测阶段输出语义目标：

```json
{
  "label": "red chair",
  "detected": true,
  "confidence": 0.966,
  "frame_id": "map",
  "position": {
    "x": 2.732,
    "y": 3.0,
    "z": 0.0
  }
}
```

查询阶段向导航模块输出：

```json
{
  "target": "red chair",
  "found_in_memory": true,
  "frame_id": "map",
  "goal_x": 2.732,
  "goal_y": 3.0
}
```

后续可转换为：

- `g1_msgs/Detection3DArray`
- `g1_msgs/SemanticObjectArray`
- 或导航模块要求的 `geometry_msgs/PoseStamped`

最终消息格式由 VLM、接口和导航控制成员共同确认。

## 9. 注意事项

本仓库不上传：

- 模型权重；
- HM3D 数据集；
- ROS bag；
- PCD 点云；
- Conda 环境；
- Hugging Face 缓存；
- 自动生成的 `semantic_memory.json`。

其他成员需要单独下载依赖和权重。

本模块当前是面向项目演示的轻量实现，不等同于完整原版 VLFM。
