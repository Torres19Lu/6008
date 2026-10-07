# Semantic target locator（接口预留版）

这个目录实现边界清晰的 VLM 语义定位模块。它只负责：

```text
目标文字 + 同步 RGB-D + CameraInfo + map<-camera TF
→ 稳定的目标物体 map 坐标
```

它不读取 costmap、不选择接近点、不规划路径、不旋转机器人，也不发布
`/cmd_vel`。原始 `vlm/` 演示代码保持不变。

当前提供两个共享同一定位核心的入口：

- `ros_node.py`：通用 YOLO + CLIP 模式；
- `ros_lite_node.py`：轻量 YOLO + HSV 颜色模式，不加载 CLIP，详情见
  [LITE_MODE.md](LITE_MODE.md)。

Windows宿主机与Ubuntu虚拟机的固定职责和分阶段启动方式见
[LIGHTWEIGHT_HOST_VM_DEPLOYMENT.md](LIGHTWEIGHT_HOST_VM_DEPLOYMENT.md)。

## 当前可运行方式

没有机器人时使用 JSON replay：

```bash
cd vlm/semantic_target_locator
python run_replay.py mock_replay.json
python -m unittest -v test_locator.py
```

Replay 与未来 ROS 版本运行相同的 `SemanticTargetLocator` 核心，只替换四个接口：

| 接口 | 当前 | 有机器人后 |
|---|---|---|
| `FrameSource` | JSON 帧 | RGB/Depth/CameraInfo 同步订阅 |
| `VisionLanguageBackend` | JSON YOLO/CLIP结果 | YOLO + CLIP 服务 |
| `TransformProvider` | JSON 4×4矩阵 | TF2 `map <- camera` |
| `ResultSink` | 内存/终端 | ROS topic 或 action result |

## 预留的 ROS topic

默认参数放在 `ros_adapter.py::RosTopics`：

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/semantic/target_query
/semantic/target_status
```

实际相机型号确定后只修改参数，不修改定位核心。深度比例通过
`depth_scale_to_m` 配置，例如毫米深度使用 `0.001`。

## 输出契约

成功时只输出目标本体坐标：

```json
{
  "status": "TARGET_LOCKED",
  "target": {
    "label": "red chair",
    "frame_id": "map",
    "position": {"x": 2.732, "y": 3.0, "z": 0.42},
    "confidence": 0.789,
    "observation_count": 3,
    "position_spread_m": 0.014
  }
}
```

导航、接近点和机器人旋转由其他模块处理。

## 轻量模式

当任务主要是“某个颜色的某种物体”或直接指定物体类别时，可以使用轻量模式：

```text
红色的椅子 -> YOLO检测chair -> HSV选择红色候选
椅子       -> YOLO检测chair -> 选择检测置信度最高候选
```

轻量模式默认只以 2 Hz 处理最新同步帧，不启动 CLIP 服务，但深度、TF、多帧锁定
和 `/semantic/target_status` 输出与通用模式一致。

## 实机接入前仍需确定

- 相机实际 topic 和 optical frame；
- RGB 与深度是否硬件对齐；
- 深度编码和单位；
- `base_link <- camera` 外参；
- YOLO/CLIP sidecar 的部署环境；
- ROS 消息最终采用 `g1_msgs/SemanticObjectArray` 还是自定义 action；
- 实机误差、延迟和超时阈值。

`ros_adapter.py` 已实现接口骨架，但当前 Windows 环境没有 ROS，因此这里只做
语法检查；必须在目标 Ubuntu/ROS 环境中进行 catkin、bag replay 和实机验证。
