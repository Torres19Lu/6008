# VLM 目标检测模块

## 1. 这个模块做什么

用户输入目标，例如：

```
red chair
```

程序会：

1. 使用 YOLOv7 找出图片中的 `chair`；
    
2. 使用 OpenCLIP 判断哪个椅子最符合 `red chair`；
    
3. 读取深度，计算椅子的三维位置；
    
4. 使用 SLAM 的 TF，把位置转换到 `map` 坐标系；
    
5. 输出并保存红色椅子的地图坐标。
    

目前已经使用 `g1_red_chair_demo.bag` 完成离线测试。
## 2. VLM 负责范围

VLM ：

- 接收目标文字，例如 `red chair`；
    
- 识别目标；
    
- 计算目标在 `map` 坐标系中的位置；
    
- 保存和查询目标位置；
    
- 将目标坐标提供给其他模块。
    

其他模块负责：

- 障碍物检测与障碍地图；
    
- 路径规划；
    
- 避障；
    
- 控制机器人行走；
    
- 选择安全停靠位置；
    
- 判断机器人是否到达目标。
    

障碍物信息应由 SLAM、深度相机或雷达提供。导航模块负责根据障碍地图规划路线。
## 3. 输入数据

当前程序从 ROS bag 读取：

```
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/tf
/tf_static
```

输入内容包括：

- RGB 彩色图片；
    
- 与彩色图片对齐的深度图；
    
- 相机内参；
    
- 相机、机器人和地图之间的坐标关系；
    
- 用户目标文字，例如 `red chair`。
    
## 4. 输出结果

示例：

```
{
  "target_text": "red chair",
  "base_class": "chair",
  "detected": true,
  "frame_id": "map",
  "object_position": {
    "x": 5.621,
    "y": 0.02,
    "z": -0.612
  },
  "depth_m": 5.525,
  "yolo_confidence": 0.593,
  "clip_similarity": 0.333,
  "observation_count": 20
}
```

字段说明：

- `target_text`：用户输入的完整目标；
    
- `base_class`：YOLO 检测的基础类别；
    
- `detected`：是否检测成功；
    
- `frame_id`：坐标所属坐标系；
    
- `object_position`：目标在 `map` 坐标系中的位置；
    
- `depth_m`：目标深度，单位为米；
    
- `yolo_confidence`：YOLO 检测置信度；
    
- `clip_similarity`：图片与目标文字的相似度；
    
- `observation_count`：参与最终计算的有效帧数量。
    
## 5. 文件说明

```
vlm/
├── README.md
├── demo_action.py
├── test_clip.py
├── mock_semantic_map.py
├── mock_slam_input.json
├── bag_extract_frames.py
├── inspect_bag.py
├── test_bag_depth.py
├── bag_vlm_localizer.py
├── scripts/
│   └── launch_vlm_servers_lite.sh
└── overlay/
    └── vlfm/
        └── vlm/
            ├── clipitm.py
            ├── yolov7.py
            └── server_wrapper.py
```

### `demo_action.py`

使用一张图片同时测试 YOLOv7 和 OpenCLIP。

YOLO 检测 `chair`，OpenCLIP 判断它是否符合 `red chair`。

这个程序不读取深度，也不计算地图坐标。

### `test_clip.py`

单独测试 OpenCLIP。

它会比较图片与以下文字的相似度：

```
red chair
blue chair
black chair
white chair
wooden chair
```

### `mock_semantic_map.py`

使用假的深度和机器人位姿，测试语义目标的输入、保存和查询格式。

该程序只用于检查代码结构，不代表真实定位结果。

### `mock_slam_input.json`

提供给 `mock_semantic_map.py` 的模拟输入数据。

### `bag_extract_frames.py`

从 ROS bag 中提取 RGB 图片，用于检查相机画面和测试模型。

### `inspect_bag.py`

查看 ROS bag 中的 Topic、图片格式、相机内参和 TF 坐标关系。

### `test_bag_depth.py`

检查 bag 中的深度图能否正确读取，以及深度单位是否为米。

### `bag_vlm_localizer.py`

主要程序。

它负责：

```
读取 bag
→ 同步 RGB 和深度
→ YOLO 检测 chair
→ OpenCLIP 选择 red chair
→ 读取目标深度
→ 计算相机三维坐标
→ 通过 TF 转换到 map 坐标
→ 保存结果
```

### `scripts/launch_vlm_servers_lite.sh`

启动当前需要的两个模型服务：

```
OpenCLIP：端口 12182
YOLOv7：端口 12184
```

当前不启动 MobileSAM 和 GroundingDINO。

### `overlay/`

保存对原版 VLFM 模型服务的修改文件。

- `clipitm.py`：使用 OpenCLIP 提供图文相似度服务；
    
- `yolov7.py`：提供 YOLOv7 检测服务；
    
- `server_wrapper.py`：发送模型请求，并将超时时间设置为60秒。
    
## 6. 第一次使用

激活环境：

```
conda activate vlm
```

设置原版 VLFM 路径：

```
export VLFM_REPO="$HOME/projects/vlfm"
export PYTHONPATH="$VLFM_REPO:${PYTHONPATH:-}"
export LD_PRELOAD="$CONDA_PREFIX/lib/libstdc++.so.6"
```

安装离线读取 bag 所需的包：

```
pip install rosbags lz4
```

如果需要将 `overlay` 中的修改文件复制到原版 VLFM：

```
cp overlay/vlfm/vlm/clipitm.py \
  ~/projects/vlfm/vlfm/vlm/clipitm.py

cp overlay/vlfm/vlm/yolov7.py \
  ~/projects/vlfm/vlfm/vlm/yolov7.py

cp overlay/vlfm/vlm/server_wrapper.py \
  ~/projects/vlfm/vlfm/vlm/server_wrapper.py
```
## 7. 启动模型

执行：

```
conda activate vlm
cd ~/projects/vlfm

bash ~/projects/6008/vlm/scripts/launch_vlm_servers_lite.sh
```

查看模型加载状态：

```
tmux attach-session -t vlm_servers_lite
```

退出查看但保持模型运行：

```
先按 Ctrl+B，松开，再按 D
```

关闭模型服务：

```
tmux kill-session -t vlm_servers_lite
```
## 8. 运行测试

进入团队仓库：

```
cd ~/projects/6008/vlm
```

### 测试 OpenCLIP

```
python test_clip.py "/path/to/chair.jpg"
```

如果图片中是红色椅子，正常情况下 `best_match` 应为：

```
red chair
```

### 测试单张图片

```
python demo_action.py \
  "/path/to/chair.jpg" \
  "red chair" \
  --base-class chair
```

### 测试模拟坐标

```
python mock_semantic_map.py mock_slam_input.json
```

输出中的：

```
source: mock_input_only
```

表示这是模拟结果，不是真实相机定位。

### 检查 bag

```
python inspect_bag.py \
  "/path/to/g1_red_chair_demo.bag"
```

### 检查真实深度

```
python test_bag_depth.py \
  "/path/to/g1_red_chair_demo.bag"
```

### 运行真实语义定位

默认的 CLIP 模式需要 OpenCLIP 和 YOLOv7 都已经启动，然后执行：

```
python bag_vlm_localizer.py \
  "/path/to/g1_red_chair_demo.bag" \
  --target "red chair" \
  --base-class chair \
  --frame-stride 5 \
  --max-pairs 20
```

保留同一套 bag、深度、TF 和 map 坐标流程，也可以只使用 YOLOv7 + HSV
颜色筛选，不启动 OpenCLIP：

```
python bag_vlm_localizer.py \
  "/path/to/g1_red_chair_demo.bag" \
  --target "red chair" \
  --base-class chair \
  --semantic-backend hsv \
  --color-score-threshold 0.03 \
  --frame-stride 5 \
  --max-pairs 20
```

`--semantic-backend` 默认为 `clip`，因此原有命令的行为保持不变。HSV 模式
支持常见颜色 + YOLO 基础类别；输出中的语义分数字段会从
`clip_similarity` 改为 `color_score`。运行 HSV 模式时只需启动 YOLOv7 服务；
现有 `launch_vlm_servers_lite.sh` 为兼容旧流程仍会同时启动 OpenCLIP，额外启动
的 CLIP 不会被 HSV 路径调用，但会占用资源。

运行成功后会：

1. 在终端显示红色椅子的 `map` 坐标；
    
2. 生成 `semantic_memory.json`。
## 9. 真机接入说明

当前的 `bag_vlm_localizer.py` 读取离线 ROS bag，主要用于验证识别、深度计算和 TF 坐标转换。

最终在部署电脑上运行时，需要增加实时 ROS 版本，从相机和 SLAM Topic 接收数据。

用户目标暂定通过部署电脑终端输入：

```
python ros_vlm_node.py --target "red chair"
```
## 10. 当前状态

当前已经完成离线 ROS bag 中红色椅子的语义识别、深度计算和 `map` 坐标转换。

下一阶段：

- 确认与接口模块之间的消息格式；
    
- 将离线 bag 输入改为实时 ROS Topic 输入（真机运行时改成持续接收机器人正在发布的数据）新建 `ros_vlm_node.py`，把“从 bag 读取”换成 ROS 订阅。具体等Topic和输出消息类型确定；
    
- 与 SLAM 和导航模块进行联调。
