# G1：SLAM、最新版 VLM 与导航工作流

## 1. 文档范围

本文与当前最新版 VLM 目录 `semantic_target_locator` 对齐，说明：

- VLM 当前已经实现什么；
- VLM 的真实输入和输出；
- SLAM、VLM、导航与任务控制模块之间如何连接；
- 没有机器人时已经验证了什么；
- 接入机器人前还需要完成什么。

早期的 `semantic_navigation_prototype` 已删除，后续 VLM 开发以
`semantic_target_locator` 为准。原始 VLM 演示代码保持不变。

## 2. 最新职责划分

```text
SLAM
  提供机器人位姿、几何地图和 map 坐标系

RGB-D 相机
  提供 RGB、对齐深度和 CameraInfo

semantic_target_locator（最新版 VLM）
  识别目标并输出目标物体的 map 坐标

导航模块
  根据目标坐标和 costmap 选择安全接近点、规划并避障

任务控制模块
  控制搜索顺序和机器人旋转，协调 VLM 与导航
```

最新版 VLM **只负责目标定位**，不负责：

- 输出障碍物坐标；
- 读取或生成 costmap；
- 选择机器人最终接近点；
- 全局或局部路径规划；
- 控制机器人旋转；
- 发布 `/cmd_vel`；
- 判断导航是否成功。

## 3. 为什么不能只把 SLAM 点云交给 VLM

当前 SLAM 点云是几何点云，包含 XYZ、强度、法向量等信息，不包含与每个点严格
对应的 RGB 颜色。SLAM 本身也不知道一个点属于“椅子”还是“墙”。

目标的全局坐标来自以下信息的组合：

```text
RGB 图像中的目标区域
        +
目标区域的对齐深度
        +
CameraInfo 相机内参
        +
图像时刻的 map <- camera TF
        =
目标物体的 map 三维坐标
```

因此：

- 检测器配合 CLIP（通用模式）或颜色规则（轻量模式）回答“目标是谁、在图像哪里”；
- 深度回答“目标离相机多远”；
- SLAM/TF 回答“相机在地图中的位置和朝向”；
- `semantic_target_locator` 完成反投影、坐标变换和多帧融合。

## 4. 完整系统运行顺序

### 4.1 阶段 A：建图

```text
启动 LiDAR、机器人状态、SLAM 前端和后端
                    ↓
g1_map_manager 进入 MAPPING
                    ↓
人工遥控机器人覆盖环境
                    ↓
确认地图质量
                    ↓
停止建图并保存地图
```

完成这一步后只有几何地图，系统仍不知道“红色椅子”在哪里。

### 4.2 阶段 B：定位、搜索目标并导航

```text
加载地图并进入 LOCALIZATION
                    ↓
等待重定位成功和 map <- base_link TF 可用
                    ↓
启动 RGB-D 相机和最新版 VLM 节点
                    ↓
用户输入“红色的椅子”
                    ↓
任务控制模块让机器人受控旋转/搜索
                    ↓
VLM 持续接收同步 RGB-D
                    ↓
检测所有 chair，并用完整语义选择 red chair
                    ↓
深度反投影得到目标 camera 坐标
                    ↓
使用图像时间戳对应的 TF 转换到 map 坐标
                    ↓
多帧位置稳定后输出 TARGET_LOCKED
                    ↓
任务控制模块停止搜索旋转
                    ↓
导航侧结合 costmap 生成安全接近点
                    ↓
以 GOAL_POSE 发送给 g1_nav
                    ↓
规划、避障、运动执行，返回导航结果
```

VLM 在该过程中只发布目标坐标。机器人旋转和停止旋转必须由任务控制模块请求，
最终速度只能由导航/运动控制链路输出。

## 5. 最新版 VLM 的代码结构

目录：`vlm/semantic_target_locator`

| 文件 | 职责 |
|---|---|
| `contracts.py` | 定义帧、目标查询、二维检测、三维观测、最终目标和状态接口 |
| `locator.py` | 定位主流程：检测、深度、TF、多帧锁定和状态输出 |
| `memory.py` | 多帧中值融合、空间关联、离群观测过滤和稳定性判断 |
| `geometry.py` | 离线回放使用的三维点与 4×4 坐标变换 |
| `ros_adapter.py` | ROS1 的同步 RGB-D、YOLO/CLIP、深度反投影、TF 和结果发布适配器 |
| `ros_node.py` | ROS1 节点入口和参数组装 |
| `lite.py` | 中文/英文固定目标解析、HSV颜色评分和视觉限频 |
| `ros_lite_adapter.py` | 只连接YOLO、不连接CLIP的ROS轻量适配器 |
| `ros_lite_node.py` | 轻量 YOLO + HSV 模式的ROS1入口 |
| `LITE_MODE.md` | 轻量模式输入、参数、限制和测试说明 |
| `replay_adapters.py` | 没有 ROS 时使用的 JSON 回放适配器 |
| `run_replay.py` | 离线接口回放入口 |
| `test_locator.py` | 定位核心单元测试 |
| `run_real_image_contract_test.py` | 真实视觉模型 + 模拟深度/TF 的接口测试 |
| `DEVELOPMENT_LOG_2026-09-16.md` | 本轮开发记录和当前完成度 |

## 6. VLM 输入

### 6.1 目标查询

当前 ROS 查询话题：

```text
/semantic/target_query
```

推荐发送明确的 JSON：

```json
{
  "target_text": "red chair",
  "base_class": "chair"
}
```

- `target_text`：完整语义，供 CLIP 或轻量颜色解析器判断属性；
- `base_class`：基础检测类别，供 YOLO 找到所有候选。

通用模式仍推荐显式JSON。轻量模式已经可以把常见的“红色的椅子”“蓝色瓶子”
或“椅子”解析为基础检测类别；没有收录的物体仍应显式提供 `base_class`。

### 6.2 相机输入

默认 ROS 话题：

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

`RosSynchronizedFrameSource` 使用近似时间同步，生成同一观测时刻的：

- RGB 图像；
- 与 RGB 像素对齐的深度图；
- `CameraInfo`；
- 图像时间戳；
- 相机 optical frame 名称。

实际机器人上的话题名称、同步容差和深度单位均可通过 ROS 参数修改。

### 6.3 SLAM/TF 输入

SLAM 主要提供：

```text
map -> odom -> base_link
```

相机标定提供：

```text
base_link -> camera_color_optical_frame
```

最新版 VLM 按 RGB 图像时间戳查询：

```text
map <- camera_color_optical_frame
```

它不把 `/slam/map` 或 `/slam/frontend/cloud_registered` 直接输入检测器。

## 7. VLM 内部处理流程

### 7.1 二维语义识别

当前提供两种可选的二维识别入口。

通用模式 `ros_node.py`：

```text
YOLOv7：检测所有 base_class 候选，例如所有 chair
CLIP：对每个候选裁剪图与 target_text 计算相似度
```

系统综合检测置信度和文本相似度选择候选。

轻量模式 `ros_lite_node.py`：

```text
YOLOv7：检测所有 base_class 候选
有颜色描述：使用HSV颜色像素比例选择候选
无颜色描述：选择检测置信度最高的候选
```

轻量模式不启动CLIP，默认只以2 Hz处理最新同步帧，适用于“颜色 + 固定类别”任务。

本次离线红椅子测试使用仓库 Plan A 的 GroundingDINO + CLIP，验证了相同的
`VisionLanguageBackend` 接口。它证明接口能够接入真实模型，但不等于 ROS 中的
YOLOv7 sidecar 已经完成实机部署验证。

### 7.2 深度反投影

当前 `AlignedDepthProvider` 在检测框中采样有效深度点，并根据相机内参计算：

```text
X_camera = (u - cx) * D / fx
Y_camera = (v - cy) * D / fy
Z_camera = D
```

然后对三维点取中值，获得目标在相机坐标系中的代表位置。

这是当前可用的基础实现。椅背空隙、椅腿之间的背景可能污染深度，后续应增加
目标分割掩膜或前景深度聚类以提高真实坐标精度。

### 7.3 转换到 map 坐标

```text
P_map = T_map_camera(image_stamp) * P_camera
```

如果深度无效或对应时间的 TF 不可用，模块发布错误状态，不生成虚假的目标坐标。

### 7.4 多帧锁定

`ObservationMemory` 对多帧三维观测进行空间关联、中值融合和离群过滤。只有满足：

- 最少观测次数；
- 关联距离阈值；
- 最大位置离散度；

才会输出 `TARGET_LOCKED`。

## 8. VLM 输出

默认状态话题：

```text
/semantic/target_status
```

成功输出示例：

```json
{
  "status": "TARGET_LOCKED",
  "message": "target position locked",
  "target": {
    "label": "red chair",
    "frame_id": "map",
    "position": {
      "x": 2.732,
      "y": 3.0,
      "z": 0.42
    },
    "confidence": 0.789,
    "observation_count": 3,
    "position_spread_m": 0.014
  }
}
```

可能状态包括：

```text
SEARCHING
NO_DETECTION
DEPTH_INVALID
TF_UNAVAILABLE
TARGET_DETECTED_3D
TARGET_UNSTABLE
TARGET_LOCKED
TIMEOUT
```

这里的 `position` 是目标物体位置，不是安全导航终点。VLM 不输出
`approach_pose`、障碍物列表或 `navigation_result`。

## 9. 为什么目标坐标不能直接发送给导航

椅子中心通常位于障碍区域。直接把椅子 `(x, y)` 作为 `GOAL_POSE`，可能导致：

- 目标点位于 costmap 障碍物内部；
- 机器人与椅子距离过近；
- 规划器判断目标不可达。

导航侧仍需实现接近点生成器：

1. 在目标周围生成保持安全距离的候选点；
2. 使用全局 costmap 删除障碍、未知或空间不足的候选；
3. 检查候选是否存在可行路径；
4. 选择代价合适的点并令机器人朝向目标；
5. 将该点作为 `GOAL_POSE` 发送给 `g1_nav`。

`g1_costmap` 已属于导航栈，不属于 VLM 或 SLAM。

## 10. 任务控制与机器人旋转

机器人“转一圈寻找目标”的逻辑应放在独立 orchestrator 状态机中：

```text
WAIT_LOCALIZATION
        ↓
START_TARGET_SEARCH
        ↓
请求导航/运动模块低速旋转
        ↓
VLM 持续处理 RGB-D
        ↓
TARGET_LOCKED 或 TIMEOUT
        ↓
停止旋转
        ↓
把目标 map 坐标交给导航侧接近点生成器
```

orchestrator 负责流程协调，但不应自己实现检测、SLAM、costmap或路径规划，
也不应绕过 `g1_nav` 直接与其他节点争抢 `/cmd_vel`。

## 11. 当前验证结果

### 11.1 核心单元测试

当前三项测试通过：

- 正常回放能够进行多帧融合并锁定 `map` 坐标；
- 没有检测结果时最终超时，不生成假坐标；
- TF 缺失时发布 `TF_UNAVAILABLE`。

运行方式：

```bash
cd vlm/semantic_target_locator
python -m unittest -v test_locator.py
```

### 11.2 红色椅子视觉测试

受控测试图包含红、蓝、灰三把椅子，输入：

```json
{"target_text": "red chair", "base_class": "chair"}
```

GroundingDINO 检测到三把椅子，CLIP 正确选择红色椅子。随后通过模拟的 2 m
对齐深度、CameraInfo 和固定 `map <- camera` 变换，完整链路输出了目标 `map`
坐标和 `TARGET_LOCKED`。

该测试中的视觉推理是真实模型推理，但深度和 TF 是模拟数据，所以只证明：

- 目标文本可以选中正确候选；
- 检测结果可以进入三维定位核心；
- 输出契约和坐标计算链路可工作。

它不证明真实机器人上的坐标误差已经满足要求。

### 11.3 轻量模式测试

轻量模式直接输入“红色的椅子”，使用真实检测器找到红、蓝、灰三把椅子，再用
HSV颜色比例完成选择。红色椅子的颜色得分为 `0.322`，蓝色和灰色候选接近
`0`，正确选择红色椅子；测试过程未加载CLIP。

轻量模式和原定位核心合计八项单元测试通过。

## 12. 当前完成度

### 已完成

- 目标定位核心接口；
- 同步帧、视觉后端、深度、TF和结果发布的依赖分层；
- RGB检测候选与文本语义选择；
- 对齐深度反投影；
- `camera -> map` 坐标变换；
- 多帧融合与目标锁定；
- JSON replay；
- ROS1 适配代码骨架；
- 不依赖CLIP的轻量 YOLO + HSV 模式；
- 常见中英文颜色与固定物体类别解析；
- 默认2 Hz最新帧处理；
- 单元测试和真实视觉模型接口测试；
- 目标 `map` 坐标输出契约。

### 尚未完成或尚未验证

- 真实机器人相机 topic、编码、内参和深度单位确认；
- `base_link <- camera` 外参标定；
- ROS Ubuntu 环境下的 catkin 构建、launch 和 rosbag 回放；
- YOLOv7/CLIP sidecar 的正式部署与联调；
- 任意中文目标描述和未收录类别的通用解析；
- 分割或深度聚类带来的前景坐标精度提升；
- 上层搜索 orchestrator；
- 导航侧安全接近点生成器；
- VLM、costmap 和 `g1_nav` 的全链路实机联调。

因此当前状态应描述为：

> 最新版 VLM 已完成“目标识别、RGB-D 三维定位、多帧融合和 map 坐标输出”的
> 核心代码及 ROS 接口预留；可以进入真实传感器接入阶段，但尚未完成实机验收。

## 13. 后续推荐顺序

1. 确定 RGB-D 相机型号和真实 ROS topic；
2. 确认深度对齐方式、编码和单位；
3. 标定并验证 `base_link <- camera`；
4. 把 `semantic_target_locator` 整理为 catkin package 并增加 launch；
5. 用录制的 RGB-D + TF rosbag 验证目标 `map` 坐标；
6. 根据实际任务扩充轻量模式中文物体别名；
7. 加入分割掩膜或前景深度聚类；
8. 实现搜索 orchestrator；
9. 在导航模块中实现安全接近点生成器；
10. 最后在有急停和安全员的条件下进行低速实机联调。

## 14. 验收条件

最新版 VLM 的实机验收至少应满足：

- RGB、深度和 CameraInfo 正确同步；
- 使用图像时间戳查询 TF，过期或缺失 TF 不输出坐标；
- 多把椅子存在时可以选择符合描述的红色椅子；
- 深度无效时不生成目标坐标；
- 同一静态目标的多帧 `map` 坐标离散度低于设定阈值；
- 重复搜索同一目标时输出位置基本一致；
- VLM 输出中不包含导航成功结果或伪造的可达点；
- 目标丢失或超时时明确返回状态，不让任务控制模块无限旋转。

完整系统另外需要验证：导航安全接近点位于可通行区域、只有 `g1_nav` 写
`/cmd_vel`，并且取消、无路径、传感器掉线和急停时机器人能够可靠停止。
