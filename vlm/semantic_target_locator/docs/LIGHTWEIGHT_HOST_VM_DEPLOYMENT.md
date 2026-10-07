# 轻量模式：Windows宿主机与Ubuntu虚拟机职责

## 1. 固定部署方案

后续轻量模式按以下边界开发和运行：

```text
Windows宿主机：使用RTX 4060运行二维物体检测模型
Ubuntu虚拟机：运行ROS、SLAM、颜色选择、三维定位、导航和G1通信
```

轻量VLM用于“某个颜色的某种物体”或直接指定物体类别，例如“红色的椅子”或
“瓶子”。它不运行CLIP，使用“基础物体检测 + HSV颜色判断”。

## 2. 数据流

```text
Ubuntu RGB图像
      ↓ HTTP
Windows GPU检测服务
      ↓ 类别、检测框、置信度
Ubuntu HSV颜色选择
      ↓
Ubuntu RGB-D深度反投影
      ↓
Ubuntu查询SLAM的map <- camera TF
      ↓
目标map坐标
      ↓
Ubuntu导航模块
      ↓
g1_locomotion -> Unitree SDK2/DDS -> G1
```

## 3. Windows宿主机职责

### 运行

- Python、PyTorch、CUDA；
- YOLO或GroundingDINO物体检测服务；
- HTTP检测端口，建议使用 `12184`。

检测服务接收RGB图像和基础类别，返回归一化二维检测框和检测置信度。

### 不运行

- ROS Master；
- SLAM、TF和地图管理；
- RGB-D深度计算；
- HSV颜色选择；
- costmap和规划器；
- `/cmd_vel`或G1控制；
- CLIP。

### 网络要求

- 服务必须监听 `0.0.0.0`，不能只监听 `127.0.0.1`；
- Windows防火墙允许虚拟机访问检测端口；
- 宿主机使用一个可从虚拟机访问的固定IP。

## 4. Ubuntu虚拟机职责

### ROS与机器人接口

- `roscore`；
- `g1_locomotion`；
- 机器人状态与TF；
- Unitree SDK2/DDS。

### 感知与定位

- LiDAR驱动；
- SLAM前端、后端与 `g1_map_manager`；
- RGB-D相机、对齐深度和 `CameraInfo`；
- `semantic_target_locator/ros_lite_node.py`；
- 中文/英文固定目标解析；
- HSV颜色选择；
- 深度反投影、`camera -> map` TF和多帧融合。

### 导航与任务控制

- `g1_costmap`；
- 安全接近点生成器（仍需实现）；
- 全局规划器、局部规划器和 `g1_nav`；
- 搜索与导航 orchestrator（仍需实现）。

Ubuntu虚拟机不运行CLIP，也不应在CPU上运行大型视觉模型。任务脚本不能绕过
`g1_nav`与其他节点争抢最终 `/cmd_vel`。

## 5. 分阶段运行

为适配当前约16GB物理内存，不同时启动所有模块。

### 阶段一：建图

Windows宿主机：不启动视觉模型。

Ubuntu虚拟机运行：

```text
roscore
g1_locomotion
LiDAR
SLAM前端和后端
g1_map_manager
RViz（仅检查地图时）
```

完成“人工遥控覆盖环境 -> 保存地图”后，关闭RViz和建图阶段不再需要的节点。

### 阶段二：定位并寻找目标

Windows宿主机运行GPU物体检测服务。

Ubuntu虚拟机运行：

```text
roscore
g1_locomotion
LiDAR与SLAM定位模式
RGB-D相机
ros_lite_node.py
搜索控制逻辑
```

执行流程：

```text
机器人受控旋转
-> Ubuntu发送RGB给Windows
-> Windows返回候选框
-> Ubuntu用HSV选择指定颜色
-> 深度与TF生成目标map坐标
-> 三帧稳定后TARGET_LOCKED
-> 停止旋转
```

### 阶段三：导航

目标是静态物体并已锁定后，可以停止Windows检测服务、轻量VLM节点和非必要的
RGB-D处理，释放内存和GPU。

Ubuntu虚拟机运行：

```text
SLAM定位与LiDAR
g1_costmap
安全接近点生成器
全局/局部规划器
g1_nav
g1_locomotion
任务总控
```

导航侧根据目标 `map` 坐标生成安全接近点，并以 `GOAL_POSE` 驱动G1。

## 6. 网络接口建议

虚拟机建议配置两个网络接口：

```text
虚拟网卡1：Host-only或可访问宿主机的网络
            用于Ubuntu访问Windows检测服务

虚拟网卡2：桥接到连接G1/LiDAR的有线网卡
            用于Unitree DDS和传感器通信
```

示例：

```text
Windows宿主机：192.168.56.1
Ubuntu虚拟机：192.168.56.101
检测服务：http://192.168.56.1:12184
```

Ubuntu虚拟机需要检查：

```bash
ping 192.168.56.1
curl http://192.168.56.1:12184
ip -br addr
```

如果检测服务没有根路径，`curl` 返回404也可以说明端口可达；连接超时或拒绝才
表示网络、监听地址或防火墙仍存在问题。

## 7. 当前代码状态

已经完成：

- `lite.py`：目标解析、HSV颜色评分和2 Hz限频；
- `ros_lite_adapter.py`：YOLO检测结果适配；
- `ros_lite_node.py`：轻量ROS节点入口；
- 深度、TF、多帧融合和目标 `map` 坐标输出；
- 红、蓝、灰三把椅子中正确选择红椅子的测试；
- 轻量模式和定位核心单元测试。

接入宿主机检测服务前仍需完成：

1. 把检测服务器地址从固定 `localhost` 改成ROS参数，例如
   `detector_host=192.168.56.1` 和 `detector_port=12184`；
2. 在Windows启动监听局域网地址的YOLO或GroundingDINO服务；
3. 在Ubuntu中补充catkin package和launch；
4. 验证虚拟机桥接网卡可以传递Unitree DDS；
5. 验证RGB-D USB直通、LiDAR和TF；
6. 实现安全接近点生成器和任务总控。

在完成第1项前，虚拟机中的轻量节点仍会连接虚拟机自己的
`localhost:12184`，不能访问Windows宿主机模型。

## 8. 安全边界

- 只有 `g1_nav` 应向最终 `/cmd_vel` 输出运动指令；
- 确认机器人状态后再调用 `/g1/arm`；
- 使用 `/g1/halt`验证软停止；
- 保留G1遥控器和实体急停；
- 网络或虚拟机卡顿时必须依靠watchdog归零速度；
- 首次联调使用低速度、空旷区域和安全员；
- 先验证断网、停止ROS和关闭虚拟机时机器人能够可靠停止。

## 9. 最终边界

```text
Windows宿主机：只做GPU二维物体检测
Ubuntu虚拟机：负责ROS、SLAM、颜色选择、三维定位、导航和G1控制
```

地图文件只给SLAM定位和导航使用。轻量VLM不读取地图文件，而是使用RGB-D和
SLAM提供的 `map <- camera` TF输出目标 `map` 坐标。
