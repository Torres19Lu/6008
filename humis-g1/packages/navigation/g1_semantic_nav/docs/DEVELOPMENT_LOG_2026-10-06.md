# 开发日志：轻量语义导航接入

日期：2026-10-06（Asia/Singapore）

## 1. 修改背景

当前仿真环境中已经跑通以下链路：

```text
Gazebo 传感器
  -> SLAM 建图与 TF
  -> 实时 VLM
  -> 红色椅子的 map 坐标
```

第一阶段不调整已经跑通的 SLAM 和 VLM，而是在导航侧补齐：

```text
VLM 目标坐标
  -> 目标旁边的安全接近点
  -> 原有 g1_nav 导航栈
  -> /cmd_vel
  -> Gazebo 机器人运动
```

另外新增仓库根目录下的 `run_red_chair_navigation.sh`，将模型服务、Gazebo、
SLAM、实时 VLM 和语义导航合并为一个非交互式启动入口。脚本默认目标为
`red chair`，支持增量构建、启动状态检查、日志分流和退出时统一清理进程。

### 1.1 真机兼容扩展

在保留全部仿真入口的基础上，本次继续增加真实 G1 支持：

- 新增 `run_g1_red_chair_navigation.sh`，只启动真实传感器与 G1 运动接口，
  不启动 Gazebo 或仿真速度桥；
- 新增 `semantic_navigation_real.launch`；
- 实时 VLM 同时兼容仿真的 `32FC1/m` 和 RealSense 的 `16UC1/mm` 深度；
- 相机、深度、CameraInfo 和模型服务地址改为 ROS 参数，默认值保持原仿真接口；
- G1 description 可以从环境变量读取测量后的 D435 安装外参；
- 真机启动链路使用 `start_state=true`、`start_locomotion=false`，避免重复
  启动 `g1_locomotion`；
- 语义目标适配器增加 `/semantic_goal_adapter/enable` 安全门，真机默认关闭；
- 真机脚本退出时优先调用 `/g1/halt`，再关闭进程；
- 自动 arm 必须同时提供 `--auto-arm` 和 `--i-understand-motion`。

## 2. 新增的导航包

新增 ROS 包：

```text
humis-g1/packages/navigation/g1_semantic_nav
```

该包不是重新实现 costmap、全局规划器或局部规划器，而是连接现有模块。

### 2.1 VLM 目标到导航目标的转换

新增 `scripts/semantic_goal_adapter.py`：

- 订阅 `/vlm/semantic_target`；
- 只接受 `stable=true` 的目标；
- 订阅 `/vlm/target_pose`，获取目标的 `map` 坐标；
- 订阅 `/nav/costmap`；
- 通过 TF 获取导航开始时机器人的实时位置；
- 在目标周围生成候选接近点；
- 排除地图外、未知区域和高代价障碍区域；
- 让接近点的朝向指向目标物体；
- 将候选点以 `g1_msgs/NavigateToAction` 的 `GOAL_POSE` 发送给
  `/navigate_to`；
- 当前候选点导航失败时，自动尝试下一个候选点。

默认优先选择距离目标约 1.2 米、位于机器人和目标连线上的位置。目标
坐标仍代表物体本身，导航终点代表目标旁边可通行的位置。

### 2.2 安全接近点算法

新增 `src/navigation_tools/goal_selection.py`：

- 与 ROS 解耦，便于单元测试；
- 支持带旋转原点的 OccupancyGrid 坐标转换；
- 默认使用 1.2、1.0、1.4 米三个候选半径；
- 默认从目标朝向机器人的方向开始，再尝试左右和目标背面的候选点；
- 拒绝 costmap 中的未知点、地图外点和超过代价阈值的点；
- 根据机器人到候选点的距离、costmap 代价和角度偏移进行排序。

### 2.3 Gazebo 速度执行桥

新增 `scripts/sim_cmd_vel_bridge.py`：

- 只订阅 `/cmd_vel`，不发布第二路速度指令；
- 将 `linear.x`、`linear.y` 和 `angular.z` 转换为 Gazebo 模型运动；
- 限制最大前进、横向和旋转速度；
- 在速度指令超时后自动停止；
- 发布 `/simulation/ground_truth_odom`，供现有仿真 Livox 桥使用。

该脚本只用于 Gazebo。真机运行时应由 `g1_locomotion` 接收 `/cmd_vel`，
不能启动这个仿真桥。

### 2.4 仿真导航启动文件

新增 `launch/semantic_navigation_sim.launch`：

- 默认启动现有 `g1_costmap`；
- 默认启动现有 `g1_global_planner`；
- 默认启动现有 `g1_local_planner`；
- 默认启动现有 `g1_nav`；
- 启动语义目标转换节点；
- 启动 Gazebo `/cmd_vel` 执行桥；
- 不重复启动已经运行的 Gazebo、SLAM 或 VLM；
- 仿真模式关闭硬件 `auto_arm`。

### 2.5 参数配置

新增 `config/navigation.yaml`，集中配置：

- VLM 状态和目标坐标话题；
- costmap 和导航 action 名称；
- 默认目标 `red chair`；
- 接近距离；
- 候选方向；
- costmap 最大可接受代价；
- 目标更新阈值与失败重试间隔。

## 3. 对已有导航模块的修改

修改 `humis-g1/packages/navigation/g1_nav/launch/nav.launch`：

- 新增 `auto_arm` 启动参数；
- 默认值仍为 `true`，保持真机原有安全行为；
- 仿真启动文件显式传入 `auto_arm:=false`，避免依赖真机的
  `/g1/arm` 和 `/g1/halt` 服务。

修改 `humis-g1/packages/navigation/g1_nav/README.md`：

- 补充 `auto_arm` 的真机和仿真使用说明。

修改 `humis-g1/packages/navigation/g1_nav/test/test_nav_launch.py`：

- 增加 `auto_arm` 启动参数契约测试；
- 确认默认仍为 `true`；
- 确认参数正确传递给 `g1_nav_node`。

## 4. 新增文件清单

```text
run_red_chair_navigation.sh
run_g1_red_chair_navigation.sh

humis-g1/packages/navigation/g1_semantic_nav/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── README.md
├── config/
│   └── navigation.yaml
├── docs/
│   └── DEVELOPMENT_LOG_2026-10-06.md
├── launch/
│   ├── semantic_navigation_sim.launch
│   └── semantic_navigation_real.launch
├── scripts/
│   ├── semantic_goal_adapter.py
│   └── sim_cmd_vel_bridge.py
├── src/navigation_tools/
│   ├── __init__.py
│   └── goal_selection.py
└── test/
    ├── test_goal_selection.py
    └── test_real_launch_contract.py
```

## 5. 修改文件清单

```text
humis-g1/packages/navigation/g1_nav/launch/nav.launch
humis-g1/packages/navigation/g1_nav/README.md
humis-g1/packages/navigation/g1_nav/test/test_nav_launch.py
humis-g1/packages/platform/g1_description/launch/description.launch
humis-g1/packages/platform/g1_description/README.md
humis-g1/packages/navigation/g1_semantic_nav/CMakeLists.txt
humis-g1/packages/navigation/g1_semantic_nav/package.xml
humis-g1/packages/navigation/g1_semantic_nav/README.md
humis-g1/packages/navigation/g1_semantic_nav/scripts/semantic_goal_adapter.py
slam/ros_vlm_node.py
```

新增的真机输入兼容文件：

```text
slam/vlm_sensor_utils.py
slam/tests/test_vlm_sensor_utils.py
```

## 6. 修改边界

导航接入阶段没有改变 VLM 的识别和定位算法。真机兼容阶段只对
`slam/ros_vlm_node.py` 做了以下输入边界调整：

- RGB、深度、CameraInfo 话题改为参数，默认值保持原仿真话题；
- YOLO、OpenCLIP 服务地址改为参数，默认地址保持不变；
- 深度图以 `passthrough` 读取，再把 `16UC1` 毫米转换为米；
- JPEG 编码前显式执行 RGB 到 BGR 转换，避免红蓝通道颠倒。

以下算法或已有资源没有修改：

- `slam/run_realtime_vlm.sh`；
- `slam/g1_simulation/` 中原有文件；
- `slam/sim_classroom/` 地图；
- `vlm/` 中原有 VLM 代码；
- SLAM 前端与后端实现；
- costmap、全局规划器和局部规划器的算法实现。

## 7. 验证结果

本次已完成的代码级验证：

- 新增安全接近点算法：3 项单元测试全部通过；
- 原有 `g1_nav` 启动与配置契约：22 项测试全部通过；
- 新增 Python 文件语法检查通过；
- `package.xml` 和 ROS launch XML 解析通过；
- Bash 启动脚本语法检查和真机脚本 `--help` 检查通过；
- 确认新包没有发布第二路 `/cmd_vel`；
- VLM 深度单位转换新增 3 项测试；
- 真机 launch 新增静态安全契约测试，检查不包含 Gazebo/仿真桥、
  `g1_locomotion` 只启动一次且语义目标门默认关闭。
- 真机脚本会等待 `/g1/loco_status`，确认机器人运动接口节点已经上线；
- 安全门在候选点计算完成和发送目标前都会再次检查；`send_goal` 与
  `cancel_goal` 使用同一动作锁串行化，关闭安全门会可靠取消当前目标。

尚未完成：

- Ubuntu ROS/catkin 实际构建；
- Gazebo 中从 VLM 坐标到自动导航的全链路运行；
- 静态和动态障碍物绕行测试；
- 真机 G1 联调。

## 8. 后续运行方式

在 Ubuntu 中构建：

```bash
cd /path/to/6008/humis-g1
./scripts/link_workspace.sh
cd catkin_ws
catkin build g1_nav g1_semantic_nav
source devel/setup.bash
```

保持已经跑通的 Gazebo、SLAM 和实时 VLM 流程运行，在另一个终端启动：

```bash
source /path/to/6008/humis-g1/catkin_ws/devel/setup.bash
roslaunch g1_semantic_nav semantic_navigation_sim.launch
```

此时预期链路为：

```text
/vlm/semantic_target + /vlm/target_pose
  -> semantic_goal_adapter
  -> /navigate_to
  -> g1_nav
  -> /cmd_vel
  -> sim_cmd_vel_bridge
  -> Gazebo G1
```

真机使用独立入口，避免误启动 Gazebo 速度桥：

```bash
cd /path/to/6008
bash ./run_g1_red_chair_navigation.sh \
  --map-path /path/to/saved_real_map \
  --interface eth0 \
  --camera-xyz "MEASURED_X MEASURED_Y MEASURED_Z" \
  --camera-rpy "MEASURED_ROLL MEASURED_PITCH MEASURED_YAW"
```

该脚本默认不切换 G1 的 FSM、不自动 arm，并让语义导航安全门保持关闭。
操作者检查传感器、TF、VLM 坐标与 costmap 后，再按现场批准的流程进入
平衡行走模式，调用 `/g1/arm`，最后显式启用
`/semantic_goal_adapter/enable`。退出脚本时会先请求 `/g1/halt`。
