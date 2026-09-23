# G1 VLM 仿真数据交接

## ROS bag

- 文件：`g1_red_chair_demo.bag`
- 时长：16.5 秒
- 压缩：LZ4
- 场景：Gazebo 教室，包含红色椅子
- ROS：ROS 1 Noetic
- 注意：这是仿真数据，时间来自 `/clock`

## Topic 接口

| 内容 | Topic | ROS 消息类型 | 说明 |
|---|---|---|---|
| RGB 图像 | `/camera/color/image_raw` | `sensor_msgs/Image` | `rgb8`，约 15 Hz |
| 对齐深度图 | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | `32FC1`，单位米，约 15 Hz |
| RGB 相机内参 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RGB 投影参数 |
| 对齐深度内参 | `/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 与彩色画面对齐 |
| SLAM 里程计 | `/slam/frontend/odom` | `nav_msgs/Odometry` | `odom -> base_link`，约 5 Hz |
| SLAM 全局点云地图 | `/slam/map` | `sensor_msgs/PointCloud2` | frame 为 `map` |
| 动态坐标变换 | `/tf` | `tf2_msgs/TFMessage` | 包含运动坐标关系 |
| 静态坐标变换 | `/tf_static` | `tf2_msgs/TFMessage` | 传感器安装关系 |
| 仿真时间 | `/clock` | `rosgraph_msgs/Clock` | 回放时必须使用 `--clock` |

RGB 和对齐深度应使用各自消息的时间戳进行近似同步。相机光学坐标系为
`camera_color_optical_frame`；语义目标最终应变换并保存到 `map` 坐标系。

## 回放

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source /home/ruruka/下载/humis-g1/humis-g1/catkin_ws/devel/setup.bash
rosparam set /use_sim_time true
rosbag play --clock g1_red_chair_demo.bag
```

## 额外地图数据

完整后端地图目录：

`packages/mapping/g1_maps/maps/sim_classroom`

其中包括 `manifest.yaml`、`pose_graph.g2o`、`scan_context.bin` 和
`keyframes/*.pcd`。仅运行 VLM 的 RGB-D 语义定位演示时，优先使用 bag 中的
`/slam/map`；需要复用后端地图或重定位时再使用完整地图目录。
