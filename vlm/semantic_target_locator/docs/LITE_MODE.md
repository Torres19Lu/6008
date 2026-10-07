# Semantic target locator 轻量模式

## 用途

轻量模式面向目标类型相对固定的任务，例如：

```text
红色的椅子
蓝色瓶子
黄色杯子
椅子
背包
```

它删除 CLIP 依赖，只保留一个基础物体检测器，并使用 HSV 颜色规则在同类候选
中选择指定颜色的物体。深度定位、TF、多帧融合和最终输出与通用模式完全共用。

## 两种模式的差异

| 项目 | 通用模式 | 轻量模式 |
|---|---|---|
| 基础检测 | YOLO | YOLO |
| 属性选择 | CLIP图文相似度 | HSV颜色像素比例 |
| 模型服务 | YOLO + CLIP | 只需要YOLO |
| 默认推理频率 | 由模型处理速度决定 | 2 Hz，可配置 |
| 目标语言 | 较灵活的完整描述 | 常见颜色 + 固定物体类别 |
| 输出 | 目标 `map` 坐标 | 相同的目标 `map` 坐标 |

## 内部流程

```text
目标文字
  ↓
解析颜色和基础类别
  ↓
YOLO检测全部基础类别候选
  ↓
有颜色：计算每个候选框的HSV颜色比例
无颜色：使用检测置信度
  ↓
选出最佳二维候选
  ↓
对齐深度反投影
  ↓
map <- camera TF
  ↓
三帧融合并输出TARGET_LOCKED
```

## 查询输入

常用中文和英文可以直接发送：

```bash
rostopic pub -1 /semantic/target_query std_msgs/String "data: '红色的椅子'"
rostopic pub -1 /semantic/target_query std_msgs/String "data: 'blue bottle'"
rostopic pub -1 /semantic/target_query std_msgs/String "data: '椅子'"
```

没有收录的物体类别可以显式指定检测器类别：

```json
{
  "target_text": "蓝色工具箱",
  "base_class": "toolbox"
}
```

注意：`base_class` 必须是所使用检测器真实支持的类别。

## 内置颜色

```text
红、橙、黄、绿、青、蓝、紫、粉、棕、黑、白、灰
```

对应英文颜色也可以使用。

## 内置常用物体解析

目前包含：

```text
chair, bottle, cup, person, backpack, couch, dining table,
potted plant, tv, laptop, cell phone, book, bed, toilet, car, bicycle
```

解析表位于 `lite.py::OBJECT_ALIASES`，可按实际任务继续增加。

## ROS入口

轻量节点：

```text
ros_lite_node.py
```

它使用：

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/semantic/target_query
/semantic/target_status
map <- camera TF
YOLOv7 HTTP sidecar（默认端口12184）
```

不连接CLIP端口。

主要参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `inference_rate_hz` | `2.0` | 每秒最多处理的同步帧数 |
| `detector_confidence` | `0.25` | YOLO最低置信度 |
| `color_score_threshold` | `0.03` | 目标颜色至少占候选框的比例 |
| `min_observations` | `3` | 锁定目标所需三维观测数 |
| `depth_scale_to_m` | `0.001` | 深度单位到米的比例 |

正式使用前仍需把目录整理成 catkin package 并增加 launch 文件；当前节点代码是预留
入口，尚未在Ubuntu/ROS真机环境中运行。

## 输出

输出契约与通用模式相同：

```json
{
  "status": "TARGET_LOCKED",
  "target": {
    "label": "红色的椅子",
    "frame_id": "map",
    "position": {"x": 2.73, "y": 3.0, "z": 0.42},
    "confidence": 0.23,
    "observation_count": 3,
    "position_spread_m": 0.02
  }
}
```

轻量模式中的 `confidence` 是检测置信度与颜色占比的组合分数，不是概率，也不能
与通用CLIP模式的数值直接比较。

## 已完成测试

- 中文“红色的椅子”可解析为 `base_class=chair`、`color=red`；
- 无颜色目标可以直接选择检测置信度最高的候选；
- 红、蓝、灰三把椅子的受控测试图中，红椅子被正确选中；
- 测试过程没有加载CLIP；
- 原有深度、TF和多帧融合测试继续通过。

运行：

```bash
python -m unittest -v test_lite.py test_locator.py
python run_lite_image_test.py
```

## 限制

- HSV颜色受照明、阴影、反光和相机白平衡影响；
- 检测框中大面积同色背景可能干扰候选选择；
- 黑、白、灰等低饱和颜色比红、蓝、绿色更容易受背景影响；
- 不理解“靠近门的椅子”“最大的杯子”等复杂关系；
- 只能识别基础检测器支持的类别；
- 真实三维精度仍取决于RGB-D对齐、相机标定、目标前景深度和TF。

如果现场颜色变化明显，可在不恢复CLIP的情况下，针对实际相机图片调整
`lite.py` 中的HSV阈值。
