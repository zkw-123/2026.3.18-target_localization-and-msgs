# target_localization workspace

本工作空间包含两个 ROS2 包，配合使用，实现超声图像与 NDI 光学追踪的实时目标定位。

---

## 包结构

```
src/
├── target_localization        # 主功能包
└── target_localization_msgs   # 自定义消息定义包
```

---

## target_localization_msgs

定义本工作空间使用的自定义消息类型。

### TargetSk.msg

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳与坐标系 ID |
| `target_m` | `geometry_msgs/Point` | 目标在世界坐标系中的 3D 位置（mm） |
| `sk` | `float32` | 感知稳定性指数（stability index） |
| `target_pix` | `geometry_msgs/Point` | 目标在图像中的像素坐标（x, y，z 置 0） |

---

## target_localization

实时目标定位主包。订阅超声视频流和 NDI Polaris 光学追踪数据，经时间同步、目标检测和坐标变换后，输出目标的 3D 世界坐标。

### 节点一览

| 节点 | 功能 |
|------|------|
| `ultrasound_publisher_node` | 采集超声图像，发布到 `/us_image_raw` |
| `polaris_publisher_node` | 读取 NDI 追踪数据，发布到 `/ndi_transforms` |
| `data_synchronizer_node` | 按时间戳配对图像与追踪数据（容差 <5ms），发布到 `/synchronized_data` |
| `target_detector_node` | 在图像中检测目标像素坐标，发布到 `/target_pixel`（当前为占位实现，需替换） |
| `target_localizer_node` | 将像素坐标经标定矩阵变换为 3D 世界坐标，发布到 `/target_3d_position` |
| `realtime_visualizer_node` | 将检测结果叠加到图像上显示或发布 |
| `data_recorder_node` | 异步保存图像与追踪数据到磁盘（可选） |

### 主要话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/us_image_raw` | `sensor_msgs/Image` | 实时超声图像 |
| `/ndi_transforms` | `std_msgs/String` | NDI 追踪数据（JSON） |
| `/synchronized_data` | `std_msgs/String` | 时间配对后的图像元数据 + 追踪数据（JSON） |
| `/target_pixel` | `std_msgs/String` | 检测到的像素坐标与置信度（JSON） |
| `/target_3d_position` | `std_msgs/String` | 目标 3D 世界坐标（JSON，单位 mm） |

### 依赖

- ROS2：`rclpy` `std_msgs` `sensor_msgs` `geometry_msgs` `cv_bridge` `message_filters`
- Python：`numpy` `opencv-python` `scikit-surgerynditracker`
- 本工作空间：`target_localization_msgs`

---

## 快速启动

```bash
# 构建
colcon build --packages-select target_localization_msgs target_localization
