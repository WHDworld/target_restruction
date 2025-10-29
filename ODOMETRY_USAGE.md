# 使用 Odometry 真值位姿说明

## 📌 概述

本目标重建系统已配置为使用**真值位姿**（Ground Truth Pose），通过订阅 `nav_msgs/Odometry` 话题获取实时相机位姿，**无需运行额外的定位/SLAM系统**。

---

## 🔧 配置修改

### 1. Launch 文件配置

文件：`launch/target_reconstruction.launch`

```xml
<!-- Ground Truth Pose from Odometry -->
<remap from="/camera/odom" to="/odom" />
```

**说明**：
- 节点订阅 `/camera/odom` 话题
- 通过 `remap` 映射到实际的 odometry 话题（如 `/odom`）
- 位姿可以来自：
  - ✅ Gazebo 仿真器（`/gazebo/odom`）
  - ✅ FAST-LIVO2 输出（`/aft_mapped_to_init`）
  - ✅ 动捕系统（Vicon、OptiTrack 等）
  - ✅ 任何提供真值的系统

### 2. 代码修改

文件：`src/main.cpp`

**修改点**：
- ✅ 包含头文件：`nav_msgs/Odometry.h`
- ✅ 订阅器类型：`nav_msgs::Odometry`
- ✅ 位姿提取：`odom_msg->pose.pose.position` 和 `odom_msg->pose.pose.orientation`

---

## 📡 话题接口

### 订阅的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/object_detection/bounding_box` | `target_reconstruction/BoundingBox` | 目标检测框 |
| **`/camera/odom`** | **`nav_msgs/Odometry`** | **相机真值位姿（新）** |

### 发布的话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/target_reconstruction/pointcloud` | `sensor_msgs/PointCloud2` | 重建的目标点云 |

---

## 🚀 使用示例

### 场景 1：Gazebo 仿真

```bash
# 1. 启动 Gazebo 仿真（自动发布 /odom）
roslaunch your_gazebo_package simulation.launch

# 2. 启动相机和深度传感器（在 Gazebo 中）

# 3. 启动目标检测
roslaunch your_detector detector.launch

# 4. 启动重建节点，映射 odom 话题
roslaunch target_reconstruction target_reconstruction.launch
```

修改 launch 文件中的映射：
```xml
<remap from="/camera/odom" to="/gazebo/odom" />
```

---

### 场景 2：使用 FAST-LIVO2 输出

```bash
# 1. 启动 FAST-LIVO2（输出 /aft_mapped_to_init）
roslaunch fast_livo mapping.launch

# 2. 启动目标检测
roslaunch your_detector detector.launch

# 3. 启动重建节点
roslaunch target_reconstruction target_reconstruction.launch
```

修改 launch 文件中的映射：
```xml
<remap from="/camera/odom" to="/aft_mapped_to_init" />
```

**注意**：FAST-LIVO2 发布的是 `nav_msgs/Odometry` 类型，可以直接使用！

---

### 场景 3：动捕系统（Vicon/OptiTrack）

```bash
# 1. 启动动捕系统 ROS 驱动（发布 /vicon/camera/odom）
roslaunch vicon_bridge vicon.launch

# 2. 启动相机
roslaunch realsense2_camera rs_camera.launch

# 3. 启动目标检测
roslaunch your_detector detector.launch

# 4. 启动重建节点
roslaunch target_reconstruction target_reconstruction.launch
```

修改 launch 文件中的映射：
```xml
<remap from="/camera/odom" to="/vicon/camera/odom" />
```

---

## 🔍 消息格式

### nav_msgs/Odometry 结构

```
Header header
  uint32 seq
  time stamp
  string frame_id

PoseWithCovariance pose
  Pose pose
    Point position      ← 相机位置 (x, y, z)
    Quaternion orientation  ← 相机姿态 (w, x, y, z)
  float64[36] covariance

TwistWithCovariance twist
  ...
```

**重建系统使用的字段**：
- `pose.pose.position` → 相机位置
- `pose.pose.orientation` → 相机旋转（四元数）
- `header.stamp` → 时间戳

---

## ⚙️ 坐标系说明

### 输入位姿

- **坐标系**：世界坐标系 → 相机坐标系
- **旋转矩阵**：`R_world_to_camera`
- **平移向量**：`t_world_to_camera`

### 变换关系

```
P_camera = R_world_to_camera * P_world + t_world_to_camera
```

或者（如果是相机在世界坐标系中的位姿）：
```
P_world = R_camera_to_world * P_camera + t_camera_to_world
```

**注意**：需要根据实际系统确认坐标系定义！

---

## 🐛 调试检查

### 1. 检查 Odometry 话题

```bash
# 查看话题列表
rostopic list | grep odom

# 查看话题信息
rostopic info /odom

# 实时显示消息
rostopic echo /odom

# 检查发布频率
rostopic hz /odom
```

### 2. 可视化位姿（RViz）

```bash
# 启动 RViz
rviz

# 添加显示项：
# - Add → TF → 查看坐标变换
# - Add → Odometry → 选择话题 /odom
```

### 3. 记录数据包

```bash
# 录制
rosbag record \
  /camera/color/image_raw \
  /camera/depth/image_raw \
  /object_detection/bounding_box \
  /odom \
  -O test_reconstruction.bag

# 回放
rosbag play test_reconstruction.bag
```

---

## ✅ 验证清单

- [ ] Odometry 话题正常发布（`rostopic hz /odom`）
- [ ] 位姿数值合理（不是全零、没有 NaN）
- [ ] 时间戳与图像同步（误差 < 100ms）
- [ ] 坐标系定义正确（右手系）
- [ ] 旋转表示正确（四元数归一化）
- [ ] 重建点云在合理位置（通过 RViz 检查）

---

## 📚 参考资料

### ROS 消息定义
- [nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)
- [geometry_msgs/Pose](http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html)

### 相关功能
- FAST-LIVO2 输出位姿
- Gazebo 仿真器
- 动捕系统集成

---

**总结**：系统已配置为直接使用真值位姿，无需额外的定位计算。只需确保 Odometry 话题正确发布即可！🎯

