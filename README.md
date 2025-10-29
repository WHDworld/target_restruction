# Target Reconstruction Framework

基于 FAST-LIVO2 视觉地图管理框架的静态目标重建系统

## 📁 项目结构

```
target_restruction/
├── include/
│   ├── common_lib.h              # 通用数据类型和常量
│   ├── feature.h                 # 单帧观测Feature
│   ├── visual_point.h            # 3D视觉地图点
│   ├── voxel_map.h               # 体素地图管理
│   └── target_reconstructor.h   # 主重建类
└── src/
    ├── feature.cpp               # Feature实现（待实现）
    ├── visual_point.cpp          # VisualPoint实现（待实现）
    ├── voxel_map.cpp             # VoxelMapManager实现（待实现）
    ├── target_reconstructor.cpp  # TargetReconstructor实现（待实现）
    └── main.cpp                  # 主程序入口（待实现）
```

## 🎯 核心设计思想

### 1. 数据流

```mermaid
graph LR
    A[RGB相机] --> E[TargetReconstructor]
    B[深度相机] --> E
    C[目标检测] --> E
    D[相机位姿] --> E
    E --> F[特征提取]
    F --> G[3D点生成]
    G --> H[VoxelMapManager]
    H --> I[视觉点融合]
    I --> J[3D重建模型]
```

### 2. 核心类关系

```mermaid
classDiagram
    class Feature {
        +V2D px_
        +float patch_[]
        +float depth_
        +M3D T_c_w_rotation_
        +V3D pos()
    }
    
    class VisualPoint {
        +V3D pos_
        +V3D normal_
        +V3D color_
        +list~Feature*~ obs_
        +float confidence_
        +addObservation()
        +updateConfidence()
    }
    
    class VOXEL_POINTS {
        +vector~VisualPoint*~ visual_points
        +V3D center_
    }
    
    class VoxelMapManager {
        +unordered_map voxel_map_
        +insertPoint()
        +getPointsInBoundingBox()
        +saveToFile()
    }
    
    class TargetReconstructor {
        +VoxelMapManager* map_manager_
        +processFrame()
        +generateVisualPoints()
        +updateVisualPoints()
    }
    
    Feature "n" --> "1" VisualPoint : 观测
    VisualPoint "n" --> "1" VOXEL_POINTS : 存储
    VOXEL_POINTS "n" --> "1" VoxelMapManager : 管理
    TargetReconstructor "1" --> "1" VoxelMapManager : 使用
```

## 🔑 关键改进点（相比FAST-LIVO2）

### ✅ 针对静态目标优化

| 特性 | FAST-LIVO2 | Target Reconstruction |
|------|-----------|----------------------|
| **应用场景** | 动态SLAM | 静态目标重建 |
| **传感器** | LiDAR + IMU + 相机 | RGB-D相机 |
| **位姿来源** | IMU预积分 + ESIKF | 外部位姿（ORB-SLAM等） |
| **体素大小** | 0.5m（大场景） | 0.05m（精细重建） |
| **观测数量** | 快速收敛（<10帧） | 充分观测（>30帧） |
| **颜色信息** | 无 | RGB颜色融合 |
| **目标过滤** | 无 | 检测框内点云 |

### ✅ 新增功能

1. **颜色重建**：`VisualPoint` 增加 `V3D color_` 和 `updateColor()`
2. **目标框过滤**：`VoxelMapManager::getPointsInBoundingBox()`
3. **置信度评估**：基于观测次数和几何一致性的 `confidence_` 评分
4. **TSDF融合**（可选）：支持体积重建
5. **质量控制**：`checkGeometricConsistency()` 和离群点检测

## 📊 内存管理机制

### 指针共享策略（继承自FAST-LIVO2）

```cpp
// 1. 创建视觉点（堆分配）
VisualPoint* pt = new VisualPoint(pos);

// 2. 插入全局地图（地图拥有所有权）
map_manager_->insertPoint(pt);  
// 内部：voxel_map_[voxel_loc]->visual_points.push_back(pt);

// 3. 临时检索（共享指针，不拷贝对象）
std::vector<VisualPoint*> visible_pts = map_manager_->getPointsInBoundingBox(...);

// 4. 更新点属性（通过任一指针修改，全局可见）
for (auto pt : visible_pts) {
    pt->addObservation(ftr);  // 修改原对象
    pt->updateConfidence();
}

// 5. 地图负责释放（析构时）
// ~VOXEL_POINTS() { for (auto vp : visual_points) delete vp; }
```

**关键原则**：
- ✅ `VisualPoint*` 是指针，多处共享同一对象
- ✅ `VoxelMapManager` 拥有所有权，负责删除
- ✅ 其他地方只持有临时指针，不能删除

## 🚀 使用流程

### 步骤1：初始化

```cpp
// 配置参数
ReconstructionConfig config;
config.voxel_size = 0.05;  // 5cm体素
config.enable_color = true;
config.min_observations = 5;

// 创建重建器
TargetReconstructor reconstructor(config);

// 初始化ROS
ros::NodeHandle nh;
reconstructor.initROS(nh);
```

### 步骤2：订阅话题

```bash
# RGB图像
/camera/color/image_raw

# 深度图像
/camera/depth/image_raw

# 目标检测框（需要自定义消息类型）
/object_detection/bounding_box

# 相机位姿（来自SLAM或其他定位系统）
/camera/pose
```

### 步骤3：处理数据

```cpp
// 在回调函数中收集数据后
reconstructor.processFrame(
    rgb_img,      // cv::Mat
    depth_img,    // cv::Mat
    bbox,         // BoundingBox
    camera_R,     // Eigen::Matrix3d
    camera_t,     // Eigen::Vector3d
    timestamp     // double
);
```

### 步骤4：保存模型

```cpp
// 优化地图（移除离群点）
reconstructor.optimizeMap();

// 保存为PLY格式
reconstructor.saveReconstruction("target_model.ply");
```

## 📝 待实现功能清单

### 高优先级

- [ ] `Feature` 类的构造函数和 `pos()` 实现
- [ ] `VisualPoint` 类的观测管理函数
- [ ] `VoxelMapManager::insertPoint()` 和空间查询
- [ ] `TargetReconstructor::generateVisualPoints()` - 特征提取
- [ ] `TargetReconstructor::updateVisualPoints()` - 多帧融合

### 中优先级

- [ ] `VisualPoint::updateConfidence()` - 置信度评估
- [ ] `VoxelMapManager::getPointsInBoundingBox()` - 目标框查询
- [ ] `TargetReconstructor::computeNormalFromDepth()` - 法向量估计
- [ ] PLY/PCD文件保存

### 低优先级（增强功能）

- [ ] TSDF融合
- [ ] Patch Warping（处理视角变化）
- [ ] 网格重建（Marching Cubes）
- [ ] 纹理映射

## 🔧 编译说明

```bash
# 在CMakeLists.txt中添加
add_executable(target_reconstruction
    src/feature.cpp
    src/visual_point.cpp
    src/voxel_map.cpp
    src/target_reconstructor.cpp
    src/main.cpp
)

target_link_libraries(target_reconstruction
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Eigen3_LIBRARIES}
)
```

## 📚 参考资料

- FAST-LIVO2 论文：基于体素地图的LIO-Visual融合
- KinectFusion：TSDF体积融合
- ORB-SLAM3：视觉SLAM位姿估计
- Open3D：点云处理库

## 🎓 核心算法伪代码

### generateVisualPoints()

```python
def generateVisualPoints(rgb_img, depth_img, bbox, R, t):
    # 1. 在检测框内网格化
    resetGrid()
    
    # 2. 对每个像素
    for (x, y) in bbox:
        if not isInGrid(x, y): continue
        
        # 3. 计算角点响应
        score = shiTomasiScore(gray_img, x, y)
        
        # 4. 在网格内保留响应最大的点
        grid_idx = getGridIndex(x, y)
        if score > grid_scores[grid_idx]:
            grid_scores[grid_idx] = score
            grid_candidates[grid_idx] = (x, y)
    
    # 5. 为选中的点创建VisualPoint
    for (x, y) in grid_candidates:
        depth = depth_img.at(x, y)
        if not isDepthValid(depth): continue
        
        # 6. 计算3D位置
        pos_3d = pixelToWorld((x, y), depth, R, t)
        
        # 7. 提取Patch和颜色
        patch = extractPatch(gray_img, (x, y))
        color = rgb_img.at(x, y)
        
        # 8. 创建VisualPoint和Feature
        pt = new VisualPoint(pos_3d, color)
        ftr = new Feature(pt, patch, (x, y), depth)
        pt->addObservation(ftr)
        
        # 9. 插入地图
        map_manager->insertPoint(pt)
```

### updateVisualPoints()

```python
def updateVisualPoints(rgb_img, visible_points, R, t):
    for pt in visible_points:
        # 1. 投影到当前帧
        px = worldToPixel(pt->pos_, R, t)
        if not isInImage(px): continue
        
        # 2. 检查几何一致性
        depth = depth_img.at(px)
        pos_new = pixelToWorld(px, depth, R, t)
        if not pt->checkGeometricConsistency(pos_new): continue
        
        # 3. 检查是否需要添加新观测
        last_ftr = pt->obs_.back()
        delta_pose = computePoseDelta(last_ftr, R, t)
        
        if delta_pose > threshold:
            # 4. 提取Patch，创建新Feature
            patch = extractPatch(gray_img, px)
            ftr = new Feature(pt, patch, px, depth)
            
            # 5. 添加观测
            pt->addObservation(ftr)
            
            # 6. 限制观测数量
            if len(pt->obs_) > MAX_OBSERVATIONS:
                removeWorstObservation(pt)
        
        # 7. 更新置信度和颜色
        pt->updateConfidence()
        pt->updateColor()
```

---

**Author**: Based on FAST-LIVO2 framework  
**License**: See LICENSE file  
**Contact**: For questions, please open an issue

