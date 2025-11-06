# 法向量计算方法对比

## 📚 目录

1. [概述](#概述)
2. [方法一：深度图梯度法](#方法一深度图梯度法)
3. [方法二：激光雷达平面拟合法](#方法二激光雷达平面拟合法)
4. [两种方法的对比](#两种方法的对比)
5. [实际应用建议](#实际应用建议)
6. [代码实现示例](#代码实现示例)

**📖 扩展阅读：** [体素八叉树详解](./VOXEL_OCTREE_GUIDE.md) - 深入了解八叉树的数据结构、插入/查询/删除操作、平面拟合算法

---

## 概述

法向量（Surface Normal）是计算机视觉和SLAM中的关键几何属性，表示**表面在某一点的垂直方向**。在 FAST-LIVO2 这样的 LiDAR-Visual-Inertial 系统中，准确的法向量对于以下任务至关重要：

- 🎯 **直接法视觉跟踪**：补偿由于视角变化导致的图像patch变形（Affine Warping）
- 📐 **平面检测与匹配**：判断点是否在同一平面上
- 🔄 **参考帧选择**：选择与法向量夹角合适的观测帧
- 🎨 **3D重建与网格生成**：表面法向量决定光照效果

### 为什么需要法向量？

在 FAST-LIVO2 的直接法中，当相机移动时，同一个3D点在不同帧中的图像patch会发生**透视形变**（perspective distortion）：

```
原始帧（正面观测）        新帧（侧面观测）
┌─────────┐               ╱─────────╲
│  Patch  │    移动→     ╱  Warped   ╲
│   9x9   │             ╱   Patch     ╲
└─────────┘             ╲─────────────╱
```

为了正确匹配这两个patch，需要：
1. 知道表面法向量 `n`
2. 计算仿射变换矩阵 `A`（Affine Warp）
3. 对新帧的patch进行反向变形

---

## 方法一：深度图梯度法

### 🎯 核心思想

利用深度图在像素空间的**梯度**（变化率）来计算3D表面的切线方向，然后通过**叉乘**得到法向量。

### 📐 数学原理

#### 1. 深度图的几何表示

深度图 `D(u, v)` 提供了每个像素对应的3D点：

```
P(u, v) = [X, Y, Z]^T = D(u, v) · K^(-1) · [u, v, 1]^T
```

其中：
- `K` 是相机内参矩阵
- `(u, v)` 是像素坐标
- `D(u, v)` 是深度值

#### 2. 计算局部切线向量

在像素 `(u, v)` 处，计算两个方向的切线向量：

**水平方向切线（U方向）：**
```
P_u = P(u+1, v) - P(u-1, v)  // 中心差分
```

**垂直方向切线（V方向）：**
```
P_v = P(u, v+1) - P(u, v-1)  // 中心差分
```

这两个向量**张成了局部平面**（tangent plane）。

#### 3. 计算法向量（叉乘）

法向量垂直于切线平面，通过叉乘得到：

```
n = P_u × P_v = | i    j    k   |
                | P_u.x P_u.y P_u.z |
                | P_v.x P_v.y P_v.z |
```

展开后：
```
n_x = P_u.y * P_v.z - P_u.z * P_v.y
n_y = P_u.z * P_v.x - P_u.x * P_v.z
n_z = P_u.x * P_v.y - P_u.y * P_v.x
```

#### 4. 归一化

```
n_normalized = n / ||n||
```

### 🔧 实现步骤

#### **Step 1: 获取邻域深度**

```cpp
// 在像素 (u, v) 处计算法向量
float d_center = depth_img.at<float>(v, u);
float d_left   = depth_img.at<float>(v, u-1);
float d_right  = depth_img.at<float>(v, u+1);
float d_up     = depth_img.at<float>(v-1, u);
float d_down   = depth_img.at<float>(v+1, u);
```

#### **Step 2: 转换到3D坐标**

```cpp
// 相机内参
float fx = 615.0, fy = 615.0;
float cx = 320.0, cy = 240.0;

// 中心点
Eigen::Vector3d P_center;
P_center.x() = (u - cx) * d_center / fx;
P_center.y() = (v - cy) * d_center / fy;
P_center.z() = d_center;

// 邻域点
Eigen::Vector3d P_left, P_right, P_up, P_down;
// ... 类似计算
```

#### **Step 3: 计算切线向量**

```cpp
// 水平切线（中心差分）
Eigen::Vector3d T_u = (P_right - P_left) / 2.0;

// 垂直切线（中心差分）
Eigen::Vector3d T_v = (P_down - P_up) / 2.0;
```

#### **Step 4: 叉乘得到法向量**

```cpp
Eigen::Vector3d normal = T_u.cross(T_v);

// 归一化
normal.normalize();

// 确保法向量指向相机（z < 0）
if (normal.z() > 0) {
    normal = -normal;
}
```

### 📊 优点

| 优点 | 说明 |
|------|------|
| ✅ **计算快速** | 只需访问邻域像素，复杂度 O(1) |
| ✅ **实时性好** | 适合高帧率的在线SLAM |
| ✅ **覆盖范围广** | 每个深度像素都可以计算 |
| ✅ **实现简单** | 不需要迭代优化 |

### ⚠️ 缺点

| 缺点 | 说明 |
|------|------|
| ❌ **对噪声敏感** | 深度图噪声会直接影响梯度 |
| ❌ **边缘不准确** | 物体边界、遮挡处容易出错 |
| ❌ **分辨率受限** | 取决于深度图分辨率 |
| ❌ **不适合曲面** | 假设局部是平面 |

### 🎨 可视化示例

```
深度图（俯视）            计算得到的切线          叉乘得到法向量
                                                      ↑
  ┌───┬───┬───┐           ┌───→───┐                 │ n
  │ 2 │ 2 │ 2 │           │   T_u  │                 │
  ├───┼───┼───┤    →      ↓───────↓      →          │
  │ 2 │ 2 │ 2 │           │  T_v   │                 │
  ├───┼───┼───┤           └────────┘                 └─────→
  │ 3 │ 3 │ 3 │                                      平面
  └───┴───┴───┘
  （深度变化）            （切线方向）            （垂直表面）
```

---

## 方法二：激光雷达平面拟合法

### 🎯 核心思想

使用激光雷达扫描的**多帧点云**，通过**PCA（主成分分析）**或**最小二乘拟合**来估计局部平面，从而得到高精度的法向量。

### 📐 数学原理

#### 1. 平面方程

平面可以用以下方程表示：

```
n · (p - c) = 0
或
n_x * x + n_y * y + n_z * z + d = 0
```

其中：
- `n = [n_x, n_y, n_z]^T` 是法向量
- `c = [c_x, c_y, c_z]^T` 是平面中心
- `d` 是平面到原点的距离

#### 2. PCA（主成分分析）方法

假设在体素内有 `N` 个激光点 `{p_1, p_2, ..., p_N}`：

**Step 1: 计算中心点**
```
c = (1/N) * Σ p_i
```

**Step 2: 计算协方差矩阵**
```
C = (1/N) * Σ (p_i - c) * (p_i - c)^T

C = [ σ_xx  σ_xy  σ_xz ]
    [ σ_yx  σ_yy  σ_yz ]
    [ σ_zx  σ_zy  σ_zz ]
```

**Step 3: 特征值分解**
```
C * v_i = λ_i * v_i
```

得到三个特征值和特征向量：
- `λ_1 ≥ λ_2 ≥ λ_3`
- 对应的特征向量 `v_1, v_2, v_3`

**Step 4: 提取法向量**

法向量是**最小特征值对应的特征向量**：
```
n = v_3  （对应 λ_3）
```

**为什么？**
- `v_1` 和 `v_2` 张成平面（变化最大的方向）
- `v_3` 垂直于平面（变化最小的方向）

#### 3. 平面性判断

通过特征值比例判断是否为平面：

```
planarity = (λ_2 - λ_3) / λ_1
```

- `planarity > 0.8`：明显的平面
- `planarity < 0.3`：散乱点或边缘

### 🔧 FAST-LIVO2 中的实现步骤

> 💡 **详细八叉树实现**：本节介绍概要流程，完整的数据结构、插入/删除/查询算法、内存管理等内容请参阅 **[体素八叉树详解文档](./VOXEL_OCTREE_GUIDE.md)**

#### **Step 1: 体素化点云**

```cpp
// 将世界坐标映射到体素网格（0.5m 分辨率）
float loc_xyz[3];
for (int j = 0; j < 3; j++) {
    loc_xyz[j] = p_w[j] / 0.5;
    if (loc_xyz[j] < 0) { 
        loc_xyz[j] -= 1.0;  // 负数向下取整
    }
}
VOXEL_LOCATION position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
```

#### **Step 2: 八叉树细分**

在每个体素内，如果点数超过阈值，递归细分为 8 个子体素：

```cpp
class VoxelOctoTree {
    VoxelOctoTree *leaves_[8];  // 8个子节点
    VoxelPlane *plane_ptr_;     // 平面信息
    
    void cut_octo_tree() {
        if (temp_points_.size() > points_size_threshold_ && layer_ < max_layer_) {
            // 递归细分
            for (int i = 0; i < 8; i++) {
                leaves_[i] = new VoxelOctoTree(...);
            }
        }
    }
};
```

#### **Step 3: 平面拟合（PCA）**

```cpp
void VoxelOctoTree::init_plane(const vector<pointWithVar> &points, VoxelPlane *plane) {
    int N = points.size();
    
    // 1. 计算中心
    V3D center = V3D::Zero();
    for (const auto& p : points) {
        center += p.point_w;
    }
    center /= N;
    
    // 2. 计算协方差矩阵
    M3D covariance = M3D::Zero();
    for (const auto& p : points) {
        V3D delta = p.point_w - center;
        covariance += delta * delta.transpose();
    }
    covariance /= N;
    
    // 3. 特征值分解
    Eigen::SelfAdjointEigenSolver<M3D> solver(covariance);
    V3D eigenvalues = solver.eigenvalues();    // λ_1, λ_2, λ_3
    M3D eigenvectors = solver.eigenvectors();  // v_1, v_2, v_3
    
    // 4. 提取法向量（最小特征值的特征向量）
    plane->normal_ = eigenvectors.col(0);  // λ_3 对应的向量
    plane->center_ = center;
    
    // 5. 计算平面参数
    plane->d_ = -plane->normal_.dot(center);
    plane->radius_ = sqrt(eigenvalues(2));  // 平面半径
    
    // 6. 判断是否为平面
    float planarity = (eigenvalues(1) - eigenvalues(0)) / eigenvalues(2);
    plane->is_plane_ = (planarity > 0.8);
}
```

#### **Step 4: 多帧更新与协方差传播**

每次新帧到来时，更新平面参数的不确定性：

```cpp
// 雅可比矩阵：点到平面距离对 [平面参数] 的偏导
J_nq = [p - c, -n]  // 6维：[中心位置(3), 法向量(3)]

// 点到平面距离的不确定性
sigma_l = J_nq * Σ_plane * J_nq^T + n^T * Σ_point * n
```

这个协方差用于判断点是否真正属于该平面（3-sigma 规则）。

#### **Step 5: 法向量方向统一**

```cpp
// 确保法向量方向一致（防止优化振荡）
if (pt->previous_normal_.dot(plane.normal_) < 0) {
    pt->normal_ = -plane.normal_;  // 翻转
} else {
    pt->normal_ = plane.normal_;
}
```

#### **Step 6: 收敛判断**

```cpp
double normal_update = (pt->normal_ - pt->previous_normal_).norm();

if (normal_update < 0.0001 && pt->obs_.size() > 10) {
    pt->is_converged_ = true;  // 法向量已收敛
}
```

### 📊 优点

| 优点 | 说明 |
|------|------|
| ✅ **高精度** | 利用多帧数据，噪声被平均化 |
| ✅ **鲁棒性强** | 对单点噪声不敏感 |
| ✅ **语义明确** | 提供平面性判断（`is_plane_`） |
| ✅ **协方差信息** | 提供法向量的不确定性 |
| ✅ **全局一致性** | 多帧融合，法向量时域稳定 |

### ⚠️ 缺点

| 缺点 | 说明 |
|------|------|
| ❌ **计算开销大** | 需要特征值分解，复杂度 O(N) |
| ❌ **延迟较高** | 需要累积多帧数据 |
| ❌ **覆盖不均** | 只在激光扫描区域有效 |
| ❌ **实现复杂** | 需要八叉树、PCA、协方差传播 |

### 🎨 可视化示例

```
多帧激光点云                PCA分解                    提取法向量
                                                          ↑
    ●  ●  ●                  λ_3 ←─────→ v_3             │ n
   ● ● ● ●                      (最小)                   │
  ● ● ● ● ●        →         λ_2 ←─────→ v_2      →     │
 ● ● ● ● ● ●                    (中等)                   │
● ● ● ● ● ● ●                λ_1 ←─────→ v_1             │
                                (最大)                   └─────→
（散点云）              （特征向量方向）              （垂直平面）
```

---

## 两种方法的对比

### 📊 详细对比表

| 维度 | 深度图梯度法 | 激光雷达平面拟合法 |
|------|-------------|-------------------|
| **精度** | ⭐⭐⭐ 中等（受深度噪声影响） | ⭐⭐⭐⭐⭐ 高（多帧融合） |
| **速度** | ⭐⭐⭐⭐⭐ 极快（O(1)） | ⭐⭐⭐ 较慢（O(N·log N)） |
| **鲁棒性** | ⭐⭐ 对噪声敏感 | ⭐⭐⭐⭐⭐ 非常鲁棒 |
| **覆盖范围** | ⭐⭐⭐⭐⭐ 全图像 | ⭐⭐⭐ 激光扫描区域 |
| **实时性** | ⭐⭐⭐⭐⭐ 优秀 | ⭐⭐⭐ 中等 |
| **边缘处理** | ⭐⭐ 易出错 | ⭐⭐⭐⭐ 可通过平面性筛选 |
| **不确定性** | ❌ 无 | ✅ 提供协方差矩阵 |
| **时域稳定性** | ⭐⭐ 单帧噪声大 | ⭐⭐⭐⭐⭐ 多帧融合稳定 |
| **实现复杂度** | ⭐ 简单 | ⭐⭐⭐⭐⭐ 复杂（PCA+八叉树） |

### 🔬 误差来源分析

#### 深度图梯度法的误差

1. **深度测量噪声**
   ```
   真实深度: 2.0m
   测量深度: 2.0 ± 0.02m  （±1cm）
   
   梯度误差: ∇D = (D_right - D_left) / Δu
            ≈ 0.04 / 2 = 0.02 rad (≈1.1°)
   ```

2. **飞点（Flying Pixels）**
   - 物体边缘处，前景和背景深度突变
   - 计算出的法向量**完全错误**

3. **量化误差**
   - 深度图分辨率有限（如 640x480）
   - 梯度计算精度受限

#### 激光雷达平面拟合法的误差

1. **点云稀疏性**
   - 远距离处点云稀疏
   - 小平面可能点数不足

2. **动态物体**
   - 多帧融合假设静态场景
   - 动态物体会污染平面估计

3. **计算截断误差**
   - 特征值分解的数值精度

### 🎯 适用场景

| 场景 | 推荐方法 | 原因 |
|------|---------|------|
| 实时视觉SLAM | 深度图梯度法 | 需要高帧率，容忍一定噪声 |
| 高精度建图 | 激光雷达平面拟合法 | 追求精度和鲁棒性 |
| 室内小物体 | 深度图梯度法 | 深度图分辨率高，覆盖范围广 |
| 大尺度室外 | 激光雷达平面拟合法 | 激光扫描范围大，精度高 |
| 纹理丰富区域 | 深度图梯度法 | 可以计算每个像素 |
| 低纹理平面 | 激光雷达平面拟合法 | 不依赖纹理，直接几何估计 |

---

## 实际应用建议

### 🎯 对于您的 RGB-D 目标重建系统

#### **推荐方案：改进的深度图梯度法**

由于您的系统是：
- ✅ RGB-D 相机（没有激光雷达）
- ✅ 目标重建（相对小尺度）
- ✅ 实时性要求

建议使用**改进的深度图梯度法** + **后处理优化**：

```cpp
V3D computeNormalFromDepth(const Mat& depth_img, int u, int v, int window_size = 1) {
    // 1. 双边滤波邻域（保持边缘的同时去噪）
    vector<V3D> neighbor_points;
    float d_center = depth_img.at<float>(v, u);
    
    for (int dv = -window_size; dv <= window_size; dv++) {
        for (int du = -window_size; du <= window_size; du++) {
            float d = depth_img.at<float>(v + dv, u + du);
            
            // 深度一致性检查（去除飞点）
            if (abs(d - d_center) < 0.05) {  // 5cm阈值
                V3D p = pixelToWorld(u + du, v + dv, d);
                neighbor_points.push_back(p);
            }
        }
    }
    
    // 2. 如果邻域点足够，使用 PCA 拟合局部平面
    if (neighbor_points.size() >= 5) {
        return fitPlaneAndGetNormal(neighbor_points);
    }
    
    // 3. 否则回退到简单梯度法
    return simpleGradientNormal(depth_img, u, v);
}
```

#### **混合方案（如果有激光雷达数据）**

如果未来能获取激光雷达数据：

```cpp
V3D getNormal(const V3D& point_w) {
    // 优先使用激光雷达平面拟合结果
    if (lidar_plane_map.has(point_w)) {
        return lidar_plane_map.getNormal(point_w);  // 高精度
    }
    
    // 回退到深度图梯度
    V2D pixel = worldToPixel(point_w);
    return computeNormalFromDepth(depth_img, pixel.x(), pixel.y());
}
```

### 📈 性能优化技巧

#### 1. **GPU 加速深度图梯度**

```cuda
__global__ void computeNormalsKernel(
    const float* depth, 
    float3* normals, 
    int width, int height) 
{
    int u = blockIdx.x * blockDim.x + threadIdx.x;
    int v = blockIdx.y * blockDim.y + threadIdx.y;
    
    // 并行计算每个像素的法向量
    // ...
}
```

#### 2. **多尺度金字塔**

```cpp
// 在低分辨率层计算法向量，然后上采样
Mat normal_pyramid[4];
for (int level = 3; level >= 0; level--) {
    if (level == 3) {
        normal_pyramid[level] = computeNormals(depth_pyramid[level]);
    } else {
        // 上一层的结果上采样 + 当前层的精细化
        normal_pyramid[level] = upsample(normal_pyramid[level+1]) 
                              + refine(depth_pyramid[level]);
    }
}
```

---

## 代码实现示例

### 🔧 完整实现：深度图梯度法

```cpp
/**
 * @brief 从深度图计算法向量（改进版）
 * @param depth_img 深度图（单通道 float）
 * @param u, v 像素坐标
 * @param fx, fy, cx, cy 相机内参
 * @return 归一化的法向量（世界坐标系）
 */
Eigen::Vector3d computeNormalFromDepth(
    const cv::Mat& depth_img, 
    int u, int v,
    double fx, double fy, 
    double cx, double cy)
{
    // ===== Step 1: 获取邻域深度 =====
    const int width = depth_img.cols;
    const int height = depth_img.rows;
    
    // 边界检查
    if (u < 2 || u >= width - 2 || v < 2 || v >= height - 2) {
        return Eigen::Vector3d(0, 0, -1);  // 默认指向相机
    }
    
    float d_center = depth_img.at<float>(v, u);
    float d_left   = depth_img.at<float>(v, u - 1);
    float d_right  = depth_img.at<float>(v, u + 1);
    float d_up     = depth_img.at<float>(v - 1, u);
    float d_down   = depth_img.at<float>(v + 1, u);
    
    // 深度有效性检查
    if (d_center <= 0.0f || d_left <= 0.0f || d_right <= 0.0f ||
        d_up <= 0.0f || d_down <= 0.0f) {
        return Eigen::Vector3d(0, 0, -1);
    }
    
    // ===== Step 2: 转换到 3D 坐标（相机系） =====
    auto pixel2cam = [&](int u, int v, float d) -> Eigen::Vector3d {
        Eigen::Vector3d p;
        p.x() = (u - cx) * d / fx;
        p.y() = (v - cy) * d / fy;
        p.z() = d;
        return p;
    };
    
    Eigen::Vector3d P_center = pixel2cam(u, v, d_center);
    Eigen::Vector3d P_left   = pixel2cam(u - 1, v, d_left);
    Eigen::Vector3d P_right  = pixel2cam(u + 1, v, d_right);
    Eigen::Vector3d P_up     = pixel2cam(u, v - 1, d_up);
    Eigen::Vector3d P_down   = pixel2cam(u, v + 1, d_down);
    
    // ===== Step 3: 计算切线向量（中心差分） =====
    Eigen::Vector3d T_u = (P_right - P_left) / 2.0;
    Eigen::Vector3d T_v = (P_down - P_up) / 2.0;
    
    // ===== Step 4: 叉乘得到法向量 =====
    Eigen::Vector3d normal = T_u.cross(T_v);
    
    // 检查退化情况
    if (normal.norm() < 1e-6) {
        return Eigen::Vector3d(0, 0, -1);
    }
    
    // ===== Step 5: 归一化 =====
    normal.normalize();
    
    // ===== Step 6: 确保法向量指向相机（z < 0） =====
    if (normal.z() > 0) {
        normal = -normal;
    }
    
    return normal;
}
```

### 🔧 完整实现：激光雷达平面拟合法

```cpp
/**
 * @brief 使用 PCA 拟合平面并提取法向量
 * @param points 点云集合（至少 3 个点）
 * @return 归一化的法向量 + 平面参数
 */
struct PlaneResult {
    Eigen::Vector3d normal;    // 法向量
    Eigen::Vector3d center;    // 平面中心
    double d;                  // 平面方程系数 d
    double planarity;          // 平面性（0-1）
    bool is_plane;             // 是否为平面
};

PlaneResult fitPlaneAndGetNormal(const std::vector<Eigen::Vector3d>& points) {
    PlaneResult result;
    int N = points.size();
    
    if (N < 3) {
        result.is_plane = false;
        result.normal = Eigen::Vector3d(0, 0, 1);
        return result;
    }
    
    // ===== Step 1: 计算中心点 =====
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        center += p;
    }
    center /= N;
    result.center = center;
    
    // ===== Step 2: 计算协方差矩阵 =====
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d delta = p - center;
        covariance += delta * delta.transpose();
    }
    covariance /= N;
    
    // ===== Step 3: 特征值分解 =====
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    Eigen::Vector3d eigenvalues = solver.eigenvalues();      // λ1, λ2, λ3（升序）
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();    // 对应的特征向量
    
    // ===== Step 4: 提取法向量（最小特征值的特征向量） =====
    result.normal = eigenvectors.col(0);  // λ_min 对应的向量
    result.normal.normalize();
    
    // ===== Step 5: 计算平面方程参数 d =====
    result.d = -result.normal.dot(center);
    
    // ===== Step 6: 判断平面性 =====
    double lambda_min = eigenvalues(0);
    double lambda_mid = eigenvalues(1);
    double lambda_max = eigenvalues(2);
    
    // 平面性指标：中间特征值与最小特征值的差异
    result.planarity = (lambda_mid - lambda_min) / lambda_max;
    result.is_plane = (result.planarity > 0.8);  // 阈值可调
    
    return result;
}
```

### 🎨 使用示例

```cpp
// ===== 场景1: 实时SLAM（深度图法） =====
void processFrame(const Mat& rgb_img, const Mat& depth_img) {
    for (auto* visual_point : visible_points) {
        V2D pixel = worldToPixel(visual_point->pos_);
        
        // 快速计算法向量
        V3D normal = computeNormalFromDepth(
            depth_img, pixel.x(), pixel.y(), fx, fy, cx, cy);
        
        visual_point->normal_ = normal;
    }
}

// ===== 场景2: 离线建图（激光雷达法） =====
void buildMap(const vector<PointCloudFrame>& frames) {
    // 累积多帧点云
    unordered_map<VOXEL_LOCATION, vector<V3D>> voxel_points;
    
    for (const auto& frame : frames) {
        for (const auto& point : frame.points) {
            VOXEL_LOCATION voxel = getVoxelLocation(point);
            voxel_points[voxel].push_back(point);
        }
    }
    
    // 每个体素拟合平面
    for (auto& [voxel, points] : voxel_points) {
        if (points.size() >= 10) {
            PlaneResult plane = fitPlaneAndGetNormal(points);
            
            if (plane.is_plane) {
                // 更新该体素内所有视觉点的法向量
                updateNormals(voxel, plane.normal);
            }
        }
    }
}
```

---

## 🎓 总结

### 关键要点

1. **深度图梯度法**：
   - 适合实时应用
   - 对噪声敏感，需要后处理
   - 计算简单，覆盖范围广

2. **激光雷达平面拟合法**：
   - 适合高精度建图
   - 鲁棒性强，提供不确定性
   - 计算开销大，需要多帧融合

3. **实际应用**：
   - RGB-D 系统：优先使用改进的深度图梯度法
   - LiDAR-Visual 系统：混合使用两种方法
   - 关键区域：可以局部使用 PCA 拟合提升精度

### 进一步改进方向

- 🔍 **深度图预处理**：双边滤波、形态学操作去除噪声
- 🌐 **多尺度融合**：结合不同分辨率的法向量估计
- 🤖 **学习方法**：神经网络预测法向量（如 Surface Normal Estimation CNN）
- 🔄 **时域一致性**：对法向量进行时间滤波（卡尔曼滤波）

---

**参考文献**

1. FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry
2. PCL - Point Cloud Library (Normal Estimation Module)
3. SVO - Semi-Direct Visual Odometry
4. Eigen Library Documentation

---

*生成时间: 2025-11-06*  
*作者: AI Assistant*  
*项目: Target Reconstruction with RGB-D*

