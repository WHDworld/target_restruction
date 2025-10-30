/*
 * Target Reconstruction - Voxel Map
 * 
 * 基于体素的3D地图存储结构
 * 用于高效的空间查询和视觉点管理
 */

#ifndef TARGET_RECON_VOXEL_MAP_H_
#define TARGET_RECON_VOXEL_MAP_H_

#include "common_lib.h"
#include "visual_point.h"
#include <unordered_map>
#include <vector>
#include <functional>

/**
 * @brief 3D整数体素坐标
 */
struct VOXEL_LOCATION
{
    int64_t x, y, z;
    
    VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}
    
    bool operator==(const VOXEL_LOCATION& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

/**
 * @brief VOXEL_LOCATION的哈希函数
 */
namespace std {
    template<>
    struct hash<VOXEL_LOCATION> {
        size_t operator()(const VOXEL_LOCATION& voxel) const {
            return ((hash<int64_t>()(voxel.x) ^ 
                    (hash<int64_t>()(voxel.y) << 1)) >> 1) ^ 
                    (hash<int64_t>()(voxel.z) << 1);
        }
    };
}

/**
 * @brief 单个体素存储的视觉点集合
 */
struct VOXEL_POINTS
{
    std::vector<VisualPoint*> visual_points;  // 该体素内的视觉点列表
    int count;                                // 点数量
    V3D center_;                              // 体素中心坐标（米）
    
    // TSDF融合相关（可选）
    float tsdf_value_;                        // 体素的TSDF值
    float tsdf_weight_;                       // TSDF权重
    V3D color_;                               // 体素平均颜色
    
    VOXEL_POINTS(int num = 0) : count(num), tsdf_value_(0.0f), tsdf_weight_(0.0f) {
        visual_points.reserve(10);
    }
    
    ~VOXEL_POINTS() {
        // 释放所有视觉点
        for (VisualPoint* vp : visual_points) {
            if (vp != nullptr) {
                delete vp;
                vp = nullptr;
            }
        }
    }
};

/**
 * @brief 目标检测框信息
 */
struct BoundingBox
{
    int x_min, y_min;     // 左上角像素坐标
    int x_max, y_max;     // 右下角像素坐标
    int class_id;         // 目标类别ID
    float confidence;     // 检测置信度
    std::string label;    // 类别标签
    
    BoundingBox() : x_min(0), y_min(0), x_max(0), y_max(0), 
                    class_id(-1), confidence(0.0f) {}
    
    bool isValid() const {
        return x_max > x_min && y_max > y_min && confidence > 0.0f;
    }
    
    bool contains(int x, int y) const {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
};

/**
 * @brief 视觉地图管理器
 * 
 * 核心功能：
 * 1. 基于体素的空间索引
 * 2. 视觉点的增删改查
 * 3. 目标检测框内的点云过滤
 * 4. 地图的保存与加载
 */
class VoxelMapManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // ========== 构造与析构 ==========
    VoxelMapManager(double voxel_size = VOXEL_SIZE, 
                    double fx = 615.0, double fy = 615.0, 
                    double cx = 320.0, double cy = 240.0);
    ~VoxelMapManager();
    
    // ========== 地图操作 ==========
    /**
     * @brief 插入视觉点到地图
     * @param point 视觉点指针（地图将接管所有权）
     */
    void insertPoint(VisualPoint* point);
    
    /**
     * @brief 批量插入视觉点
     */
    void insertPoints(const std::vector<VisualPoint*>& points);
    
    /**
     * @brief 删除指定体素位置的点
     */
    void removePoint(const VOXEL_LOCATION& voxel_loc, VisualPoint* point);
    
    /**
     * @brief 清空整个地图
     */
    void clear();
    
    // ========== 空间查询 ==========
    /**
     * @brief 根据3D坐标获取体素位置
     */
    VOXEL_LOCATION getVoxelLocation(const V3D& pos) const;
    
    /**
     * @brief 查询指定体素内的所有点
     */
    std::vector<VisualPoint*> getPointsInVoxel(const VOXEL_LOCATION& voxel_loc) const;
    
    /**
     * @brief 查询3D点周围的所有点（邻域搜索）
     * @param center 中心点
     * @param radius 搜索半径（米）
     */
    std::vector<VisualPoint*> getPointsInRadius(const V3D& center, double radius) const;
    
    /**
     * @brief 获取目标框内的所有点
     * @param bbox 目标检测框
     * @param camera_R 相机旋转矩阵
     * @param camera_t 相机平移向量
     * @param depth_img 深度图
     */
    std::vector<VisualPoint*> getPointsInBoundingBox(
        const BoundingBox& bbox,
        const M3D& camera_R,
        const V3D& camera_t,
        const Mat& depth_img) const;
    
    /**
     * @brief 获取在相机视野内的所有点
     */
    std::vector<VisualPoint*> getPointsInFrustum(
        const M3D& camera_R,
        const V3D& camera_t,
        int img_width,
        int img_height,
        double min_depth = 0.1,
        double max_depth = 10.0) const;
    
    // ========== 地图统计 ==========
    /**
     * @brief 获取地图中的总点数
     */
    size_t getTotalPoints() const;
    
    /**
     * @brief 获取体素总数
     */
    size_t getTotalVoxels() const { return voxel_map_.size(); }
    
    /**
     * @brief 获取地图的边界框
     */
    void getBoundingBox(V3D& min_bound, V3D& max_bound) const;
    
    // ========== 地图优化 ==========
    /**
     * @brief 移除低置信度的点
     */
    void removeOutliers(float confidence_threshold = 0.3f);
    
    /**
     * @brief 移除观测次数过少的点
     */
    void removeUnderObservedPoints(int min_observations = 3);
    
    /**
     * @brief 下采样：合并邻近点
     */
    void downsample(double merge_threshold = 0.01);
    
    // ========== 地图保存与加载 ==========
    /**
     * @brief 保存地图到文件
     * @param filename 文件路径（支持.ply, .pcd）
     * @param save_color 是否保存颜色
     */
    bool saveToFile(const std::string& filename, bool save_color = true) const;
    
    /**
     * @brief 从文件加载地图
     */
    bool loadFromFile(const std::string& filename);
    
    // ========== 可视化辅助 ==========
    /**
     * @brief 导出为点云（用于可视化）
     * @param points 输出：点坐标
     * @param colors 输出：点颜色（可选）
     * @param normals 输出：点法向量（可选）
     */
    void exportPointCloud(
        std::vector<V3D>& points,
        std::vector<V3D>* colors = nullptr,
        std::vector<V3D>* normals = nullptr) const;
    
private:
    // ========== 内部数据结构 ==========
    std::unordered_map<VOXEL_LOCATION, VOXEL_POINTS*> voxel_map_;  // 主地图
    double voxel_size_;                                             // 体素大小
    
    // ========== 相机内参 ==========
    double fx_, fy_;  // 焦距
    double cx_, cy_;  // 主点
    
    // ========== 辅助函数 ==========
    /**
     * @brief 获取邻域体素位置
     */
    std::vector<VOXEL_LOCATION> getNeighborVoxels(
        const VOXEL_LOCATION& center, 
        int range = 1) const;
};

#endif // TARGET_RECON_VOXEL_MAP_H_

