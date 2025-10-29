/*
 * Target Reconstruction - Main Reconstructor
 * 
 * 目标重建的主类
 * 整合深度图、RGB图像、目标检测框，构建3D模型
 */

#ifndef TARGET_RECONSTRUCTOR_H_
#define TARGET_RECONSTRUCTOR_H_

#include "common_lib.h"
#include "voxel_map.h"
#include "visual_point.h"
#include "feature.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>

/**
 * @brief 重建配置参数
 */
struct ReconstructionConfig
{
    // 图像处理
    int image_width;
    int image_height;
    int grid_size;                  // 网格划分大小（像素）
    int border_pixels;              // 图像边界（像素）
    
    // 深度过滤
    double min_depth;               // 最小深度（米）
    double max_depth;               // 最大深度（米）
    double depth_noise_threshold;   // 深度噪声阈值
    
    // 特征选择
    double min_shi_tomasi_score;    // 最小Shi-Tomasi角点响应值
    int max_points_per_grid;        // 每个网格的最大点数
    
    // 质量控制
    int min_observations;           // 点的最小观测次数
    double max_reprojection_error;  // 最大重投影误差（像素）
    double min_confidence;          // 最小置信度
    
    // 地图管理
    double voxel_size;              // 体素大小（米）
    bool enable_color;              // 是否使用颜色
    bool enable_tsdf;               // 是否使用TSDF融合
    
    ReconstructionConfig() 
        : image_width(640), image_height(480), grid_size(40), border_pixels(20),
          min_depth(0.1), max_depth(5.0), depth_noise_threshold(0.05),
          min_shi_tomasi_score(5.0), max_points_per_grid(1),
          min_observations(3), max_reprojection_error(2.0), min_confidence(0.3),
          voxel_size(VOXEL_SIZE), enable_color(true), enable_tsdf(false) {}
};

/**
 * @brief 目标重建器主类
 * 
 * 工作流程：
 * 1. 订阅RGB图像、深度图、目标检测框、相机位姿
 * 2. 在检测框内提取特征点
 * 3. 利用深度图计算3D位置
 * 4. 多帧融合，更新视觉地图
 * 5. 导出重建模型
 */
class TargetReconstructor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // ========== 构造与初始化 ==========
    TargetReconstructor(const ReconstructionConfig& config = ReconstructionConfig());
    ~TargetReconstructor();
    
    /**
     * @brief 初始化ROS节点和订阅器
     */
    void initROS(ros::NodeHandle& nh);
    
    // ========== 核心处理函数 ==========
    /**
     * @brief 处理单帧数据（使用检测框）
     * @param rgb_img RGB图像
     * @param depth_img 深度图
     * @param bbox 目标检测框
     * @param camera_R 相机旋转矩阵（世界到相机）
     * @param camera_t 相机平移向量
     * @param timestamp 时间戳
     */
    void processFrame(
        const Mat& rgb_img,
        const Mat& depth_img,
        const BoundingBox& bbox,
        const M3D& camera_R,
        const V3D& camera_t,
        double timestamp);
    
    /**
     * @brief 处理单帧数据（使用分割 Mask）
     * @param rgb_img RGB图像
     * @param depth_img 深度图
     * @param mask 分割 mask（255=目标，0=背景）
     * @param bbox 从 mask 计算的包围框（用于优化）
     * @param camera_R 相机旋转矩阵（世界到相机）
     * @param camera_t 相机平移向量
     * @param timestamp 时间戳
     */
    void processFrameWithMask(
        const Mat& rgb_img,
        const Mat& depth_img,
        const Mat& mask,
        const BoundingBox& bbox,
        const M3D& camera_R,
        const V3D& camera_t,
        double timestamp);
    
    /**
     * @brief 从检测框内生成新的视觉点
     */
    void generateVisualPoints(
        const Mat& rgb_img,
        const Mat& depth_img,
        const BoundingBox& bbox,
        const M3D& camera_R,
        const V3D& camera_t);
    
    /**
     * @brief 从检测框内生成新的视觉点（使用 Mask 进行精确筛选）
     */
    void generateVisualPointsWithMask(
        const Mat& rgb_img,
        const Mat& depth_img,
        const Mat& mask,
        const BoundingBox& bbox,
        const M3D& camera_R,
        const V3D& camera_t);
    
    /**
     * @brief 从地图中检索可见的视觉点
     */
    std::vector<VisualPoint*> retrieveVisiblePoints(
        const BoundingBox& bbox,
        const M3D& camera_R,
        const V3D& camera_t);
    
    /**
     * @brief 更新已观测的视觉点
     */
    void updateVisualPoints(
        const Mat& rgb_img,
        const std::vector<VisualPoint*>& visible_points,
        const M3D& camera_R,
        const V3D& camera_t);
    
    /**
     * @brief 更新已观测的视觉点（使用 Mask 进行精确筛选）
     */
    void updateVisualPointsWithMask(
        const Mat& rgb_img,
        const Mat& mask,
        const std::vector<VisualPoint*>& visible_points,
        const M3D& camera_R,
        const V3D& camera_t);
    
    // ========== 特征提取 ==========
    /**
     * @brief 在检测框内提取候选点
     * @param gray_img 灰度图像
     * @param depth_img 深度图
     * @param bbox 目标框
     * @return 候选点的像素坐标
     */
    std::vector<V2D> extractCandidatePoints(
        const Mat& gray_img,
        const Mat& depth_img,
        const BoundingBox& bbox);
    
    /**
     * @brief 计算Shi-Tomasi角点响应
     */
    float computeShiTomasiScore(const Mat& img, int x, int y);
    
    /**
     * @brief 提取图像Patch
     */
    void extractImagePatch(const Mat& img, const V2D& px, float* patch);
    
    // ========== 几何计算 ==========
    /**
     * @brief 像素坐标 + 深度 -> 3D世界坐标
     */
    V3D pixelToWorld(const V2D& px, float depth, const M3D& R_c_w, const V3D& t_c_w) const;
    
    /**
     * @brief 3D世界坐标 -> 像素坐标
     */
    V2D worldToPixel(const V3D& pos_w, const M3D& R_c_w, const V3D& t_c_w) const;
    
    /**
     * @brief 计算法向量（基于深度图梯度）
     */
    V3D computeNormalFromDepth(const Mat& depth_img, int x, int y) const;
    
    // ========== 地图管理 ==========
    /**
     * @brief 优化地图（移除离群点、下采样）
     */
    void optimizeMap();
    
    /**
     * @brief 保存重建结果
     */
    bool saveReconstruction(const std::string& filename) const;
    
    /**
     * @brief 获取地图管理器（用于外部访问）
     */
    VoxelMapManager* getMapManager() { return map_manager_; }
    
    // ========== ROS回调函数 ==========
    void rgbCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void bboxCallback(const geometry_msgs::PoseStampedConstPtr& msg);  // 临时使用，后续改为自定义消息
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    
private:
    // ========== 配置参数 ==========
    ReconstructionConfig config_;
    
    // ========== 核心数据 ==========
    VoxelMapManager* map_manager_;      // 视觉地图管理器
    
    // 当前帧数据（缓存）
    Mat current_rgb_;
    Mat current_depth_;
    BoundingBox current_bbox_;
    M3D current_R_;                     // 相机到世界的旋转
    V3D current_t_;                     // 相机到世界的平移
    double current_timestamp_;
    
    // 相机内参
    double fx_, fy_, cx_, cy_;          // 焦距和主点
    
    // 网格划分（用于特征选择）
    int grid_n_width_;                  // 网格列数
    int grid_n_height_;                 // 网格行数
    std::vector<float> grid_scores_;    // 每个网格的最大响应值
    std::vector<V2D> grid_candidates_;  // 每个网格选中的候选点
    
    // 统计信息
    int frame_count_;
    int total_points_created_;
    int total_observations_;
    
    // ========== 辅助函数 ==========
    /**
     * @brief 重置网格
     */
    void resetGrid();
    
    /**
     * @brief 检查深度是否有效
     */
    bool isDepthValid(float depth) const {
        return depth > config_.min_depth && depth < config_.max_depth;
    }
    
    /**
     * @brief 检查像素是否在检测框内
     */
    bool isInBoundingBox(int x, int y, const BoundingBox& bbox) const {
        return bbox.contains(x, y);
    }
    
    /**
     * @brief 检查像素是否在图像范围内
     */
    bool isInImage(int x, int y) const {
        return x >= config_.border_pixels && x < config_.image_width - config_.border_pixels &&
               y >= config_.border_pixels && y < config_.image_height - config_.border_pixels;
    }
};

#endif // TARGET_RECONSTRUCTOR_H_

