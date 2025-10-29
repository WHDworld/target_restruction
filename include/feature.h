/*
 * Target Reconstruction - Feature
 * 
 * 单帧观测的视觉特征
 * 存储图像Patch、相机位姿、时间戳等信息
 */

#ifndef TARGET_RECON_FEATURE_H_
#define TARGET_RECON_FEATURE_H_

#include "common_lib.h"
#include <boost/noncopyable.hpp>

// 前向声明
class VisualPoint;

/**
 * @brief 单帧观测的Feature
 * 
 * 相比FAST-LIVO2的简化：
 * - 移除曝光时间估计（假设相机参数稳定）
 * - 移除IMU相关的位姿信息（使用外部位姿）
 * - 增加深度信息（来自深度图）
 */
class Feature : boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ========== 基本信息 ==========
    int id_;                        // Feature唯一ID（帧序号）
    VisualPoint* point_;            // 所属的3D视觉点
    
    // ========== 图像信息 ==========
    V2D px_;                        // 像素坐标
    V3D f_;                         // 归一化平面坐标（bearing vector）
    float patch_[PATCH_SIZE_TOTAL]; // 图像Patch（灰度值）
    Mat img_;                       // 所属图像（用于Warping，可选存储）
    int level_;                     // 金字塔层级
    
    // ========== 几何信息 ==========
    M3D T_c_w_rotation_;            // 相机到世界的旋转（来自外部位姿）
    V3D T_c_w_translation_;         // 相机到世界的平移
    float depth_;                   // 深度值（来自深度图）
    bool depth_valid_;              // 深度是否有效
    
    // ========== 构造函数 ==========
    Feature(VisualPoint* point, const float* patch, const V2D& px, 
            const V3D& f, int level = 0);
    
    Feature(VisualPoint* point, const float* patch, const V2D& px, 
            const V3D& f, const M3D& R_c_w, const V3D& t_c_w, 
            float depth, int level = 0);
    
    ~Feature();
    
    // ========== 辅助函数 ==========
    /**
     * @brief 获取Feature在世界坐标系中的位置
     * @return 3D世界坐标（如果深度有效）
     */
    V3D pos() const;
    
    /**
     * @brief 检查深度是否有效
     */
    bool hasValidDepth() const { return depth_valid_ && depth_ > 0.0f; }
};

#endif // TARGET_RECON_FEATURE_H_

