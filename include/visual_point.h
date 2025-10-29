/*
 * Target Reconstruction - Visual Point
 * 
 * 3D视觉地图点
 * 存储多帧观测、法向量、协方差等信息
 */

#ifndef TARGET_RECON_VISUAL_POINT_H_
#define TARGET_RECON_VISUAL_POINT_H_

#include "common_lib.h"
#include "feature.h"
#include <boost/noncopyable.hpp>
#include <list>

/**
 * @brief 3D视觉地图点
 * 
 * 相比FAST-LIVO2的改进：
 * - 增加颜色信息（RGB）用于纹理重建
 * - 增加置信度评分（基于观测次数和几何一致性）
 * - 简化收敛判定（静态场景无需快速收敛）
 * - 增加TSDF相关信息（可选，用于体积重建）
 */
class VisualPoint : boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ========== 几何信息 ==========
    V3D pos_;                       // 3D位置（世界坐标系）
    V3D normal_;                    // 表面法向量
    M3D covariance_;                // 位置协方差矩阵
    
    // ========== 外观信息 ==========
    V3D color_;                     // RGB颜色（0-255）
    bool has_color_;                // 是否有颜色信息
    
    // ========== 观测信息 ==========
    std::list<Feature*> obs_;       // 多帧观测列表
    Feature* ref_patch_;            // 参考Patch（最佳观测）
    bool has_ref_patch_;            // 是否已选择参考Patch
    
    // ========== 状态标志 ==========
    bool is_normal_initialized_;    // 法向量是否已初始化
    bool is_converged_;             // 是否收敛（观测充分）
    bool is_outlier_;               // 是否为离群点
    
    // ========== 质量评估 ==========
    float confidence_;              // 置信度评分 [0, 1]
    int num_observations_;          // 观测次数
    float avg_photometric_error_;   // 平均光度误差
    
    // ========== TSDF信息（可选）==========
    float tsdf_value_;              // TSDF距离值
    float tsdf_weight_;             // TSDF权重（融合次数）
    
    // ========== 构造与析构 ==========
    VisualPoint(const V3D& pos);
    VisualPoint(const V3D& pos, const V3D& color);
    ~VisualPoint();
    
    // ========== 观测管理 ==========
    /**
     * @brief 添加新的观测
     */
    void addObservation(Feature* ftr);
    
    /**
     * @brief 删除指定观测
     */
    void deleteObservation(Feature* ftr);
    
    /**
     * @brief 删除除参考Patch外的所有观测（节省内存）
     */
    void deleteNonRefObservations();
    
    /**
     * @brief 查找最佳参考Patch
     * @param current_pos 当前相机位置
     * @param ref_ftr 输出：参考Feature
     * @return 是否找到合适的参考
     */
    bool findBestReferenceObservation(const V3D& current_pos, Feature*& ref_ftr) const;
    
    /**
     * @brief 查找得分最低的观测（用于替换）
     */
    void findMinScoreObservation(const V3D& framepos, Feature*& ftr) const;
    
    // ========== 质量更新 ==========
    /**
     * @brief 更新置信度评分
     * 基于：观测次数、光度误差、几何一致性
     */
    void updateConfidence();
    
    /**
     * @brief 更新法向量（基于多帧观测的平均）
     */
    void updateNormal();
    
    /**
     * @brief 更新颜色（基于多帧观测的平均）
     */
    void updateColor();
    
    // ========== 几何检查 ==========
    /**
     * @brief 检查新观测是否与现有观测几何一致
     * @param new_pos 新观测推算的3D位置
     * @param threshold 阈值（米）
     * @return 是否一致
     */
    bool checkGeometricConsistency(const V3D& new_pos, double threshold = 0.05) const;
    
    /**
     * @brief 计算重投影误差
     */
    double computeReprojectionError(const Feature* ftr) const;
};

#endif // TARGET_RECON_VISUAL_POINT_H_

