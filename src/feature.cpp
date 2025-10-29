/*
 * Target Reconstruction - Feature Implementation
 */

#include "feature.h"
#include "visual_point.h"
#include <cstring>

Feature::Feature(VisualPoint* point, const float* patch, const V2D& px, 
                 const V3D& f, int level)
    : id_(-1), point_(point), px_(px), f_(f), level_(level),
      T_c_w_rotation_(M3D::Identity()), T_c_w_translation_(V3D::Zero()),
      depth_(0.0f), depth_valid_(false)
{
    std::memcpy(patch_, patch, PATCH_SIZE_TOTAL * sizeof(float));
}

Feature::Feature(VisualPoint* point, const float* patch, const V2D& px, 
                 const V3D& f, const M3D& R_c_w, const V3D& t_c_w, 
                 float depth, int level)
    : id_(-1), point_(point), px_(px), f_(f), level_(level),
      T_c_w_rotation_(R_c_w), T_c_w_translation_(t_c_w),
      depth_(depth), depth_valid_(depth > 0.0f)
{
    std::memcpy(patch_, patch, PATCH_SIZE_TOTAL * sizeof(float));
}

Feature::~Feature()
{
    // 不删除point_，因为它由VoxelMapManager管理
}

V3D Feature::pos() const
{
    if (!depth_valid_) {
        return V3D::Zero();
    }
    
    // 归一化平面坐标 * 深度 = 相机坐标系中的3D点
    V3D pt_camera = f_ * depth_;
    
    // 相机坐标系 -> 世界坐标系
    // X_world = R_w_c * X_camera + t_w_c
    // 其中 R_w_c = R_c_w^T, t_w_c = -R_c_w^T * t_c_w
    V3D pt_world = T_c_w_rotation_.transpose() * (pt_camera - T_c_w_translation_);
    
    return pt_world;
}

