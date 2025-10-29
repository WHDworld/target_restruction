/*
 * Target Reconstruction - VisualPoint Implementation
 */

#include "visual_point.h"
#include <algorithm>
#include <cmath>
#include <limits>

VisualPoint::VisualPoint(const V3D& pos)
    : pos_(pos), normal_(V3D::Zero()), covariance_(M3D::Identity()),
      color_(V3D::Zero()), has_color_(false),
      ref_patch_(nullptr), has_ref_patch_(false),
      is_normal_initialized_(false), is_converged_(false), is_outlier_(false),
      confidence_(0.0f), num_observations_(0), avg_photometric_error_(0.0f),
      tsdf_value_(0.0f), tsdf_weight_(0.0f)
{
    obs_.clear();
}

VisualPoint::VisualPoint(const V3D& pos, const V3D& color)
    : pos_(pos), normal_(V3D::Zero()), covariance_(M3D::Identity()),
      color_(color), has_color_(true),
      ref_patch_(nullptr), has_ref_patch_(false),
      is_normal_initialized_(false), is_converged_(false), is_outlier_(false),
      confidence_(0.0f), num_observations_(0), avg_photometric_error_(0.0f),
      tsdf_value_(0.0f), tsdf_weight_(0.0f)
{
    obs_.clear();
}

VisualPoint::~VisualPoint()
{
    // 删除所有观测Feature
    for (auto ftr : obs_) {
        if (ftr != nullptr) {
            delete ftr;
        }
    }
    obs_.clear();
}

void VisualPoint::addObservation(Feature* ftr)
{
    if (ftr == nullptr) return;
    
    obs_.push_back(ftr);
    num_observations_++;
    
    // 自动更新置信度
    updateConfidence();
    
    // 如果观测足够，标记为收敛
    if (num_observations_ >= 10) {
        is_converged_ = true;
    }
}

void VisualPoint::deleteObservation(Feature* ftr)
{
    if (ftr == nullptr) return;
    
    auto it = std::find(obs_.begin(), obs_.end(), ftr);
    if (it != obs_.end()) {
        delete *it;
        obs_.erase(it);
        num_observations_ = obs_.size();
    }
    
    // 如果是参考Patch，清除标记
    if (ftr == ref_patch_) {
        ref_patch_ = nullptr;
        has_ref_patch_ = false;
    }
}

void VisualPoint::deleteNonRefObservations()
{
    if (!has_ref_patch_ || ref_patch_ == nullptr) {
        return;
    }
    
    // 只保留参考Patch
    for (auto it = obs_.begin(); it != obs_.end(); ) {
        if (*it != ref_patch_) {
            delete *it;
            it = obs_.erase(it);
        } else {
            ++it;
        }
    }
    
    num_observations_ = obs_.size();
}

bool VisualPoint::findBestReferenceObservation(const V3D& current_pos, Feature*& ref_ftr) const
{
    if (obs_.empty()) {
        return false;
    }
    
    // 如果已有参考Patch，直接返回
    if (has_ref_patch_ && ref_patch_ != nullptr) {
        ref_ftr = ref_patch_;
        return true;
    }
    
    // 查找距离当前位置最近的观测
    double min_dist = std::numeric_limits<double>::max();
    Feature* best_ftr = nullptr;
    
    for (auto ftr : obs_) {
        // 计算观测位置与当前位置的距离
        V3D obs_pos = -ftr->T_c_w_rotation_.transpose() * ftr->T_c_w_translation_;
        double dist = (obs_pos - current_pos).norm();
        
        if (dist < min_dist) {
            min_dist = dist;
            best_ftr = ftr;
        }
    }
    
    if (best_ftr != nullptr) {
        ref_ftr = best_ftr;
        return true;
    }
    
    return false;
}

void VisualPoint::findMinScoreObservation(const V3D& framepos, Feature*& ftr) const
{
    if (obs_.empty()) {
        ftr = nullptr;
        return;
    }
    
    // 查找距离当前位置最远的观测（得分最低）
    double max_dist = 0.0;
    Feature* worst_ftr = nullptr;
    
    for (auto f : obs_) {
        V3D obs_pos = -f->T_c_w_rotation_.transpose() * f->T_c_w_translation_;
        double dist = (obs_pos - framepos).norm();
        
        if (dist > max_dist) {
            max_dist = dist;
            worst_ftr = f;
        }
    }
    
    ftr = worst_ftr;
}

void VisualPoint::updateConfidence()
{
    // 基于观测次数的得分（0-1）
    float obs_score = std::min(1.0f, num_observations_ / 10.0f);
    
    // 基于光度误差的得分（0-1）
    float error_score = 1.0f;
    if (avg_photometric_error_ > 0) {
        error_score = std::exp(-avg_photometric_error_ / 100.0f);
    }
    
    // 基于是否已初始化法向量
    float normal_score = is_normal_initialized_ ? 1.0f : 0.5f;
    
    // 综合评分
    confidence_ = 0.4f * obs_score + 0.4f * error_score + 0.2f * normal_score;
    
    // 限制在[0, 1]范围
    confidence_ = std::max(0.0f, std::min(1.0f, confidence_));
}

void VisualPoint::updateNormal()
{
    if (obs_.empty() || !is_normal_initialized_) {
        return;
    }
    
    // 简化版本：保持当前法向量
    // 完整版本可以基于多帧深度图梯度平均
    // 这里暂不实现复杂的法向量融合
}

void VisualPoint::updateColor()
{
    if (!has_color_ || obs_.empty()) {
        return;
    }
    
    // 简化版本：保持当前颜色
    // 完整版本需要从各个Feature的图像中提取颜色并平均
    // 由于Feature中没有存储完整图像，这里暂时保持原始颜色
}

bool VisualPoint::checkGeometricConsistency(const V3D& new_pos, double threshold) const
{
    double distance = (new_pos - pos_).norm();
    return distance < threshold;
}

double VisualPoint::computeReprojectionError(const Feature* ftr) const
{
    if (ftr == nullptr || !ftr->depth_valid_) {
        return std::numeric_limits<double>::max();
    }
    
    // 将3D点投影到Feature对应的图像
    // 世界坐标 -> 相机坐标
    V3D pt_camera = ftr->T_c_w_rotation_ * pos_ + ftr->T_c_w_translation_;
    
    // 相机坐标 -> 归一化平面
    if (pt_camera.z() <= 0) {
        return std::numeric_limits<double>::max();
    }
    
    V3D pt_normalized = pt_camera / pt_camera.z();
    
    // 计算像素误差
    // 这里简化处理，假设f_已经是归一化坐标
    double error = (pt_normalized - ftr->f_ * ftr->depth_).norm();
    
    return error;
}

