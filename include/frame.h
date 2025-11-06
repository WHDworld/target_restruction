/*
 * Frame Data Structure for Target Reconstruction
 * 
 * 参考 FAST-LIVO2 Frame 类设计
 * 存储单帧的图像、位姿、点云和mask信息
 */

#ifndef TARGET_RECONSTRUCTION_FRAME_H_
#define TARGET_RECONSTRUCTION_FRAME_H_

#include <boost/noncopyable.hpp>
#include <vikit/abstract_camera.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>
#include "common_lib.h"

using namespace Eigen;

// ========== 数据帧结构 ==========
/// FrameData - 参考 FAST-LIVO2 Frame 类设计
/// 存储单帧的图像、位姿、点云和mask信息
class FrameData : boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ========== 基础信息 ==========
    double timestamp;              //!< 帧时间戳
    int id_;                       //!< 帧ID（可选）
    vk::AbstractCamera* cam_;      //!< 相机模型（用于像素与归一化坐标转换）
    
    // ========== 图像数据 ==========
    cv::Mat rgb_img;               //!< RGB图像
    
    // ========== 点云数据 ==========
    std::vector<pointWithVar> pg;  //!< 深度图转换的点云（带协方差）
    std::vector<Eigen::Vector2d> px_mask;  //!< mask中的白色像素坐标
    
    // ========== 位姿数据（使用SE3表示，与FAST-LIVO2一致）==========
    SE3 T_w_c_;                    //!< Transform: World from Camera（相机到世界的变换，p_w = T_w_c * p_c）
    SE3 T_w_i_;                    //!< Transform: World from IMU/Body（机体到世界的变换，p_w = T_w_i * p_i）
    
    // ========== 包围框 ==========
    BoundingBox bbox;              //!< 从mask计算的包围框
    
    // ========== 标志位 ==========
    bool has_valid_mask;           //!< 是否有有效的mask
    
    // ========== 构造/析构 ==========
    FrameData() 
        : timestamp(0.0), id_(-1), cam_(nullptr), has_valid_mask(false) {}
    
    FrameData(vk::AbstractCamera* cam) 
        : timestamp(0.0), id_(-1), cam_(cam), has_valid_mask(false) {}
    
    ~FrameData() {}

    // ========== 坐标变换函数（参考 Frame 类设计）==========
    
    /// 世界坐标 -> 相机像素坐标
    /// @param xyz_w: 世界坐标系下的3D点
    /// @return: 像素坐标 (u, v)
    inline Vector2d w2c(const Vector3d& xyz_w) const { 
        if (cam_ == nullptr) {
            // ROS_ERROR_THROTTLE(1.0, "Camera model is nullptr!");
            return Vector2d::Zero();
        }
        return cam_->world2cam(T_w_c_.inverse() * xyz_w);  // p_c = T_w_c^-1 * p_w
    }
    
    /// 世界坐标 -> 相机坐标系
    /// @param xyz_w: 世界坐标系下的3D点
    /// @return: 相机坐标系下的3D点
    inline Vector3d w2f(const Vector3d& xyz_w) const { 
        return T_w_c_.inverse() * xyz_w;  // p_c = T_w_c^-1 * p_w
    }
    
    /// 相机坐标系 -> 世界坐标
    /// @param xyz_c: 相机坐标系下的3D点
    /// @return: 世界坐标系下的3D点
    inline Vector3d f2w(const Vector3d& xyz_c) const { 
        return T_w_c_ * xyz_c;  // p_w = T_w_c * p_c
    }
    
    /// 像素坐标 -> 相机坐标系归一化平面（单位球面）
    /// @param px: 像素坐标 (u, v)
    /// @return: 相机坐标系下的单位方向向量
    inline Vector3d c2f(const Vector2d& px) const { 
        if (cam_ == nullptr) {
            // ROS_ERROR_THROTTLE(1.0, "Camera model is nullptr!");
            return Vector3d::Zero();
        }
        return cam_->cam2world(px[0], px[1]); 
    }
    
    /// 像素坐标 -> 相机坐标系归一化平面（单位球面）
    inline Vector3d c2f(const double x, const double y) const { 
        if (cam_ == nullptr) {
            // ROS_ERROR_THROTTLE(1.0, "Camera model is nullptr!");
            return Vector3d::Zero();
        }
        return cam_->cam2world(x, y); 
    }
    
    /// 相机坐标系 -> 像素坐标
    /// @param xyz_c: 相机坐标系下的3D点
    /// @return: 像素坐标 (u, v)
    inline Vector2d f2c(const Vector3d& xyz_c) const { 
        if (cam_ == nullptr) {
            // ROS_ERROR_THROTTLE(1.0, "Camera model is nullptr!");
            return Vector2d::Zero();
        }
        return cam_->world2cam(xyz_c); 
    }
    
    /// 返回相机在世界坐标系中的位置
    inline Vector3d cam_pos() const { 
        return T_w_c_.translation();  // T_w_c 的平移部分就是相机在世界系中的位置
    }
    
    /// 返回机体在世界坐标系中的位置
    inline Vector3d imu_pos() const { 
        return T_w_i_.translation();  // T_w_i 的平移部分就是机体在世界系中的位置
    }
    
    // ========== IMU/机体坐标系变换 ==========
    
    /// 世界坐标 -> 机体坐标
    inline Vector3d w2i(const Vector3d& xyz_w) const { 
        return T_w_i_.inverse() * xyz_w;  // p_i = T_w_i^-1 * p_w
    }
    
    /// 机体坐标 -> 世界坐标
    inline Vector3d i2w(const Vector3d& xyz_i) const { 
        return T_w_i_ * xyz_i;  // p_w = T_w_i * p_i
    }
    
    // ========== 辅助函数 ==========
    
    /// 检查像素坐标是否在图像内
    inline bool isInFrame(const Vector2i& px, int boundary = 0) const {
        if (cam_ == nullptr) return false;
        return cam_->isInFrame(px, boundary);
    }
    
    /// 设置相机位姿（从旋转矩阵和平移向量）
    /// @param R_w_c: 相机到世界的旋转矩阵
    /// @param t_w_c: 相机在世界系中的位置
    void setCameraPose(const Matrix3d& R_w_c, const Vector3d& t_w_c) {
        T_w_c_ = SE3(R_w_c, t_w_c);
    }
    
    /// 设置机体位姿（从旋转矩阵和平移向量）
    /// @param R_w_i: 机体到世界的旋转矩阵
    /// @param t_w_i: 机体在世界系中的位置
    void setIMUPose(const Matrix3d& R_w_i, const Vector3d& t_w_i) {
        T_w_i_ = SE3(R_w_i, t_w_i);
    }
};

typedef std::shared_ptr<FrameData> FrameDataPtr;

#endif // TARGET_RECONSTRUCTION_FRAME_H_

