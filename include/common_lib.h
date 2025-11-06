/*
 * Target Reconstruction - Common Library
 * 
 * 通用数据类型和常量定义
 * 基于 FAST-LIVO2 框架，针对静态目标重建优化
 */

#ifndef TARGET_RECON_COMMON_LIB_H_
#define TARGET_RECON_COMMON_LIB_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.h>
#include <vector>
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>

// 使用 Sophus 命名空间（与 FAST-LIVO2 保持一致）
using namespace Sophus;

// Eigen 类型别名
using V2D = Eigen::Vector2d;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using M6D = Eigen::Matrix<double, 6, 6>;
using Quaterniond = Eigen::Quaterniond;

// OpenCV 类型
using cv::Mat;

typedef struct pointWithVar
{
  // ==================== 位置信息（不同坐标系） ====================
  Eigen::Vector3d point_c;     // LiDAR体坐标系（Body frame）下的点坐标
  Eigen::Vector3d point_i;     // IMU体坐标系（IMU frame）下的点坐标（经过外参变换）
  Eigen::Vector3d point_w;     // 世界坐标系（World frame）下的点坐标（用于建图和匹配）
  
  // ==================== 协方差矩阵（不确定性建模） ====================
  Eigen::Matrix3d var_nostate; // 去除状态协方差影响后的点位置协方差（仅包含测量噪声）
  Eigen::Matrix3d body_var;    // 点在LiDAR体坐标系下的协方差（由测距误差和角度误差建模）
  Eigen::Matrix3d var;         // 点在世界坐标系下的完整协方差（包含状态不确定性传播）
                               // var = R*body_var*R^T + 状态协方差传播项
  
  // ==================== 雅可比矩阵相关 ====================
  Eigen::Matrix3d point_crossmat; // 点坐标的反对称矩阵 [point]_×（用于计算旋转雅可比）
                                  // point_crossmat = skew(point_i) = [0 -z y; z 0 -x; -y x 0]
                                  // 用途：∂(R*p)/∂R = -R*[p]_×
  
  // ==================== 几何信息 ====================
  Eigen::Vector3d normal;      // 点所在平面的法向量（由体素地图平面拟合得到）
                               // 用于：1) 点到平面距离计算  2) 视觉点选择（观测角度判断）
  
  pointWithVar()
  {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_c = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  };
} pointWithVar;

// 常量定义
constexpr int PATCH_SIZE = 5;              // Patch边长（像素）
constexpr int PATCH_SIZE_HALF = 2;         // Patch半径
constexpr int PATCH_SIZE_TOTAL = 25;       // Patch总像素数
constexpr int PYRAMID_LEVELS = 3;          // 图像金字塔层数

constexpr double VOXEL_SIZE = 0.05;        // 体素大小（米）- 针对目标重建，比SLAM更精细
constexpr int MAX_OBSERVATIONS = 50;       // 每个视觉点的最大观测数

// 预留容量
constexpr int SIZE_SMALL = 100;
constexpr int SIZE_MEDIUM = 500;
constexpr int SIZE_LARGE = 2000;

#endif // TARGET_RECON_COMMON_LIB_H_

