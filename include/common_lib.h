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
#include <vector>
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>

// Eigen 类型别名
using V2D = Eigen::Vector2d;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using M6D = Eigen::Matrix<double, 6, 6>;
using Quaterniond = Eigen::Quaterniond;

// OpenCV 类型
using cv::Mat;

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

