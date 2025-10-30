/*
 * Target Reconstruction - TargetReconstructor Implementation
 */

#include "target_reconstructor.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <algorithm>

TargetReconstructor::TargetReconstructor(const ReconstructionConfig& config)
    : config_(config), frame_count_(0), total_points_created_(0), total_observations_(0)
{
    // 地图管理器将在initROS中创建（需要先读取相机内参）
    map_manager_ = nullptr;
    
    // 初始化相机内参（默认值）
    fx_ = 615.0;
    fy_ = 615.0;
    cx_ = config_.image_width / 2.0;
    cy_ = config_.image_height / 2.0;
    
    // 初始化网格
    grid_n_width_ = config_.image_width / config_.grid_size;
    grid_n_height_ = config_.image_height / config_.grid_size;
    int grid_total = grid_n_width_ * grid_n_height_;
    
    grid_scores_.resize(grid_total, 0.0f);
    grid_candidates_.resize(grid_total, V2D(-1, -1));
    
    // 初始化当前位姿为单位矩阵
    current_R_ = M3D::Identity();
    current_t_ = V3D::Zero();
    current_timestamp_ = 0.0;
    current_bbox_ = BoundingBox();
    
    ROS_INFO("TargetReconstructor initialized");
    ROS_INFO("  Grid: %d x %d", grid_n_width_, grid_n_height_);
    ROS_INFO("  Voxel size: %.3f m", config_.voxel_size);
}

TargetReconstructor::~TargetReconstructor()
{
    if (map_manager_ != nullptr) {
        delete map_manager_;
        map_manager_ = nullptr;
    }
}

void TargetReconstructor::initROS(ros::NodeHandle& nh)
{
    // 读取相机内参（从YAML配置文件）
    nh.param("camera/fx", fx_, 615.0);
    nh.param("camera/fy", fy_, 615.0);
    nh.param("camera/cx", cx_, config_.image_width / 2.0);
    nh.param("camera/cy", cy_, config_.image_height / 2.0);
    
    ROS_INFO("Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
             fx_, fy_, cx_, cy_);
    
    // 创建地图管理器（使用读取到的相机内参）
    if (map_manager_ != nullptr) {
        delete map_manager_;
    }
    map_manager_ = new VoxelMapManager(config_.voxel_size, fx_, fy_, cx_, cy_);
    ROS_INFO("VoxelMapManager created with camera intrinsics");
    
    // 订阅话题（暂时注释，等创建自定义消息后启用）
    // ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 1, 
    //     &TargetReconstructor::rgbCallback, this);
    // ros::Subscriber depth_sub = nh.subscribe("/camera/depth/image_raw", 1,
    //     &TargetReconstructor::depthCallback, this);
    
    // 发布重建点云
    static ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
        "/target_reconstruction/pointcloud", 1);
}

void TargetReconstructor::processFrame(
    const Mat& rgb_img,
    const Mat& depth_img,
    const BoundingBox& bbox,
    const M3D& camera_R,
    const V3D& camera_t,
    double timestamp)
{
    if (!bbox.isValid()) {
        ROS_WARN("Invalid bounding box, skipping frame");
        return;
    }
    
    // 更新当前帧信息
    frame_count_++;
    current_rgb_ = rgb_img.clone();
    current_depth_ = depth_img.clone();
    current_bbox_ = bbox;
    current_R_ = camera_R;
    current_t_ = camera_t;
    current_timestamp_ = timestamp;
    
    ROS_INFO("========== Processing Frame %d ==========", frame_count_);
    
    // Step 1: 生成新的视觉点
    ROS_INFO("Step 1: Generating new visual points...");
    generateVisualPoints(rgb_img, depth_img, bbox, camera_R, camera_t);
    
    // Step 2: 检索可见的历史点
    ROS_INFO("Step 2: Retrieving visible points...");
    auto visible_points = retrieveVisiblePoints(bbox, camera_R, camera_t);
    ROS_INFO("  Found %zu visible points", visible_points.size());
    
    // Step 3: 更新已观测的点
    if (!visible_points.empty()) {
        ROS_INFO("Step 3: Updating visual points...");
        updateVisualPoints(rgb_img, visible_points, camera_R, camera_t);
    }
    
    // Step 4: 定期优化地图
    if (frame_count_ % 10 == 0) {
        ROS_INFO("Step 4: Optimizing map...");
        optimizeMap();
    }
    
    // 打印统计信息
    size_t total_pts = map_manager_->getTotalPoints();
    ROS_INFO("Frame %d summary: Total points = %zu, Created = %d, Observations = %d",
             frame_count_, total_pts, total_points_created_, total_observations_);
    ROS_INFO("==========================================\n");
}

void TargetReconstructor::processFrameWithMask(
    const Mat& rgb_img,
    const Mat& depth_img,
    const Mat& mask,
    const BoundingBox& bbox,
    const M3D& camera_R,
    const V3D& camera_t,
    double timestamp)
{
    if (!bbox.isValid()) {
        ROS_WARN("Invalid bounding box, skipping frame");
        return;
    }
    
    // 检查 mask 有效性
    if (mask.empty() || mask.rows != rgb_img.rows || mask.cols != rgb_img.cols) {
        ROS_WARN("Invalid mask, falling back to bbox-only processing");
        processFrame(rgb_img, depth_img, bbox, camera_R, camera_t, timestamp);
        return;
    }
    
    // 更新当前帧信息
    frame_count_++;
    current_rgb_ = rgb_img.clone();
    current_depth_ = depth_img.clone();
    current_bbox_ = bbox;
    current_R_ = camera_R;
    current_t_ = camera_t;
    current_timestamp_ = timestamp;
    
    ROS_INFO("========== Processing Frame %d (with Mask) ==========", frame_count_);
    
    // Step 1: 生成新的视觉点（使用 mask 进行精确筛选）
    ROS_INFO("Step 1: Generating new visual points with mask...");
    generateVisualPointsWithMask(rgb_img, depth_img, mask, bbox, camera_R, camera_t);
    
    // Step 2: 检索可见的历史点
    ROS_INFO("Step 2: Retrieving visible points...");
    auto visible_points = retrieveVisiblePoints(bbox, camera_R, camera_t);
    ROS_INFO("  Found %zu visible points", visible_points.size());
    
    // Step 3: 更新已观测的点（可选：使用 mask 过滤）
    if (!visible_points.empty()) {
        ROS_INFO("Step 3: Updating visual points...");
        updateVisualPointsWithMask(rgb_img, mask, visible_points, camera_R, camera_t);
    }
    
    // Step 4: 定期优化地图
    if (frame_count_ % 10 == 0) {
        ROS_INFO("Step 4: Optimizing map...");
        optimizeMap();
    }
    
    // 打印统计信息
    size_t total_pts = map_manager_->getTotalPoints();
    ROS_INFO("Frame %d summary: Total points = %zu, Created = %d, Observations = %d",
             frame_count_, total_pts, total_points_created_, total_observations_);
    ROS_INFO("==========================================\n");
}

void TargetReconstructor::generateVisualPoints(
    const Mat& rgb_img,
    const Mat& depth_img,
    const BoundingBox& bbox,
    const M3D& camera_R,
    const V3D& camera_t)
{
    // 转换为灰度图
    Mat gray_img;
    if (rgb_img.channels() == 3) {
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
    } else {
        gray_img = rgb_img.clone();
    }
    
    // 提取候选点
    auto candidates = extractCandidatePoints(gray_img, depth_img, bbox);
    
    if (candidates.empty()) {
        ROS_WARN("  No candidate points found");
        return;
    }
    
    int created_count = 0;
    
    for (const auto& px : candidates) {
        int x = static_cast<int>(px.x());
        int y = static_cast<int>(px.y());
        
        // 获取深度（假设depth_img是float类型，单位：米）
        float depth = depth_img.at<float>(y, x);
        
        if (!isDepthValid(depth)) continue;
        
        // 像素 + 深度 -> 3D世界坐标
        V3D pos_3d = pixelToWorld(px, depth, camera_R, camera_t);
        
        // 获取RGB颜色
        V3D color(128, 128, 128);  // 默认灰色
        if (rgb_img.channels() == 3) {
            cv::Vec3b bgr = rgb_img.at<cv::Vec3b>(y, x);
            color = V3D(bgr[2], bgr[1], bgr[0]);  // BGR -> RGB
        }
        
        // 提取图像Patch
        float patch[PATCH_SIZE_TOTAL];
        extractImagePatch(gray_img, px, patch);
        
        // 创建VisualPoint
        VisualPoint* pt = new VisualPoint(pos_3d, color);
        
        // 计算法向量（从深度图）
        pt->normal_ = computeNormalFromDepth(depth_img, x, y);
        pt->is_normal_initialized_ = true;
        
        // 创建归一化平面坐标
        V3D f((x - cx_) / fx_, (y - cy_) / fy_, 1.0);
        f.normalize();
        
        // 创建首次观测Feature
        Feature* ftr = new Feature(pt, patch, px, f, camera_R, camera_t, depth, 0);
        ftr->id_ = frame_count_;
        ftr->img_ = gray_img.clone();  // 保存图像用于后续Warping
        
        // 添加观测
        pt->addObservation(ftr);
        
        // 插入地图
        map_manager_->insertPoint(pt);
        created_count++;
    }
    
    total_points_created_ += created_count;
    ROS_INFO("  Created %d new visual points", created_count);
}

void TargetReconstructor::generateVisualPointsWithMask(
    const Mat& rgb_img,
    const Mat& depth_img,
    const Mat& mask,
    const BoundingBox& bbox,
    const M3D& camera_R,
    const V3D& camera_t)
{
    // 转换为灰度图
    Mat gray_img;
    if (rgb_img.channels() == 3) {
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
    } else {
        gray_img = rgb_img.clone();
    }
    
    // 提取候选点
    auto candidates = extractCandidatePoints(gray_img, depth_img, bbox);
    
    if (candidates.empty()) {
        ROS_WARN("  No candidate points found");
        return;
    }
    
    int created_count = 0;
    int filtered_by_mask = 0;
    
    for (const auto& px : candidates) {
        int x = static_cast<int>(px.x());
        int y = static_cast<int>(px.y());
        
        // **关键：使用 mask 过滤**
        if (x < 0 || x >= mask.cols || y < 0 || y >= mask.rows) continue;
        if (mask.at<uint8_t>(y, x) < 128) {  // 不在目标区域内
            filtered_by_mask++;
            continue;
        }
        
        // 获取深度（假设depth_img是float类型，单位：米）
        float depth = depth_img.at<float>(y, x);
        
        if (!isDepthValid(depth)) continue;
        
        // 像素 + 深度 -> 3D世界坐标
        V3D pos_3d = pixelToWorld(px, depth, camera_R, camera_t);
        
        // 获取RGB颜色
        V3D color(128, 128, 128);  // 默认灰色
        if (rgb_img.channels() == 3) {
            cv::Vec3b bgr = rgb_img.at<cv::Vec3b>(y, x);
            color = V3D(bgr[2], bgr[1], bgr[0]);  // BGR -> RGB
        }
        
        // 提取图像Patch
        float patch[PATCH_SIZE_TOTAL];
        extractImagePatch(gray_img, px, patch);
        
        // 创建VisualPoint
        VisualPoint* pt = new VisualPoint(pos_3d, color);
        
        // 计算法向量（从深度图）
        pt->normal_ = computeNormalFromDepth(depth_img, x, y);
        pt->is_normal_initialized_ = true;
        
        // 创建归一化平面坐标
        V3D f((x - cx_) / fx_, (y - cy_) / fy_, 1.0);
        f.normalize();
        
        // 创建首次观测Feature
        Feature* ftr = new Feature(pt, patch, px, f, camera_R, camera_t, depth, 0);
        ftr->id_ = frame_count_;
        ftr->img_ = gray_img.clone();  // 保存图像用于后续Warping
        
        // 添加观测
        pt->addObservation(ftr);
        
        // 插入地图
        map_manager_->insertPoint(pt);
        created_count++;
    }
    
    total_points_created_ += created_count;
    ROS_INFO("  Created %d new visual points (%d filtered by mask)", 
             created_count, filtered_by_mask);
}

std::vector<VisualPoint*> TargetReconstructor::retrieveVisiblePoints(
    const BoundingBox& bbox,
    const M3D& camera_R,
    const V3D& camera_t)
{
    // 使用VoxelMapManager的功能获取检测框内的点
    return map_manager_->getPointsInBoundingBox(bbox, camera_R, camera_t, current_depth_);
}

void TargetReconstructor::updateVisualPoints(
    const Mat& rgb_img,
    const std::vector<VisualPoint*>& visible_points,
    const M3D& camera_R,
    const V3D& camera_t)
{
    // 转换为灰度图
    Mat gray_img;
    if (rgb_img.channels() == 3) {
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
    } else {
        gray_img = rgb_img.clone();
    }
    
    int updated_count = 0;
    
    for (auto pt : visible_points) {
        if (pt == nullptr || pt->is_outlier_) continue;
        
        // 投影到当前帧
        V2D px = worldToPixel(pt->pos_, camera_R, camera_t);
        
        // 检查是否在图像范围内
        int x = static_cast<int>(px.x());
        int y = static_cast<int>(px.y());
        
        if (!isInImage(x, y)) continue;
        
        // 获取深度并进行几何一致性检查
        float depth = current_depth_.at<float>(y, x);
        
        if (!isDepthValid(depth)) continue;
        
        V3D pos_new = pixelToWorld(px, depth, camera_R, camera_t);
        
        if (!pt->checkGeometricConsistency(pos_new, 0.1)) {
            // 几何不一致，可能是遮挡或误匹配
            continue;
        }
        
        // 判断是否需要添加新观测
        bool add_flag = false;
        
        if (pt->obs_.empty()) {
            add_flag = true;
        } else {
            // 检查与最后一次观测的差异
            Feature* last_ftr = pt->obs_.back();
            
            // 计算位姿变化
            V3D last_cam_pos = -last_ftr->T_c_w_rotation_.transpose() * last_ftr->T_c_w_translation_;
            V3D current_cam_pos = -camera_R.transpose() * camera_t;
            
            double delta_pos = (current_cam_pos - last_cam_pos).norm();
            double delta_pixel = (px - last_ftr->px_).norm();
            
            // 位姿变化超过阈值或像素位置变化较大
            if (delta_pos > 0.3 || delta_pixel > 30) {
                add_flag = true;
            }
        }
        
        if (add_flag) {
            // 提取Patch
            float patch[PATCH_SIZE_TOTAL];
            extractImagePatch(gray_img, px, patch);
            
            // 创建归一化平面坐标
            V3D f((x - cx_) / fx_, (y - cy_) / fy_, 1.0);
            f.normalize();
            
            // 创建新观测
            Feature* ftr = new Feature(pt, patch, px, f, camera_R, camera_t, depth, 0);
            ftr->id_ = frame_count_;
            ftr->img_ = gray_img.clone();
            
            // 添加观测
            pt->addObservation(ftr);
            updated_count++;
            total_observations_++;
            
            // 限制观测数量
            if (pt->obs_.size() > MAX_OBSERVATIONS) {
                Feature* worst_ftr = nullptr;
                V3D current_cam_pos = -camera_R.transpose() * camera_t;
                pt->findMinScoreObservation(current_cam_pos, worst_ftr);
                
                if (worst_ftr != nullptr) {
                    pt->deleteObservation(worst_ftr);
                }
            }
            
            // 更新点的属性
            pt->updateConfidence();
            pt->updateColor();
        }
    }
    
    ROS_INFO("  Updated %d visual points with new observations", updated_count);
}

void TargetReconstructor::updateVisualPointsWithMask(
    const Mat& rgb_img,
    const Mat& mask,
    const std::vector<VisualPoint*>& visible_points,
    const M3D& camera_R,
    const V3D& camera_t)
{
    // 转换为灰度图
    Mat gray_img;
    if (rgb_img.channels() == 3) {
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
    } else {
        gray_img = rgb_img.clone();
    }
    
    int updated_count = 0;
    int filtered_by_mask = 0;
    
    for (auto pt : visible_points) {
        if (pt == nullptr || pt->is_outlier_) continue;
        
        // 投影到当前帧
        V2D px = worldToPixel(pt->pos_, camera_R, camera_t);
        
        // 检查是否在图像范围内
        int x = static_cast<int>(px.x());
        int y = static_cast<int>(px.y());
        
        if (!isInImage(x, y)) continue;
        
        // **关键：使用 mask 过滤**
        if (x >= 0 && x < mask.cols && y >= 0 && y < mask.rows) {
            if (mask.at<uint8_t>(y, x) < 128) {  // 不在目标区域内
                filtered_by_mask++;
                continue;
            }
        }
        
        // 获取深度并进行几何一致性检查
        float depth = current_depth_.at<float>(y, x);
        
        if (!isDepthValid(depth)) continue;
        
        V3D pos_new = pixelToWorld(px, depth, camera_R, camera_t);
        
        if (!pt->checkGeometricConsistency(pos_new, 0.1)) {
            // 几何不一致，可能是遮挡或误匹配
            continue;
        }
        
        // 判断是否需要添加新观测
        bool add_flag = false;
        
        if (pt->obs_.empty()) {
            add_flag = true;
        } else {
            // 检查与最后一次观测的差异
            Feature* last_ftr = pt->obs_.back();
            
            // 计算位姿变化
            V3D last_cam_pos = -last_ftr->T_c_w_rotation_.transpose() * last_ftr->T_c_w_translation_;
            V3D current_cam_pos = -camera_R.transpose() * camera_t;
            
            double delta_pos = (current_cam_pos - last_cam_pos).norm();
            double delta_pixel = (px - last_ftr->px_).norm();
            
            // 位姿变化超过阈值或像素位置变化较大
            if (delta_pos > 0.3 || delta_pixel > 30) {
                add_flag = true;
            }
        }
        
        if (add_flag) {
            // 提取Patch
            float patch[PATCH_SIZE_TOTAL];
            extractImagePatch(gray_img, px, patch);
            
            // 创建归一化平面坐标
            V3D f((x - cx_) / fx_, (y - cy_) / fy_, 1.0);
            f.normalize();
            
            // 创建新观测
            Feature* ftr = new Feature(pt, patch, px, f, camera_R, camera_t, depth, 0);
            ftr->id_ = frame_count_;
            ftr->img_ = gray_img.clone();
            
            // 添加观测
            pt->addObservation(ftr);
            updated_count++;
            total_observations_++;
            
            // 限制观测数量
            if (pt->obs_.size() > MAX_OBSERVATIONS) {
                Feature* worst_ftr = nullptr;
                V3D current_cam_pos = -camera_R.transpose() * camera_t;
                pt->findMinScoreObservation(current_cam_pos, worst_ftr);
                
                if (worst_ftr != nullptr) {
                    pt->deleteObservation(worst_ftr);
                }
            }
            
            // 更新点的属性
            pt->updateConfidence();
            pt->updateColor();
        }
    }
    
    ROS_INFO("  Updated %d visual points with new observations (%d filtered by mask)", 
             updated_count, filtered_by_mask);
}

std::vector<V2D> TargetReconstructor::extractCandidatePoints(
    const Mat& gray_img,
    const Mat& depth_img,
    const BoundingBox& bbox)
{
    resetGrid();
    
    // 遍历检测框内的像素（跳步采样以提高效率）
    int step = 2;
    
    for (int y = bbox.y_min; y <= bbox.y_max; y += step) {
        for (int x = bbox.x_min; x <= bbox.x_max; x += step) {
            if (!isInImage(x, y)) continue;
            
            // 检查深度
            float depth = depth_img.at<float>(y, x);
            if (!isDepthValid(depth)) continue;
            
            // 计算Shi-Tomasi角点响应
            float score = computeShiTomasiScore(gray_img, x, y);
            
            if (score < config_.min_shi_tomasi_score) continue;
            
            // 网格索引
            int grid_x = x / config_.grid_size;
            int grid_y = y / config_.grid_size;
            int grid_idx = grid_y * grid_n_width_ + grid_x;
            
            if (grid_idx < 0 || grid_idx >= grid_scores_.size()) continue;
            
            // 在网格内竞争：保留响应最大的点
            if (score > grid_scores_[grid_idx]) {
                grid_scores_[grid_idx] = score;
                grid_candidates_[grid_idx] = V2D(x, y);
            }
        }
    }
    
    // 收集选中的候选点
    std::vector<V2D> candidates;
    candidates.reserve(grid_scores_.size());
    
    for (const auto& px : grid_candidates_) {
        if (px.x() >= 0 && px.y() >= 0) {  // 有效点
            candidates.push_back(px);
        }
    }
    
    return candidates;
}

float TargetReconstructor::computeShiTomasiScore(const Mat& img, int x, int y)
{
    const int patch_size = 2;
    
    // 边界检查
    if (x < patch_size || x >= img.cols - patch_size ||
        y < patch_size || y >= img.rows - patch_size) {
        return 0.0f;
    }
    
    // 计算图像梯度的自相关矩阵
    float dx2 = 0.0f;  // Ix^2
    float dy2 = 0.0f;  // Iy^2
    float dxy = 0.0f;  // Ix*Iy
    
    for (int v = -patch_size; v <= patch_size; ++v) {
        for (int u = -patch_size; u <= patch_size; ++u) {
            // 使用Sobel算子计算梯度
            float Ix = (img.at<uchar>(y + v, x + u + 1) - 
                       img.at<uchar>(y + v, x + u - 1)) / 2.0f;
            float Iy = (img.at<uchar>(y + v + 1, x + u) - 
                       img.at<uchar>(y + v - 1, x + u)) / 2.0f;
            
            dx2 += Ix * Ix;
            dy2 += Iy * Iy;
            dxy += Ix * Iy;
        }
    }
    
    // 计算Shi-Tomasi得分（矩阵的较小特征值）
    float trace = dx2 + dy2;
    float det = dx2 * dy2 - dxy * dxy;
    
    // 较小特征值 = (trace - sqrt(trace^2 - 4*det)) / 2
    float discriminant = trace * trace - 4.0f * det;
    
    if (discriminant < 0) return 0.0f;
    
    float score = (trace - std::sqrt(discriminant)) / 2.0f;
    
    return score;
}

void TargetReconstructor::extractImagePatch(const Mat& img, const V2D& px, float* patch)
{
    int x = static_cast<int>(px.x());
    int y = static_cast<int>(px.y());
    
    int idx = 0;
    for (int v = -PATCH_SIZE_HALF; v <= PATCH_SIZE_HALF; ++v) {
        for (int u = -PATCH_SIZE_HALF; u <= PATCH_SIZE_HALF; ++u) {
            int px_x = x + u;
            int px_y = y + v;
            
            // 边界检查
            if (px_x >= 0 && px_x < img.cols && px_y >= 0 && px_y < img.rows) {
                patch[idx] = static_cast<float>(img.at<uchar>(px_y, px_x));
            } else {
                patch[idx] = 0.0f;
            }
            idx++;
        }
    }
}

V3D TargetReconstructor::pixelToWorld(const V2D& px, float depth, 
                                      const M3D& R_c_w, const V3D& t_c_w) const
{
    // 像素 -> 归一化平面
    double x_norm = (px.x() - cx_) / fx_;
    double y_norm = (px.y() - cy_) / fy_;
    
    // 归一化平面 -> 相机坐标系
    V3D pt_camera(x_norm * depth, y_norm * depth, depth);
    
    // 相机坐标系 -> 世界坐标系
    // X_world = R_w_c * X_camera + t_w_c
    // 其中 R_w_c = R_c_w^T, t_w_c = -R_c_w^T * t_c_w
    V3D pt_world = R_c_w.transpose() * (pt_camera - t_c_w);
    
    return pt_world;
}

V2D TargetReconstructor::worldToPixel(const V3D& pos_w, const M3D& R_c_w, const V3D& t_c_w) const
{
    // 世界坐标系 -> 相机坐标系
    V3D pt_camera = R_c_w * pos_w + t_c_w;
    
    if (pt_camera.z() <= 0) {
        return V2D(-1, -1);  // 在相机后面
    }
    
    // 相机坐标系 -> 像素
    double u = fx_ * pt_camera.x() / pt_camera.z() + cx_;
    double v = fy_ * pt_camera.y() / pt_camera.z() + cy_;
    
    return V2D(u, v);
}

V3D TargetReconstructor::computeNormalFromDepth(const Mat& depth_img, int x, int y) const
{
    // 使用深度梯度估计法向量
    const int step = 2;
    
    if (x < step || x >= depth_img.cols - step ||
        y < step || y >= depth_img.rows - step) {
        return V3D(0, 0, 1);  // 默认朝向相机
    }
    
    float depth_center = depth_img.at<float>(y, x);
    float depth_right = depth_img.at<float>(y, x + step);
    float depth_down = depth_img.at<float>(y + step, x);
    
    if (depth_center <= 0 || depth_right <= 0 || depth_down <= 0) {
        return V3D(0, 0, 1);
    }
    
    // 计算3D点
    V3D p_center((x - cx_) * depth_center / fx_, (y - cy_) * depth_center / fy_, depth_center);
    V3D p_right((x + step - cx_) * depth_right / fx_, (y - cy_) * depth_right / fy_, depth_right);
    V3D p_down((x - cx_) * depth_down / fx_, (y + step - cy_) * depth_down / fy_, depth_down);
    
    // 叉积计算法向量
    V3D v1 = p_right - p_center;
    V3D v2 = p_down - p_center;
    V3D normal = v1.cross(v2);
    
    if (normal.norm() < 1e-6) {
        return V3D(0, 0, 1);
    }
    
    normal.normalize();
    
    // 确保法向量朝向相机
    if (normal.z() > 0) {
        normal = -normal;
    }
    
    return normal;
}

void TargetReconstructor::optimizeMap()
{
    // 移除低置信度的点
    map_manager_->removeOutliers(config_.min_confidence);
    
    // 移除观测次数过少的点
    map_manager_->removeUnderObservedPoints(config_.min_observations);
    
    ROS_INFO("  Map optimized: Total points = %zu", map_manager_->getTotalPoints());
}

bool TargetReconstructor::saveReconstruction(const std::string& filename) const
{
    return map_manager_->saveToFile(filename, config_.enable_color);
}

void TargetReconstructor::resetGrid()
{
    std::fill(grid_scores_.begin(), grid_scores_.end(), 0.0f);
    std::fill(grid_candidates_.begin(), grid_candidates_.end(), V2D(-1, -1));
}

// ROS回调函数（暂时简化实现）
void TargetReconstructor::rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // 将在完整集成时实现
}

void TargetReconstructor::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // 将在完整集成时实现
}

void TargetReconstructor::bboxCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // 将在完整集成时实现
}

void TargetReconstructor::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // 将在完整集成时实现
}

