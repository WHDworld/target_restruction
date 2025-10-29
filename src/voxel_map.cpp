/*
 * Target Reconstruction - VoxelMapManager Implementation
 */

#include "voxel_map.h"
#include <fstream>
#include <cmath>
#include <algorithm>

VoxelMapManager::VoxelMapManager(double voxel_size)
    : voxel_size_(voxel_size)
{
    voxel_map_.clear();
}

VoxelMapManager::~VoxelMapManager()
{
    clear();
}

VOXEL_LOCATION VoxelMapManager::getVoxelLocation(const V3D& pos) const
{
    int64_t x = static_cast<int64_t>(std::floor(pos.x() / voxel_size_));
    int64_t y = static_cast<int64_t>(std::floor(pos.y() / voxel_size_));
    int64_t z = static_cast<int64_t>(std::floor(pos.z() / voxel_size_));
    
    return VOXEL_LOCATION(x, y, z);
}

void VoxelMapManager::insertPoint(VisualPoint* point)
{
    if (point == nullptr) return;
    
    VOXEL_LOCATION voxel_loc = getVoxelLocation(point->pos_);
    
    auto iter = voxel_map_.find(voxel_loc);
    if (iter != voxel_map_.end()) {
        // 体素已存在，直接添加点
        iter->second->visual_points.push_back(point);
        iter->second->count++;
    } else {
        // 创建新体素
        VOXEL_POINTS* voxel = new VOXEL_POINTS(1);
        voxel->visual_points.push_back(point);
        
        // 计算体素中心
        voxel->center_ = V3D(
            voxel_loc.x * voxel_size_ + voxel_size_ / 2.0,
            voxel_loc.y * voxel_size_ + voxel_size_ / 2.0,
            voxel_loc.z * voxel_size_ + voxel_size_ / 2.0
        );
        
        voxel_map_[voxel_loc] = voxel;
    }
}

void VoxelMapManager::insertPoints(const std::vector<VisualPoint*>& points)
{
    for (auto pt : points) {
        insertPoint(pt);
    }
}

void VoxelMapManager::removePoint(const VOXEL_LOCATION& voxel_loc, VisualPoint* point)
{
    auto iter = voxel_map_.find(voxel_loc);
    if (iter == voxel_map_.end()) return;
    
    auto& points = iter->second->visual_points;
    auto it = std::find(points.begin(), points.end(), point);
    
    if (it != points.end()) {
        delete *it;
        points.erase(it);
        iter->second->count = points.size();
        
        // 如果体素为空，删除体素
        if (points.empty()) {
            delete iter->second;
            voxel_map_.erase(iter);
        }
    }
}

void VoxelMapManager::clear()
{
    for (auto& pair : voxel_map_) {
        if (pair.second != nullptr) {
            delete pair.second;  // VOXEL_POINTS的析构函数会删除所有VisualPoint
        }
    }
    voxel_map_.clear();
}

std::vector<VisualPoint*> VoxelMapManager::getPointsInVoxel(const VOXEL_LOCATION& voxel_loc) const
{
    auto iter = voxel_map_.find(voxel_loc);
    if (iter != voxel_map_.end()) {
        return iter->second->visual_points;
    }
    return std::vector<VisualPoint*>();
}

std::vector<VisualPoint*> VoxelMapManager::getPointsInRadius(const V3D& center, double radius) const
{
    std::vector<VisualPoint*> result;
    
    // 计算搜索范围（体素坐标）
    int range = static_cast<int>(std::ceil(radius / voxel_size_));
    VOXEL_LOCATION center_voxel = getVoxelLocation(center);
    
    // 遍历邻域体素
    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            for (int dz = -range; dz <= range; ++dz) {
                VOXEL_LOCATION voxel_loc(
                    center_voxel.x + dx,
                    center_voxel.y + dy,
                    center_voxel.z + dz
                );
                
                auto iter = voxel_map_.find(voxel_loc);
                if (iter != voxel_map_.end()) {
                    for (auto pt : iter->second->visual_points) {
                        // 检查实际距离
                        if ((pt->pos_ - center).norm() <= radius) {
                            result.push_back(pt);
                        }
                    }
                }
            }
        }
    }
    
    return result;
}

std::vector<VisualPoint*> VoxelMapManager::getPointsInBoundingBox(
    const BoundingBox& bbox,
    const M3D& camera_R,
    const V3D& camera_t,
    const Mat& depth_img) const
{
    std::vector<VisualPoint*> result;
    
    // 相机内参（这里使用默认值，实际应从外部传入）
    const double fx = 615.0;
    const double fy = 615.0;
    const double cx = 320.0;
    const double cy = 240.0;
    
    for (const auto& pair : voxel_map_) {
        for (auto pt : pair.second->visual_points) {
            if (pt == nullptr || pt->is_outlier_) continue;
            
            // 世界坐标 -> 相机坐标
            V3D pt_camera = camera_R * pt->pos_ + camera_t;
            
            // 深度检查
            if (pt_camera.z() <= 0.01) continue;
            
            // 相机坐标 -> 像素坐标
            int u = static_cast<int>(fx * pt_camera.x() / pt_camera.z() + cx);
            int v = static_cast<int>(fy * pt_camera.y() / pt_camera.z() + cy);
            
            // 检查是否在检测框内
            if (bbox.contains(u, v)) {
                result.push_back(pt);
            }
        }
    }
    
    return result;
}

std::vector<VisualPoint*> VoxelMapManager::getPointsInFrustum(
    const M3D& camera_R,
    const V3D& camera_t,
    int img_width,
    int img_height,
    double min_depth,
    double max_depth) const
{
    std::vector<VisualPoint*> result;
    
    // 相机内参
    const double fx = 615.0;
    const double fy = 615.0;
    const double cx = img_width / 2.0;
    const double cy = img_height / 2.0;
    
    for (const auto& pair : voxel_map_) {
        for (auto pt : pair.second->visual_points) {
            if (pt == nullptr || pt->is_outlier_) continue;
            
            // 世界坐标 -> 相机坐标
            V3D pt_camera = camera_R * pt->pos_ + camera_t;
            
            // 深度范围检查
            if (pt_camera.z() < min_depth || pt_camera.z() > max_depth) continue;
            
            // 相机坐标 -> 像素坐标
            int u = static_cast<int>(fx * pt_camera.x() / pt_camera.z() + cx);
            int v = static_cast<int>(fy * pt_camera.y() / pt_camera.z() + cy);
            
            // 视野范围检查
            if (u >= 0 && u < img_width && v >= 0 && v < img_height) {
                result.push_back(pt);
            }
        }
    }
    
    return result;
}

size_t VoxelMapManager::getTotalPoints() const
{
    size_t total = 0;
    for (const auto& pair : voxel_map_) {
        total += pair.second->visual_points.size();
    }
    return total;
}

void VoxelMapManager::getBoundingBox(V3D& min_bound, V3D& max_bound) const
{
    if (voxel_map_.empty()) {
        min_bound = V3D::Zero();
        max_bound = V3D::Zero();
        return;
    }
    
    min_bound = V3D(1e10, 1e10, 1e10);
    max_bound = V3D(-1e10, -1e10, -1e10);
    
    for (const auto& pair : voxel_map_) {
        for (auto pt : pair.second->visual_points) {
            min_bound.x() = std::min(min_bound.x(), pt->pos_.x());
            min_bound.y() = std::min(min_bound.y(), pt->pos_.y());
            min_bound.z() = std::min(min_bound.z(), pt->pos_.z());
            
            max_bound.x() = std::max(max_bound.x(), pt->pos_.x());
            max_bound.y() = std::max(max_bound.y(), pt->pos_.y());
            max_bound.z() = std::max(max_bound.z(), pt->pos_.z());
        }
    }
}

void VoxelMapManager::removeOutliers(float confidence_threshold)
{
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ) {
        auto& points = iter->second->visual_points;
        
        for (auto it = points.begin(); it != points.end(); ) {
            if ((*it)->confidence_ < confidence_threshold || (*it)->is_outlier_) {
                delete *it;
                it = points.erase(it);
            } else {
                ++it;
            }
        }
        
        iter->second->count = points.size();
        
        // 如果体素为空，删除
        if (points.empty()) {
            delete iter->second;
            iter = voxel_map_.erase(iter);
        } else {
            ++iter;
        }
    }
}

void VoxelMapManager::removeUnderObservedPoints(int min_observations)
{
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ) {
        auto& points = iter->second->visual_points;
        
        for (auto it = points.begin(); it != points.end(); ) {
            if ((*it)->num_observations_ < min_observations) {
                delete *it;
                it = points.erase(it);
            } else {
                ++it;
            }
        }
        
        iter->second->count = points.size();
        
        if (points.empty()) {
            delete iter->second;
            iter = voxel_map_.erase(iter);
        } else {
            ++iter;
        }
    }
}

void VoxelMapManager::downsample(double merge_threshold)
{
    // 简化版本：暂不实现复杂的下采样
    // 完整版本需要合并距离很近的点
}

bool VoxelMapManager::saveToFile(const std::string& filename, bool save_color) const
{
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        return false;
    }
    
    size_t total_points = getTotalPoints();
    
    // PLY文件头
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << total_points << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    
    if (save_color) {
        ofs << "property uchar red\n";
        ofs << "property uchar green\n";
        ofs << "property uchar blue\n";
    }
    
    ofs << "end_header\n";
    
    // 点云数据
    for (const auto& pair : voxel_map_) {
        for (auto pt : pair.second->visual_points) {
            ofs << pt->pos_.x() << " " << pt->pos_.y() << " " << pt->pos_.z();
            
            if (save_color && pt->has_color_) {
                ofs << " " << static_cast<int>(pt->color_.x())
                    << " " << static_cast<int>(pt->color_.y())
                    << " " << static_cast<int>(pt->color_.z());
            } else if (save_color) {
                ofs << " 128 128 128";  // 默认灰色
            }
            
            ofs << "\n";
        }
    }
    
    ofs.close();
    return true;
}

bool VoxelMapManager::loadFromFile(const std::string& filename)
{
    // 暂不实现加载功能
    return false;
}

void VoxelMapManager::exportPointCloud(
    std::vector<V3D>& points,
    std::vector<V3D>* colors,
    std::vector<V3D>* normals) const
{
    points.clear();
    if (colors) colors->clear();
    if (normals) normals->clear();
    
    for (const auto& pair : voxel_map_) {
        for (auto pt : pair.second->visual_points) {
            points.push_back(pt->pos_);
            
            if (colors && pt->has_color_) {
                colors->push_back(pt->color_);
            } else if (colors) {
                colors->push_back(V3D(128, 128, 128));
            }
            
            if (normals && pt->is_normal_initialized_) {
                normals->push_back(pt->normal_);
            } else if (normals) {
                normals->push_back(V3D(0, 0, 1));
            }
        }
    }
}

std::vector<VOXEL_LOCATION> VoxelMapManager::getNeighborVoxels(
    const VOXEL_LOCATION& center, 
    int range) const
{
    std::vector<VOXEL_LOCATION> neighbors;
    neighbors.reserve((2*range+1) * (2*range+1) * (2*range+1));
    
    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            for (int dz = -range; dz <= range; ++dz) {
                neighbors.emplace_back(
                    center.x + dx,
                    center.y + dy,
                    center.z + dz
                );
            }
        }
    }
    
    return neighbors;
}

