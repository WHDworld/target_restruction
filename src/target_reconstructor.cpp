/*
 * Target Reconstruction - TargetReconstructor Implementation
 */

#include "target_reconstructor.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <vikit/vision.h>
#include <cmath>
#include <algorithm>
#include <numeric>

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
    
    current_timestamp_ = 0.0;
    
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
    
    // 发布重建点云
    static ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
        "/target_reconstruction/pointcloud", 1);
}


void TargetReconstructor::processFrameWithMask(const FrameDataPtr& new_frame)
{
    // 保存当前帧指针（用于后续 retrieveFromVisualSparseMap 访问）
    new_frame_ = new_frame;
    
    // 检查 mask 有效性
    if (!new_frame_->has_valid_mask) 
    {
        ROS_WARN("Invalid mask, falling back to bbox-only processing");
        return;
    }
    
    // 更新当前帧信息
    frame_count_++;
    current_rgb_ = new_frame_->rgb_img.clone();
    current_pg_ = new_frame_->pg;
    current_px_mask_ = new_frame_->px_mask;
    current_timestamp_ = new_frame_->timestamp;
    current_R_ = new_frame_->T_w_c_.rotation_matrix();
    current_t_ = new_frame_->T_w_c_.translation();
    // 转换为灰度图
    Mat gray_img;
    if (current_rgb_.channels() == 3) {
        cv::cvtColor(current_rgb_, gray_img, cv::COLOR_BGR2GRAY);
    } else {
        gray_img = current_rgb_.clone();
    }

    ROS_INFO("========== Processing Frame %d (with Mask) ==========", frame_count_);
    
    ROS_INFO("========== Processing Frame %d ==========", frame_count_);
    // Step 1: 检索可见的历史点 
    retrieveFromVisualSparseMap(current_pg_, visible_voxel_points);
    // Step 2: 生成新的视觉点
    ROS_INFO("Step 1: Generating new visual points...");
    generateVisualPoints(gray_img, current_pg_);
    ROS_INFO("  Found %zu visible points", visible_voxel_points.size());
    
    // Step 3: 更新已观测的点（可选：使用 mask 过滤）
    updateVisualPoints(gray_img);
    
    // TODO: updateReferencePatch 需要激光雷达的平面地图
    // 对于 RGB-D，可以使用深度图计算法向量，暂时注释掉
    // updateReferencePatch(map_manager_->voxel_map_);
    // 打印统计信息
    size_t total_pts = map_manager_->getTotalPoints();
    ROS_INFO("Frame %d summary: Total points = %zu, Created = %d, Observations = %d",
             frame_count_, total_pts, total_points_created_, total_observations_);
    ROS_INFO("==========================================\n");
}

void TargetReconstructor::generateVisualPoints(const Mat& gray_img, std::vector<pointWithVar> &pg)
{
    if(pg.size() <= 5) return;
    
    int skip_count = 0;
    int in_frame_count = 0;
    int type_map_blocked = 0;
    int added_to_grid = 0;

    for(int i = 0; i < pg.size(); i++)
    {
        if(pg[i].normal == V3D(0, 0, 0))
        {
            skip_count++;
            continue;
        }
        Eigen::Vector3d pt = pg[i].point_w;
        Eigen::Vector2d pc(new_frame_->w2c(pt));

        if(new_frame_->cam_->isInFrame(pc.cast<int>(), config_.border))
        {
            in_frame_count++;
            int index = static_cast<int>(pc[1] / config_.grid_size) * grid_n_width_ + static_cast<int>(pc[0] / config_.grid_size);

            if(grid_num[index] != 1)
            {
                float cur_value = vk::shiTomasiScore(gray_img, pc[0], pc[1]);

                if(cur_value > grid_scores_[index])
                {
                    grid_scores_[index] = cur_value;
                    append_voxel_points[index] = pg[i];
                    grid_num[index] = 2;
                    added_to_grid++;
                }
            }
            else
            {
                type_map_blocked++;
            }
        }
    }

    printf("[ Target Reconstructor ] Found %d in-frame points, %d added to grid, %d type_map_blocked\n", in_frame_count, added_to_grid, type_map_blocked);

    int add = 0;
    for(int i = 0; i < length; i++)
    {
        if(grid_num[i] == 2)
        {
            pointWithVar pt_var = append_voxel_points[i];

            V3D pt = pt_var.point_w;

            V2D pc(new_frame_->w2c(pt));
            float *patch = new float[config_.patch_size_total];
            getImagePatch(gray_img, pc, patch, 0);

            VisualPoint *pt_new = new VisualPoint(pt);
            pt_new->covariance_ = pt_var.var;
            pt_new->is_normal_initialized_ = true;

            Vector3d f = new_frame_->cam_->cam2world(pc); // 像素 → 相机坐标系射线
            Feature *ftr_new = new Feature(pt_new, patch, pc, f, new_frame_->T_w_c_.inverse(), 0);
            ftr_new->id_ = new_frame_->id_;
            ftr_new->img_ = gray_img;
            pt_new->addObservation(ftr_new);
            pt_new->normal_ = pt_var.normal;
            pt_new->previous_normal_ = pt_new->normal_;
            map_manager_->insertPoint(pt_new);
            add++;
        }
    }
}

void TargetReconstructor::updateVisualPoints(const Mat& gray_img)
{
    if(total_points_ == 0) return;

  int update_num = 0;
  SE3 pose_cur = new_frame_->T_w_c_.inverse();
  for (int i = 0; i < total_points_; i++)
  {
    VisualPoint *pt = visible_voxel_points[i];
    if (pt == nullptr) continue;
    if (pt->is_converged_) // 收敛点处理：<br>• 如果点已收敛（优化完成）
    { 
      pt->deleteNonRefPatchFeatures();
      continue;
    }

    V2D pc(new_frame_->w2c(pt->pos_));
    bool add_flag = false;
    
    float *patch_temp = new float[config_.patch_size_total];
    getImagePatch(gray_img, pc, patch_temp, 0);
    // TODO: condition: distance and view_angle
    // Step 1: time
    Feature *last_feature = pt->obs_.back();
    // if(new_frame_->id_ >= last_feature->id_ + 10) add_flag = true; // 10

    // 判断条件 2：位姿变化
    SE3 pose_ref = last_feature->T_f_w_;
    SE3 delta_pose = pose_ref * pose_cur.inverse();
    double delta_p = delta_pose.translation().norm();
    double delta_theta = (delta_pose.rotation_matrix().trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (delta_pose.rotation_matrix().trace() - 1));
    if (delta_p > 0.5 || delta_theta > 0.3) add_flag = true; // 0.5 || 0.3

    // 判断条件 3: 像素距离
    Vector2d last_px = last_feature->px_;
    double pixel_dist = (pc - last_px).norm();
    if (pixel_dist > 40) add_flag = true;

    // Maintain the size of 3D point observation features.
    if (pt->obs_.size() >= 30)
    {
      Feature *ref_ftr;
      pt->findMinScoreFeature(new_frame_->imu_pos(), ref_ftr);
      pt->deleteObservation(ref_ftr);
      // cout<<"pt->obs_.size() exceed 20 !!!!!!"<<endl;
    }
    if (add_flag)
    {
      update_num += 1;
      update_flag[i] = 1;
      Vector3d f = new_frame_->cam_->cam2world(pc);
      Feature *ftr_new = new Feature(pt, patch_temp, pc, f, new_frame_->T_w_c_.inverse(), search_levels[i]);
      ftr_new->img_ = gray_img;
      ftr_new->id_ = new_frame_->id_;
      pt->addObservation(ftr_new);
    }
  }
}

void TargetReconstructor::getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level)
{
  const float u_ref = pc[0];
  const float v_ref = pc[1];
  const int scale = (1 << level);
  const int u_ref_i = floorf(pc[0] / scale) * scale;
  const int v_ref_i = floorf(pc[1] / scale) * scale;
  const float subpix_u_ref = (u_ref - u_ref_i) / scale;
  const float subpix_v_ref = (v_ref - v_ref_i) / scale;
  const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
  const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
  const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
  const float w_ref_br = subpix_u_ref * subpix_v_ref;
  for (int x = 0; x < config_.patch_size; x++)
  {
    uint8_t *img_ptr = (uint8_t *)img.data + (v_ref_i - config_.patch_size_half * scale + x * scale) * config_.image_width + (u_ref_i - config_.patch_size_half * scale);
    for (int y = 0; y < config_.patch_size; y++, img_ptr += scale)
    {
      patch_tmp[config_.patch_size_total * level + x * config_.patch_size + y] =
          w_ref_tl * img_ptr[0] + w_ref_tr * img_ptr[scale] + w_ref_bl * img_ptr[scale * config_.image_width] + w_ref_br * img_ptr[scale * config_.image_width + scale];
    }
  }
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

void TargetReconstructor::updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map)
{
  if (total_points_ == 0) return;
    float voxel_size = 0.1;
  for (int i = 0; i < visible_voxel_points.size(); i++)
  {
    VisualPoint *pt = visible_voxel_points[i];

    if (!pt->is_normal_initialized_) continue;
    if (pt->is_converged_) continue;
    if (pt->obs_.size() <= 5) continue;
    if (update_flag[i] == 0) continue;

    const V3D &p_w = pt->pos_;
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = plane_map.find(position);
    if (iter != plane_map.end())
    {
      VoxelOctoTree *current_octo;
      current_octo = iter->second->find_correspond(p_w);
      if (current_octo->plane_ptr_->is_plane_)
      {
        VoxelPlane &plane = *current_octo->plane_ptr_;
        float dis_to_plane = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
        float dis_to_plane_abs = fabs(dis_to_plane);
        float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) +
                              (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) + (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);
        if (range_dis <= 3 * plane.radius_)
        {
          Eigen::Matrix<double, 1, 6> J_nq;
          J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
          J_nq.block<1, 3>(0, 3) = -plane.normal_;
          double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
          sigma_l += plane.normal_.transpose() * pt->covariance_ * plane.normal_;

          if (dis_to_plane_abs < 3 * sqrt(sigma_l))
          {
            // V3D norm_vec(new_frame_->T_f_w_.rotation_matrix() * plane.normal_);
            // V3D pf(new_frame_->T_f_w_ * pt->pos_);
            // V3D pf_ref(pt->ref_patch->T_f_w_ * pt->pos_);
            // V3D norm_vec_ref(pt->ref_patch->T_f_w_.rotation_matrix() *
            // plane.normal); double cos_ref = pf_ref.dot(norm_vec_ref);
            
            if (pt->previous_normal_.dot(plane.normal_) < 0) { pt->normal_ = -plane.normal_; }
            else { pt->normal_ = plane.normal_; }

            double normal_update = (pt->normal_ - pt->previous_normal_).norm();

            pt->previous_normal_ = pt->normal_;

            if (normal_update < 0.0001 && pt->obs_.size() > 10)
            {
              pt->is_converged_ = true;
              // visual_converged_point.push_back(pt);
            }
          }
        }
      }
    }

    float score_max = -1000.;
    for (auto it = pt->obs_.begin(), ite = pt->obs_.end(); it != ite; ++it)
    {
      Feature *ref_patch_temp = *it;
      float *patch_temp = ref_patch_temp->patch_;
      float NCC_up = 0.0;
      float NCC_down1 = 0.0;
      float NCC_down2 = 0.0;
      float NCC = 0.0;
      float score = 0.0;
      int count = 0;

      V3D pf = ref_patch_temp->T_f_w_ * pt->pos_;
      V3D norm_vec = ref_patch_temp->T_f_w_.rotation_matrix() * pt->normal_;
      pf.normalize();
      double cos_angle = pf.dot(norm_vec);
      // if(fabs(cos_angle) < 0.86) continue; // 20 degree

      float ref_mean;
      if (abs(ref_patch_temp->mean_) < 1e-6)
      {
        float ref_sum = std::accumulate(patch_temp, patch_temp + config_.patch_size_total, 0.0);
        ref_mean = ref_sum / config_.patch_size_total;
        ref_patch_temp->mean_ = ref_mean;
      }

      for (auto itm = pt->obs_.begin(), itme = pt->obs_.end(); itm != itme; ++itm)
      {
        if ((*itm)->id_ == ref_patch_temp->id_) continue;
        float *patch_cache = (*itm)->patch_;

        float other_mean;
        if (abs((*itm)->mean_) < 1e-6)
        {
          float other_sum = std::accumulate(patch_cache, patch_cache + config_.patch_size_total, 0.0);
          other_mean = other_sum / config_.patch_size_total;
          (*itm)->mean_ = other_mean;
        }

        for (int ind = 0; ind < config_.patch_size_total; ind++)
        {
          NCC_up += (patch_temp[ind] - ref_mean) * (patch_cache[ind] - other_mean);
          NCC_down1 += (patch_temp[ind] - ref_mean) * (patch_temp[ind] - ref_mean);
          NCC_down2 += (patch_cache[ind] - other_mean) * (patch_cache[ind] - other_mean);
        }
        NCC += fabs(NCC_up / sqrt(NCC_down1 * NCC_down2));
        count++;
      }

      NCC = NCC / count;

      score = NCC + cos_angle;

      ref_patch_temp->score_ = score;

      if (score > score_max)
      {
        score_max = score;
        pt->ref_patch_ = ref_patch_temp;
        pt->has_ref_patch_ = true;
      }
    }

  }
}

void TargetReconstructor::retrieveFromVisualSparseMap(std::vector<pointWithVar> &pg, std::vector<VisualPoint*>& visible_points)
{
    if(pg.size() == 0) return;

    cv::Mat depth_img = cv::Mat::zeros(config_.image_height, config_.image_width, CV_32FC1);
    float *it = (float *)depth_img.data;

    float voxel_size = 0.1;
    sub_feat_map.clear();
    visible_voxel_points.clear();

    int loc_xyz[3];
    // ===== 步骤1: 标记LiDAR覆盖体素（同上）=====
    for(int i = 0; i < pg.size(); i++)
    {
        V3D pt_w = pg[i].point_w;

        for(int j = 0; j < 3; j++)
        {
            loc_xyz[j] = floor(pt_w[j] / voxel_size);
            if(loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
        }

        VOXEL_LOCATION position(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
        sub_feat_map[position] = 0;

        V3D pt_cam = new_frame_->w2f(pt_w);
        if(pt_cam[2] > 0)
        {
            V2D px = new_frame_->cam_->world2cam(pt_cam);
            if(new_frame_->cam_->isInFrame(px.cast<int>(), config_.border))
            {
                float depth = pt_cam[2];
                int col = static_cast<int>(px[0]);
                int row = static_cast<int>(px[1]);
                it[config_.image_width * row + col] = depth;
            }
        }
    }
    // ===== 步骤2: 检索并进行网格竞争 =====
    for(auto& iter : sub_feat_map)
    {
        VOXEL_LOCATION position = iter.first;
        auto corre_voxel = map_manager_->voxel_map_.find(position);
        if(corre_voxel == map_manager_->voxel_map_.end()) continue;

        bool voxel_in_fov = false;
        std::vector<VisualPoint*> &voxel_points = corre_voxel->second->visual_points;
        int voxel_num = voxel_points.size();
        for(int i = 0; i < voxel_num; i++)
        {
            VisualPoint* pt = voxel_points[i];

            if(pt == nullptr) continue;
            if(pt->obs_.size() == 0) continue;

            V3D pt_cam = new_frame_->w2f(pt->pos_);
            if(pt_cam[2] <= 0) continue;

            V2D pc = new_frame_->w2c(pt_cam);
            if(new_frame_->cam_->isInFrame(pc.cast<int>(), config_.border))
            {
                int index = static_cast<int>(pc[1] / config_.grid_size) * grid_n_width_ + static_cast<int>(pc[0] / config_.grid_size);
                grid_num[index] = 1;

                V3D obs_vec(new_frame_->imu_pos() - pt->pos_);
                if( obs_vec.norm() <= map_dist[index])
                {
                    map_dist[index] = obs_vec.norm();
                    retrieve_voxel_points[index] = pt;
                }
            }
        }
    }
    
    // ===== 步骤3: 收集结果 =====
    visible_points.clear();
    for (int i = 0; i < length; i++) 
    {
        if (grid_num[i] == 1 && retrieve_voxel_points[i] != nullptr) 
        {
            VisualPoint* pt = retrieve_voxel_points[i];

            V3D pt_cam =new_frame_->w2f(pt->pos_);
            V2D pc = new_frame_->w2c(pt->pos_);
            bool depth_continous = false;

            for(int u = -2; u <= 2; u++)
            {
                for(int v = -2; v <= 2; v++)
                {
                    if(u == 0 && v == 0) continue;

                    float depth = it[config_.image_width * (int(pc[1]) + v) + (int(pc[0]) + u)];
                    if(depth == 0.) continue;

                    double delta_dist = abs(pt_cam[2] - depth);
                    if(delta_dist > 0.1)
                    {
                        depth_continous = true;
                        break;
                    }
                }
                if(depth_continous) break;
            }
            if(depth_continous)
            {
                visible_points.push_back(retrieve_voxel_points[i]);
            }
        }
    }
    total_points_ = visible_points.size();
}
