/*
 * Target Reconstruction - Main Entry Point
 * 
 * ROS节点主程序 - 实时发布目标重建点云
 * 
 * 架构设计：生产者-消费者模式
 * - 回调函数（生产者）：轻量级数据接收 + 缓存到队列
 * - 处理线程（消费者）：从队列取数据 + 执行重建计算
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <XmlRpcValue.h>
#include <mutex>
#include <thread>
#include <deque>
#include <condition_variable>
#include <vikit/pinhole_camera.h>  // 相机模型
#include "target_reconstructor.h"  // 包含 frame.h

class TargetReconstructionNode
{
public:
    TargetReconstructionNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
        : nh_(nh), nh_private_(nh_private), reconstructor_(nullptr), cam_(nullptr),
          got_first_data_(false), got_first_mask_(false), 
          processing_thread_running_(true), max_queue_size_(10)
    {
        // 加载配置参数
        ReconstructionConfig config;
        
        // 图像配置
        nh_private_.param("image/width", config.image_width, 640);
        nh_private_.param("image/height", config.image_height, 480);
        nh_private_.param("image/grid_size", config.grid_size, 40);
        nh_private_.param("image/border", config.border, 40);
        nh_private_.param("image/sampling_step_inside_mask", config.sampling_step_inside_mask, 1);
        nh_private_.param("image/sampling_step_outside_mask", config.sampling_step_outside_mask, 5);
        
        // 深度配置
        nh_private_.param("depth/min_depth", config.min_depth, 0.1);
        nh_private_.param("depth/max_depth", config.max_depth, 5.0);
        
        // 质量控制
        nh_private_.param("quality/min_observations", config.min_observations, 3);
        nh_private_.param("quality/min_confidence", config.min_confidence, 0.3);
        
        // 地图管理
        nh_private_.param("map/voxel_size", config.voxel_size, 0.05);
        nh_private_.param("map/enable_color", config.enable_color, true);
        nh_private_.param("map/enable_tsdf", config.enable_tsdf, false);
        
        // 发布频率
        nh_private_.param("publishing/rate", publish_rate_, 5.0);
        
        // IMU(无人机本体)到相机的外参变换
        // 从YAML读取R_i_c (3x3矩阵)
        std::vector<double> R_i_c_row0, R_i_c_row1, R_i_c_row2;
        if (nh_private_.getParam("extrinsics/R_i_c", R_i_c_row0)) {
            // 如果是扁平化的9个元素
            if (R_i_c_row0.size() == 9) {
                R_i_c_ << R_i_c_row0[0], R_i_c_row0[1], R_i_c_row0[2],
                          R_i_c_row0[3], R_i_c_row0[4], R_i_c_row0[5],
                          R_i_c_row0[6], R_i_c_row0[7], R_i_c_row0[8];
            } else {
                ROS_WARN("R_i_c format error, using default values");
                R_i_c_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;
            }
        } else {
            // 尝试读取嵌套列表 [[row0], [row1], [row2]]
            XmlRpc::XmlRpcValue R_list;
            if (nh_private_.getParam("extrinsics/R_i_c", R_list) && R_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                if (R_list.size() == 3) {
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            R_i_c_(i, j) = static_cast<double>(R_list[i][j]);
                        }
                    }
                } else {
                    ROS_WARN("R_i_c format error, using default values");
                    R_i_c_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;
                }
            } else {
                ROS_WARN("Cannot read R_i_c, using default values");
                R_i_c_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;
            }
        }
        
        // 从YAML读取t_i_c (3x1向量)
        std::vector<double> t_i_c_vec;
        if (nh_private_.getParam("extrinsics/t_i_c", t_i_c_vec) && t_i_c_vec.size() == 3) {
            t_i_c_ << t_i_c_vec[0], t_i_c_vec[1], t_i_c_vec[2];
        } else {
            ROS_WARN("Cannot read t_i_c, using default values");
            t_i_c_ << 0.1, 0.0, 0.0;
        }
        
        ROS_INFO("IMU to Camera Extrinsics:");
        ROS_INFO("  R_i_c = [%6.3f %6.3f %6.3f]", R_i_c_(0,0), R_i_c_(0,1), R_i_c_(0,2));
        ROS_INFO("          [%6.3f %6.3f %6.3f]", R_i_c_(1,0), R_i_c_(1,1), R_i_c_(1,2));
        ROS_INFO("          [%6.3f %6.3f %6.3f]", R_i_c_(2,0), R_i_c_(2,1), R_i_c_(2,2));
        ROS_INFO("  t_i_c = [%6.3f %6.3f %6.3f]", t_i_c_(0), t_i_c_(1), t_i_c_(2));
        
        // 打印配置
        ROS_INFO("==========================================================");
        ROS_INFO("           Target Reconstruction System");
        ROS_INFO("           Based on FAST-LIVO2 Framework");
        ROS_INFO("==========================================================");
        ROS_INFO("Configuration:");
        ROS_INFO("  - Image Size: %d x %d", config.image_width, config.image_height);
        ROS_INFO("  - Grid Size: %d pixels", config.grid_size);
        ROS_INFO("  - Border: %d pixels", config.border);
        ROS_INFO("  - Voxel Size: %.3f m", config.voxel_size);
        ROS_INFO("  - Depth Range: [%.2f, %.2f] m", config.min_depth, config.max_depth);
        ROS_INFO("  - Min Observations: %d", config.min_observations);
        ROS_INFO("  - Publish Rate: %.1f Hz", publish_rate_);
        
        // 创建重建器
        reconstructor_ = new TargetReconstructor(config);
        reconstructor_->initROS(nh_);
        
        // 创建相机模型（使用重建器中的相机内参）
        // PinholeCamera(width, height, scale, fx, fy, cx, cy, d0, d1, d2, d3, d4)
        cam_ = new vk::PinholeCamera(config.image_width, config.image_height, 1.0,
                                     reconstructor_->fx_, reconstructor_->fy_,
                                     reconstructor_->cx_, reconstructor_->cy_);
        ROS_INFO("Camera model initialized: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                 reconstructor_->fx_, reconstructor_->fy_, 
                 reconstructor_->cx_, reconstructor_->cy_);
        
        // 从参数服务器读取话题名称
        std::string rgb_topic, depth_topic, mask_topic, odom_topic;
        std::string cloud_global_topic, cloud_body_topic;
        std::string depth_cloud_body_topic, depth_cloud_global_topic;
        
        nh_private_.param<std::string>("topics/rgb_image", rgb_topic, "/camera/color/image_raw");
        nh_private_.param<std::string>("topics/depth_image", depth_topic, "/camera/depth/image_raw");
        nh_private_.param<std::string>("topics/person_mask", mask_topic, "/yolo/person_mask");
        nh_private_.param<std::string>("topics/odometry", odom_topic, "/camera/odom");
        nh_private_.param<std::string>("topics/reconstructed_cloud_global", cloud_global_topic, "/target_reconstruction/reconstructed_cloud_global");
        nh_private_.param<std::string>("topics/reconstructed_cloud_body", cloud_body_topic, "/target_reconstruction/reconstructed_cloud_body");
        nh_private_.param<std::string>("topics/depth_cloud_body", depth_cloud_body_topic, "/target_reconstruction/depth_cloud_body");
        nh_private_.param<std::string>("topics/depth_cloud_global", depth_cloud_global_topic, "/target_reconstruction/depth_cloud_global");
        
        // 发布器
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_global_topic, 1);
        reconstructed_cloud_body_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_body_topic, 1);
        depth_cloud_body_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(depth_cloud_body_topic, 1);
        depth_cloud_global_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(depth_cloud_global_topic, 1);
        
        // 订阅器（RGB+Depth+Odom 使用同步，Mask 单独订阅）
        rgb_sub_.subscribe(nh_, rgb_topic, 1);
        depth_sub_.subscribe(nh_, depth_topic, 1);
        odom_sub_.subscribe(nh_, odom_topic, 1);
        
        // Mask 单独订阅（因为有推理延迟，且不连续发布）
        mask_sub_ = nh_.subscribe(mask_topic, 1, 
                                   &TargetReconstructionNode::maskCallback, this);
        
        // 同步策略（只同步 RGB + Depth + Odom）
        sync_.reset(new Sync(MySyncPolicy(10), rgb_sub_, depth_sub_, odom_sub_));
        sync_->registerCallback(boost::bind(&TargetReconstructionNode::syncCallback, this, _1, _2, _3));
        
        // 定时发布点云
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                         &TargetReconstructionNode::publishCallback, this);
        
        // 读取队列大小参数
        nh_private_.param("threading/max_queue_size", max_queue_size_, 10);
        
        // 启动处理线程
        processing_thread_ = std::thread(&TargetReconstructionNode::processingThreadFunc, this);
        
        ROS_INFO("Target Reconstruction Node initialized successfully!");
        ROS_INFO("Architecture: Producer-Consumer with processing thread");
        ROS_INFO("  Max queue size: %d frames", max_queue_size_);
        ROS_INFO("==========================================================");
        ROS_INFO("Subscribed Topics:");
        ROS_INFO("  - RGB:   %s (synchronized)", rgb_topic.c_str());
        ROS_INFO("  - Depth: %s (synchronized)", depth_topic.c_str());
        ROS_INFO("  - Odom:  %s (synchronized)", odom_topic.c_str());
        ROS_INFO("  - Mask:  %s (separate, cached)", mask_topic.c_str());
        ROS_INFO("Published Topics:");
        ROS_INFO("  - Reconstructed Cloud (Global): %s (frame: map, accumulated)", cloud_global_topic.c_str());
        ROS_INFO("  - Reconstructed Cloud (Body):   %s (frame: base_link, current frame)", cloud_body_topic.c_str());
        ROS_INFO("  - Depth Cloud (Body):           %s (frame: base_link)", depth_cloud_body_topic.c_str());
        ROS_INFO("  - Depth Cloud (Global):         %s (frame: map)", depth_cloud_global_topic.c_str());
        ROS_INFO("==========================================================");
        ROS_INFO("Waiting for synchronized data...");
    }
    
    ~TargetReconstructionNode()
    {
        // 停止处理线程
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            processing_thread_running_ = false;
            queue_cv_.notify_all();
        }
        
        if (processing_thread_.joinable()) {
            ROS_INFO("Waiting for processing thread to finish...");
            processing_thread_.join();
        }
        
        if (reconstructor_ != nullptr) {
            // 打印最终统计信息
            ROS_INFO("========== Final Statistics ==========");
            ROS_INFO("Total frames received: %d", frame_count_received_);
            ROS_INFO("Total frames processed: %d", frame_count_processed_);
            ROS_INFO("Total frames dropped: %d", frame_count_dropped_);
            ROS_INFO("Total map points: %zu", reconstructor_->getMapManager()->getTotalPoints());
            ROS_INFO("=====================================");
            
            // 保存最终重建结果
            std::string output_file;
            nh_private_.param<std::string>("output/file", output_file, "target_reconstruction.ply");
            
            ROS_INFO("Saving reconstruction to: %s", output_file.c_str());
            if (reconstructor_->saveReconstruction(output_file)) {
                ROS_INFO("Reconstruction saved successfully!");
            }
            
            delete reconstructor_;
        }
        
        // 删除相机模型
        if (cam_ != nullptr) {
            delete cam_;
        }
    }
    
private:
    // ========== 回调函数（生产者）==========
    
    // Mask 回调（单独订阅，缓存最新的 mask）
    void maskCallback(const sensor_msgs::ImageConstPtr& mask_msg)
    {
        std::lock_guard<std::mutex> lock(mask_mutex_);
        latest_mask_msg_ = mask_msg;
        mask_update_time_ = ros::Time::now();
        
        if (!got_first_mask_) {
            got_first_mask_ = true;
            ROS_INFO("Received first mask from YOLO!");
        }
    }
    
    // RGB + Depth + Odom 同步回调（轻量级处理，仅存入队列）
    void syncCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                     const sensor_msgs::ImageConstPtr& depth_msg,
                     const nav_msgs::OdometryConstPtr& odom_msg)
    {
        if (!got_first_data_) {
            got_first_data_ = true;
            ROS_INFO("Received first synchronized data!");
        }
        
        frame_count_received_++;
        
        // 创建帧数据结构（使用智能指针，传入相机模型）
        FrameDataPtr frame = std::make_shared<FrameData>(cam_);
        frame->timestamp = rgb_msg->header.stamp.toSec();
        
        // 1. 转换RGB图像
            cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        frame->rgb_img = rgb_ptr->image.clone();  // 深拷贝
        
        // 2. 解析位姿并转换到相机坐标系（必须在点云转换之前）
        // Odom给的是IMU(无人机本体)坐标系的位姿
        Eigen::Quaterniond q_imu(odom_msg->pose.pose.orientation.w,
                                 odom_msg->pose.pose.orientation.x,
                                 odom_msg->pose.pose.orientation.y,
                                 odom_msg->pose.pose.orientation.z);
        Eigen::Matrix3d R_w_i = q_imu.toRotationMatrix();  // 世界系到IMU系
        Eigen::Vector3d t_w_i(odom_msg->pose.pose.position.x,
                              odom_msg->pose.pose.position.y,
                              odom_msg->pose.pose.position.z);
        
        // 设置机体位姿（IMU/Body to World）
        frame->setIMUPose(R_w_i, t_w_i);
        
        // 坐标系变换：相机到世界 (Camera to World)
        // T_c_w：从相机坐标系到世界坐标系的变换
        // R_c_w = R_w_i * R_i_c（相机到世界的旋转）
        // t_c_w = R_w_i * t_i_c + t_w_i（相机在世界坐标系下的位置）
        // 使用公式：p_world = R_c_w * p_camera + t_c_w
        Eigen::Matrix3d R_c_w = R_w_i * R_i_c_;
        Eigen::Vector3d t_c_w = R_w_i * t_i_c_ + t_w_i;
        frame->setCameraPose(R_c_w, t_c_w);
        
        // 3. 将深度图转换为点云（使用上面计算的位姿）
        int step = 3;
        frame->pg.reserve((depth_msg->height / step) * (depth_msg->width / step));
        
        for (int v = 0; v < depth_msg->height; v += step)
        {
            for (int u = 0; u < depth_msg->width; u += step)
            {
                float depth = depth_msg->data[v * depth_msg->width + u];
                
                if (depth <= reconstructor_->config_.min_depth || 
                    depth >= reconstructor_->config_.max_depth) continue;
                
                // 深度图反投影到相机坐标系（使用相机模型）
                // cam2world 返回单位方向向量（bearing vector），乘以深度得到3D点
                Eigen::Vector3d bearing = frame->c2f(u, v);  // 像素 -> 归一化平面（单位向量）
                Eigen::Vector3d point_cam = bearing * depth;  // 方向向量 * 深度 = 3D点
                
                // 转换到世界坐标系（使用FrameData的f2w方法）
                Eigen::Vector3d point_world = frame->f2w(point_cam);
                
                // 填充点云结构
                pointWithVar pt;
                pt.point_c = point_cam;                           // 相机坐标系
                pt.point_i = R_i_c_ * point_cam + t_i_c_;        // IMU坐标系
                pt.point_w = point_world;                         // 世界坐标系
                
                // 协方差（深度误差与距离成正比）
                double sigma_depth = 0.01 * depth;
                pt.var_nostate = Eigen::Matrix3d::Identity() * sigma_depth * sigma_depth;
                pt.body_var = pt.var_nostate;
                pt.var = pt.var_nostate;
                
                // 法向量（相机光轴方向）
                pt.normal = Eigen::Vector3d(0, 0, 1);
                
                frame->pg.push_back(pt);
            }
        }
        
        // 4. 从mask中提取白色像素坐标
        std::lock_guard<std::mutex> lock(mask_mutex_);
        frame->px_mask.clear();
        
        // 检查是否有 mask，且不能太旧（1秒内）
        if (latest_mask_msg_ && (ros::Time::now() - mask_update_time_).toSec() < 1.0) 
        {
            cv_bridge::CvImagePtr mask_ptr = cv_bridge::toCvCopy(
                latest_mask_msg_, sensor_msgs::image_encodings::MONO8);
            
            if (mask_ptr->image.rows == frame->rgb_img.rows && 
                mask_ptr->image.cols == frame->rgb_img.cols)
            {
                const cv::Mat& mask = mask_ptr->image;
                
                // 计算包围框
                frame->bbox = computeBoundingBoxFromMask(mask);
                
                // 只在bbox内遍历，提取白色像素
                for (int v = frame->bbox.y_min; v <= frame->bbox.y_max; v++)
                {
                    const uchar* row_ptr = mask.ptr<uchar>(v);
                    for (int u = frame->bbox.x_min; u <= frame->bbox.x_max; u++)
                    {
                        if (row_ptr[u] >= 128)
                        {
                            frame->px_mask.emplace_back(u, v);
                        }
                    }
                }
                
                if (!frame->px_mask.empty())
                {
                    frame->has_valid_mask = true;
                    ROS_DEBUG_THROTTLE(1.0, "Extracted %zu white pixels from mask", 
                                      frame->px_mask.size());
                }
            }
        }
        
        // 5. 如果没有有效的 mask，跳过该帧
        if (!frame->has_valid_mask) {
            ROS_WARN_THROTTLE(2.0, "No valid mask available, skipping frame");
            return;
        }
        
        // 6. 将帧数据加入处理队列
        std::unique_lock<std::mutex> queue_lock(queue_mutex_);
        
        // 如果队列满了，丢弃最旧的帧
        if (frame_queue_.size() >= static_cast<size_t>(max_queue_size_)) {
            frame_queue_.pop_front();
            frame_count_dropped_++;
            ROS_WARN_THROTTLE(1.0, "Frame queue full! Dropping oldest frame. Dropped: %d", 
                             frame_count_dropped_);
        }
        
        frame_queue_.push_back(frame);
        queue_cv_.notify_one();  // 通知处理线程
        
        ROS_INFO_THROTTLE(5.0, "Producer: Queue size = %zu, Received = %d, Processed = %d, Dropped = %d", 
                         frame_queue_.size(), frame_count_received_, 
                         frame_count_processed_, frame_count_dropped_);
    }
    
    // ========== 处理线程（消费者）==========
    
    void processingThreadFunc()
    {
        ROS_INFO("Processing thread started!");
        
        while (ros::ok() && processing_thread_running_) 
        {
            FrameDataPtr frame;
            
            // 从队列中取出一帧数据
            std::unique_lock<std::mutex> lock(queue_mutex_);
            
            // 等待队列非空或线程停止信号
            queue_cv_.wait(lock, [this] { 
                return !frame_queue_.empty() || !processing_thread_running_; 
            });
            
            // 检查是否需要退出
            if (!processing_thread_running_ && frame_queue_.empty()) {
                break;
            }
            
            // 取出队首数据
            if (!frame_queue_.empty()) {
                frame = frame_queue_.front();
                frame_queue_.pop_front();
            }
            
            lock.unlock();  // 手动解锁，避免在处理数据时持有锁
            
            // 处理数据（无锁，可以长时间运行）
            if (frame && frame->has_valid_mask && reconstructor_ != nullptr) {
                // 1. 处理重建
                reconstructor_->processFrameWithMask(frame);
                
                // // 2. 发布原始深度点云（机体系 + 全局系）
                // publishDepthClouds(frame);
                
                // // 3. 发布当前帧重建点云（机体系）
                // publishCurrentFrameReconstructedCloud(frame);
                
                // // 4. 更新最后一帧的位姿（用于累积地图的机体系发布）
                // pose_mutex_.lock();
                // last_R_w_i_ = frame->T_w_i_.rotation_matrix();
                // last_t_w_i_ = frame->T_w_i_.translation();
                // pose_mutex_.unlock();
                
                // frame_count_processed_++;
            }
        }
        
    ROS_INFO("Processing thread stopped!");
    }
    
    // 发布当前帧重建点云（机体坐标系）
    void publishCurrentFrameReconstructedCloud(const FrameData& frame)
    {
        if (reconstructed_cloud_body_pub_.getNumSubscribers() == 0) return;
        
        // 从重建器获取当前帧生成的点（相机坐标系）
        std::vector<V3D> points_camera, colors;
        reconstructor_->getLastFramePoints(points_camera, colors);
        
        if (points_camera.empty()) {
            return;
        }
        
        // 转换到机体坐标系: p_body = R_i_c^T * (p_camera - t_i_c)
        std::vector<Eigen::Vector3d> points_body;
        points_body.reserve(points_camera.size());
        
        Eigen::Matrix3d R_c_i = R_i_c_.transpose();
        for (const auto& p_cam : points_camera) {
            Eigen::Vector3d p_body = R_i_c_ * p_cam + t_i_c_;
            points_body.push_back(p_body);
        }
        
        // 创建点云消息
        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "base_link";
        cloud_msg.height = 1;
        cloud_msg.width = points_body.size();
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(points_body.size());
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
        
        for (size_t i = 0; i < points_body.size(); ++i) {
            *iter_x = points_body[i].x();
            *iter_y = points_body[i].y();
            *iter_z = points_body[i].z();
            *iter_r = static_cast<uint8_t>(colors[i].x());
            *iter_g = static_cast<uint8_t>(colors[i].y());
            *iter_b = static_cast<uint8_t>(colors[i].z());
            
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b;
        }
        
        reconstructed_cloud_body_pub_.publish(cloud_msg);
        
        ROS_INFO_THROTTLE(5.0, "Published current frame reconstructed cloud: %zu points (base_link frame)", 
                         points_body.size());
    }
    
    // 从 mask 计算包围框
    BoundingBox computeBoundingBoxFromMask(const cv::Mat& mask)
    {
        BoundingBox bbox;
        bbox.x_min = mask.cols;
        bbox.y_min = mask.rows;
        bbox.x_max = 0;
        bbox.y_max = 0;
        bbox.confidence = 1.0f;
        bbox.label = "person";
        
        // 遍历 mask 找到边界
        for (int y = 0; y < mask.rows; y++) {
            for (int x = 0; x < mask.cols; x++) {
                if (mask.at<uint8_t>(y, x) > 128) {  // 阈值 128，大于则认为是目标
                    if (x < bbox.x_min) bbox.x_min = x;
                    if (x > bbox.x_max) bbox.x_max = x;
                    if (y < bbox.y_min) bbox.y_min = y;
                    if (y > bbox.y_max) bbox.y_max = y;
                }
            }
        }
        
        // 确保包围框有效
        if (bbox.x_max < bbox.x_min) {
            bbox.x_min = bbox.y_min = 0;
            bbox.x_max = bbox.y_max = 0;
        }
        
        return bbox;
    }
    
    // 发布深度点云（机体坐标系和全局坐标系）- 使用已转换的点云数据
    void publishDepthClouds(const FrameDataPtr& frame)
    {
        if (!frame || frame->pg.empty()) return;
        
        // 直接使用 FrameData 中已经转换好的点云数据
        std::vector<Eigen::Vector3d> points_body;
        std::vector<Eigen::Vector3d> points_global;
        std::vector<Eigen::Vector3i> colors(frame->pg.size(), Eigen::Vector3i(255, 255, 255));
        
        points_body.reserve(frame->pg.size());
        points_global.reserve(frame->pg.size());
        
        for (const auto& pt : frame->pg)
        {
            points_body.push_back(pt.point_i);  // IMU/Body坐标系
            points_global.push_back(pt.point_w);  // 世界坐标系
            // TODO: 如果需要颜色，可以从 rgb_img 中根据像素坐标提取
        }
        
        // 1. 发布机体坐标系点云（frame_id: body）
        if (!points_body.empty() && depth_cloud_body_pub_.getNumSubscribers() > 0) 
        {
            sensor_msgs::PointCloud2 cloud_body_msg;
            cloud_body_msg.header.stamp = ros::Time::now();
            cloud_body_msg.header.frame_id = "base_link";
            cloud_body_msg.height = 1;
            cloud_body_msg.width = points_body.size();
            cloud_body_msg.is_dense = true;
            cloud_body_msg.is_bigendian = false;
            
            sensor_msgs::PointCloud2Modifier modifier(cloud_body_msg);
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
            modifier.resize(points_body.size());
            
            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_body_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_body_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_body_msg, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_body_msg, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_body_msg, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_body_msg, "b");
            
            for (size_t i = 0; i < points_body.size(); ++i) {
                *iter_x = points_body[i].x();
                *iter_y = points_body[i].y();
                *iter_z = points_body[i].z();
                *iter_r = static_cast<uint8_t>(colors[i].x());
                *iter_g = static_cast<uint8_t>(colors[i].y());
                *iter_b = static_cast<uint8_t>(colors[i].z());
                
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_r; ++iter_g; ++iter_b;
            }
            
            depth_cloud_body_pub_.publish(cloud_body_msg);
        }
        
        // 2. 发布全局坐标系点云（frame_id: map）
        if (!points_global.empty() && depth_cloud_global_pub_.getNumSubscribers() > 0) 
        {
            sensor_msgs::PointCloud2 cloud_global_msg;
            cloud_global_msg.header.stamp = ros::Time::now();
            cloud_global_msg.header.frame_id = "map";
            cloud_global_msg.height = 1;
            cloud_global_msg.width = points_global.size();
            cloud_global_msg.is_dense = true;
            cloud_global_msg.is_bigendian = false;
            
            sensor_msgs::PointCloud2Modifier modifier(cloud_global_msg);
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
            modifier.resize(points_global.size());
            
            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_global_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_global_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_global_msg, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_global_msg, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_global_msg, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_global_msg, "b");
            
            for (size_t i = 0; i < points_global.size(); ++i) {
                *iter_x = points_global[i].x();
                *iter_y = points_global[i].y();
                *iter_z = points_global[i].z();
                *iter_r = static_cast<uint8_t>(colors[i].x());
                *iter_g = static_cast<uint8_t>(colors[i].y());
                *iter_b = static_cast<uint8_t>(colors[i].z());
                
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_r; ++iter_g; ++iter_b;
            }
            
            depth_cloud_global_pub_.publish(cloud_global_msg);
            
            ROS_INFO_THROTTLE(5.0, "Published depth clouds: base_link=%zu points, global=%zu points", 
                             points_body.size(), points_global.size());
        }
    }
    
    void publishCallback(const ros::TimerEvent&)
    {
        if (!got_first_data_) return;
        
        // 从地图导出点云（全局坐标系累积地图）
        std::vector<V3D> points_global;
        std::vector<V3D> colors;
        reconstructor_->getMapManager()->exportPointCloud(points_global, &colors);
        
        if (points_global.empty()) {
            ROS_WARN_THROTTLE(10.0, "No points to publish yet");
            return;
        }
        
        // 发布全局系累积重建点云 (frame_id: map)
        if (cloud_pub_.getNumSubscribers() > 0) {
            sensor_msgs::PointCloud2 cloud_msg_global;
            cloud_msg_global.header.stamp = ros::Time::now();
            cloud_msg_global.header.frame_id = "map";
            cloud_msg_global.height = 1;
            cloud_msg_global.width = points_global.size();
            cloud_msg_global.is_dense = false;
            cloud_msg_global.is_bigendian = false;
            
            sensor_msgs::PointCloud2Modifier modifier_global(cloud_msg_global);
            modifier_global.setPointCloud2FieldsByString(2, "xyz", "rgb");
            modifier_global.resize(points_global.size());
            
            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_global, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_global, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_global, "z");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg_global, "r");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg_global, "g");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg_global, "b");
            
            for (size_t i = 0; i < points_global.size(); ++i) {
                *iter_x = points_global[i].x();
                *iter_y = points_global[i].y();
                *iter_z = points_global[i].z();
                *iter_r = static_cast<uint8_t>(colors[i].x());
                *iter_g = static_cast<uint8_t>(colors[i].y());
                *iter_b = static_cast<uint8_t>(colors[i].z());
                
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_r; ++iter_g; ++iter_b;
            }
            
            cloud_pub_.publish(cloud_msg_global);
            
            ROS_INFO_THROTTLE(5.0, "Published accumulated reconstructed cloud: %zu points (global frame)", 
                              points_global.size());
        }
    }
    
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    TargetReconstructor* reconstructor_;
    
    // 消息同步（只同步 RGB + Depth + Odom）
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, 
        sensor_msgs::Image,
        nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    boost::shared_ptr<Sync> sync_;
    
    // Mask 单独订阅（异步，缓存最新的）
    ros::Subscriber mask_sub_;
    sensor_msgs::ImageConstPtr latest_mask_msg_;
    ros::Time mask_update_time_;
    std::mutex mask_mutex_;
    
    // 发布器
    ros::Publisher cloud_pub_;                      // 全局系重建点云
    ros::Publisher reconstructed_cloud_body_pub_;   // 机体系重建点云
    ros::Publisher depth_cloud_body_pub_;           // 机体系原始深度点云
    ros::Publisher depth_cloud_global_pub_;         // 全局系原始深度点云
    ros::Timer publish_timer_;
    
    double publish_rate_;
    
    // 运行状态（注意：初始化顺序要与声明顺序一致）
    bool got_first_data_;
    bool got_first_mask_;
    
    // ========== 处理线程相关 ==========
    std::thread processing_thread_;
    bool processing_thread_running_;
    
    // 帧数据队列（线程安全）- 使用智能指针避免数据拷贝
    std::deque<FrameDataPtr> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    int max_queue_size_;
    
    // 统计信息
    int frame_count_received_ = 0;
    int frame_count_processed_ = 0;
    int frame_count_dropped_ = 0;
    
    // ========== 相机模型 ==========
    vk::AbstractCamera* cam_;  //!< 相机模型（用于坐标投影）
    
    // ========== 外参标定 ==========
    // IMU(无人机本体)到相机的外参变换
    Eigen::Matrix3d R_i_c_;  // 旋转矩阵
    Eigen::Vector3d t_i_c_;  // 平移向量
    
    // ========== 当前位姿（用于坐标转换） ==========
    Eigen::Matrix3d last_R_w_i_;  // 最后一帧的机体姿态（世界到机体）
    Eigen::Vector3d last_t_w_i_;  // 最后一帧的机体位置（世界系）
    std::mutex pose_mutex_;       // 位姿互斥锁
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_reconstruction_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    TargetReconstructionNode node(nh, nh_private);
    
    ros::spin();
    
    return 0;
}

