/*
 * Target Reconstruction - Feature
 * 
 * 单帧观测的视觉特征
 * 存储图像Patch、相机位姿、时间戳等信息
 */

#ifndef TARGET_RECON_FEATURE_H_
#define TARGET_RECON_FEATURE_H_

#include "common_lib.h"
#include <boost/noncopyable.hpp>

// 前向声明
class VisualPoint;

/**
 * @brief 单帧观测的Feature
 * 
 * 相比FAST-LIVO2的简化：
 * - 移除曝光时间估计（假设相机参数稳定）
 * - 移除IMU相关的位姿信息（使用外部位姿）
 * - 增加深度信息（来自深度图）
 */
class Feature : boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum FeatureType
  {
    CORNER,
    EDGELET
  };
  int id_;
  FeatureType type_;     //!< Type can be corner or edgelet.
  cv::Mat img_;          //!< Image associated with the patch feature
  Vector2d px_;          // 像素坐标
  Vector3d f_;           //!< Unit-bearing vector of the patch feature.
  int level_;            //!< Image pyramid level where patch feature was extracted.
  VisualPoint *point_;   // 关联的3D点
  Vector2d grad_;        //!< Dominant gradient direction for edglets, normalized.
  SE3 T_f_w_;            // 观测时的位姿
  float *patch_;         // 图像patch (9x9像素)
  float score_;          //!< Score of the patch feature.
  float mean_;           //!< Mean intensity of the image patch feature, used for normalization.
  double inv_expo_time_; // 逆曝光时间
    
    // ========== 构造函数 ==========
    Feature(VisualPoint* point, float* patch, const V2D& px, 
            const V3D& f, const SE3& _T_f_w, int level = 0) :
            id_(-1), px_(px), f_(f), level_(level), point_(point),
            T_f_w_(_T_f_w), patch_(patch), score_(0.0), mean_(0.0)
        {

        }
    ~Feature()
    {
        delete[] patch_;
    }
    
    // ========== 辅助函数 ==========
    /**
     * @brief 获取Feature在世界坐标系中的位置
     * @return 3D世界坐标（如果深度有效）
     */
    V3D pos() const {return T_f_w_.inverse().translation();};
    
};

#endif // TARGET_RECON_FEATURE_H_

