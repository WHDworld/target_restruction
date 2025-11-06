/*
 * Target Reconstruction - VisualPoint Implementation
 */

#include "visual_point.h"
#include <algorithm>
#include <cmath>
#include <limits>

VisualPoint::VisualPoint(const Vector3d &pos)
    : pos_(pos), previous_normal_(Vector3d::Zero()), normal_(Vector3d::Zero()),
      is_converged_(false), is_normal_initialized_(false), has_ref_patch_(false)
{
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
    
    obs_.push_front(ftr);
    num_observations_++;
        
}

void VisualPoint::deleteObservation(Feature* ftr)
{
    if (ftr == nullptr) return;
    
    // 如果是参考Patch，清除标记
    if (ftr == ref_patch_) {
        ref_patch_ = nullptr;
        has_ref_patch_ = false;
    }

    for(auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
    {
        if((*it) == ftr)
        {
            delete(*it);
            obs_.erase(it);
            return;
        }
    }
}

void VisualPoint::findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const
{
    auto min_it = obs_.begin();
    float min_score = std::numeric_limits<float>::max();

    for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
    {
        if ((*it)->score_ < min_score)
        {
            min_score = (*it)->score_;
            min_it = it;
        }
    }
    ftr = *min_it;
}

void VisualPoint::deleteNonRefPatchFeatures()
{
    for (auto it = obs_.begin(); it != obs_.end();)
    {
        if (*it != ref_patch_)  // 注意：你的代码使用 ref_patch_ 而非 ref_patch
        {
            delete *it;
            it = obs_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}


