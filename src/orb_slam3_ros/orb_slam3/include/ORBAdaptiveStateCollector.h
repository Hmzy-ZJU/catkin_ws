/**
 * ORBAdaptiveStateCollector.h
 * ORB-SLAM3 adapter for the portable AdaptiveIDVO policy core.
 */
#ifndef ORB_ADAPTIVE_STATE_COLLECTOR_H
#define ORB_ADAPTIVE_STATE_COLLECTOR_H

#include "AdaptiveIDVO.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>

namespace ORB_SLAM3
{

class Frame;

class ORBAdaptiveStateCollector
{
public:
    static AdaptiveState Collect(
        Frame& frame,
        const cv::Mat& image,
        const std::vector<int>& fim_indices,
        int candidate_count,
        int selected_count,
        int inlier_count,
        const Eigen::Matrix<double,6,6>& H_ref,
        bool has_ref,
        double cumulative_degradation,
        double lambda,
        double tracking_time_ms,
        double recent_local_ba_time_ms,
        int number_of_keyframes,
        int number_of_map_points,
        int keyframe_interval);

    static void UpdateFIMMetrics(
        AdaptiveState& state,
        const Eigen::Matrix<double,6,6>& H,
        const Eigen::Matrix<double,6,6>& H_ref,
        bool has_ref,
        double lambda);

    static double SafeLogDetNatural(
        const Eigen::Matrix<double,6,6>& H,
        double lambda);
};

} // namespace ORB_SLAM3

#endif // ORB_ADAPTIVE_STATE_COLLECTOR_H
