/**
* AdaptiveIDVO.h
* Lightweight adaptive controller for information-driven visual odometry.
*/
#ifndef ADAPTIVE_IDVO_H
#define ADAPTIVE_IDVO_H

#include <fstream>
#include <string>

namespace ORB_SLAM3
{

enum class AdaptivePolicyType
{
    Fixed = 0,
    RuleBased = 1,
    Learned = 2
};

struct AdaptiveState
{
    long unsigned int frame_id = 0;
    double timestamp = 0.0;

    int candidate_point_number = 0;
    int selected_point_number = 0;
    double selection_ratio = 0.0;

    double logdet_H = 0.0;
    double min_eigenvalue_H = 0.0;
    double condition_number_H = 0.0;
    double information_variation_delta_E = 0.0;
    double cumulative_degradation_D_t = 0.0;

    double median_reprojection_error = 0.0;
    double inlier_ratio = 0.0;
    int tracked_map_points = 0;

    double image_contrast = 0.0;
    double blur_score = 0.0;
    double feature_spatial_entropy = 0.0;

    double tracking_time_ms = 0.0;
    double recent_local_ba_time_ms = 0.0;
    int number_of_keyframes = 0;
    int number_of_map_points = 0;
    int keyframe_interval = 0;

    bool imu_ready = false;
    bool adaptive_enabled_this_frame = false;
    bool adaptive_bypassed = false;
    std::string bypass_reason;
    AdaptivePolicyType policy_type = AdaptivePolicyType::Fixed;

    double fim_time_ms = 0.0;
    double idps_time_ms = 0.0;
    double idkd_time_ms = 0.0;
    double policy_time_ms = 0.0;
    double logger_time_ms = 0.0;
    double total_adaptive_time_ms = 0.0;
};

struct AdaptiveParams
{
    int kappa_top = 400;
    double alpha = 0.25;
    double tau0 = 1.0;
    double theta_drop = 0.5;
    double keyframe_aggressiveness = 1.0;
};

struct AdaptiveConfig
{
    bool enable_adaptive_idvo = false;
    AdaptivePolicyType policy_type = AdaptivePolicyType::Fixed;
    bool enable_logging = false;
    std::string log_path = "adaptive_idvo_log.csv";

    int min_kappa_top = 120;
    int max_kappa_top = 420;
    double min_tau0 = 0.01;
    double max_tau0 = 2.0;
    double tracking_time_budget_ms = 30.0;
    double smooth_factor = 0.8;
    bool disable_before_imu_ready = true;
    int imu_ready_stable_frames = 15;

    double low_logdet_H = 0.0;
    double poor_condition_number = 1.0e6;
    double low_inlier_ratio = 0.35;
    double blur_threshold = 60.0;
};

class AdaptivePolicy
{
public:
    virtual ~AdaptivePolicy() {}
    virtual AdaptiveParams Decide(
        const AdaptiveState& state,
        const AdaptiveParams& previous_params,
        const AdaptiveParams& fixed_params,
        const AdaptiveConfig& config) = 0;
};

class FixedAdaptivePolicy : public AdaptivePolicy
{
public:
    AdaptiveParams Decide(
        const AdaptiveState& state,
        const AdaptiveParams& previous_params,
        const AdaptiveParams& fixed_params,
        const AdaptiveConfig& config) override;
};

class RuleBasedAdaptivePolicy : public AdaptivePolicy
{
public:
    AdaptiveParams Decide(
        const AdaptiveState& state,
        const AdaptiveParams& previous_params,
        const AdaptiveParams& fixed_params,
        const AdaptiveConfig& config) override;
};

class AdaptiveLogger
{
public:
    AdaptiveLogger() {}
    ~AdaptiveLogger();

    void Configure(bool enabled, const std::string& path);
    void Log(
        const AdaptiveState& state,
        const AdaptiveParams& params,
        bool keyframe_inserted,
        bool tracking_lost);

private:
    bool mEnabled = false;
    bool mHeaderWritten = false;
    std::string mPath;
    std::ofstream mStream;
};

AdaptivePolicyType ParseAdaptivePolicyType(const std::string& name);
const char* AdaptivePolicyTypeName(AdaptivePolicyType type);
AdaptiveParams SmoothAdaptiveParams(
    const AdaptiveParams& target,
    const AdaptiveParams& previous,
    const AdaptiveConfig& config);
void ClampAdaptiveParams(AdaptiveParams& params, const AdaptiveConfig& config);

} // namespace ORB_SLAM3

#endif // ADAPTIVE_IDVO_H
