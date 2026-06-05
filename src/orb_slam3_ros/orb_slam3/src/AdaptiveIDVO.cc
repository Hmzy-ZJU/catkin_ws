/**
* AdaptiveIDVO.cc
* Lightweight adaptive controller for information-driven visual odometry.
*/

#include "AdaptiveIDVO.h"

#include <algorithm>
#include <cmath>
#include <iomanip>

namespace ORB_SLAM3
{

namespace
{

static double ClampDouble(double value, double lo, double hi)
{
    return std::max(lo, std::min(hi, value));
}

static int ClampInt(int value, int lo, int hi)
{
    return std::max(lo, std::min(hi, value));
}

} // namespace

AdaptivePolicyType ParseAdaptivePolicyType(const std::string& name)
{
    if(name == "RuleBased" || name == "rulebased" || name == "Rule")
        return AdaptivePolicyType::RuleBased;
    if(name == "Learned" || name == "learned")
        return AdaptivePolicyType::Learned;
    return AdaptivePolicyType::Fixed;
}

const char* AdaptivePolicyTypeName(AdaptivePolicyType type)
{
    switch(type)
    {
        case AdaptivePolicyType::RuleBased:
            return "RuleBased";
        case AdaptivePolicyType::Learned:
            return "Learned";
        case AdaptivePolicyType::Fixed:
        default:
            return "Fixed";
    }
}

AdaptiveParams FixedAdaptivePolicy::Decide(
    const AdaptiveState&,
    const AdaptiveParams&,
    const AdaptiveParams& fixed_params,
    const AdaptiveConfig& config)
{
    AdaptiveParams out = fixed_params;
    ClampAdaptiveParams(out, config);
    return out;
}

AdaptiveParams RuleBasedAdaptivePolicy::Decide(
    const AdaptiveState& state,
    const AdaptiveParams& previous_params,
    const AdaptiveParams& fixed_params,
    const AdaptiveConfig& config)
{
    AdaptiveParams target = fixed_params;

    double risk = 0.0;
    if(state.condition_number_H > config.poor_condition_number)
        risk += 0.25;
    if(state.logdet_H < config.low_logdet_H)
        risk += 0.20;
    if(state.inlier_ratio > 0.0 && state.inlier_ratio < config.low_inlier_ratio)
        risk += 0.20;
    if(state.blur_score > 0.0 && state.blur_score < config.blur_threshold)
        risk += 0.15;
    if(state.min_eigenvalue_H <= 1.0e-6)
        risk += 0.10;
    if(state.tracked_map_points > 0 && state.tracked_map_points < fixed_params.kappa_top / 2)
        risk += 0.10;
    risk = ClampDouble(risk, 0.0, 1.0);

    double overload = 0.0;
    if(config.tracking_time_budget_ms > 1.0 &&
       state.tracking_time_ms > config.tracking_time_budget_ms)
    {
        overload = ClampDouble(
            (state.tracking_time_ms - config.tracking_time_budget_ms) /
            config.tracking_time_budget_ms,
            0.0,
            1.0);
    }

    const bool strong_information =
        state.logdet_H >= config.low_logdet_H &&
        state.condition_number_H < config.poor_condition_number * 0.5 &&
        state.inlier_ratio >= std::max(0.55, config.low_inlier_ratio);

    if(risk > 0.0)
    {
        target.kappa_top = fixed_params.kappa_top +
            static_cast<int>(risk * (config.max_kappa_top - fixed_params.kappa_top));
        target.tau0 = fixed_params.tau0 -
            risk * (fixed_params.tau0 - config.min_tau0);
        target.theta_drop = fixed_params.theta_drop *
            ClampDouble(1.0 - 0.35 * risk, 0.5, 1.0);
        target.alpha = ClampDouble(fixed_params.alpha + 0.25 * risk, 0.0, 1.0);
        target.keyframe_aggressiveness = 1.0 + 0.5 * risk;
    }

    if(strong_information && overload > 0.0)
    {
        target.kappa_top = fixed_params.kappa_top -
            static_cast<int>(overload * (fixed_params.kappa_top - config.min_kappa_top));
        target.tau0 = fixed_params.tau0 +
            overload * (config.max_tau0 - fixed_params.tau0);
        target.theta_drop = fixed_params.theta_drop *
            ClampDouble(1.0 + 0.35 * overload, 1.0, 1.5);
        target.alpha = ClampDouble(fixed_params.alpha - 0.15 * overload, 0.0, 1.0);
        target.keyframe_aggressiveness = 1.0 - 0.3 * overload;
    }

    ClampAdaptiveParams(target, config);
    return SmoothAdaptiveParams(target, previous_params, config);
}

void ClampAdaptiveParams(AdaptiveParams& params, const AdaptiveConfig& config)
{
    params.kappa_top = ClampInt(params.kappa_top, config.min_kappa_top, config.max_kappa_top);
    params.alpha = ClampDouble(params.alpha, 0.0, 1.0);
    params.tau0 = ClampDouble(params.tau0, config.min_tau0, config.max_tau0);
    params.theta_drop = std::max(0.0, params.theta_drop);
    params.keyframe_aggressiveness = ClampDouble(params.keyframe_aggressiveness, 0.25, 2.0);
}

AdaptiveParams SmoothAdaptiveParams(
    const AdaptiveParams& target,
    const AdaptiveParams& previous,
    const AdaptiveConfig& config)
{
    const double s = ClampDouble(config.smooth_factor, 0.0, 1.0);
    AdaptiveParams out;
    out.kappa_top = static_cast<int>(
        std::round(s * previous.kappa_top + (1.0 - s) * target.kappa_top));
    out.alpha = s * previous.alpha + (1.0 - s) * target.alpha;
    out.tau0 = s * previous.tau0 + (1.0 - s) * target.tau0;
    out.theta_drop = s * previous.theta_drop + (1.0 - s) * target.theta_drop;
    out.keyframe_aggressiveness =
        s * previous.keyframe_aggressiveness + (1.0 - s) * target.keyframe_aggressiveness;

    ClampAdaptiveParams(out, config);
    return out;
}

AdaptiveLogger::~AdaptiveLogger()
{
    if(mStream.is_open())
        mStream.close();
}

void AdaptiveLogger::Configure(bool enabled, const std::string& path)
{
    mEnabled = enabled;
    mPath = path;
    if(mStream.is_open())
        mStream.close();
    mHeaderWritten = false;
}

void AdaptiveLogger::Log(
    const AdaptiveState& state,
    const AdaptiveParams& params,
    bool keyframe_inserted,
    bool tracking_lost)
{
    if(!mEnabled || mPath.empty())
        return;

    if(!mStream.is_open())
        mStream.open(mPath, std::ios::app);
    if(!mStream.is_open())
        return;

    if(!mHeaderWritten)
    {
        if(mStream.tellp() == 0)
        {
            mStream << "frame_id,timestamp,"
                    << "candidate_point_number,selected_point_number,selection_ratio,"
                    << "logdet_H,min_eigenvalue_H,condition_number_H,"
                    << "information_variation_delta_E,cumulative_degradation_D_t,"
                    << "median_reprojection_error,inlier_ratio,tracked_map_points,"
                    << "image_contrast,blur_score,feature_spatial_entropy,"
                    << "tracking_time_ms,recent_local_ba_time_ms,"
                    << "number_of_keyframes,number_of_map_points,keyframe_interval,"
                    << "kappa_top,alpha,tau0,theta_drop,keyframe_aggressiveness,"
                    << "keyframe_insertion_flag,tracking_lost_flag,ate,rpe\n";
        }
        mHeaderWritten = true;
    }

    mStream << std::fixed << std::setprecision(6)
            << state.frame_id << ","
            << state.timestamp << ","
            << state.candidate_point_number << ","
            << state.selected_point_number << ","
            << state.selection_ratio << ","
            << state.logdet_H << ","
            << state.min_eigenvalue_H << ","
            << state.condition_number_H << ","
            << state.information_variation_delta_E << ","
            << state.cumulative_degradation_D_t << ","
            << state.median_reprojection_error << ","
            << state.inlier_ratio << ","
            << state.tracked_map_points << ","
            << state.image_contrast << ","
            << state.blur_score << ","
            << state.feature_spatial_entropy << ","
            << state.tracking_time_ms << ","
            << state.recent_local_ba_time_ms << ","
            << state.number_of_keyframes << ","
            << state.number_of_map_points << ","
            << state.keyframe_interval << ","
            << params.kappa_top << ","
            << params.alpha << ","
            << params.tau0 << ","
            << params.theta_drop << ","
            << params.keyframe_aggressiveness << ","
            << (keyframe_inserted ? 1 : 0) << ","
            << (tracking_lost ? 1 : 0) << ","
            << "," << "\n";
    mStream.flush();
}

} // namespace ORB_SLAM3
