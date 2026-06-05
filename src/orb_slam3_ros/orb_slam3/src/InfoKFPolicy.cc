/**
* InfoKFPolicy.cc
* Implementation of Fisher Information-based keyframe gating
*/

#include "InfoKFPolicy.h"
#include "Frame.h"
#include "KeyFrame.h"
#include <iostream>
#include <cmath>
#include <iomanip>
namespace ORB_SLAM3
{

/**
 * @brief Compute information drop between current and reference Fisher matrix
 * 
 * Delta_E = 0.5 * log2(det(H_curr) / det(H_ref))
 * 
 * Positive value = information gain
 * Negative value = information loss
 */
double InfoKFPolicy::ComputeInformationDrop(
    const Eigen::Matrix<double,6,6>& H_curr,
    const Eigen::Matrix<double,6,6>& H_ref,
    double lambda)
{
    double log_det_curr = SafeLogDet(H_curr, lambda);
    double log_det_ref = SafeLogDet(H_ref, lambda);

    // Check for invalid determinants
    if(log_det_curr < -1e8 || log_det_ref < -1e8) {
        // Unable to compute, assume information is lost
        return -1e9;
    }

    // Delta_E = 0.5 * (log_det_curr - log_det_ref)
    double delta_bits = 0.5 * (log_det_curr - log_det_ref);

    return delta_bits;
}

/**
 * @brief Main keyframe gating decision function
 * 
 * Decision logic:
 * 1. If not initialized, always allow (to create reference)
 * 2. Compute information drop: delta_E = 0.5*log2(det(H_curr)/det(H_ref))
 * 3. If delta_E < -allowBitsDrop, reject keyframe (information not sufficient)
 * 4. Otherwise, allow keyframe
 */
// AFTER (完整替换)
bool InfoKFPolicy::AllowNewKF(
    const Eigen::Matrix<double,6,6>& H_curr,
    int n_matches_curr,
    InfoKFState& state,
    const InfoKFParams& params)
{
    if(!params.use) return true;

    // ---- 0) 初始化阶段：允许创建第一个参考 ----
    if(!state.initialized) {
        std::cout << "[InfoKF] First KF - initializing reference\n";
        return true;
    }

    // ---- 1) 基本信息差 ΔE_t ----
    const double delta_bits = ComputeInformationDrop(H_curr, state.H_ref, params.lambdaMean);

    // ---- 2) 基于匹配数的 tracking 质量 r_t ∈ [0,1] ----
    // nref 取参考关键帧对应的匹配数，若为 0 则退化为 1 防止除零
    const int nref = std::max(1, state.refMatches);
    const double r = std::max(
        0.0,
        std::min(1.0, static_cast<double>(n_matches_curr) / static_cast<double>(nref))
    );

    // ---- 3) 对称信息阈值 T_eff(r) = clip( T0 * (1 + alpha*(1-r)), [Tmin, Tmax] ) ----
    // 其中 T0 = allowBitsDrop 是你要主要调节的“信息变化容忍度”
    double Teff = params.allowBitsDrop * (1.0 - params.dynAlpha * (1.0 - r));
    Teff = std::max(params.dynTauMin, std::min(params.dynTauMax, Teff));

    // ---- 4) 累计/平滑信息下降 D_t = ρ D_{t-1} + min(0, ΔE_t) ----
    state.cumDrop = params.cumDecay * state.cumDrop + std::min(0.0, delta_bits);

    // ---- 5) 最大帧距强制插帧 ----
    const bool force_by_gap = (params.maxFramesForce > 0) &&
                              (state.framesSinceRef >= params.maxFramesForce);

    std::cout << "[InfoKF] ΔE=" << std::fixed << std::setprecision(3) << delta_bits
              << "  |ΔE|_thr=" << Teff
              << "  r="        << std::setprecision(3) << r
              << "  D_cum="    << state.cumDrop
              << "  gap="      << state.framesSinceRef
              << (force_by_gap ? " (FORCE)" : "")
              << std::endl;

    // ---- 6) 允许插帧的 OR 条件（对称 |ΔE| 阈值 + 累积下降 + 最大帧距）----
    const bool allow =
        (std::fabs(delta_bits) > Teff) ||      // 对称信息阈值：信息显著变化（增益/退化）
        (state.cumDrop < -params.cumThr) ||    // 长期信息缓慢下降
        force_by_gap;                          // 帧距兜底

    if(allow) {
        std::cout << "[InfoKF] ALLOW\n";
    } else {
        std::cout << "[InfoKF] REJECT\n";
    }
    return allow;
}

/**
 * @brief Update state after creating a new keyframe
 * 
 * 1. Update reference Fisher matrix: H_ref = H_new
 * 2. Update running mean: H_mean = alpha * H_mean + (1-alpha) * H_new
 * 3. Reset frame counter
 */
void InfoKFPolicy::OnKeyFrameCreated(
    const Eigen::Matrix<double,6,6>& H_new,
    InfoKFState& state,
    const InfoKFParams& params)
{
    // Update reference
    state.H_ref = H_new;

    // Update running mean using exponential moving average
    if(state.initialized) {
        state.H_mean = params.alpha * state.H_mean 
                     + (1.0 - params.alpha) * H_new;
    } else {
        // First keyframe: initialize mean
        state.H_mean = H_new;
        state.initialized = true;
    }

    // Reset counter

    state.numKFsSinceRef = 0; // 如果还保留这个字段
    state.framesSinceRef = 0; // 新增：重置帧距
    state.cumDrop = 0.0;      // 新增：累计下降清零
    std::cout << "[InfoKF] Reference updated after new KF creation" << std::endl;
}

} // namespace ORB_SLAM3
