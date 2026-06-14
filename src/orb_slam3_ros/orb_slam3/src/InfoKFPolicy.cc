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
// AFTER (瀹屾暣鏇挎崲)
bool InfoKFPolicy::AllowNewKF(
    const Eigen::Matrix<double,6,6>& H_curr,
    int n_matches_curr,
    InfoKFState& state,
    const InfoKFParams& params)
{
    if(!params.use) return true;

    // ---- 0) 鍒濆鍖栭樁娈碉細鍏佽鍒涘缓绗竴涓弬鑰?----
    if(!state.initialized) {
        if(params.verbose)
            std::cout << "[InfoKF] First KF - initializing reference\n";
        return true;
    }

    // ---- 1) 鍩烘湰淇℃伅宸?螖E_t ----
    const double delta_bits = ComputeInformationDrop(H_curr, state.H_ref, params.lambdaMean);

    // ---- 2) 鍩轰簬鍖归厤鏁扮殑 tracking 璐ㄩ噺 r_t 鈭?[0,1] ----
    // nref 鍙栧弬鑰冨叧閿抚瀵瑰簲鐨勫尮閰嶆暟锛岃嫢涓?0 鍒欓€€鍖栦负 1 闃叉闄ら浂
    const int nref = std::max(1, state.refMatches);
    const double r = std::max(
        0.0,
        std::min(1.0, static_cast<double>(n_matches_curr) / static_cast<double>(nref))
    );

    // Effective information-change threshold.
    double Teff = params.allowBitsDrop * (1.0 - params.dynAlpha * (1.0 - r));
    Teff = std::max(params.dynTauMin, std::min(params.dynTauMax, Teff));

    // ---- 4) 绱/骞虫粦淇℃伅涓嬮檷 D_t = 蟻 D_{t-1} + min(0, 螖E_t) ----
    state.cumDrop = params.cumDecay * state.cumDrop + std::min(0.0, delta_bits);

    // ---- 5) 鏈€澶у抚璺濆己鍒舵彃甯?----
    const bool force_by_gap = (params.maxFramesForce > 0) &&
                              (state.framesSinceRef >= params.maxFramesForce);

    if(params.verbose)
    {
    std::cout << "[InfoKF] 螖E=" << std::fixed << std::setprecision(3) << delta_bits
              << "  |螖E|_thr=" << Teff
              << "  r="        << std::setprecision(3) << r
              << "  D_cum="    << state.cumDrop
              << "  gap="      << state.framesSinceRef
              << (force_by_gap ? " (FORCE)" : "")
              << std::endl;
    }

    // ---- 6) 鍏佽鎻掑抚鐨?OR 鏉′欢锛堝绉?|螖E| 闃堝€?+ 绱Н涓嬮檷 + 鏈€澶у抚璺濓級----
    const bool allow =
        (std::fabs(delta_bits) > Teff) ||      // 瀵圭О淇℃伅闃堝€硷細淇℃伅鏄捐憲鍙樺寲锛堝鐩?閫€鍖栵級
        (state.cumDrop < -params.cumThr) ||    // 闀挎湡淇℃伅缂撴參涓嬮檷
        force_by_gap;                          // 甯ц窛鍏滃簳

    if(params.verbose)
    {
        if(allow) {
            std::cout << "[InfoKF] ALLOW\n";
        } else {
            std::cout << "[InfoKF] REJECT\n";
        }
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

    // Reset counters after accepting a new reference keyframe.
    state.numKFsSinceRef = 0;
    state.framesSinceRef = 0;
    state.cumDrop = 0.0;

    if(params.verbose)
        std::cout << "[InfoKF] Reference updated after new KF creation" << std::endl;
}

} // namespace ORB_SLAM3

