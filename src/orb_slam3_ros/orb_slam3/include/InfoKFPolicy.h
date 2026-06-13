/**
* InfoKFPolicy.h
* Fisher Information-based keyframe gating policy
* 
* This module decides whether to insert a new keyframe based on 
* information gain rather than traditional heuristics.
*/

#ifndef INFOKFPOLICY_H
#define INFOKFPOLICY_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace ORB_SLAM3
{

// Forward declarations
class Frame;
class KeyFrame;

/**
 * @brief Parameters for information-based keyframe policy
 */
struct InfoKFParams
{
    bool use;                    // Enable information gating
    double allowBitsDrop;        // Max allowed information drop (bits)
    double lambdaMean;           // Regularization for mean Fisher matrix
    double alpha;                // Smoothing factor for exponential moving average
    // --- 新增：动态阈值 ---
    double dynAlpha;     // α
    double dynBeta;      // β
    double dynTauMin;    // τ_min
    double dynTauMax;    // τ_max

    // --- 新增：累计下降 ---
    double cumDecay;     // ρ
    double cumThr;       // Θ_cum

    // --- 新增：最大帧距 ---
    int    maxFramesForce; // G_max
    bool   verbose;
    InfoKFParams()
        : use(true)
        , allowBitsDrop(1.0)
        , lambdaMean(1.0e-3)
        , alpha(0.9)
        , dynAlpha(0.25)
        , dynBeta(0.0)
        , dynTauMin(0.1)
        , dynTauMax(5.0)
        , cumDecay(0.95)
        , cumThr(0.5)
        , maxFramesForce(60)
        , verbose(true)
    {}
};

/**
 * @brief State maintained for information-based keyframe policy
 */
struct InfoKFState
{
    Eigen::Matrix<double,6,6> H_ref;     // Fisher Information of reference KF
    Eigen::Matrix<double,6,6> H_mean;    // Running mean of Fisher Information
    bool initialized;                     // Whether reference is initialized
    int numKFsSinceRef;                  // Number of frames since reference KF
    int  framesSinceRef{0}; // 新增
    int  refMatches{0};     // 新增
    double cumDrop{0.0};    // 新增
    InfoKFState()
        : initialized(false)
        , numKFsSinceRef(0)
    {
        H_ref.setZero();
        H_mean.setZero();
    }
};

/**
 * @brief Information-based keyframe insertion policy
 */
class InfoKFPolicy
{
public:
    /**
     * @brief Decide whether to allow new keyframe based on information gain
     * 
     * @param H_curr Fisher Information matrix of current frame
     * @param state Current state (reference KF info, running mean)
     * @param params Policy parameters
     * @return true if new keyframe should be created
     */
    static bool AllowNewKF(
        const Eigen::Matrix<double,6,6>& H_curr,
                int n_matches_curr,
                InfoKFState& state,
                const InfoKFParams& params);

    /**
     * @brief Update state after a keyframe is created
     * 
     * @param H_new Fisher Information of the new keyframe
     * @param state State to update
     * @param params Policy parameters
     */
    static void OnKeyFrameCreated(
        const Eigen::Matrix<double,6,6>& H_new,
        InfoKFState& state,
        const InfoKFParams& params);

private:
    /**
     * @brief Compute information drop in bits
     * 
     * Delta_E = 0.5 * log2(det(H_curr) / det(H_ref))
     * 
     * @param H_curr Current Fisher Information matrix
     * @param H_ref Reference Fisher Information matrix
     * @param lambda Regularization parameter
     * @return Information drop in bits (negative = information loss)
     */
    static double ComputeInformationDrop(
        const Eigen::Matrix<double,6,6>& H_curr,
        const Eigen::Matrix<double,6,6>& H_ref,
        double lambda);

    /**
     * @brief Compute log determinant safely
     * 
     * @param H Matrix to compute log-determinant
     * @param lambda Regularization parameter
     * @return log2(det(H + lambda*I))
     */
    static inline double SafeLogDet(
        const Eigen::Matrix<double,6,6>& H,
        double lambda)
    {
        // Add regularization for numerical stability
        Eigen::Matrix<double,6,6> H_reg = H + lambda * Eigen::Matrix<double,6,6>::Identity();
        
        // Use Cholesky decomposition for stable determinant computation
        Eigen::LLT<Eigen::Matrix<double,6,6>> llt(H_reg);
        
        if(llt.info() != Eigen::Success) {
            // Matrix is not positive definite, return large negative value
            return -1e9;
        }
        
        // det(H) = product of diagonal elements of L (Cholesky factor)
        // log(det(H)) = 2 * sum(log(diag(L)))
        Eigen::Matrix<double,6,6> L = llt.matrixL();
        double log_det = 0.0;
        
        for(int i = 0; i < 6; ++i) {
            double diag_val = L(i, i);
            if(diag_val > 0) {
                log_det += std::log(diag_val);
            }
        }
        
        log_det *= 2.0; // Cholesky: H = L*L^T => det(H) = det(L)^2
        
        // Convert to log2
        return log_det / std::log(2.0);
    }
};

} // namespace ORB_SLAM3

#endif // INFOKFPOLICY_H
