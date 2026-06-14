/**
* InfoGain.h
* Information-theoretic feature selection for ORB-SLAM3
* 
* This module implements Fisher Information-based feature selection
* to retain only the most informative keypoints for pose estimation.
* 淇℃伅璁虹壒寰侀€夋嫨妯″潡澶存枃浠讹紝鐢ㄤ簬 ORB-SLAM3銆?*
* 瀹炵幇鍩轰簬 Fisher 淇℃伅鐭╅樀鐨勭壒寰侀€夋嫨锛屼粎淇濈暀瀵逛綅濮夸及璁℃渶鏈夌敤鐨勫叧閿偣銆?*/
#ifndef INFOGAIN_H
#define INFOGAIN_H
// ==== 澶存枃浠朵緷璧?====
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
namespace ORB_SLAM3
{

// Forward declarations
class Frame;
class MapPoint;

/**
 * @brief Parameters for information-based feature selection
 */
struct InfoSelectParams
{
    int topK;
    float w_uniform;
    float minPxDist;
    bool useUniform;
    double lambdaInit;
    bool stereoSafeKeep;
    bool verbose;
    bool greedySelect;

    InfoSelectParams()
        : topK(400)
        , w_uniform(0.25f)
        , minPxDist(8.0f)
        , useUniform(true)
        , lambdaInit(1.0e-3)
        , stereoSafeKeep(true)
        , verbose(true)
        , greedySelect(true)
    {}
};

/**
 * @brief 鍖归厤鐐圭粨鏋勪綋锛岀敤浜庣壒寰侀€夋嫨
 */
struct MatchInfo
{
    int idx;               // Keypoint index in Frame
    MapPoint* pMP;         // Associated MapPoint
    float uniformScore;    // Spatial uniformity score [0,1]

    MatchInfo() : idx(-1), pMP(nullptr), uniformScore(0.0f) {}
    MatchInfo(int i, MapPoint* p) : idx(i), pMP(p), uniformScore(0.0f) {}
};

/**
 * @brief 淇℃伅澧炵泭閫夋嫨涓荤被
 */
class InfoGain
{
public:
    /**
     * @brief 涓诲嚱鏁帮細鏍规嵁淇℃伅澧炵泭閫夋嫨 top-K 鐗瑰緛鐐?     * @param F 褰撳墠甯у璞?     * @param matches 鍏抽敭鐐逛笌鍦板浘鐐瑰尮閰嶅垪琛?     * @param params 鐗瑰緛閫夋嫨鍙傛暟
     * @return 琚€夋嫨鐨勭壒寰佺偣绱㈠紩鍒楄〃
     */
    static std::vector<int> SelectByInformationGain(
        Frame& F,
        std::vector<MatchInfo>& matches,
        const InfoSelectParams& params);

    /**
     * @brief 璁＄畻鎵€閫夌壒寰佺偣瀵瑰綋鍓嶇浉鏈哄Э鎬佺殑 Fisher 淇℃伅鐭╅樀
     * @param F 褰撳墠甯?     * @param selectedIndices 琚€変腑鐐圭殑绱㈠紩
     * @param lambda 姝ｅ垯鍙傛暟锛岄伩鍏嶅寮傜煩闃?     * @return 6x6 鐨?Fisher 淇℃伅鐭╅樀锛圚essian锛?     */
    static Eigen::Matrix<double,6,6> ComputePoseInformation(
        Frame& F,
        const std::vector<int>& selectedIndices,
        double lambda = 1.0e-3);

private:
    /**
     * @brief 鏋勫缓涓€涓壒寰佺偣鍦ㄥ綋鍓嶄綅濮夸笅瀵逛綅濮跨殑 Jacobian 鐭╅樀
     * @param Pw 涓栫晫鍧愭爣绯讳笅鐨勪笁缁寸偣
     * @param Tcw 鐩告満浣嶅Э锛堜笘鐣?>鐩告満锛?     * @param fx 鐩告満鐒﹁窛 fx
     * @param fy 鐩告満鐒﹁窛 fy
     * @return 2x6 鐨勯泤鍙瘮鐭╅樀锛岃〃绀?u,v 瀵?6 DoF 浣嶅Э鐨勫亸瀵?     */
    static Eigen::Matrix<double,2,6> BuildJacobianForPoint(
        const Eigen::Vector3f& Pw,
        const Sophus::SE3f& Tcw,
        float fx, float fy);

    /**
     * @brief 浠?Sophus 鐨?SE3 鎻愬彇鏃嬭浆鍜屽钩绉?     * @param Tcw 鐩告満浣嶅Э
     * @param R 杈撳嚭锛?x3 鏃嬭浆鐭╅樀
     * @param t 杈撳嚭锛?x1 骞崇Щ鍚戦噺
     */
    static inline void ExtractPoseRt(
        const Sophus::SE3f& Tcw,
        Eigen::Matrix3f& R,
        Eigen::Vector3f& t)
    {
        R = Tcw.rotationMatrix();
        t = Tcw.translation();
    }

    /**
     * @brief 璁＄畻鍔犲叆涓€涓壒寰佺偣鍚庡鍔犵殑淇℃伅閲忥紙浠?bit 涓哄崟浣嶏級
     * @param H 褰撳墠 Fisher 淇℃伅鐭╅樀
     * @param J 褰撳墠鐗瑰緛鐐圭殑 Jacobian锛?x6锛?     * @param lambda 姝ｅ垯椤?     * @return 淇℃伅澧炵泭锛坆it锛?     */
    static inline double DeltaBits(
        const Eigen::Matrix<double,6,6>& H,
        const Eigen::Matrix<double,2,6>& J,
        double lambda)
    {
        // Compute H_inv using Cholesky decomposition with regularization
        Eigen::Matrix<double,6,6> H_reg = H + lambda * Eigen::Matrix<double,6,6>::Identity();
        Eigen::LLT<Eigen::Matrix<double,6,6>> llt(H_reg);
        
        if(llt.info() != Eigen::Success) {
            // 濡傛灉鍒嗚В澶辫触锛岃繑鍥為粯璁ゆ渶灏忓鐩?            return 0.1;
        }
        
        Eigen::Matrix<double,6,6> H_inv = llt.solve(Eigen::Matrix<double,6,6>::Identity());
        
        // 浣跨敤 Woodbury 鎭掔瓑寮忚绠楀鐩婏細
        // det(H + J^T*J) / det(H) = det(I + J*H_inv*J^T)
        Eigen::Matrix<double,2,2> inner = Eigen::Matrix<double,2,2>::Identity() 
                                         + J * H_inv * J.transpose();
        
        double det_inner = inner.determinant();
        if(det_inner <= 0.0) {
            return 0.1;
        }
        
        // 淇℃伅澧炵泭 = 0.5 * log2(det_ratio)
        return 0.5 * std::log2(det_inner);
    }

    /**
     * @brief 浣跨敤 Woodbury 鍏紡鏇存柊 Fisher 淇℃伅鐭╅樀
     * @param H 杈撳叆/杈撳嚭锛氬綋鍓嶄俊鎭煩闃?     * @param J 琚姞鍏ョ壒寰佺偣鐨勯泤鍙瘮鐭╅樀
     */
    static inline void WoodburyUpdate(
        Eigen::Matrix<double,6,6>& H,
        const Eigen::Matrix<double,2,6>& J)
    {
        // H_new = H + J^T * J
        H += J.transpose() * J;
    }
};

} // namespace ORB_SLAM3

#endif // INFOGAIN_H
