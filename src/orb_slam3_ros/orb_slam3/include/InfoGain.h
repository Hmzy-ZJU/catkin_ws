/**
* InfoGain.h
* Information-theoretic feature selection for ORB-SLAM3
* 
* This module implements Fisher Information-based feature selection
* to retain only the most informative keypoints for pose estimation.
* 信息论特征选择模块头文件，用于 ORB-SLAM3。
*
* 实现基于 Fisher 信息矩阵的特征选择，仅保留对位姿估计最有用的关键点。
*/
#ifndef INFOGAIN_H
#define INFOGAIN_H
// ==== 头文件依赖 ====
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

    int topK;              // 保留的最多特征点数
    float w_uniform;       // 空间均匀性得分权重（0 ~ 1）
    float minPxDist;       // 最小像素间距，用于计算密度
    bool useUniform;       // 是否使用空间均匀性
    double lambdaInit;     // Fisher 信息矩阵的初始正则项

    // 默认构造函数，设定默认参数
    InfoSelectParams()
        : topK(400)
        , w_uniform(0.25f)
        , minPxDist(8.0f)
        , useUniform(true)
        , lambdaInit(1.0e-3)
    {}
};

/**
 * @brief 匹配点结构体，用于特征选择
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
 * @brief 信息增益选择主类
 */
class InfoGain
{
public:
    /**
     * @brief 主函数：根据信息增益选择 top-K 特征点
     * @param F 当前帧对象
     * @param matches 关键点与地图点匹配列表
     * @param params 特征选择参数
     * @return 被选择的特征点索引列表
     */
    static std::vector<int> SelectByInformationGain(
        Frame& F,
        std::vector<MatchInfo>& matches,
        const InfoSelectParams& params);

    /**
     * @brief 计算所选特征点对当前相机姿态的 Fisher 信息矩阵
     * @param F 当前帧
     * @param selectedIndices 被选中点的索引
     * @param lambda 正则参数，避免奇异矩阵
     * @return 6x6 的 Fisher 信息矩阵（Hessian）
     */
    static Eigen::Matrix<double,6,6> ComputePoseInformation(
        Frame& F,
        const std::vector<int>& selectedIndices,
        double lambda = 1.0e-3);

private:
    /**
     * @brief 构建一个特征点在当前位姿下对位姿的 Jacobian 矩阵
     * @param Pw 世界坐标系下的三维点
     * @param Tcw 相机位姿（世界->相机）
     * @param fx 相机焦距 fx
     * @param fy 相机焦距 fy
     * @return 2x6 的雅可比矩阵，表示 u,v 对 6 DoF 位姿的偏导
     */
    static Eigen::Matrix<double,2,6> BuildJacobianForPoint(
        const Eigen::Vector3f& Pw,
        const Sophus::SE3f& Tcw,
        float fx, float fy);

    /**
     * @brief 从 Sophus 的 SE3 提取旋转和平移
     * @param Tcw 相机位姿
     * @param R 输出：3x3 旋转矩阵
     * @param t 输出：3x1 平移向量
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
     * @brief 计算加入一个特征点后增加的信息量（以 bit 为单位）
     * @param H 当前 Fisher 信息矩阵
     * @param J 当前特征点的 Jacobian（2x6）
     * @param lambda 正则项
     * @return 信息增益（bit）
     */
    static inline double DeltaBits(
        const Eigen::Matrix<double,6,6>& H,
        const Eigen::Matrix<double,2,6>& J,
        double lambda)
    {
        // Compute H_inv using Cholesky decomposition with regularization
        Eigen::Matrix<double,6,6> H_reg = H + lambda * Eigen::Matrix<double,6,6>::Identity();
        Eigen::LLT<Eigen::Matrix<double,6,6>> llt(H_reg);
        
        if(llt.info() != Eigen::Success) {
            // 如果分解失败，返回默认最小增益
            return 0.1;
        }
        
        Eigen::Matrix<double,6,6> H_inv = llt.solve(Eigen::Matrix<double,6,6>::Identity());
        
        // 使用 Woodbury 恒等式计算增益：
        // det(H + J^T*J) / det(H) = det(I + J*H_inv*J^T)
        Eigen::Matrix<double,2,2> inner = Eigen::Matrix<double,2,2>::Identity() 
                                         + J * H_inv * J.transpose();
        
        double det_inner = inner.determinant();
        if(det_inner <= 0.0) {
            return 0.1;
        }
        
        // 信息增益 = 0.5 * log2(det_ratio)
        return 0.5 * std::log2(det_inner);
    }

    /**
     * @brief 使用 Woodbury 公式更新 Fisher 信息矩阵
     * @param H 输入/输出：当前信息矩阵
     * @param J 被加入特征点的雅可比矩阵
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
