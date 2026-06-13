/**
* InfoGain.cc
* Implementation of information-theoretic feature selection

* 实现信息论特征选择功能模块
*/
#include "InfoGain.h"
#include "Frame.h"
#include "MapPoint.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace ORB_SLAM3
{

/**
 * @brief 构建某个特征点对应的雅可比矩阵（Jacobian）
 * 
 * 输入为某个 3D 点 Pw，在相机当前位姿 Tcw 下，计算其像素坐标 (u,v)
 * 相对于位姿参数 [旋转omega和平移t] 的偏导数，共6个自由度
 * 
 * 返回 2x6 的雅可比矩阵：d[u,v]/d[omega_x, omega_y, omega_z, tx, ty, tz]
 */
Eigen::Matrix<double,2,6> InfoGain::BuildJacobianForPoint(
    const Eigen::Vector3f& Pw,
    const Sophus::SE3f& Tcw,
    float fx, float fy)
{
    Eigen::Matrix<double,2,6> J;
    J.setZero();

    // 从Tcw中提取旋转矩阵 R 和 平移向量 t
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    ExtractPoseRt(Tcw, R, t);

    // 将世界坐标 Pw 转换到相机坐标系 Pc
    Eigen::Vector3f Pc = R * Pw + t;
    
    float X = Pc(0);
    float Y = Pc(1);
    float Z = Pc(2);

    // Check for valid depth
    // 如果深度太小，认为是无效点，直接返回零矩阵
    if(Z <= 0.01f) {
        return J; // Return zero Jacobian for invalid points
    }

    float Z_inv = 1.0f / Z;
    float Z_inv2 = Z_inv * Z_inv;

    // ============================================
    // Jacobian w.r.t. rotation (left perturbation)
    // ============================================
    // d(u)/d(omega) = fx * d(X/Z)/d(omega)
    // For left perturbation: delta_R = exp([omega]_x) * R
    // => d(Pc)/d(omega) = -[Pc]_x (skew-symmetric matrix)
    
    // ============================================
    // 对旋转扰动的雅可比（采用左扰动模型）
    // Pc在相机坐标系中，其扰动对旋转的导数为 -[Pc]_x（反对称矩阵）
    // ============================================

    Eigen::Matrix3f skew_Pc;
    skew_Pc << 0,   -Z,   Y,
               Z,    0,  -X,
              -Y,    X,   0;

    // d(u)/d(omega) = fx * [d(X/Z)/dPc] * [dPc/d(omega)]
    // d(X/Z)/dPc = [1/Z, 0, -X/Z^2]
    Eigen::RowVector3f du_dPc;
    du_dPc << Z_inv, 0.0f, -X * Z_inv2;
    
    Eigen::RowVector3f dv_dPc;
    dv_dPc << 0.0f, Z_inv, -Y * Z_inv2;

    // Rotation part: J[:, 0:3]
    J.block<1,3>(0, 0) = (fx * du_dPc * (-skew_Pc)).cast<double>();
    J.block<1,3>(1, 0) = (fy * dv_dPc * (-skew_Pc)).cast<double>();
    // ============================================
    // Jacobian w.r.t. translation
    // ============================================
    // d(Pc)/d(t) = R (for world-to-camera)
    // Since we parameterize t directly in camera frame: d(Pc)/d(t) = I
    
    // Translation part: J[:, 3:6]
    J.block<1,3>(0, 3) = (fx * du_dPc).cast<double>();
    J.block<1,3>(1, 3) = (fy * dv_dPc).cast<double>();

    return J;
}

/**
 * @brief Compute Fisher Information matrix for selected features
 */
 /**
 * @brief 计算一帧中已选特征点对位姿的 Fisher 信息矩阵
 * @param F 当前帧
 * @param selectedIndices 选中的关键点索引
 * @param lambda 正则化项，避免奇异矩阵
 */
Eigen::Matrix<double,6,6> InfoGain::ComputePoseInformation(
    Frame& F,
    const std::vector<int>& selectedIndices,
    double lambda)
{
    Eigen::Matrix<double,6,6> H;
    H.setZero();

    // Add regularization to diagonal
    H += lambda * Eigen::Matrix<double,6,6>::Identity();

    if(!F.HasPose())
        return H;

    // Get camera parameters
    float fx = F.fx;
    float fy = F.fy;

    // Get current pose
    Sophus::SE3f Tcw = F.GetPose();

    // Accumulate information from each selected feature
    for(size_t i = 0; i < selectedIndices.size(); ++i)
    {
        int idx = selectedIndices[i];
        if(idx < 0 || idx >= F.N)
            continue;

        MapPoint* pMP = F.mvpMapPoints[idx];
        if(!pMP || pMP->isBad())
            continue;

        // Get 3D point in world coordinates
        Eigen::Vector3f Pw = pMP->GetWorldPos();

        // Compute Jacobian
        Eigen::Matrix<double,2,6> J = BuildJacobianForPoint(Pw, Tcw, fx, fy);

        // Update Fisher Information: H += J^T * J
        H += J.transpose() * J;
    }

    return H;
}

/**
 * @brief Main feature selection function using greedy information gain
 * 
 * Algorithm:
 * 1. Initialize Fisher Information matrix H with lambda*I
 * 2. For each candidate feature:
 *    - Compute information gain: delta_bits
 *    - Compute spatial uniformity score
 *    - Compute total score = (1-w)*delta_bits + w*uniform_score
 * 3. Greedily select top-K features with highest scores
 * 4. Update H after each selection using Woodbury formula
 */
 /**
 * @brief 主要特征选择函数，采用贪婪信息增益策略
 * 
 * 算法步骤：
 * 1. 初始化信息矩阵 H = lambda * I
 * 2. 对每个候选特征：
 *    - 计算信息增益 delta_bits
 *    - 如果开启，计算空间均匀性得分
 *    - 得分 = (1-w)*信息增益 + w*均匀性得分
 * 3. 贪婪地选择得分前 top-K 的特征点
 * 4. 每次选择后更新 H
 */
std::vector<int> InfoGain::SelectByInformationGain(
    Frame& F,
    std::vector<MatchInfo>& matches,
    const InfoSelectParams& params)
{
    std::vector<int> selectedIndices;
    
    if(matches.empty()) {
        return selectedIndices;
    }

    // Check if frame has valid pose
    if(!F.HasPose()) {
        // Fallback: return all matches
        for(size_t i = 0; i < matches.size(); ++i) {
            selectedIndices.push_back(matches[i].idx);
        }
        return selectedIndices;
    }

    // Get camera parameters
    float fx = F.fx;
    float fy = F.fy;
    Sophus::SE3f Tcw = F.GetPose();

    // ========================================
    // Step 1: Initialize Fisher Information matrix
    // ========================================
    Eigen::Matrix<double,6,6> H;
    H.setZero();
    H += params.lambdaInit * Eigen::Matrix<double,6,6>::Identity();

    // ========================================
    // Step 2: Compute spatial uniformity scores
    // ========================================
    if(params.useUniform)
    {
        for(size_t i = 0; i < matches.size(); ++i)
        {
            int idx = matches[i].idx;
            if(idx < 0 || idx >= (int)F.mvKeysUn.size())
                continue;

            const cv::KeyPoint& kp = F.mvKeysUn[idx];
            float u = kp.pt.x;
            float v = kp.pt.y;

            // Count nearby keypoints within minPxDist
            int countNearby = 0;
            for(size_t j = 0; j < F.mvKeysUn.size(); ++j)
            {
                float du = F.mvKeysUn[j].pt.x - u;
                float dv = F.mvKeysUn[j].pt.y - v;
                float dist = std::sqrt(du*du + dv*dv);
                
                if(dist < params.minPxDist) {
                    countNearby++;
                }
            }

            // Uniformity score: lower density = higher score
            // Normalize by image area
            float density = (float)countNearby / (params.minPxDist * params.minPxDist);
            matches[i].uniformScore = 1.0f / (1.0f + density);
        }
    }
    else
    {
        // No uniformity scoring
        for(size_t i = 0; i < matches.size(); ++i) {
            matches[i].uniformScore = 0.0f;
        }
    }

    // ========================================
    // Step 3: Greedy selection with information gain
    // ========================================
    std::vector<bool> selected(matches.size(), false);
    int numToSelect = std::min(params.topK, (int)matches.size());

    for(int iter = 0; iter < numToSelect; ++iter)
    {
        double maxScore = -1e9;
        int bestIdx = -1;

        // Find feature with maximum combined score
        for(size_t i = 0; i < matches.size(); ++i)
        {
            if(selected[i])
                continue;

            MapPoint* pMP = matches[i].pMP;
            if(!pMP || pMP->isBad())
                continue;

            // Get 3D point
            Eigen::Vector3f Pw = pMP->GetWorldPos();

            // Compute Jacobian
            Eigen::Matrix<double,2,6> J = BuildJacobianForPoint(Pw, Tcw, fx, fy);

            // Check if Jacobian is valid (non-zero)
            if(J.norm() < 1e-6)
                continue;

            // Compute information gain
            double delta_bits = DeltaBits(H, J, params.lambdaInit);

            // Combine with uniformity score
            double score = (1.0f - params.w_uniform) * delta_bits 
                         + params.w_uniform * matches[i].uniformScore;

            if(score > maxScore) {
                maxScore = score;
                bestIdx = i;
            }
        }

        if(bestIdx == -1)
            break; // No more valid features

        // Mark as selected
        selected[bestIdx] = true;
        selectedIndices.push_back(matches[bestIdx].idx);

        // Update Fisher Information matrix
        MapPoint* pMP = matches[bestIdx].pMP;
        Eigen::Vector3f Pw = pMP->GetWorldPos();
        Eigen::Matrix<double,2,6> J = BuildJacobianForPoint(Pw, Tcw, fx, fy);
        
        WoodburyUpdate(H, J);
    }

    if(params.verbose)
    {
        std::cout << "[InfoGain] Selected " << selectedIndices.size()
                  << " / " << matches.size() << " features" << std::endl;
    }

    return selectedIndices;
}

} // namespace ORB_SLAM3
