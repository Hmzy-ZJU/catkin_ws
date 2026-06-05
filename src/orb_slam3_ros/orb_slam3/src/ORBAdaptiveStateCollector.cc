/**
 * ORBAdaptiveStateCollector.cc
 * ORB-SLAM3 measurements used by the portable AdaptiveIDVO policy core.
 */

#include "ORBAdaptiveStateCollector.h"

#include "Frame.h"
#include "InfoGain.h"
#include "MapPoint.h"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>

namespace ORB_SLAM3
{

namespace
{

int ClampInt(int value, int lo, int hi)
{
    return std::max(lo, std::min(hi, value));
}

double Median(std::vector<double>& values)
{
    if(values.empty())
        return 0.0;

    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double median = values[mid];
    if(values.size() % 2 == 0)
    {
        std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
        median = 0.5 * (median + values[mid - 1]);
    }
    return median;
}

cv::Mat ToGray(const cv::Mat& image)
{
    if(image.empty() || image.channels() == 1)
        return image;

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

double ComputeContrast(const cv::Mat& image)
{
    const cv::Mat gray = ToGray(image);
    if(gray.empty())
        return 0.0;

    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);
    return stddev[0];
}

double ComputeBlurScore(const cv::Mat& image)
{
    const cv::Mat gray = ToGray(image);
    if(gray.empty())
        return 0.0;

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    return stddev[0] * stddev[0];
}

double ComputeSpatialEntropy(const Frame& frame, int grid_cols = 4, int grid_rows = 4)
{
    if(frame.mvKeysUn.empty() || grid_cols <= 0 || grid_rows <= 0)
        return 0.0;

    const double min_x = Frame::mnMinX;
    const double min_y = Frame::mnMinY;
    const double width = std::max(1.0, static_cast<double>(Frame::mnMaxX - Frame::mnMinX));
    const double height = std::max(1.0, static_cast<double>(Frame::mnMaxY - Frame::mnMinY));

    std::vector<int> bins(grid_cols * grid_rows, 0);
    for(const cv::KeyPoint& keypoint : frame.mvKeysUn)
    {
        int col = static_cast<int>(grid_cols * (keypoint.pt.x - min_x) / width);
        int row = static_cast<int>(grid_rows * (keypoint.pt.y - min_y) / height);
        col = ClampInt(col, 0, grid_cols - 1);
        row = ClampInt(row, 0, grid_rows - 1);
        ++bins[row * grid_cols + col];
    }

    const double total = static_cast<double>(frame.mvKeysUn.size());
    double entropy = 0.0;
    for(int count : bins)
    {
        if(count <= 0)
            continue;
        const double probability = static_cast<double>(count) / total;
        entropy -= probability * std::log(probability);
    }

    const double max_entropy = std::log(static_cast<double>(bins.size()));
    return max_entropy > 0.0 ? entropy / max_entropy : 0.0;
}

double ComputeMedianReprojectionError(Frame& frame)
{
    if(!frame.HasPose() || !frame.mpCamera)
        return 0.0;

    const Sophus::SE3f Tcw = frame.GetPose();
    std::vector<double> errors;
    errors.reserve(frame.N);
    for(int i = 0; i < frame.N; ++i)
    {
        MapPoint* map_point = frame.mvpMapPoints[i];
        if(!map_point || map_point->isBad() || frame.mvbOutlier[i] ||
           i >= static_cast<int>(frame.mvKeysUn.size()))
            continue;

        const Eigen::Vector3f point_camera = Tcw * map_point->GetWorldPos();
        if(point_camera[2] <= 0.0f)
            continue;

        const Eigen::Vector2f projected = frame.mpCamera->project(point_camera);
        const cv::Point2f observed = frame.mvKeysUn[i].pt;
        const double dx = static_cast<double>(projected[0]) - observed.x;
        const double dy = static_cast<double>(projected[1]) - observed.y;
        errors.push_back(std::sqrt(dx * dx + dy * dy));
    }
    return Median(errors);
}

int CountTrackedMapPoints(Frame& frame)
{
    int count = 0;
    for(int i = 0; i < frame.N; ++i)
    {
        MapPoint* map_point = frame.mvpMapPoints[i];
        if(map_point && !map_point->isBad() && !frame.mvbOutlier[i])
            ++count;
    }
    return count;
}

} // namespace

double ORBAdaptiveStateCollector::SafeLogDetNatural(
    const Eigen::Matrix<double,6,6>& H,
    double lambda)
{
    Eigen::Matrix<double,6,6> regularized =
        H + lambda * Eigen::Matrix<double,6,6>::Identity();
    regularized = 0.5 * (regularized + regularized.transpose());

    Eigen::LLT<Eigen::Matrix<double,6,6>> llt(regularized);
    if(llt.info() != Eigen::Success)
        return -1.0e9;

    const Eigen::Matrix<double,6,6> lower = llt.matrixL();
    double log_det = 0.0;
    for(int i = 0; i < 6; ++i)
    {
        if(lower(i, i) <= 0.0)
            return -1.0e9;
        log_det += std::log(lower(i, i));
    }
    return 2.0 * log_det;
}

void ORBAdaptiveStateCollector::UpdateFIMMetrics(
    AdaptiveState& state,
    const Eigen::Matrix<double,6,6>& H,
    const Eigen::Matrix<double,6,6>& H_ref,
    bool has_ref,
    double lambda)
{
    const Eigen::Matrix<double,6,6> regularized =
        0.5 * (H + H.transpose()) + lambda * Eigen::Matrix<double,6,6>::Identity();
    state.logdet_H = SafeLogDetNatural(H, lambda);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> solver(regularized);
    if(solver.info() == Eigen::Success)
    {
        const Eigen::Matrix<double,6,1> eigenvalues = solver.eigenvalues();
        state.min_eigenvalue_H = eigenvalues.minCoeff();
        state.condition_number_H = state.min_eigenvalue_H > 1.0e-12
            ? eigenvalues.maxCoeff() / state.min_eigenvalue_H
            : 1.0e12;
    }
    else
    {
        state.min_eigenvalue_H = 0.0;
        state.condition_number_H = 1.0e12;
    }

    if(has_ref)
    {
        const double current = SafeLogDetNatural(H, lambda);
        const double reference = SafeLogDetNatural(H_ref, lambda);
        state.information_variation_delta_E = current > -1.0e8 && reference > -1.0e8
            ? 0.5 * (current - reference) / std::log(2.0)
            : -1.0e9;
    }
}

AdaptiveState ORBAdaptiveStateCollector::Collect(
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
    int keyframe_interval)
{
    AdaptiveState state;
    state.frame_id = frame.mnId;
    state.timestamp = frame.mTimeStamp;
    state.candidate_point_number = candidate_count;
    state.selected_point_number = selected_count;
    state.selection_ratio = candidate_count > 0
        ? static_cast<double>(selected_count) / candidate_count
        : 0.0;
    state.cumulative_degradation_D_t = cumulative_degradation;
    state.tracking_time_ms = tracking_time_ms;
    state.recent_local_ba_time_ms = recent_local_ba_time_ms;
    state.number_of_keyframes = number_of_keyframes;
    state.number_of_map_points = number_of_map_points;
    state.keyframe_interval = keyframe_interval;
    state.tracked_map_points = CountTrackedMapPoints(frame);
    state.inlier_ratio = candidate_count > 0
        ? static_cast<double>(inlier_count) / candidate_count
        : 0.0;
    state.image_contrast = ComputeContrast(image);
    state.blur_score = ComputeBlurScore(image);
    state.feature_spatial_entropy = ComputeSpatialEntropy(frame);
    state.median_reprojection_error = ComputeMedianReprojectionError(frame);

    const Eigen::Matrix<double,6,6> H =
        InfoGain::ComputePoseInformation(frame, fim_indices, lambda);
    UpdateFIMMetrics(state, H, H_ref, has_ref, lambda);
    return state;
}

} // namespace ORB_SLAM3
