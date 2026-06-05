#include "orb_slam3_ros/uw_fusion.h"
#include <algorithm>
#include <numeric>
#include <cmath>

namespace uwfusion {

// ------------------------ 小工具函数 ------------------------

static cv::Mat simpleColorBalance(const cv::Mat& bgr8, double percent) {
    // 按通道在低/高百分位上截断，并线性拉伸
    // percent 是左右各百分比（例如 1.0 表示两端各截掉 1%）
    CV_Assert(bgr8.type() == CV_8UC3);
    percent = std::max(0.0, std::min(percent, 10.0)); // 防止过大

    std::vector<cv::Mat> ch;
    cv::split(bgr8, ch);

    int histSize = 256;
    float range[] = {0.f, 256.f};
    const float* histRange = {range};

    for (int c = 0; c < 3; ++c) {
        cv::Mat hist;
        cv::calcHist(&ch[c], 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);

        // 累积分布
        std::vector<float> acc(256, 0.f);
        acc[0] = hist.at<float>(0);
        for (int i = 1; i < 256; ++i) acc[i] = acc[i - 1] + hist.at<float>(i);
        float total = acc[255];
        if (total <= 0.f) continue;

        float lowCount  = static_cast<float>(percent * 0.01 * total);
        float highCount = static_cast<float>((1.0 - percent * 0.01) * total);

        int low = 0;
        while (low < 255 && acc[low] < lowCount) ++low;
        int high = 255;
        while (high > 0 && acc[high] > highCount) --high;
        if (high <= low) { low = 0; high = 255; }

        // 线性拉伸到 [0,255]
        ch[c].forEach<uchar>([&](uchar& px, const int*) {
            int v = px;
            if (v < low) v = low;
            if (v > high) v = high;
            px = static_cast<uchar>( (v - low) * 255.0 / std::max(1, high - low) );
        });
    }

    cv::Mat out;
    cv::merge(ch, out);
    return out;
}

static cv::Mat computeSaliencyFT(const cv::Mat& bgr8) {
    // 频域调谐法近似：Lab 空间每像素到图像均值的距离
    CV_Assert(bgr8.type() == CV_8UC3);
    cv::Mat lab;
    cv::cvtColor(bgr8, lab, cv::COLOR_BGR2Lab);
    lab.convertTo(lab, CV_32F);

    std::vector<cv::Mat> ch;
    cv::split(lab, ch);

    // 轻微平滑
    for (int i = 0; i < 3; ++i)
        cv::GaussianBlur(ch[i], ch[i], cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT);

    cv::Scalar meanL = cv::mean(ch[0]);
    cv::Scalar meana = cv::mean(ch[1]);
    cv::Scalar meanb = cv::mean(ch[2]);

    cv::Mat sal;
    {
        cv::Mat dl = ch[0] - meanL[0];
        cv::Mat da = ch[1] - meana[0];
        cv::Mat db = ch[2] - meanb[0];
        sal = dl.mul(dl) + da.mul(da) + db.mul(db);
        cv::sqrt(sal, sal);
    }

    double minv, maxv;
    cv::minMaxLoc(sal, &minv, &maxv);
    if (maxv > 1e-6) sal = (sal - minv) / (maxv - minv);
    else sal.setTo(0);

    return sal; // CV_32F [0,1]
}

static void separableBinomial5x5(const cv::Mat& src, cv::Mat& dst) {
    // h = (1/16)[1,4,6,4,1]
    static const cv::Matx<float,1,5> k = (1.f/16.f) * cv::Matx<float,1,5>(1,4,6,4,1);
    cv::sepFilter2D(src, dst, CV_32F, k, k, cv::Point(-1,-1), 0, cv::BORDER_REFLECT101);
}

static cv::Mat laplacianContrast(const cv::Mat& grayFloat01) {
    // lap kernel [[0,1,0],[1,-4,1],[0,1,0]] / 8
    CV_Assert(grayFloat01.type() == CV_32F);
    static const cv::Mat kernel = (1.f/8.f) * (cv::Mat_<float>(3,3) <<
        0, 1, 0,
        1,-4, 1,
        0, 1, 0);
    cv::Mat lap;
    cv::filter2D(grayFloat01, lap, CV_32F, kernel, cv::Point(-1,-1), 0, cv::BORDER_REFLECT101);
    cv::Mat w = cv::abs(lap);
    return w; // CV_32F
}

static cv::Mat localContrast(const cv::Mat& grayFloat01) {
    CV_Assert(grayFloat01.type() == CV_32F);
    cv::Mat blurred;
    separableBinomial5x5(grayFloat01, blurred);
    // 论文实现中有个 "whc" 裁剪（高频阻断阈值）；这里延用你 Python 的值
    const float whc = static_cast<float>(M_PI / 2.75); // ≈1.142…，对 [0,1] 灰度来说基本无影响
    cv::Mat clipped;
    cv::min(blurred, whc, clipped);
    cv::Mat w = (grayFloat01 - clipped);
    w = w.mul(w);
    return w; // CV_32F
}

static void buildGaussianPyr(const cv::Mat& img, int levels, std::vector<cv::Mat>& pyr) {
    pyr.clear();
    pyr.reserve(std::max(1, levels));
    cv::Mat cur = img;
    pyr.push_back(cur);
    for (int i = 1; i < levels; ++i) {
        cv::Mat down;
        cv::pyrDown(cur, down);
        pyr.push_back(down);
        cur = down;
    }
}

static void buildLaplacianPyr(const cv::Mat& img, int levels, std::vector<cv::Mat>& lap) {
    std::vector<cv::Mat> gauss;
    buildGaussianPyr(img, levels, gauss);
    lap.resize(levels);
    for (int i = 0; i < levels - 1; ++i) {
        cv::Mat up;
        cv::pyrUp(gauss[i+1], up, gauss[i].size());
        lap[i] = gauss[i] - up;
    }
    lap[levels - 1] = gauss[levels - 1]; // 最底层直接拿 Gaussian
}

static cv::Mat reconstructFromLaplacian(const std::vector<cv::Mat>& lap) {
    cv::Mat cur = lap.back();
    for (int i = static_cast<int>(lap.size()) - 2; i >= 0; --i) {
        cv::Mat up;
        cv::pyrUp(cur, up, lap[i].size());
        cur = up + lap[i];
    }
    return cur;
}

static void normalize01(cv::Mat& f) {
    double minv, maxv;
    cv::minMaxLoc(f, &minv, &maxv);
    if (maxv > minv) f = (f - minv) / (maxv - minv);
    else f.setTo(0);
}

// ------------------------ 主流程 ------------------------

void EnhanceFusionBGR(const cv::Mat& bgr8_in, cv::Mat& bgr8_out, const Params& P)
{
    CV_Assert(!bgr8_in.empty() && bgr8_in.type() == CV_8UC3);

    if (!P.enable) {
        bgr8_out = bgr8_in.clone();
        return;
    }

    // -------- 输入图像1：SCB 颜色平衡 --------
    cv::Mat img1_bgr = simpleColorBalance(bgr8_in, P.scb_percent);

    // -------- 构造输入图像2：Lab 的 L 通道双边滤波 + CLAHE --------
    // OpenCV 的 Lab: L∈[0,255]，a,b~[0,255]（8bit表示）
    cv::Mat lab;
    cv::cvtColor(img1_bgr, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_ch;
    cv::split(lab, lab_ch); // L in [0..255] (8U)

    // 双边滤波在 8U 上即可；把 sigmaColor 从“[0..100]域”映射到“[0..255]域”
    double sigmaColor8 = P.bilateral_sigma_color * (255.0 / 100.0);
    cv::Mat L_bi;
    cv::bilateralFilter(lab_ch[0], L_bi, P.bilateral_d, sigmaColor8, P.bilateral_sigma_space, cv::BORDER_DEFAULT);

    // CLAHE on 8U L
    int tile = std::max(1, P.clahe_tile);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(P.clahe_clip, cv::Size(tile, tile));
    cv::Mat L_eq;
    clahe->apply(L_bi, L_eq);

    lab_ch[0] = L_eq;
    cv::merge(lab_ch, lab);
    cv::Mat img2_bgr;
    cv::cvtColor(lab, img2_bgr, cv::COLOR_Lab2BGR);

    // -------- 计算权重（使用 L 通道归一化到 [0,1]）--------
    // R1/R2：用两幅图各自的 L 通道
    cv::Mat lab1, lab2;
    cv::cvtColor(img1_bgr, lab1, cv::COLOR_BGR2Lab);
    cv::cvtColor(img2_bgr, lab2, cv::COLOR_BGR2Lab);

    std::vector<cv::Mat> ch1, ch2;
    cv::split(lab1, ch1);
    cv::split(lab2, ch2);

    cv::Mat R1f, R2f;
    ch1[0].convertTo(R1f, CV_32F, 1.0/255.0);
    ch2[0].convertTo(R2f, CV_32F, 1.0/255.0);

    // 1) Laplacian contrast
    cv::Mat WL1 = laplacianContrast(R1f);
    cv::Mat WL2 = laplacianContrast(R2f);

    // 2) Local contrast（5x5二项式核）
    cv::Mat WLC1 = localContrast(R1f);
    cv::Mat WLC2 = localContrast(R2f);

    // 3) Saliency（在 BGR 上做 Lab-FT）
    cv::Mat WS1 = computeSaliencyFT(img1_bgr);
    cv::Mat WS2 = computeSaliencyFT(img2_bgr);

    // 4) Exposedness（高斯型，中心=0.5）
    auto exposed = [&](const cv::Mat& R)->cv::Mat{
        cv::Mat diff = R - static_cast<float>(P.exposed_avg);
        cv::Mat w;
        float denom = 2.0f * static_cast<float>(P.exposed_sigma * P.exposed_sigma);
        cv::exp( (-diff.mul(diff)) / std::max(1e-6f, denom), w );
        return w;
    };
    cv::Mat WE1 = exposed(R1f);
    cv::Mat WE2 = exposed(R2f);

    // 归一化（避免除零）
    cv::Mat denom = WL1 + WLC1 + WS1 + WE1 + WL2 + WLC2 + WS2 + WE2;
    cv::Mat eps(denom.size(), denom.type(), cv::Scalar(1e-12));
    denom = denom + eps;

    cv::Mat W1 = (WL1 + WLC1 + WS1 + WE1) / denom;
    cv::Mat W2 = (WL2 + WLC2 + WS2 + WE2) / denom;

    // -------- 多分辨率融合（拉普拉斯金字塔）--------
    int levels = std::max(1, P.levels);

    // 把图像转为 [0,1] float
    cv::Mat I1f, I2f;
    img1_bgr.convertTo(I1f, CV_32F, 1.0/255.0);
    img2_bgr.convertTo(I2f, CV_32F, 1.0/255.0);

    // 权重金字塔（Gaussian）
    std::vector<cv::Mat> Gw1, Gw2;
    buildGaussianPyr(W1, levels, Gw1);
    buildGaussianPyr(W2, levels, Gw2);

    // 图像拉普拉斯金字塔（按通道）
    std::vector<cv::Mat> L1, L2;
    buildLaplacianPyr(I1f, levels, L1);
    buildLaplacianPyr(I2f, levels, L2);

    // 每层归一化权重并做加权
    std::vector<cv::Mat> Lblend(levels);
    for (int l = 0; l < levels; ++l) {
        cv::Mat w1 = Gw1[l], w2 = Gw2[l];
        // 确保尺寸与图像层一致
        if (w1.size() != L1[l].size())  cv::resize(w1, w1, L1[l].size(), 0, 0, cv::INTER_LINEAR);
        if (w2.size() != L2[l].size())  cv::resize(w2, w2, L2[l].size(), 0, 0, cv::INTER_LINEAR);

        cv::Mat wd = w1 + w2 + 1e-12f;
        w1 = w1 / wd;
        w2 = w2 / wd;

        // 将单通道权重扩展到 3 通道
        cv::Mat w1c, w2c;
        cv::Mat w1_merge[] = {w1, w1, w1};
        cv::Mat w2_merge[] = {w2, w2, w2};
        cv::merge(w1_merge, 3, w1c);
        cv::merge(w2_merge, 3, w2c);

        Lblend[l] = w1c.mul(L1[l]) + w2c.mul(L2[l]);
    }

    // 重建
    cv::Mat fused = reconstructFromLaplacian(Lblend);
    cv::Mat out8;
    fused = cv::min(cv::max(fused, 0.0f), 1.0f);
    fused.convertTo(out8, CV_8UC3, 255.0);

    bgr8_out = out8;
}

} // namespace uwfusion
