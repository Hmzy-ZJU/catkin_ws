#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace uwfusion {

// ROS 可配置参数
struct Params {
  bool   enable            = false; // 开关：false=关闭增强，true=启用增强
  int    levels            = 5;     // 金字塔层数（建议3~5）
  // 简化颜色平衡（SCB）参数：截断百分比，左右各 p%
  double scb_percent       = 1.0;   // 0.5~2.0 常见
  // 双边滤波（用在 L 通道，Lab 浮点域）
  int    bilateral_d       = 5;
  double bilateral_sigma_color = 5.0;  // 注意：这里 L∈[0,100] 的域
  double bilateral_sigma_space = 5.0;
  // CLAHE（在 8bit L 通道上）
  double clahe_clip        = 2.0;
  int    clahe_tile        = 8;        // 8×8 tile
  // 显著性与曝光度
  double exposed_avg       = 0.5;  // 目标中灰
  double exposed_sigma     = 0.25; // 曝光度 σ
};

// 入口：给定 BGR8 输入，输出增强后的 BGR8（尺寸不变）
void EnhanceFusionBGR(const cv::Mat& bgr8_in, cv::Mat& bgr8_out, const Params& P);

} // namespace uwfusion
