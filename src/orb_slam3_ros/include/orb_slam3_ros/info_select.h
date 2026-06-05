#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>

// 前向声明，避免大范围 include 侵入
namespace ORB_SLAM3 { class Frame; }

namespace infoselect {

// 运行参数（ROS 传入）
struct Params {
  bool   enable        = false; // 开关
  int    max_points    = 500;   // 保留的点数上限（建议：300~800）
  float  radius_pix    = 12.f;  // 空间抑制半径（像素）
  float  w_response    = 1.0f;  // 角点响应权重（信息度）
  float  w_radius      = 0.5f;  // 分散项权重（多样性）
};

// 统计信息（用于 CSV）
struct Stats {
  int    num_candidates = 0;   // 原始 ORB 数
  int    num_selected   = 0;   // 筛选后数
  double ms_select      = 0.0; // 选择耗时
};

// 设置/读取参数
void SetParams(const Params& p);
const Params& GetParams();

// 获取最近一次筛选统计
const Stats& GetLastStats();

// 对当前帧做二次筛选（就地修改 Frame 的关键点与描述子）
void FilterFrame(ORB_SLAM3::Frame& F);

} // namespace infoselect
