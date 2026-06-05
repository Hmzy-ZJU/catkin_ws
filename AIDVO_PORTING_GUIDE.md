# AIDVO 迁移说明

## 模块边界

- `AdaptiveIDVO.h/.cc`：可移植核心，只包含状态、参数、策略、平滑、限幅和 CSV 日志，不依赖 ORB-SLAM3、OpenCV 或 MapPoint。
- `ORBAdaptiveStateCollector.h/.cc`：ORB-SLAM3 适配层，负责从 `Frame`、图像和 pose FIM 构造 `AdaptiveState`。
- `Tracking.cc`：集成层，在 IDPS 前调用策略，在 IDKD 前读取动态参数，跟踪结束后写日志。

## 迁移到其他 VO/SLAM 项目

1. 复制 `AdaptiveIDVO.h/.cc` 到目标项目。
2. 实现项目自己的状态采集适配器，填充 `AdaptiveState`。不需要修改策略核心。
3. 在点选择前调用 `AdaptivePolicy::Decide()`，将 `kappa_top` 和 `alpha` 传给原点选择模块。
4. 在关键帧判断前将 `tau0` 和 `theta_drop` 传给原关键帧模块。
5. 在每帧结束后调用 `AdaptiveLogger::Log()`。
6. Learned policy 后续实现同一 `AdaptivePolicy` 接口即可，现有 IDPS/IDKD 无需改动。

## 兼容要求

- `EnableAdaptiveIDVO=false`：使用原固定 IDPS/IDKD 参数。
- `AdaptivePolicyType=Fixed`：经过策略接口但输出固定参数。
- 非惯性模式：地图和当前帧有效后即可运行。
- 惯性模式：建议等待 IMU 初始化和惯性 BA 稳定后再运行。
