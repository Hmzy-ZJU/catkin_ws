# AIDVO ORB-SLAM3 与 AQUA-SLAM 实验计划

## 当前结论确认

根据目前 EuRoC、Harbor/AQUALOC 和 Tank 的测试结果，可以把 ORB-SLAM3 上的 AIDVO 验证边界明确为：

1. EuRoC 的单目、双目、单目惯性、双目惯性四类模式目前都可以正常测试和输出。轨迹完整性和轨迹形状基本可比，可以用于评估 `off / fixed / rule` 三种 AIDVO 消融策略。
2. Harbor/AQUALOC 只有单目模式目前适合作为 ORB-SLAM3 上的有效策略评估。单目惯性轨迹完整性和形状较差，不建议作为主实验依据。Harbor 本身没有双目话题，因此双目和双目惯性不应列入 ORB-SLAM3 主实验。
3. Tank 更难、更接近水下真实退化场景。当前 ORB-SLAM3 在 Tank 的水下惯性重初始化、多子地图恢复、低纹理和强旋转片段中表现不稳定。Tank 可作为失败分析和 AQUA-SLAM 迁移动机，不建议作为 ORB-SLAM3 主实验的核心结论来源。
4. 因此，AIDVO 在 ORB-SLAM3 上可以体现策略有效性，但主要适用于稳定可评估的数据组合。水下困难场景的失败更多反映 ORB-SLAM3 作为通用视觉/视觉惯性 SLAM baseline 的上限，而不是 AIDVO 策略本身的充分否定。

## 推荐实验分层

### 论文主实验：离线直接读取

主实验建议采用直接读取图像、IMU 和时间戳的离线方式，而不是 ROS 在线 rosbag 播放。原因：

- 排除 ROS 播放速率、callback 队列、RViz、服务保存超时和机器负载对结果的影响。
- 更容易保证每次运行使用完全一致的帧序列和时间戳。
- 更适合报告 ATE、RPE、轨迹完整性、输出频率、关键帧数量、选点数量和 AIDVO 参数变化。

当前脚本入口：

```bash
bash test_aidvo_orb3_offline_main_20260608.sh
```

该脚本会先把 rosbag 抽取成 EuRoC-like 离线目录，再调用 ORB-SLAM3 direct-reader example。输出目录为：

```text
results/aidvo_offline_main_<RUN_TAG>/offline_summary.csv
dataset_<name>/results/aidvo_offline_<RUN_TAG>/
offline_euroc_cache/<dataset>/<sequence>/
```

推荐先做短测试：

```bash
cd ~/catkin_ws; source /opt/ros/noetic/setup.bash; DO_BUILD=1 RUN_ALL_BAGS=0 BAG_DURATION=60 RUNS_PER_CASE=1 ONLY_DATASET=tank ONLY_SENSOR=stereo AIDVO_MODES=off,rule bash test_aidvo_orb3_offline_main_20260608.sh
```

确认离线抽取和 direct-reader 可用后，再跑 `RUN_PROFILE=full` 的完整离线矩阵。Tank/Harbor 的结果仍需谨慎解释：离线方式可以排除 ROS 播放干扰，但不能消除 ORB-SLAM3 在水下困难场景中的初始化和重定位弱点。

### 工程部署验证：ROS 在线测试

ROS 在线测试保留为部署验证，验证 AIDVO 在 ORB-SLAM3 ROS wrapper 中能否正常运行、记录日志、生成轨迹和评估文件。

当前推荐入口：

```bash
bash test_aidvo_orb3_ros_deploy_20260608.sh
```

默认 `RUN_PROFILE=meaningful`，只跑当前适合作为 ORB-SLAM3 策略评估的组合：

- EuRoC mono
- EuRoC stereo
- EuRoC mono-inertial
- EuRoC stereo-inertial
- Harbor mono

如果需要完整工程压力测试：

```bash
RUN_PROFILE=full bash test_aidvo_orb3_ros_deploy_20260608.sh
```

完整测试结果可用于部署稳定性分析，但不建议把 Tank/Harbor 惯性失败直接作为 AIDVO 主结论。

## ORB-SLAM3 主实验建议矩阵

| 数据集 | 模式 | 是否进入 ORB3 主实验 | 原因 |
| --- | --- | --- | --- |
| EuRoC | mono | 是 | 轨迹完整，可评估策略 |
| EuRoC | stereo | 是 | 轨迹完整，可评估策略 |
| EuRoC | mono-inertial | 是 | 轨迹完整，可评估策略 |
| EuRoC | stereo-inertial | 是 | 轨迹完整，可评估策略 |
| Harbor | mono | 是 | 水下单目可用，可评估策略 |
| Harbor | mono-inertial | 否 | 当前 ORB3 轨迹完整性差 |
| Harbor | stereo | 否 | 数据集无双目话题 |
| Harbor | stereo-inertial | 否 | 数据集无双目话题 |
| Tank | mono | 诊断 | 可作为困难水下场景补充，不作为主结论 |
| Tank | stereo | 诊断 | 可作为工程验证，不作为主结论 |
| Tank | mono-inertial | 否 | ORB3 水下重初始化失败明显 |
| Tank | stereo-inertial | 否 | ORB3 水下惯性稳定性不足 |

## AQUA-SLAM 迁移方案

### 迁移动机

AQUA-SLAM 是基于 ORB-SLAM3 改造的水下 acoustic-visual-inertial SLAM 系统，融合 stereo camera、IMU 和 DVL，并针对水下传感器标定和退化视觉场景做了系统升级。AIDVO 迁移到 AQUA-SLAM 后，可以验证：

- AIDVO 作为前端信息预算调节策略，在水下专用 SLAM 框架中是否带来更稳定收益。
- FIM-based IDPS/IDKD 是否能和 DVL/IMU/视觉紧耦合后端互补。
- 规则策略或 learned policy 是否能根据视觉退化、FIM 状态和计算负载动态调整前端预算。

### 迁移模块边界

可直接迁移：

- `AdaptiveIDVO.h/.cc`
- `AdaptiveState`
- `AdaptiveParams`
- `AdaptivePolicy`
- `RuleBasedAdaptivePolicy`
- `AdaptiveLogger`

需要重写适配层：

- `ORBAdaptiveStateCollector`
- IDPS 候选点入口
- IDKD 关键帧判定入口
- AQUA-SLAM 中 tracking/local mapping 的日志输出位置

### 迁移步骤

1. 在 AQUA-SLAM 工程中建立 `aidvo/` 或 `include/aidvo/` 模块目录，复制 AIDVO 策略核心代码。
2. 找到 AQUA-SLAM 的 ORB-SLAM3 tracking 前端入口，确认其 Frame、MapPoint、KeyFrame 是否仍保持 ORB-SLAM3 数据结构。
3. 将 IDPS 接入 pose optimization 前的候选 map point matches 过滤阶段，保持原始 FIM point scoring、uniformity scoring 和 greedy selection。
4. 将 IDKD 接入 NeedNewKeyFrame 或等价关键帧插入判断前，保持 FIM information variation、adaptive threshold 和 cumulative degradation 逻辑。
5. 实现 `AquaAdaptiveStateCollector`，采集：
   - FIM 指标：logdet、lambda_min、condition number、delta_E、D_t
   - tracking 指标：inlier ratio、tracked map points、median reprojection error
   - 图像质量：contrast、blur、spatial entropy
   - 计算负载：tracking time、local BA time
   - 水下传感器状态：DVL availability、DVL residual、IMU init state、depth/pressure availability
6. 在 AQUA-SLAM 配置文件中加入与 ORB-SLAM3 版本一致的开关：
   - `EnableAdaptiveIDVO`
   - `AdaptivePolicyType`
   - `MinKappaTop / MaxKappaTop`
   - `MinTau0 / MaxTau0`
   - `TrackingTimeBudget`
   - `SmoothFactor`
   - `EnableAdaptiveLogging`
7. 先跑 `Fixed` 与 `RuleBased`，确认：
   - `off` 能退化为 AQUA-SLAM 原始前端。
   - `fixed` 等价固定参数 IDVO。
   - `rule` 的 `kappa_top / tau0 / alpha` 有动态变化。
8. 使用 Tank 全序列作为 AQUA-SLAM 主水下验证集，优先测试 stereo-inertial/acoustic-visual-inertial 配置。
9. 将日志作为 offline dataset，后续训练 contextual bandit 或 DQN policy，再替换 rule-based policy。

### 论文表达建议

ORB-SLAM3 实验用于证明 AIDVO 的可迁移性和策略接口有效性：

```text
We first integrate AIDVO into ORB-SLAM3 to evaluate the generality of adaptive information regulation in a standard visual and visual-inertial SLAM baseline.
```

AQUA-SLAM 实验用于证明水下专用框架中的实际收益：

```text
We further port AIDVO to AQUA-SLAM, an underwater acoustic-visual-inertial SLAM system built upon ORB-SLAM3, to evaluate the proposed strategy under challenging underwater sensing conditions.
```

这两个实验层次分开后，ORB-SLAM3 在 Tank/Harbor 惯性模式下的失败不会削弱 AIDVO 主贡献，反而可以自然引出 AQUA-SLAM 迁移实验的必要性。
