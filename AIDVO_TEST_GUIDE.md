# UW-AIDVO 分模式测试与验收指南

本文用于逐项验证 UW-AIDVO 在以下四种 ORB-SLAM3 模式中的功能：

1. 单目（Monocular）
2. 双目（Stereo）
3. 单目惯性（Monocular-Inertial）
4. 双目惯性（Stereo-Inertial）

建议严格按照本文顺序测试。先完成一种模式的 `off -> fixed -> rule` 三组测试并保存结果，再进入下一种模式。

---

## 1. 测试目标

本次测试不仅检查程序能否运行，还要确认下列模块确实产生了作用。

| 模块 | 需要验证的内容 | 主要证据 |
|---|---|---|
| 配置开关 | AIDVO 可以关闭、固定运行或规则自适应运行 | `roslaunch.log` |
| AdaptiveStateCollector | 每帧采集 FIM、跟踪、图像质量和负载状态 | `adaptive_frames.csv` |
| RuleBasedAdaptivePolicy | 根据状态改变参数 | CSV 中参数随帧变化 |
| 参数平滑 | 参数不会在相邻帧剧烈跳变 | CSV 相邻行差值 |
| IDPS | 使用当前帧的 `kappa_top` 和 `alpha` 选点 | candidate/selected/ratio 和 InfoSel 日志 |
| IDKD | 使用当前帧的 `tau0` 和 `theta_drop` 决定关键帧 | delta_E、D_t、参数和插帧标志 |
| AdaptiveLogger | 每帧完整写出状态、参数和结果 | CSV 表头、行数和字段值 |
| 兼容回退 | 关闭 AIDVO 后保持固定参数 UW-IDVO 行为 | `off` 与 `fixed` 对比 |
| 四模式兼容 | 四种传感器入口均可启动并输出轨迹 | 节点、日志、轨迹文件 |

---

## 2. 三种运行状态

测试脚本通过 `AIDVO_MODE` 生成临时 YAML：

| `AIDVO_MODE` | YAML 等效设置 | 用途 |
|---|---|---|
| `off` | `EnableAdaptiveIDVO: 0`, `Fixed` | 验证关闭回退到原固定参数 UW-IDVO |
| `fixed` | `EnableAdaptiveIDVO: 1`, `Fixed` | 验证统一策略接口的固定参数路径 |
| `rule` | `EnableAdaptiveIDVO: 1`, `RuleBased` | 验证动态参数调节 |

注意：

- `EnableAdaptiveIDVO: 0` 只关闭自适应策略，不关闭原来的 IDPS/IDKD。
- IDPS 是否启用由 `InfoSelector.Enable` 控制。
- IDKD 是否启用由 `InfoKF.Use` 控制。
- `Learned` 目前只有接口，本轮不测试学习策略。

---

## 3. 测试前准备

### 3.1 环境

建议环境：Ubuntu 20.04、ROS Noetic、已安装 ORB-SLAM3 所需依赖。

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
```

确认工具存在：

```bash
command -v catkin_make
command -v roslaunch
command -v rosbag
command -v rosservice
python3 --version
```

五条命令都应正常输出。缺少任何一项都不要开始数据集测试。

### 3.2 编译 AIDVO

```bash
cd ~/catkin_ws
catkin_make -DORB3_USE_INFOSEL=ON
source devel/setup.bash
```

成功标准：

1. 编译结束没有 `error:`。
2. 编译输出包含 `Information-theoretic feature selection: ENABLED`。
3. 四个节点均存在：

```bash
test -x devel/lib/orb_slam3_ros/ros_mono
test -x devel/lib/orb_slam3_ros/ros_stereo
test -x devel/lib/orb_slam3_ros/ros_mono_inertial
test -x devel/lib/orb_slam3_ros/ros_stereo_inertial
echo $?
```

最后输出 `0` 表示四个节点都存在。

### 3.3 检查数据包话题

测试前先执行：

```bash
rosbag info /绝对路径/sequence.bag
```

需要确认时间长度不是 0，并且话题与对应模式匹配：

| 模式 | 需要的话题 |
|---|---|
| 单目 EuRoC | `/cam0/image_raw` |
| 双目 EuRoC | `/cam0/image_raw`, `/cam1/image_raw` |
| 单目惯性 EuRoC | `/cam0/image_raw`, `/imu0` |
| 双目惯性 EuRoC | `/cam0/image_raw`, `/cam1/image_raw`, `/imu0` |
| Tank | 参照对应 launch 中的相机和 IMU remap |

惯性 bag 必须包含连续、高频 IMU 数据，且建议完整播放以完成初始化。

---

## 4. 脚本参数与结果目录

公共驱动：

```text
Parameters_test/aidvo_smoke_common.sh
```

常用环境变量：

| 变量 | 默认值 | 说明 |
|---|---:|---|
| `WS` | `$HOME/catkin_ws` | ROS 工作空间 |
| `AIDVO_MODE` | `rule` | `off/fixed/rule` |
| `BAG_FILE` | 空 | 指定一条 bag 的绝对路径 |
| `BAG_DURATION` | `0` | 播放秒数，0 为完整播放 |
| `BAG_RATE` | `1.0` | rosbag 播放速度 |
| `RUN_ALL_BAGS` | `0` | 1 表示运行目录内全部 bag |
| `TRACKING_TIME_BUDGET` | `30.0` | 规则策略跟踪耗时预算，毫秒 |
| `SMOOTH_FACTOR` | `0.8` | 参数平滑系数 |
| `MIN_KAPPA_TOP` | `60` | 最小点预算 |
| `MAX_KAPPA_TOP` | `180` | 最大点预算 |
| `MIN_TAU0` | `0.1` | 最小信息变化阈值 |
| `MAX_TAU0` | `5.0` | 最大信息变化阈值 |

每次结果保存在：

```text
<数据集根目录>/aidvo_results/<传感器模式>/<数据集>/<off|fixed|rule>/<bag名称>/
```

关键文件：

```text
settings_<mode>.yaml     实际运行配置
roslaunch.log            节点启动与运行日志
adaptive_frames.csv      每帧自适应状态和参数
```

轨迹通过 `/orb_slam3/save_traj` 保存，通常位于 `$ROS_HOME`，默认是 `~/.ros`。

---

## 5. 每种模式的统一测试顺序

每种传感器模式均执行下面三轮。三轮必须使用同一条 bag、相同播放速度和相同播放时长。

### 第 1 轮：关闭自适应

```bash
AIDVO_MODE=off BAG_FILE=/绝对路径/sequence.bag bash <对应脚本>
```

成功标准：

- `roslaunch.log` 包含 `EnableAdaptiveIDVO: NO`。
- `AdaptivePolicyType: Fixed`。
- ORB-SLAM3 能完成跟踪并保存轨迹。
- CSV 中 `kappa_top/alpha/tau0/theta_drop` 保持固定。

### 第 2 轮：固定策略接口

```bash
AIDVO_MODE=fixed BAG_FILE=/绝对路径/sequence.bag bash <对应脚本>
```

成功标准：

- `roslaunch.log` 包含 `EnableAdaptiveIDVO: YES`。
- `AdaptivePolicyType: Fixed`。
- 参数仍保持固定。
- 与 `off` 相比，选点数量、关键帧数量和轨迹应大体一致。允许线程调度引起少量差异。

### 第 3 轮：规则策略

```bash
AIDVO_MODE=rule BAG_FILE=/绝对路径/sequence.bag bash <对应脚本>
```

成功标准：

- `roslaunch.log` 包含 `EnableAdaptiveIDVO: YES`。
- `AdaptivePolicyType: RuleBased`。
- CSV 中至少一个动态参数在部分帧发生变化。
- 参数始终位于配置上下界内，且相邻帧变化平滑。
- 系统仍能完成跟踪和轨迹保存。

---

## 6. 单目模式测试

### 6.1 推荐首测：EuRoC 单目

建议先使用 `MH_01`。设置实际 bag 路径：

```bash
cd ~/catkin_ws
source devel/setup.bash
export BAG=~/catkin_ws/dataset_EuRoc/data/MH_01.bag
```

依次运行：

```bash
AIDVO_MODE=off   BAG_FILE=$BAG bash Parameters_test/V-slam/mono/aidvo_euroc.sh
AIDVO_MODE=fixed BAG_FILE=$BAG bash Parameters_test/V-slam/mono/aidvo_euroc.sh
AIDVO_MODE=rule  BAG_FILE=$BAG bash Parameters_test/V-slam/mono/aidvo_euroc.sh
```

单目专项观察：

1. 初始化后能够建立地图，而不是长期停留在初始化状态。
2. `roslaunch.log` 中出现 `[InfoSel] (UNIFIED)`。
3. `candidates(before)` 大于 0。
4. `selected(after)` 大于 0 且不大于候选数。
5. rule 模式下选点预算可随状态变化。

单目通过标准：

- 节点不崩溃、不持续重置地图。
- `adaptive_frames.csv` 至少有几十行有效帧。
- FIM 和图像质量字段出现有效非零数据。
- 能生成相机轨迹。

### 6.2 其他单目数据集

```bash
bash Parameters_test/V-slam/mono/aidvo_archaeo.sh
bash Parameters_test/V-slam/mono/aidvo_cave.sh
bash Parameters_test/V-slam/mono/aidvo_harbor.sh
bash Parameters_test/V-slam/mono/aidvo_tank.sh
```

建议先设置 `BAG_FILE`，避免脚本默认选择到非目标序列。

---

## 7. 双目模式测试

### 7.1 推荐首测：EuRoC 双目

```bash
cd ~/catkin_ws
source devel/setup.bash
export BAG=~/catkin_ws/dataset_EuRoc/data/MH_01.bag

AIDVO_MODE=off   BAG_FILE=$BAG bash Parameters_test/V-slam/stereo/aidvo_euroc.sh
AIDVO_MODE=fixed BAG_FILE=$BAG bash Parameters_test/V-slam/stereo/aidvo_euroc.sh
AIDVO_MODE=rule  BAG_FILE=$BAG bash Parameters_test/V-slam/stereo/aidvo_euroc.sh
```

双目专项观察：

1. `/cam0/image_raw` 和 `/cam1/image_raw` 都有消息。
2. 节点类型是 `ros_stereo`。
3. 初始化速度通常应快于单目。
4. 双目深度点可以正常创建，地图点数量持续增长。
5. IDPS 只减少进入 pose optimization 的匹配，不应破坏双目建图。

运行时可在另一终端检查：

```bash
source ~/catkin_ws/devel/setup.bash
rosnode info /orb_slam3
rostopic hz /cam0/image_raw
rostopic hz /cam1/image_raw
```

双目通过标准：

- 左右图像频率正常且接近。
- `[InfoSel]` 日志持续出现。
- `number_of_map_points` 和 `number_of_keyframes` 不是长期为 0。
- `selected_point_number <= candidate_point_number`。
- 三种 AIDVO_MODE 均能保存轨迹。

### 7.2 Tank 双目

```bash
AIDVO_MODE=rule BAG_FILE=/绝对路径/Tank序列.bag \
  bash Parameters_test/V-slam/stereo/aidvo_tank.sh
```

Tank launch 包含压缩图像解压节点。若无图像，首先检查 bag 中的 compressed topic 是否与 launch 一致。

---

## 8. 单目惯性模式测试

### 8.1 推荐首测：EuRoC 单目惯性

惯性模式不要先用 30 秒截断测试。首次验收应完整播放 `MH_01`。

```bash
cd ~/catkin_ws
source devel/setup.bash
export BAG=~/catkin_ws/dataset_EuRoc/data/MH_01.bag

AIDVO_MODE=off   BAG_FILE=$BAG bash Parameters_test/VI-slam/mono-inertial/aidvo_euroc.sh
AIDVO_MODE=fixed BAG_FILE=$BAG bash Parameters_test/VI-slam/mono-inertial/aidvo_euroc.sh
AIDVO_MODE=rule  BAG_FILE=$BAG bash Parameters_test/VI-slam/mono-inertial/aidvo_euroc.sh
```

单目惯性专项观察：

1. `/cam0/image_raw` 和 `/imu0` 均有消息。
2. 节点类型是 `ros_mono_inertial`。
3. IMU 初始化前，AIDVO 信息筛选保持休眠。
4. IMU 初始化和第二次惯性 BA 完成后，`[InfoSel]` 开始出现。
5. 初始化过程中不能因为 IDPS 削减匹配而反复重置。

检查话题：

```bash
rostopic hz /cam0/image_raw
rostopic hz /imu0
```

单目惯性通过标准：

- IMU 频率明显高于相机频率。
- 系统能够完成惯性初始化。
- 初始化完成后 CSV 的 FIM 字段开始出现有效数据。
- rule 模式参数只在运行就绪后开始响应状态。
- 节点完整运行并输出轨迹。

若整个序列都没有 `[InfoSel]`，优先检查 IMU 初始化是否完成，而不是立即判断 AIDVO 失效。

### 8.2 Harbor 和 Tank 单目惯性

```bash
bash Parameters_test/VI-slam/mono-inertial/aidvo_harbor.sh
bash Parameters_test/VI-slam/mono-inertial/aidvo_tank.sh
```

必须先用 `rosbag info` 核对实际 IMU topic 与对应 launch 的 remap。

---

## 9. 双目惯性模式测试

### 9.1 推荐首测：EuRoC 双目惯性

```bash
cd ~/catkin_ws
source devel/setup.bash
export BAG=~/catkin_ws/dataset_EuRoc/data/MH_01.bag

AIDVO_MODE=off   BAG_FILE=$BAG bash Parameters_test/VI-slam/stereo-inertial/aidvo_euroc.sh
AIDVO_MODE=fixed BAG_FILE=$BAG bash Parameters_test/VI-slam/stereo-inertial/aidvo_euroc.sh
AIDVO_MODE=rule  BAG_FILE=$BAG bash Parameters_test/VI-slam/stereo-inertial/aidvo_euroc.sh
```

双目惯性专项观察：

1. 左图、右图和 IMU 三路输入都有稳定消息。
2. 节点类型是 `ros_stereo_inertial`。
3. AIDVO 在惯性初始化前不介入，在初始化稳定后自动启用。
4. 双目地图点创建、惯性优化和动态选点能够同时工作。

检查命令：

```bash
rostopic hz /cam0/image_raw
rostopic hz /cam1/image_raw
rostopic hz /imu0
rosnode info /orb_slam3
```

双目惯性通过标准：

- 三路输入频率正常。
- 惯性初始化完成。
- 初始化后出现连续的 InfoSel 和自适应 CSV 数据。
- 地图点、关键帧和轨迹均正常生成。
- rule 模式参数变化不会导致持续跟踪丢失或频繁重置。

### 9.2 Tank 双目惯性

```bash
AIDVO_MODE=rule BAG_FILE=/绝对路径/Tank序列.bag \
  bash Parameters_test/VI-slam/stereo-inertial/aidvo_tank.sh
```

Tank 双目惯性 launch 默认 IMU remap 为 `/rtimulib_node/imu`。必须确认 bag 中确实存在该话题，否则需要修改 launch remap。

---

## 10. 验证 AdaptiveStateCollector

找到本次运行的 CSV：

```bash
find ~/catkin_ws -path '*aidvo_results*' -name adaptive_frames.csv -print
export CSV=/找到的绝对路径/adaptive_frames.csv
```

执行完整性检查：

```bash
python3 - "$CSV" <<'PY'
import csv, math, sys

path = sys.argv[1]
required = [
    'frame_id', 'timestamp', 'candidate_point_number',
    'selected_point_number', 'selection_ratio', 'logdet_H',
    'min_eigenvalue_H', 'condition_number_H',
    'information_variation_delta_E', 'cumulative_degradation_D_t',
    'median_reprojection_error', 'inlier_ratio', 'tracked_map_points',
    'image_contrast', 'blur_score', 'feature_spatial_entropy',
    'tracking_time_ms', 'recent_local_ba_time_ms',
    'number_of_keyframes', 'number_of_map_points', 'keyframe_interval'
]
with open(path, newline='') as f:
    rows = list(csv.DictReader(f))
assert rows, 'CSV has no frame rows'
missing = [name for name in required if name not in rows[0]]
assert not missing, f'missing columns: {missing}'
finite_rows = 0
for row in rows:
    values = [float(row[name]) for name in required if name not in ('frame_id',)]
    if all(math.isfinite(v) for v in values):
        finite_rows += 1
assert finite_rows > 0, 'no finite state row'
assert any(float(r['image_contrast']) > 0 for r in rows), 'contrast was never collected'
assert any(float(r['blur_score']) > 0 for r in rows), 'blur score was never collected'
assert any(float(r['tracked_map_points']) > 0 for r in rows), 'no tracked map points'
assert any(float(r['condition_number_H']) > 0 for r in rows), 'no FIM metrics'
print(f'PASS: {len(rows)} rows, {finite_rows} finite rows')
PY
```

通过标准：脚本输出 `PASS`。惯性模式应在初始化完成后的帧中通过这些条件。

---

## 11. 验证 AdaptiveParams、规则策略和参数平滑

```bash
python3 - "$CSV" <<'PY'
import csv, sys

rows = list(csv.DictReader(open(sys.argv[1], newline='')))
names = ['kappa_top', 'alpha', 'tau0', 'theta_drop', 'keyframe_aggressiveness']
for name in names:
    values = [float(r[name]) for r in rows]
    print(name, 'min=', min(values), 'max=', max(values), 'unique=', len(set(values)))

kappa = [float(r['kappa_top']) for r in rows]
tau = [float(r['tau0']) for r in rows]
assert all(60 <= v <= 180 for v in kappa), 'kappa_top out of configured bounds'
assert all(0.1 <= v <= 5.0 for v in tau), 'tau0 out of configured bounds'
assert all(0 <= float(r['alpha']) <= 1 for r in rows), 'alpha out of bounds'

max_kappa_jump = max((abs(b-a) for a,b in zip(kappa, kappa[1:])), default=0)
max_tau_jump = max((abs(b-a) for a,b in zip(tau, tau[1:])), default=0)
print('max adjacent kappa jump:', max_kappa_jump)
print('max adjacent tau jump:', max_tau_jump)
PY
```

判定方法：

- `off`、`fixed`：各参数 `unique` 应为 1，或只存在初始化阶段的极少量差异。
- `rule`：至少一个参数的 `unique` 应大于 1。
- 参数必须在上下界内。
- `SmoothFactor=0.8` 时不应一帧直接从最小值跳到最大值。

如果普通序列中 rule 参数始终不变，可进行压力测试：

```bash
TRACKING_TIME_BUDGET=1 SMOOTH_FACTOR=0.5 AIDVO_MODE=rule \
  BAG_FILE=$BAG bash <对应脚本>
```

这会降低耗时预算并加快策略响应。压力测试中参数发生变化即可证明规则策略路径有效，但该配置不用于精度评价。

---

## 12. 验证 IDPS 动态点选择

检查 CSV 约束：

```bash
python3 - "$CSV" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1], newline='')))
valid = []
for r in rows:
    c = int(float(r['candidate_point_number']))
    s = int(float(r['selected_point_number']))
    ratio = float(r['selection_ratio'])
    if c > 0:
        assert 0 <= s <= c, (c, s)
        assert abs(ratio - s/c) < 1e-3, (ratio, s/c)
        valid.append((c, s, int(float(r['kappa_top']))))
assert valid, 'no valid IDPS frames'
assert any(s < c for c,s,k in valid), 'IDPS never removed a candidate point'
print('PASS: valid IDPS frames =', len(valid))
print('first samples:', valid[:10])
PY
```

同时检查运行日志：

```bash
grep -E '\[InfoSel\].*candidates.*selected' /结果目录/roslaunch.log | head -20
```

IDPS 成功标准：

1. 存在候选点大于 0 的帧。
2. 选中点不超过候选点。
3. 至少部分帧确实剔除了点。
4. rule 模式中 `kappa_top` 变化后，选点预算随之受到影响。
5. pose optimization 后仍有足够内点，系统没有因此持续丢失。

---

## 13. 验证 IDKD 动态关键帧判断

```bash
python3 - "$CSV" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1], newline='')))
inserted = [r for r in rows if int(float(r['keyframe_insertion_flag'])) == 1]
delta = [float(r['information_variation_delta_E']) for r in rows]
degradation = [float(r['cumulative_degradation_D_t']) for r in rows]
tau = [float(r['tau0']) for r in rows]
theta = [float(r['theta_drop']) for r in rows]
print('frames:', len(rows))
print('inserted keyframes:', len(inserted))
print('delta_E range:', min(delta), max(delta))
print('D_t range:', min(degradation), max(degradation))
print('tau0 range:', min(tau), max(tau))
print('theta_drop range:', min(theta), max(theta))
assert any(int(float(r['number_of_keyframes'])) > 0 for r in rows), 'no keyframes in map'
assert inserted, 'no keyframe insertion was recorded; use a longer bag'
print('PASS: IDKD data and insertion events are present')
PY
```

IDKD 成功标准：

1. `delta_E`、`D_t`、`tau0`、`theta_drop` 均被记录。
2. 完整序列中至少出现一次 `keyframe_insertion_flag=1`。
3. rule 模式下 `tau0` 或 `theta_drop` 能随状态变化。
4. 关键帧间隔不是每帧固定插入，也不是整个序列完全不插入。
5. off/fixed 模式保持原固定 IDKD 参数。

短时冒烟测试可能没有新关键帧，IDKD 验收必须使用足够长的序列。

---

## 14. 验证 AdaptiveLogger

```bash
head -2 "$CSV"
wc -l "$CSV"
tail -2 "$CSV"
```

成功标准：

- 第一行是完整 CSV 表头。
- 行数大于 2。
- `frame_id` 和 `timestamp` 随运行推进。
- 包含状态字段、参数字段、关键帧标志和跟踪丢失标志。
- 程序结束后文件可以正常打开，没有半行或明显截断。

日志行数不要求严格等于 rosbag 图像数，因为初始化、丢帧和提前返回会影响记录数量。

---

## 15. 验证关闭回退兼容性

对同一 bag 比较 `off` 和 `fixed`：

```bash
python3 - /off/adaptive_frames.csv /fixed/adaptive_frames.csv <<'PY'
import csv, sys

def values(path):
    rows = list(csv.DictReader(open(path, newline='')))
    names = ['kappa_top', 'alpha', 'tau0', 'theta_drop']
    return {n: sorted(set(r[n] for r in rows)) for n in names}

off = values(sys.argv[1])
fixed = values(sys.argv[2])
print('off  :', off)
print('fixed:', fixed)
assert off == fixed, 'off and fixed parameter outputs differ'
print('PASS: fixed-parameter compatibility confirmed')
PY
```

成功标准：

- 两者动态参数输出一致。
- 两者都能完成轨迹输出。
- 跟踪成功率、关键帧数和选点数在合理误差范围内接近。
- `off` 不应出现 RuleBased 参数变化。

由于 ROS 多线程调度，轨迹不要求逐字节相同。

---

## 16. 建议的逐项测试记录

每完成一项，在下面记录结果：

| 编号 | 模式 | 数据集/序列 | AIDVO_MODE | 编译/启动 | CSV | IDPS | IDKD | 轨迹 | 结论 |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 单目 | EuRoC MH_01 | off |  |  |  |  |  |  |
| 2 | 单目 | EuRoC MH_01 | fixed |  |  |  |  |  |  |
| 3 | 单目 | EuRoC MH_01 | rule |  |  |  |  |  |  |
| 4 | 双目 | EuRoC MH_01 | off |  |  |  |  |  |  |
| 5 | 双目 | EuRoC MH_01 | fixed |  |  |  |  |  |  |
| 6 | 双目 | EuRoC MH_01 | rule |  |  |  |  |  |  |
| 7 | 单目惯性 | EuRoC MH_01 | off |  |  |  |  |  |  |
| 8 | 单目惯性 | EuRoC MH_01 | fixed |  |  |  |  |  |  |
| 9 | 单目惯性 | EuRoC MH_01 | rule |  |  |  |  |  |  |
| 10 | 双目惯性 | EuRoC MH_01 | off |  |  |  |  |  |  |
| 11 | 双目惯性 | EuRoC MH_01 | fixed |  |  |  |  |  |  |
| 12 | 双目惯性 | EuRoC MH_01 | rule |  |  |  |  |  |  |

建议只有当前模式的三轮都通过后，才测试下一种模式。

---

## 17. 常见失败与定位

### 编译找不到 AIDVO 文件

确认 CMake 包含：

```text
orb_slam3/src/AdaptiveIDVO.cc
orb_slam3/src/ORBAdaptiveStateCollector.cc
```

### 日志显示 AIDVO 没有打开

检查本次结果目录中的 `settings_rule.yaml`，不要只检查源配置文件。

### 单目/双目没有 InfoSel 日志

检查：

1. `InfoSelector.Enable: 1`。
2. 当前帧已经有有效位姿。
3. 跟踪状态已经进入 `OK`。
4. local map matching 后存在候选 MapPoint。

### 惯性模式没有 InfoSel 日志

检查 IMU topic、频率、时间戳、相机-IMU标定以及 IMU 初始化是否完成。AIDVO 在初始化前休眠属于正常行为。

### CSV 为空

检查：

1. `EnableAdaptiveLogging: 1`。
2. `AdaptiveLogPath` 所在目录可写。
3. `roslaunch.log` 是否包含配置加载信息。
4. 节点是否在第一帧前就退出。

### rule 参数不变化

先完整播放退化较明显的序列；再使用 `TRACKING_TIME_BUDGET=1` 做策略压力测试。若状态明显变化而参数始终完全固定，再检查 `AdaptivePolicyType` 是否实际为 `RuleBased`。

### 参数变化后跟踪频繁丢失

先恢复默认范围：

```bash
MIN_KAPPA_TOP=60 MAX_KAPPA_TOP=180 \
MIN_TAU0=0.1 MAX_TAU0=5.0 \
SMOOTH_FACTOR=0.8 TRACKING_TIME_BUDGET=30 \
AIDVO_MODE=rule BAG_FILE=$BAG bash <对应脚本>
```

然后检查 `kappa_top` 是否过低、`tau0` 是否过小，以及原固定参数本身是否适合该数据集。
