# 原始项目功能回归测试文档

本文档用于重新熟悉当前 `catkin_ws-main` ROS 工作空间，并在运行 UW-IDVO 四模式实验前，先确认原有 ORB-SLAM3-ROS、UW-Fusion、UW-IDVO 固定参数功能是否正常。

注意：当前论文主线已经收敛为 `UW-IDVO = IDPS + IDKD`。为了回归测试固定参数版功能，配置中必须保持：

```yaml
EnableAdaptiveIDVO: 0
AdaptivePolicyType: "Fixed"
EnableAdaptiveLogging: 0
```

主配置文件 [Aqualoc_harbor.yaml](D:/home/catkin_ws-main/src/orb_slam3_ros/config/Monocular/Aqualoc_harbor.yaml) 已按这个状态写入。

## 1. 工作空间组成

根目录：`D:/home/catkin_ws-main`

| 路径 | 作用 |
|---|---|
| `src/orb_slam3_ros` | 主 SLAM ROS 包，包含 ORB-SLAM3 core、ROS 节点、launch、配置、UW-Fusion、UW-IDVO |
| `src/hmzy_tools` | 辅助工具包，目前主要是 rosbag/IMU 同步修复脚本 |
| `Parameters_test` | 批量参数实验脚本，包含增强参数、点选择参数、关键帧参数扫描 |
| `plot_traj_ate.py` | 轨迹 ATE 绘图/评估脚本 |
| `run_orb.sh` | EuRoC mono 示例自动启动脚本，包含 roslaunch、rosbag play、保存轨迹 |
| `run_orb_data.sh` | EuRoC mono-inertial 示例自动启动脚本 |

## 2. `orb_slam3_ros` 包文件组成

### 2.1 ROS 包顶层

| 路径 | 作用 |
|---|---|
| `CMakeLists.txt` | catkin 构建入口，编译 ORB-SLAM3 core 动态库、ROS 节点、UW-Fusion |
| `package.xml` | ROS 依赖声明 |
| `README.md` | 原始 ORB-SLAM3-ROS 使用说明 |
| `Dependencies.md` | ORB-SLAM3 第三方依赖说明 |
| `srv/SaveMap.srv` | 保存地图/轨迹服务请求格式：`string name`，返回 `bool success` |

### 2.2 ROS 节点源码

| 文件 | 节点/用途 |
|---|---|
| `src/ros_mono.cc` | 单目 SLAM 节点 `ros_mono` |
| `src/ros_mono_inertial.cc` | 单目+IMU 节点 `ros_mono_inertial` |
| `src/ros_stereo.cc` | 双目 SLAM 节点 `ros_stereo` |
| `src/ros_stereo_inertial.cc` | 双目+IMU 节点 `ros_stereo_inertial` |
| `src/ros_rgbd.cc` | RGB-D 节点 `ros_rgbd` |
| `src/ros_rgbd_inertial.cc` | RGB-D+IMU 节点 `ros_rgbd_inertial` |
| `src/ros_stereo_minimal.cc` | 简化双目测试节点 `ros_stereo_minimal` |
| `src/common.cc` | 公共发布器、TF、点云、图像、保存地图/轨迹服务 |
| `src/uw_fusion.cc` | 水下图像增强模块 |
| `src/run_logger.cc` | 占位/日志相关源码 |

### 2.3 ORB-SLAM3 core

| 路径 | 作用 |
|---|---|
| `orb_slam3/include` | ORB-SLAM3 头文件，包括 `System`、`Tracking`、`Frame`、`Optimizer`、`InfoGain`、`InfoKFPolicy` |
| `orb_slam3/src` | ORB-SLAM3 核心实现，包括 Tracking、LocalMapping、LoopClosing、Optimizer 等 |
| `orb_slam3/Thirdparty/DBoW2` | 词袋库 |
| `orb_slam3/Thirdparty/g2o` | 图优化库 |
| `orb_slam3/Thirdparty/Sophus` | Lie group 位姿库 |
| `orb_slam3/Vocabulary/ORBvoc.txt.bin` | ORB 词典，运行必需 |

原固定参数 UW-IDVO 相关文件：

| 文件 | 作用 |
|---|---|
| `orb_slam3/include/InfoGain.h` / `src/InfoGain.cc` | IDPS：FIM-based informative point selection |
| `orb_slam3/include/InfoKFPolicy.h` / `src/InfoKFPolicy.cc` | IDKD：FIM 信息变化与累计退化关键帧门控 |
| `orb_slam3/include/Frame.h` | 保存 `mvInfoSelected`、`mvInfoCandidate` 等可视化标志 |
| `orb_slam3/src/FrameDrawer.cc` | 绘制 InfoSelector 候选点/选中点 |
| `orb_slam3/src/Tracking.cc` | IDPS/IDKD 接入 Tracking 流程的位置 |

### 2.4 配置目录

| 路径 | 作用 |
|---|---|
| `config/Monocular` | 单目配置，含 Aqualoc、EuRoC、KITTI、TUM 等 |
| `config/Monocular-Inertial` | 单目+IMU 配置 |
| `config/Stereo` | 双目配置 |
| `config/Stereo-Inertial` | 双目+IMU 配置 |
| `config/RGB-D` | RGB-D 配置 |
| `config/*.rviz` | RViz 可视化配置 |
| `config/Calibration` | 标定辅助文件 |

重点水下配置：

| 文件/目录 | 用途 |
|---|---|
| `config/Monocular/Aqualoc_harbor.yaml` | Harbor 单目主配置，含 UW-Fusion、InfoSelector、InfoKF 参数 |
| `config/Monocular/Aquacular_harbor/*` | Harbor 参数扫描配置 |
| `config/Monocular/Aquacular_archaeo/*` | Archaeo 参数扫描配置 |

### 2.5 launch 文件

常用入口：

| launch | 节点类型 | 典型数据 |
|---|---|---|
| `aqualoc_harbor_mono.launch` | `ros_mono` | Aqualoc harbor 单目 |
| `aqualoc_harbor_mono_off.launch` | `ros_mono` | Harbor 单目，关闭 UW-Fusion |
| `aqualoc_harbor_mono_inertial.launch` | `ros_mono_inertial` | Harbor 单目+IMU |
| `aqualoc_archaeo_mono.launch` | `ros_mono` | Aqualoc archaeo 单目 |
| `tank_mono.launch` | `ros_mono` | tank 单目，含压缩图像 republish |
| `tank_stereo.launch` | `ros_stereo` | tank 双目 |
| `tank_stereo_inertial.launch` | `ros_stereo_inertial` | tank 双目+IMU |
| `euroc_mono.launch` | `ros_mono` | EuRoC 单目 |
| `euroc_mono_inertial.launch` | `ros_mono_inertial` | EuRoC 单目+IMU |
| `euroc_stereo.launch` | `ros_stereo` | EuRoC 双目 |
| `euroc_stereo_inertial.launch` | `ros_stereo_inertial` | EuRoC 双目+IMU |
| `tum_rgbd.launch` | `ros_rgbd` | TUM RGB-D |
| `uw_mono.launch` | `ros_mono` | UW-Fusion 参数化单目入口 |

## 3. `hmzy_tools` 包文件组成

| 路径 | 作用 |
|---|---|
| `src/hmzy_tools/package.xml` | ROS 包声明 |
| `src/hmzy_tools/CMakeLists.txt` | catkin 构建入口 |
| `src/hmzy_tools/scripts/fix_rosbag_vio_sync.py` | 修复/检查 rosbag 中图像与 IMU 时间同步 |

## 4. `Parameters_test` 文件组成

| 路径 | 作用 |
|---|---|
| `Parameters_test/harbor_test/enh_*.sh` | UW-Fusion 参数扫描：SCB、CLAHE、bilateral、exposure 等 |
| `Parameters_test/harbor_test/point_*.sh` | InfoSelector 点选择参数扫描，主要是 `InfoSelector.TopK` |
| `Parameters_test/harbor_test/kf_*.sh` | InfoKF 关键帧参数扫描，主要是 `InfoKF.AllowBitsDrop` |
| `Parameters_test/harbor_test/new/*` | 新一组点选择/关键帧扫描脚本 |
| `Parameters_test/V-slam/mono/*` | 单目视觉 SLAM 批量实验脚本 |
| `Parameters_test/V-slam/stereo/*` | 双目视觉 SLAM 批量实验脚本 |
| `Parameters_test/VI-slam/*` | 视觉惯性 SLAM 批量实验脚本 |

## 5. 原始功能测试前准备

建议在 Ubuntu 20.04 + ROS Noetic 环境中执行。当前 Windows PowerShell 环境无法直接运行 `catkin_make` / `catkin build`。

### 5.1 环境检查

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
rosversion -d
which catkin_make
which catkin
python3 -c "import cv2; print(cv2.__version__)"
pkg-config --modversion eigen3
```

期望：

| 项 | 期望 |
|---|---|
| ROS | `noetic` |
| OpenCV | 4.x，项目 CMake 默认查找 OpenCV 4.2 |
| Eigen | 能被 pkg-config 或 CMake 找到 |
| Pangolin | 已安装，CMake 能找到 |

### 5.2 数据与词典检查

```bash
test -f ~/catkin_ws/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin && echo OK
ls ~/catkin_ws/datasets
```

若测试 EuRoC，至少准备：

```text
~/catkin_ws/datasets/MH_01_easy.bag
```

若测试 Aqualoc / harbor / tank，请确认 bag 中话题和对应 launch 的 remap 匹配。

## 6. 构建测试

### 6.1 清理旧构建缓存，可选

```bash
cd ~/catkin_ws
rm -rf build devel
```

### 6.2 构建

优先使用项目 README 推荐的 catkin build：

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

如果只安装了 `catkin_make`：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 6.3 构建结果检查

```bash
rospack find orb_slam3_ros
roscd orb_slam3_ros
ls devel/lib/orb_slam3_ros 2>/dev/null || ls ~/catkin_ws/devel/lib/orb_slam3_ros
```

期望能看到：

```text
ros_mono
ros_mono_inertial
ros_stereo
ros_stereo_inertial
ros_rgbd
ros_rgbd_inertial
ros_stereo_minimal
```

## 7. ROS 基础功能测试

### 7.1 启动 roscore

终端 1：

```bash
roscore
```

### 7.2 启动一个最小 SLAM 节点

终端 2：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch orb_slam3_ros euroc_mono.launch
```

期望：

| 检查项 | 期望 |
|---|---|
| 控制台 | 能加载 vocabulary 和 settings |
| 节点 | `/orb_slam3` 存在 |
| RViz | 能启动，未播放数据时无轨迹属正常 |

检查命令：

```bash
rosnode list
rosparam get /orb_slam3/voc_file
rosparam get /orb_slam3/settings_file
rosservice list | grep orb_slam3
rostopic list | grep orb_slam3
```

应至少看到服务：

```text
/orb_slam3/save_map
/orb_slam3/save_traj
```

应至少看到话题：

```text
/orb_slam3/camera_pose
/orb_slam3/tracking_image
/orb_slam3/tracked_points
/orb_slam3/all_points
/orb_slam3/kf_markers
```

IMU 模式还应看到：

```text
/orb_slam3/body_odom
```

## 8. EuRoC 单目回归测试

这是最小、最容易复现的通用测试。

终端 1：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch orb_slam3_ros euroc_mono.launch
```

终端 2：

```bash
cd ~/catkin_ws/datasets
source ~/catkin_ws/devel/setup.bash
rosparam set use_sim_time true
rosbag play MH_01_easy.bag --clock -r 0.8
```

运行中检查：

```bash
rostopic hz /orb_slam3/camera_pose
rostopic hz /orb_slam3/tracking_image
rostopic echo -n 1 /orb_slam3/camera_pose
rostopic echo -n 1 /orb_slam3/tracked_points
```

期望：

| 项 | 期望 |
|---|---|
| 控制台 | Tracking 状态从初始化进入 OK |
| RViz | camera pose、关键帧、地图点逐渐出现 |
| `/orb_slam3/camera_pose` | 持续发布 |
| `/orb_slam3/tracking_image` | 持续发布带特征点图像 |

保存轨迹：

```bash
rosservice call /orb_slam3/save_traj mh01
ls ~/.ros/mh01_cam_traj.txt ~/.ros/mh01_kf_traj.txt
```

## 9. EuRoC 单目+IMU 回归测试

终端 1：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch orb_slam3_ros euroc_mono_inertial.launch
```

终端 2：

```bash
cd ~/catkin_ws/datasets
source ~/catkin_ws/devel/setup.bash
rosparam set use_sim_time true
rosbag play MH_01_easy.bag --clock -r 0.8
```

额外检查：

```bash
rostopic hz /orb_slam3/body_odom
rostopic echo -n 1 /tf
```

期望：

| 项 | 期望 |
|---|---|
| `/orb_slam3/body_odom` | IMU 模式下持续发布 |
| TF | `world -> camera` 和 `world -> imu` 可见 |
| 控制台 | IMU 初始化后进入稳定 Tracking |

## 10. Aqualoc harbor 单目回归测试

用于测试水下单目、UW-Fusion、InfoSelector、InfoKF 固定参数版。

终端 1：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch orb_slam3_ros aqualoc_harbor_mono.launch
```

终端 2，根据实际 bag 路径调整：

```bash
cd ~/catkin_ws/datasets
source ~/catkin_ws/devel/setup.bash
rosparam set use_sim_time true
rosbag play YOUR_HARBOR.bag --clock -r 0.8
```

如果 bag 的图像话题不是 `/camera/image_raw`，用 launch 参数或 remap 修正：

```bash
roslaunch orb_slam3_ros aqualoc_harbor_mono.launch image_topic:=/your/image/topic
```

检查固定参数 UW-IDVO 是否启用：

```bash
grep -n "InfoSelector.Enable\|InfoSelector.TopK\|InfoKF.Use\|InfoKF.AllowBitsDrop\|EnableAdaptiveIDVO" \
  ~/catkin_ws/src/orb_slam3_ros/config/Monocular/Aqualoc_harbor.yaml
```

期望：

```text
InfoSelector.Enable: 1
InfoKF.Use: 1
EnableAdaptiveIDVO: 0
AdaptivePolicyType: "Fixed"
```

运行中观察控制台：

| 日志关键词 | 含义 |
|---|---|
| `[InfoGain] Selected ... features` | IDPS 正在进行 FIM-based 点选择 |
| `[InfoSel] (UNIFIED) Frame ... candidates ... selected ...` | 当前帧候选点/选中点数量 |
| `[InfoKF]` | IDKD 正在计算信息变化和关键帧门控 |
| `[TrackLocalMap] Frame ... 有效匹配点数` | 跟踪内点数量 |

功能期望：

| 项 | 期望 |
|---|---|
| UW-Fusion | 图像增强后 tracking image 视觉上更清晰 |
| InfoSelector | 候选点与选中点数量随帧更新 |
| InfoKF | 关键帧插入不再只由 ORB-SLAM3 原始规则决定 |
| 轨迹 | `/orb_slam3/camera_pose` 持续发布 |

## 11. UW-Fusion 开关对照测试

目标：确认水下图像增强模块仍然正常，且可以关闭回到原图像输入。

启动增强版：

```bash
roslaunch orb_slam3_ros aqualoc_harbor_mono.launch
```

启动关闭增强版：

```bash
roslaunch orb_slam3_ros aqualoc_harbor_mono_off.launch
```

对照检查：

```bash
rostopic echo -n 1 /orb_slam3/tracking_image/header
rostopic hz /orb_slam3/tracking_image
```

建议记录：

| 项 | 增强开启 | 增强关闭 |
|---|---|---|
| 初始化是否成功 |  |  |
| 平均内点数量 |  |  |
| 是否频繁 lost |  |  |
| 轨迹是否连续 |  |  |

## 12. InfoSelector 固定参数检查

目标：确认 IDPS 原始固定参数版功能完好。

配置检查：

```bash
grep -n "InfoSelector" ~/catkin_ws/src/orb_slam3_ros/config/Monocular/Aqualoc_harbor.yaml
```

关键参数：

| 参数 | 作用 |
|---|---|
| `InfoSelector.Enable` | 是否启用 IDPS |
| `InfoSelector.TopK` | 固定点预算 kappa_top |
| `InfoSelector.w_uniform` | 信息得分与均匀性得分权重 |
| `InfoSelector.MinPxDist` | 空间均匀性统计像素距离 |
| `InfoSelector.LambdaInit` | FIM 正则项 |

测试方法：

1. 保持 `InfoSelector.Enable: 1`，运行 harbor 单目。
2. 控制台应输出 `[InfoGain]` 与 `[InfoSel]`。
3. 临时改为 `InfoSelector.Enable: 0`，重新构建不是必须，重新 roslaunch 即可。
4. 再次运行时应不再输出 `[InfoGain]` 与 `[InfoSel]`。

记录：

| 配置 | 是否初始化 | 平均 selected | 平均 tracking FPS | 备注 |
|---|---|---|---|---|
| Enable=1 |  |  |  |  |
| Enable=0 |  |  |  |  |

## 13. InfoKF 固定参数检查

目标：确认 IDKD 原始固定参数版功能完好。

配置检查：

```bash
grep -n "InfoKF" ~/catkin_ws/src/orb_slam3_ros/config/Monocular/Aqualoc_harbor.yaml
```

关键参数：

| 参数 | 作用 |
|---|---|
| `InfoKF.Use` | 是否启用 IDKD |
| `InfoKF.AllowBitsDrop` | 信息变化基础阈值 tau0 |
| `InfoKF.LambdaMean` | FIM 均值/参考正则 |
| `InfoKF.Cum.Decay` | 累计退化衰减 |
| `InfoKF.Cum.Thr` | 累计退化阈值 theta_drop |
| `InfoKF.MaxFramesForce` | 最大帧间隔强制插入 |

测试方法：

1. 保持 `InfoKF.Use: 1`，运行 harbor 单目。
2. 控制台应输出 `[InfoKF] ΔE=... D_cum=... gap=...`。
3. 临时改为 `InfoKF.Use: 0`，重新 roslaunch。
4. 再次运行时应不再出现 InfoKF 门控日志，关键帧由原 ORB-SLAM3 规则主导。

记录：

| 配置 | KF 数量 | 是否频繁拒绝 KF | 是否 lost | 备注 |
|---|---|---|---|---|
| InfoKF.Use=1 |  |  |  |  |
| InfoKF.Use=0 |  |  |  |  |

## 14. 地图保存和加载服务测试

保存地图：

```bash
rosservice call /orb_slam3/save_map test_map
ls ~/.ros/test_map.osa
```

保存轨迹：

```bash
rosservice call /orb_slam3/save_traj test_run
ls ~/.ros/test_run_cam_traj.txt ~/.ros/test_run_kf_traj.txt
```

期望：

| 服务 | 期望 |
|---|---|
| `/orb_slam3/save_map` | 返回 `success: True` 并生成 `.osa` |
| `/orb_slam3/save_traj` | 返回 `success: True` 并生成 camera/keyframe 轨迹 |

## 15. 轨迹评估

如果有 ground truth，可用根目录脚本：

```bash
cd ~/catkin_ws
python3 plot_traj_ate.py --help
```

建议记录：

| 数据集 | 配置 | ATE | RPE | 跟踪是否 lost | 备注 |
|---|---|---|---|---|---|
| EuRoC MH_01 | mono |  |  |  |  |
| EuRoC MH_01 | mono-inertial |  |  |  |  |
| Harbor | UW-IDVO fixed |  |  |  |  |

## 16. 常见故障排查

| 现象 | 优先检查 |
|---|---|
| `roslaunch` 找不到包 | 是否 `source ~/catkin_ws/devel/setup.bash` |
| 找不到 vocabulary | `ORBvoc.txt.bin` 是否存在，launch 的 `voc_file` 是否正确 |
| OpenCV / Pangolin CMake 报错 | 系统依赖是否安装，OpenCV 版本是否满足 CMake |
| 无图像输入 | `rostopic list` 检查 bag 图像话题，确认 launch remap |
| `use_sim_time` 下时间不动 | rosbag 是否使用 `--clock` |
| 单目一直初始化失败 | 图像质量、运动视差、相机参数、topic 是否正确 |
| IMU 模式不初始化 | IMU topic、时间同步、外参、频率和噪声参数 |
| RViz 无轨迹 | 先看 `/orb_slam3/camera_pose` 是否发布，再看 frame id |
| 保存轨迹失败 | 是否已经完成初始化并有有效关键帧 |

## 17. 建议本轮执行顺序

1. 构建通过：`catkin build` 或 `catkin_make`。
2. ROS 基础检查：节点、topic、service 正常。
3. EuRoC mono 跑通并保存轨迹。
4. EuRoC mono-inertial 跑通并确认 `/orb_slam3/body_odom`。
5. Aqualoc harbor mono 跑通，确认 UW-Fusion、InfoSelector、InfoKF 日志。
6. 对照测试 `aqualoc_harbor_mono.launch` 与 `aqualoc_harbor_mono_off.launch`。
7. 临时开关 `InfoSelector.Enable` 和 `InfoKF.Use` 做原始模块消融。
8. 所有原功能确认后，再进入 UW-IDVO 四模式测试。

## 18. 测试记录模板

```text
日期：
机器/系统：
ROS：
OpenCV：
数据集：
launch：
配置文件：

构建结果：
启动结果：
输入话题：
输出话题：
是否初始化：
是否 lost：
平均 tracking hz：
关键帧数量：
地图点数量：
轨迹文件：
异常日志：
结论：
```
