#!/bin/bash
set -euo pipefail

# ====== 可按需修改的参数 ======
WS=~/catkin_ws
DATA_DIR=/home/hmzy/catkin_ws/datasets
SEQ=MH_01_easy                         # EuRoC 序列名（不带扩展名）
BAG="${DATA_DIR}/${SEQ}.bag"
TRAJ_TAG="mh01"                        # 传给 /orb_slam3/save_traj 的文件名前缀
ROS_HOME_DIR="${ROS_HOME:-$HOME/.ros}" # 轨迹初始保存位置（由包决定）
# =================================

# 第一个终端：启动 ROS launch
gnome-terminal -- bash -c "
cd ${WS};
source ${WS}/devel/setup.bash;
roslaunch orb_slam3_ros euroc_mono.launch;
exec bash"

# 等待 10 秒再启动第二个终端（保持你原脚本设置）
sleep 10

# 第二个终端：播放 rosbag，结束后保存轨迹并提示
gnome-terminal -- bash -c "
set -euo pipefail
source ${WS}/devel/setup.bash
cd ${DATA_DIR}
rosparam set use_sim_time true
# 播放数据
rosbag play ${BAG} --clock -r 0.8

echo '[INFO] rosbag 播放结束，准备保存轨迹…'
sleep 5  # 给 SLAM 一点时间写完最后状态

# 保存相机/关键帧轨迹到 ROS_HOME（由包提供的服务）
# 会生成：~/.ros/\${TRAJ_TAG}_cam_traj.txt 和 \${TRAJ_TAG}_kf_traj.txt
rosservice call /orb_slam3/save_traj ${TRAJ_TAG}

# 拷贝到 datasets，并按序列名重命名
mkdir -p ${DATA_DIR}
cp '${ROS_HOME_DIR}'/${TRAJ_TAG}_cam_traj.txt '${DATA_DIR}'/${SEQ}_cam_traj.txt
cp '${ROS_HOME_DIR}'/${TRAJ_TAG}_kf_traj.txt  '${DATA_DIR}'/${SEQ}_kf_traj.txt
echo '[INFO] 轨迹已保存到: ${DATA_DIR}/${SEQ}_cam_traj.txt / ${DATA_DIR}/${SEQ}_kf_traj.txt'

# 提示保存完成
echo '[INFO] 保存轨迹完成！'
# =================================

# 可选：结束后关闭 SLAM 节点
rosnode kill /orb_slam3 || true

exec bash"

