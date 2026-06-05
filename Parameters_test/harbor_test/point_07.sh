#!/usr/bin/env bash
set -euo pipefail

export MPLBACKEND=Agg

########################################
# 0. 基本路径配置（按需改）
########################################

WS="${WS:-$HOME/catkin_ws}"

# harbor 数据根目录
HARBOR_ROOT="${HARBOR_ROOT:-$WS/dataset_harbor}"
BAG_DIR="${BAG_DIR:-$HARBOR_ROOT/data}"
GT_DIR="${GT_DIR:-$HARBOR_ROOT/GT}"
RESULTS_ROOT="${RESULTS_ROOT:-$HARBOR_ROOT/results}"

# ROS 轨迹默认保存位置
ROS_HOME_DIR="${ROS_HOME:-$HOME/.ros}"

# 词典与 launch 文件
VOC_FILE="${VOC_FILE:-$WS/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin}"
LAUNCH_PKG="${LAUNCH_PKG:-orb_slam3_ros}"
LAUNCH_NAME="${LAUNCH_NAME:-aqualoc_harbor_mono.launch}"

# 8 套 YAML 所在目录
CONFIG_DIR="${CONFIG_DIR:-$WS/src/orb_slam3_ros/config/Monocular/Aquacular_harbor/point_3}"
# 8 套配置名字（必须与 .yaml 文件名对应）/home/hmzy/catkin_ws/src/orb_slam3_ros/config/Monocular/Aquacular_harbor/point_3/harbor_MinPxDist_2.yaml
CONFIG_NAMES=(
  "harbor_MinPxDist_1"
  "harbor_MinPxDist_3"
  "harbor_MinPxDist_5"
  "harbor_MinPxDist_7"
  "harbor_MinPxDist_9"

)

# 要跑的序列索引（1..7）
# SEQ_INDICES=(1 2 3 4 5 6 7)
SEQ_INDICES=(1 3)
# 每个 config+sequence 重复次数
RUNS_PER_SEQ="${RUNS_PER_SEQ:-3}"

########################################
# 1. 评估参数（ATE / RPE）
########################################

RPE_ENABLED="${RPE_ENABLED:-1}"    # 1=计算 RPE，0=只算 ATE
RPE_UNIT="${RPE_UNIT:-f}"         # f=按帧, m=按米
RPE_DELTA="${RPE_DELTA:-1}"       # RPE 间隔
FPS="${FPS:-20}"                  # GT 为帧索引时：t = frame_id / FPS
T_MAX_DIFF="${T_MAX_DIFF:-0.05}"  # 允许的时间对齐阈值（秒）

# rosbag 播放速率
BAG_RATE="${BAG_RATE:-1}"

########################################
# 2. 预检
########################################

command -v roslaunch >/dev/null 2>&1 || { echo "[ERR] 需要 roslaunch 在 PATH 中"; exit 1; }
command -v rosbag    >/dev/null 2>&1 || { echo "[ERR] 需要 rosbag 在 PATH 中"; exit 1; }

[[ -d "${WS}" ]]          || { echo "[ERR] 工作空间不存在: ${WS}"; exit 1; }
[[ -d "${HARBOR_ROOT}" ]] || { echo "[ERR] harbor 根目录不存在: ${HARBOR_ROOT}"; exit 1; }
[[ -d "${BAG_DIR}" ]]     || { echo "[ERR] 数据目录不存在: ${BAG_DIR}"; exit 1; }
[[ -d "${GT_DIR}" ]]      || { echo "[ERR] GT 目录不存在: ${GT_DIR}"; exit 1; }
[[ -d "${CONFIG_DIR}" ]]  || { echo "[ERR] YAML 目录不存在: ${CONFIG_DIR}"; exit 1; }
[[ -f "${VOC_FILE}" ]]    || { echo "[ERR] 词典不存在: ${VOC_FILE}"; exit 1; }

mkdir -p "${RESULTS_ROOT}"

# 原始每次运行的记录 CSV（带 run 列）
RAW_CSV="${RESULTS_ROOT}/harbor_eval_runs.csv"
echo "config,sequence,run,uw_tag,ate_rmse,ate_mean,ate_median,ate_std,rpe_rmse,rpe_mean,rpe_median,rpe_std,elapsed_sec,track_ms_mean,ba_ms_mean,enh_ms_mean,sel_pts_mean,num_kfs,mem_mb,completed" > "${RAW_CSV}"

# 最终“中位数”汇总表
SUMMARY_CSV="${RESULTS_ROOT}/harbor_eval_summary.csv"

# ROS 环境
source "${WS}/devel/setup.bash"

# 安装 / 检查 evo
python3 -m pip show evo >/dev/null 2>&1 || python3 -m pip install -U "evo>=1.25" --no-binary evo

########################################
# 3. 工具函数
########################################

# 从 evo zip 结果中提取指标（rmse/mean/median/std）
get_metric_from_zip() {
  local zipfile="$1"
  local metric="$2"   # rmse / mean / median / std ...

  if [[ ! -f "$zipfile" ]]; then
    echo ""
    return
  fi

  evo_res "$zipfile" --no_warnings -v 2>/dev/null | \
    awk -v m="$metric" '$1 == m {print $2; exit}'
}

# 计算某个 (config, sequence) 在 RAW_CSV 中某一列的中位数
# col_index 按 RAW_CSV 的列序号：
# 1=config 2=sequence 3=run 4=uw_tag
# 5=ate_rmse 6=ate_mean 7=ate_median 8=ate_std
# 9=rpe_rmse 10=rpe_mean 11=rpe_median 12=rpe_std
# 13=elapsed_sec
# 14=track_ms_mean 15=ba_ms_mean 16=enh_ms_mean
# 17=sel_pts_mean 18=num_kfs 19=mem_mb 20=completed
median_for_group() {
  local cfg="$1"
  local seq="$2"
  local col="$3"

  awk -F',' -v cfg="$cfg" -v seq="$seq" -v col="$col" '
    NR>1 && $1==cfg && $2==seq && $col != "" {print $col}
  ' "${RAW_CSV}" | sort -n | awk '
    {a[NR]=$1}
    END{
      if(NR==0) exit;
      if(NR%2==1){
        print a[(NR+1)/2]
      } else {
        print (a[NR/2]+a[NR/2+1])/2.0
      }
    }'
}

# 计算某 (config,sequence) 的完成率
completion_rate_for_group() {
  local cfg="$1"
  local seq="$2"
  awk -F',' -v cfg="$cfg" -v seq="$seq" '
    NR>1 && $1==cfg && $2==seq && $20 != "" {
      sum += $20; n++
    }
    END{
      if(n>0) printf("%.3f", sum / n);
    }' "${RAW_CSV}"
}

########################################
# 4. 运行单个配置 + 单个序列 + 单次 run
########################################
run_one_sequence() {
  local cfg_name="$1"   # 配置名
  local cfg_yaml="$2"   # yaml 路径
  local idx="$3"        # 序列号 1..7
  local run_id="$4"     # 本序列第几次 run (1..RUNS_PER_SEQ)

  # 计时起点
  local t_start t_end ELAPSED_SEC
  t_start=$(date +%s)

  local seq_name="harbor_sequence_${idx}"
  local bag_file="${BAG_DIR}/${seq_name}.bag"
  local gt_file
  gt_file=$(printf "%s/harbor_colmap_traj_sequence_%02d.txt" "${GT_DIR}" "${idx}")

  # 从 YAML 里解析 uwfusion.enable，用来标记 uw_on / uw_off
  local uw_enable_raw UW_TAG
  uw_enable_raw=$(grep -E '^[[:space:]]*uwfusion.enable' "${cfg_yaml}" | tail -n1 | sed 's/.*://; s/[[:space:]]//g' | tr 'A-Z' 'a-z' || true)
  if [[ "${uw_enable_raw}" == "1" || "${uw_enable_raw}" == "true" || "${uw_enable_raw}" == "yes" ]]; then
    UW_TAG="uw_on"
  else
    UW_TAG="uw_off"
  fi

  # 本配置的结果目录
  local cfg_results_dir="${RESULTS_ROOT}/${cfg_name}"
  mkdir -p "${cfg_results_dir}"

  # 这一次 run 单独的轨迹输出文件
  local est_cam_dst="${cfg_results_dir}/${seq_name}_r${run_id}_cam_traj_${UW_TAG}.txt"
  local est_kf_dst="${cfg_results_dir}/${seq_name}_r${run_id}_kf_traj_${UW_TAG}.txt"

  echo "============================================================"
  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 开始处理序列: ${seq_name}"
  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] YAML       : ${cfg_yaml}"
  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] rosbag     : ${bag_file}"
  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] GT         : ${gt_file}"
  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] UW_TAG     : ${UW_TAG}"
  echo "============================================================"

  # 单次预检
  [[ -f "${cfg_yaml}" ]] || { echo "[ERR][CFG ${cfg_name}] YAML 不存在: ${cfg_yaml}"; return 1; }
  [[ -f "${bag_file}" ]] || { echo "[ERR][CFG ${cfg_name}] [SEQ ${idx}] rosbag 不存在: ${bag_file}"; return 1; }
  [[ -f "${gt_file}"  ]] || { echo "[ERR][CFG ${cfg_name}] [SEQ ${idx}] GT 不存在: ${gt_file}"; return 1; }

  # 每次评估一个独立目录
  local eval_tag="${cfg_name}_${seq_name}_r${run_id}"
  local work_dir="${cfg_results_dir}/${eval_tag}_eval_${UW_TAG}_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${work_dir}"
  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 评估输出目录: ${work_dir}"

  ########################################
  # 4.1 启动 ORB-SLAM3（roslaunch）在后台
  ########################################

  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 启动 roslaunch..."
  roslaunch "${LAUNCH_PKG}" "${LAUNCH_NAME}" \
    voc_file:="${VOC_FILE}" \
    settings_file:="${cfg_yaml}" \
    use_sim_time:=true \
    > "${work_dir}/roslaunch_output.log" 2>&1 &
  local slam_pid=$!


  ########################################
  # 4.2 等待 ROS master / save_traj
  ########################################

  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 等待 ROS master..."
  until rosparam list >/dev/null 2>&1; do
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}]  ROS master 未就绪，等待中..."
    sleep 1
  done
  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] ROS master 已就绪。"

  # 用仿真时间
  rosparam set use_sim_time true || true

  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 等待 /orb_slam3/save_traj 服务..."
  timeout 60s bash -c 'until rosservice info /orb_slam3/save_traj >/dev/null 2>&1; do sleep 1; done' || true

  ########################################
  # 4.3 播放 rosbag
  ########################################

  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 开始播放 rosbag: ${bag_file} (rate=${BAG_RATE})"
  rosbag play "${bag_file}" --clock -r "${BAG_RATE}" >/dev/null 2>&1 || true

  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] rosbag 播放结束，准备保存轨迹…"
  sleep 5

  ########################################
  # 4.4 保存轨迹（调用 /orb_slam3/save_traj）
  ########################################

  local traj_tag="${cfg_name}_${seq_name}_r${run_id}_${UW_TAG}"

  if ! rosservice info /orb_slam3/save_traj >/dev/null 2>&1; then
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] [WARN] /orb_slam3/save_traj 暂不可用，继续等待 20s"
    timeout 20s bash -c 'until rosservice info /orb_slam3/save_traj >/dev/null 2>&1; do sleep 1; done' || true
  fi

  if rosservice info /orb_slam3/save_traj >/dev/null 2>&1; then
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 调用 /orb_slam3/save_traj ${traj_tag}"
    rosservice call /orb_slam3/save_traj "${traj_tag}" || echo "[WARN] save_traj 调用失败"
  else
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] [ERROR] /orb_slam3/save_traj 不可用，无法保存轨迹。"
  fi

  # 轨迹源文件（ROS 默认写在 ROS_HOME_DIR）
  local est_cam_src="${ROS_HOME_DIR}/${traj_tag}_cam_traj.txt"
  local est_kf_src="${ROS_HOME_DIR}/${traj_tag}_kf_traj.txt"

  if [[ -f "${est_cam_src}" ]]; then
    cp "${est_cam_src}" "${est_cam_dst}"
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 相机轨迹: ${est_cam_dst}"
  else
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] [WARN] 未找到相机轨迹: ${est_cam_src}"
  fi

  if [[ -f "${est_kf_src}" ]]; then
    cp "${est_kf_src}" "${est_kf_dst}"
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 关键帧轨迹: ${est_kf_dst}"
  else
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] [WARN] 未找到关键帧轨迹: ${est_kf_src}"
  fi

  ########################################
  # 4.5 关闭 SLAM 节点 + roslaunch
  ########################################
  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 关闭 /orb_slam3 和 rviz..."
  rosnode kill /orb_slam3 >/dev/null 2>&1 || true
  rosnode kill /rviz      >/dev/null 2>&1 || true

  # 再明确杀掉 roslaunch 进程（否则会留后台）
  if ps -p "${slam_pid}" >/dev/null 2>&1; then
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 杀掉 roslaunch (PID=${slam_pid})..."
    kill "${slam_pid}" >/dev/null 2>&1 || true
    wait "${slam_pid}" 2>/dev/null || true
  fi

  ########################################
  # 4.6 如果没有轨迹，就不做评估，只记耗时
  ########################################

  if [[ ! -f "${est_cam_dst}" ]]; then
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] [ERROR] 估计轨迹不存在，跳过评估: ${est_cam_dst}"
    t_end=$(date +%s)
    ELAPSED_SEC=$((t_end - t_start))
    echo "${cfg_name},${seq_name},${run_id},${UW_TAG},,,,,,,,,${ELAPSED_SEC},,,,,,0" >> "${RAW_CSV}"
    return 1
  fi

  ########################################
  # 4.7 ATE / RPE 评估 之前：读取性能统计
  ########################################

  local perf_csv="${ROS_HOME_DIR}/${traj_tag}_perf.csv"
  local track_ms_mean="" ba_ms_mean="" enh_ms_mean="" sel_pts_mean="" num_kfs="" mem_mb=""
  if [[ -f "${perf_csv}" ]]; then
    IFS=',' read -r track_ms_mean ba_ms_mean enh_ms_mean sel_pts_mean num_kfs _ mem_mb < <(tail -n1 "${perf_csv}")
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 性能统计: ${perf_csv}"
  else
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] [WARN] 未找到性能统计: ${perf_csv}"
  fi

  ########################################
  # 4.7 ATE / RPE 评估
  ########################################

  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 开始评估 ATE/RPE..."
  cd "${work_dir}"

  # 4.7.1 GT: COLMAP -> TUM
  awk -v fps="${FPS}" '
    $1 ~ /^#/ { next }
    NF>=8 {
      frame = $1 + 0
      ts = frame / fps
      printf("%.9f %s %s %s %s %s %s %s\n", ts, $2, $3, $4, $5, $6, $7, $8)
    }
  ' "${gt_file}" | sort -k1,1n > groundtruth.tum
  evo_traj tum groundtruth.tum --save_plot gt_traj_xz.pdf --plot_mode xy --no_warnings

  # 4.7.2 估计轨迹 -> TUM（自动识别时间单位）
  cp "${est_cam_dst}" est.raw
  local MAXTS SCALE
  MAXTS=$(awk 'NF>=8 && $1 !~ /^#/ {if($1>m)m=$1} END{print m+0}' est.raw)
  SCALE=$(awk -v m="${MAXTS}" 'BEGIN{ if (m>1e12) print 1e9; else if (m>1e10) print 1e3; else print 1; }')
  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] Detected est timestamp scale: divide by ${SCALE}"

  awk -v s="${SCALE}" '
    BEGIN{OFS=" "}
    $1 ~ /^#/ {next}
    NF>=8 {
      printf("%.9f %s %s %s %s %s %s %s\n", $1/s, $2,$3,$4,$5,$6,$7,$8)
    }
  ' est.raw | sort -k1,1n > est.tum
  evo_traj tum est.tum --save_plot est_traj_xz.pdf --plot_mode xy --no_warnings

  # 4.7.3 时间归一化
  awk 'BEGIN{OFS=" "} $1!~/^#/ && NF>=8 { if(!seen++){t0=$1} printf("%.9f %s %s %s %s %s %s %s\n",$1-t0,$2,$3,$4,$5,$6,$7,$8) }' est.tum > est_norm.tum
  awk 'BEGIN{OFS=" "} $1!~/^#/ && NF>=8 { if(!seen++){t0=$1} printf("%.9f %s %s %s %s %s %s %s\n",$1-t0,$2,$3,$4,$5,$6,$7,$8) }' groundtruth.tum > groundtruth_norm.tum
  evo_traj tum est_norm.tum --save_plot est_norm_xz.pdf --plot_mode xy --no_warnings
  evo_traj tum groundtruth_norm.tum --save_plot gt_norm_xz.pdf --plot_mode xy --no_warnings

  # 4.7.4 ATE
  local ATE_ZIP="ATE_${eval_tag}_${UW_TAG}.zip"
  evo_ape tum est_norm.tum groundtruth_norm.tum \
    -a --align --correct_scale \
    --t_max_diff "${T_MAX_DIFF}" \
    -s -v \
    --save_results "${ATE_ZIP}" \
    --save_plot "ATE_${eval_tag}_${UW_TAG}_xz.pdf" --plot_mode xy \
    --no_warnings

  # 4.7.5 RPE（可选）
  local RPE_ZIP=""
  if [[ "${RPE_ENABLED}" -eq 1 ]]; then
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] RPE enabled: unit=${RPE_UNIT}, delta=${RPE_DELTA}"
    RPE_ZIP="RPE_${eval_tag}_${UW_TAG}.zip"
    evo_rpe tum est_norm.tum groundtruth_norm.tum \
      -a --align --correct_scale \
      --t_max_diff "${T_MAX_DIFF}" \
      -s -v \
      -r trans_part \
      -d "${RPE_DELTA}" -u "${RPE_UNIT}" \
      --save_results "${RPE_ZIP}" \
      --save_plot "RPE_${eval_tag}_${UW_TAG}_xz.pdf" --plot_mode xy \
      --no_warnings
  else
    echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] RPE disabled"
  fi

  ########################################
  # 4.8 从 zip 抽取指标，写入 RAW_CSV（单次 RUN）
  ########################################

  # ATE 指标
  local ATE_RMSE ATE_MEAN ATE_MED ATE_STD
  ATE_RMSE=$(get_metric_from_zip "${ATE_ZIP}" "rmse")
  ATE_MEAN=$(get_metric_from_zip "${ATE_ZIP}" "mean")
  ATE_MED=$(get_metric_from_zip  "${ATE_ZIP}" "median")
  ATE_STD=$(get_metric_from_zip  "${ATE_ZIP}" "std")

  # RPE 指标
  local RPE_RMSE="" RPE_MEAN="" RPE_MED="" RPE_STD=""
  if [[ -n "${RPE_ZIP}" && -f "${RPE_ZIP}" ]]; then
    RPE_RMSE=$(get_metric_from_zip "${RPE_ZIP}" "rmse")
    RPE_MEAN=$(get_metric_from_zip "${RPE_ZIP}" "mean")
    RPE_MED=$(get_metric_from_zip  "${RPE_ZIP}" "median")
    RPE_STD=$(get_metric_from_zip  "${RPE_ZIP}" "std")
  fi

  # 计算耗时
  t_end=$(date +%s)
  ELAPSED_SEC=$((t_end - t_start))

  local completed=1

  # 回到工作空间，写 RAW_CSV（每一行是一条 run）
  cd "${WS}"
  echo "${cfg_name},${seq_name},${run_id},${UW_TAG},${ATE_RMSE},${ATE_MEAN},${ATE_MED},${ATE_STD},${RPE_RMSE},${RPE_MEAN},${RPE_MED},${RPE_STD},${ELAPSED_SEC},${track_ms_mean},${ba_ms_mean},${enh_ms_mean},${sel_pts_mean},${num_kfs},${mem_mb},${completed}" >> "${RAW_CSV}"

  echo "[CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 完成，结果目录: ${work_dir}"
}

########################################
# 5. 主循环：8 个配置 × 7 个序列 × RUNS_PER_SEQ 次
########################################

echo "[INFO] 开始批量实验..."
echo "[INFO] WS=${WS}"
echo "[INFO] HARBOR_ROOT=${HARBOR_ROOT}"
echo "[INFO] CONFIG_DIR=${CONFIG_DIR}"
echo "[INFO] RESULTS_ROOT=${RESULTS_ROOT}"
echo "[INFO] RAW_CSV=${RAW_CSV}"
echo "[INFO] RUNS_PER_SEQ=${RUNS_PER_SEQ}"

for cfg_name in "${CONFIG_NAMES[@]}"; do
  cfg_yaml="${CONFIG_DIR}/${cfg_name}.yaml"
  for idx in "${SEQ_INDICES[@]}"; do
    for run_id in $(seq 1 "${RUNS_PER_SEQ}"); do
      run_one_sequence "${cfg_name}" "${cfg_yaml}" "${idx}" "${run_id}" || {
        echo "[WARN] [CFG ${cfg_name}] [SEQ ${idx}] [RUN ${run_id}] 运行失败，继续下一个。"
      }
    done
  done
done

########################################
# 6. 生成“中位数”汇总表
########################################

echo "[INFO] 所有 run 完成，开始汇总中位数到: ${SUMMARY_CSV}"
echo "config,sequence,uw_tag,ate_rmse_med,ate_mean_med,ate_median_med,ate_std_med,rpe_rmse_med,rpe_mean_med,rpe_median_med,rpe_std_med,elapsed_sec_med,track_ms_med,ba_ms_med,enh_ms_med,sel_pts_med,num_kfs_med,mem_mb_med,completion_rate" > "${SUMMARY_CSV}"

for cfg_name in "${CONFIG_NAMES[@]}"; do
  for idx in "${SEQ_INDICES[@]}"; do
    seq_name="harbor_sequence_${idx}"

    # 找出这个 (config,sequence) 对应的 uw_tag（理论上所有 run 都一样）
    uw_tag=$(awk -F',' -v cfg="$cfg_name" -v seq="$seq_name" 'NR>1 && $1==cfg && $2==seq && $4!="" {print $4; exit}' "${RAW_CSV}" || true)
    # 如果一个 run 都没有成功，这组就跳过
    if [[ -z "${uw_tag}" ]]; then
      continue
    fi

    # 对每一列求中位数
    ate_rmse_med=$(   median_for_group "${cfg_name}" "${seq_name}" 5 )
    ate_mean_med=$(   median_for_group "${cfg_name}" "${seq_name}" 6 )
    ate_median_med=$( median_for_group "${cfg_name}" "${seq_name}" 7 )
    ate_std_med=$(    median_for_group "${cfg_name}" "${seq_name}" 8 )

    rpe_rmse_med=$(   median_for_group "${cfg_name}" "${seq_name}" 9 )
    rpe_mean_med=$(   median_for_group "${cfg_name}" "${seq_name}" 10 )
    rpe_median_med=$( median_for_group "${cfg_name}" "${seq_name}" 11 )
    rpe_std_med=$(    median_for_group "${cfg_name}" "${seq_name}" 12 )

    elapsed_med=$(    median_for_group "${cfg_name}" "${seq_name}" 13 )

    track_ms_med=$( median_for_group "${cfg_name}" "${seq_name}" 14 )
    ba_ms_med=$(    median_for_group "${cfg_name}" "${seq_name}" 15 )
    enh_ms_med=$(   median_for_group "${cfg_name}" "${seq_name}" 16 )
    sel_pts_med=$(  median_for_group "${cfg_name}" "${seq_name}" 17 )
    num_kfs_med=$(  median_for_group "${cfg_name}" "${seq_name}" 18 )
    mem_mb_med=$(   median_for_group "${cfg_name}" "${seq_name}" 19 )

    completion_rate=$( completion_rate_for_group "${cfg_name}" "${seq_name}" )

    echo "${cfg_name},${seq_name},${uw_tag},${ate_rmse_med},${ate_mean_med},${ate_median_med},${ate_std_med},${rpe_rmse_med},${rpe_mean_med},${rpe_median_med},${rpe_std_med},${elapsed_med},${track_ms_med},${ba_ms_med},${enh_ms_med},${sel_pts_med},${num_kfs_med},${mem_mb_med},${completion_rate}" >> "${SUMMARY_CSV}"
  done
done

echo "[INFO] 全部实验结束。"
echo "[INFO] 每次运行的原始记录在: ${RAW_CSV}"
echo "[INFO] 运行按中位数汇总的结果在: ${SUMMARY_CSV}"
