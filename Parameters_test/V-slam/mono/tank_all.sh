#!/usr/bin/env bash
set -euo pipefail

export MPLBACKEND=Agg

########################################
# 0. 基本路径配置（按需改）
########################################

WS="${WS:-$HOME/catkin_ws}"

# Tank 数据根目录
TANK_ROOT="${TANK_ROOT:-$WS/dataset_tank}"
BAG_DIR="${BAG_DIR:-$TANK_ROOT/data}"
GT_DIR="${GT_DIR:-$TANK_ROOT/GT}"
RESULTS_ROOT="${RESULTS_ROOT:-$TANK_ROOT/results}"

# ROS 轨迹默认保存位置
ROS_HOME_DIR="${ROS_HOME:-$HOME/.ros}"

# 词典与 launch 文件
VOC_FILE="${VOC_FILE:-$WS/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin}"
LAUNCH_PKG="${LAUNCH_PKG:-orb_slam3_ros}"
LAUNCH_NAME="${LAUNCH_NAME:-tank_mono.launch}"

# Tank YAML 所在目录
CONFIG_DIR="${CONFIG_DIR:-$WS/src/orb_slam3_ros/config/Monocular/Tank}"

CONFIG_NAMES=(
  "tank_off_00"
  "tank_off_01"
  "tank_off_10"
  "tank_off_11"
  "tank_on_00"
  "tank_on_01"
  "tank_on_10"
  "tank_on_11"
)

# 要跑的 Tank 序列名（与 .bag 和 _gt.csv 文件名对应）
SEQ_NAMES=(
   "Structure_Easy"
  "Structure_Medium"
  # "Structure_Hard"
  # "HalfTank_Easy"
  # "HalfTank_Medium"
  # "HalfTank_Hard"
  # "WholeTank_Medium"
  # "WholeTank_Hard"
)

RUNS_PER_SEQ="${RUNS_PER_SEQ:-2}"

########################################
# 1. 评估参数（ATE / RPE）
########################################

RPE_ENABLED="${RPE_ENABLED:-1}"
RPE_UNIT="${RPE_UNIT:-f}"
RPE_DELTA="${RPE_DELTA:-1}"
FPS="${FPS:-20}"
T_MAX_DIFF="${T_MAX_DIFF:-0.05}"

BAG_RATE="${BAG_RATE:-1}"

########################################
# 2. 预检
########################################

command -v roslaunch >/dev/null 2>&1 || { echo "[ERR] 需要 roslaunch 在 PATH 中"; exit 1; }
command -v rosbag    >/dev/null 2>&1 || { echo "[ERR] 需要 rosbag 在 PATH 中"; exit 1; }

[[ -d "${WS}" ]]        || { echo "[ERR] 工作空间不存在: ${WS}"; exit 1; }
[[ -d "${TANK_ROOT}" ]] || { echo "[ERR] Tank 根目录不存在: ${TANK_ROOT}"; exit 1; }
[[ -d "${BAG_DIR}" ]]   || { echo "[ERR] 数据目录不存在: ${BAG_DIR}"; exit 1; }
[[ -d "${GT_DIR}" ]]    || { echo "[ERR] GT 目录不存在: ${GT_DIR}"; exit 1; }
[[ -d "${CONFIG_DIR}" ]]|| { echo "[ERR] YAML 目录不存在: ${CONFIG_DIR}"; exit 1; }
[[ -f "${VOC_FILE}" ]]  || { echo "[ERR] 词典不存在: ${VOC_FILE}"; exit 1; }

mkdir -p "${RESULTS_ROOT}"

RAW_CSV="${RESULTS_ROOT}/tank_eval_runs.csv"
echo "config,sequence,run,uw_tag,ate_rmse,ate_mean,ate_median,ate_std,rpe_rmse,rpe_mean,rpe_median,rpe_std,elapsed_sec,track_ms_mean,ba_ms_mean,enh_ms_mean,sel_pts_mean,num_kfs,mem_mb,completed" > "${RAW_CSV}"

SUMMARY_CSV="${RESULTS_ROOT}/tank_eval_summary.csv"

source "${WS}/devel/setup.bash"

python3 -m pip show evo >/dev/null 2>&1 || python3 -m pip install -U "evo>=1.25" --no-binary evo

########################################
# 3. 工具函数
########################################

# ★修复点：从 zip 内 stats.json 读取指标（避免解析 evo_res 文本导致拿不到数）
get_metric_from_zip() {
  local zipfile="$1"
  local metric="$2"
  [[ -f "$zipfile" ]] || { echo ""; return 0; }

  python3 - "$zipfile" "$metric" <<'PY' 2>/dev/null || true
import sys, json, zipfile
zp, key = sys.argv[1], sys.argv[2]
try:
    with zipfile.ZipFile(zp, "r") as z:
        names = [n for n in z.namelist() if n.endswith("stats.json")]
        if not names:
            sys.exit(0)
        names = sorted(names, key=len)
        data = json.loads(z.read(names[0]).decode("utf-8"))
        v = data.get(key, None)
        if v is None:
            sys.exit(0)
        print(v)
except Exception:
    pass
PY
}

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

completion_rate_for_group() {
  local cfg="$1"
  local seq="$2"
  awk -F',' -v cfg="$cfg" -v seq="$seq" '
    NR>1 && $1==cfg && $2==seq && $20 != "" { sum += $20; n++ }
    END{ if(n>0) printf("%.3f", sum / n); }
  ' "${RAW_CSV}"
}

########################################
# 4. 运行单个配置 + 单个序列 + 单次 run
########################################

run_one_sequence() {
  local cfg_name="$1"
  local cfg_yaml="$2"
  local seq_name="$3"
  local run_id="$4"

  local t_start t_end ELAPSED_SEC
  t_start=$(date +%s)

  local bag_file="${BAG_DIR}/${seq_name}.bag"
  local gt_file="${GT_DIR}/${seq_name}_gt.csv"

  local uw_enable_raw UW_TAG
  uw_enable_raw=$(grep -E '^[[:space:]]*uwfusion.enable' "${cfg_yaml}" | tail -n1 | sed 's/.*://; s/[[:space:]]//g' | tr 'A-Z' 'a-z' || true)
  if [[ "${uw_enable_raw}" == "1" || "${uw_enable_raw}" == "true" || "${uw_enable_raw}" == "yes" ]]; then
    UW_TAG="uw_on"
  else
    UW_TAG="uw_off"
  fi

  local cfg_results_dir="${RESULTS_ROOT}/${cfg_name}"
  mkdir -p "${cfg_results_dir}"

  local est_cam_dst="${cfg_results_dir}/${seq_name}_r${run_id}_cam_traj_${UW_TAG}.txt"
  local est_kf_dst="${cfg_results_dir}/${seq_name}_r${run_id}_kf_traj_${UW_TAG}.txt"

  echo "============================================================"
  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 开始处理序列"
  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] YAML       : ${cfg_yaml}"
  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] rosbag     : ${bag_file}"
  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] GT         : ${gt_file}"
  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] UW_TAG     : ${UW_TAG}"
  echo "============================================================"

  [[ -f "${cfg_yaml}" ]] || { echo "[ERR][CFG ${cfg_name}] YAML 不存在: ${cfg_yaml}"; return 1; }
  [[ -f "${bag_file}" ]] || { echo "[ERR][CFG ${cfg_name}] [SEQ ${seq_name}] rosbag 不存在: ${bag_file}"; return 1; }
  [[ -f "${gt_file}"  ]] || { echo "[ERR][CFG ${cfg_name}] [SEQ ${seq_name}] GT 不存在: ${gt_file}"; return 1; }

  local eval_tag="${cfg_name}_${seq_name}_r${run_id}"
  local work_dir="${cfg_results_dir}/${eval_tag}_eval_${UW_TAG}_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${work_dir}"
  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 评估输出目录: ${work_dir}"

  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 启动 roslaunch..."
  roslaunch "${LAUNCH_PKG}" "${LAUNCH_NAME}" \
    voc_file:="${VOC_FILE}" \
    settings_file:="${cfg_yaml}" \
    use_sim_time:=true \
    > "${work_dir}/roslaunch_output.log" 2>&1 &
  local slam_pid=$!

  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 等待 ROS master..."
  until rosparam list >/dev/null 2>&1; do
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}]  ROS master 未就绪，等待中..."
    sleep 1
  done

  rosparam set use_sim_time true || true

  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 等待 /orb_slam3/save_traj 服务..."
  timeout 60s bash -c 'until rosservice info /orb_slam3/save_traj >/dev/null 2>&1; do sleep 1; done' || true

  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 开始播放 rosbag: ${bag_file} (rate=${BAG_RATE})"
  rosbag play "${bag_file}" --clock -r "${BAG_RATE}" >/dev/null 2>&1 || true
  sleep 5

  local traj_tag="${cfg_name}_${seq_name}_r${run_id}_${UW_TAG}"

  if rosservice info /orb_slam3/save_traj >/dev/null 2>&1; then
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 调用 /orb_slam3/save_traj ${traj_tag}"
    rosservice call /orb_slam3/save_traj "${traj_tag}" || echo "[WARN] save_traj 调用失败"
  else
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] [ERROR] /orb_slam3/save_traj 不可用，无法保存轨迹。"
  fi

  local est_cam_src="${ROS_HOME_DIR}/${traj_tag}_cam_traj.txt"
  local est_kf_src="${ROS_HOME_DIR}/${traj_tag}_kf_traj.txt"

  if [[ -f "${est_cam_src}" ]]; then
    cp "${est_cam_src}" "${est_cam_dst}"
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 相机轨迹: ${est_cam_dst}"
  else
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] [WARN] 未找到相机轨迹: ${est_cam_src}"
  fi

  if [[ -f "${est_kf_src}" ]]; then
    cp "${est_kf_src}" "${est_kf_dst}"
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 关键帧轨迹: ${est_kf_dst}"
  else
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] [WARN] 未找到关键帧轨迹: ${est_kf_src}"
  fi

  # 点云（若有）
  local est_map_src="${ROS_HOME_DIR}/${traj_tag}_map.ply"
  local est_map_dst="${cfg_results_dir}/${seq_name}_r${run_id}_map_${UW_TAG}.ply"
  local est_map_work="${work_dir}/${traj_tag}_map.ply"

  if [[ -f "${est_map_src}" ]]; then
    cp "${est_map_src}" "${est_map_dst}"
    cp "${est_map_src}" "${est_map_work}"
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 地图点云: ${est_map_dst}"
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 点云已同步到评估目录: ${est_map_work}"
  else
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] [WARN] 未找到点云: ${est_map_src}"
  fi

  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 关闭 /orb_slam3 和 rviz..."
  rosnode kill /orb_slam3 >/dev/null 2>&1 || true
  rosnode kill /rviz      >/dev/null 2>&1 || true

  if ps -p "${slam_pid}" >/dev/null 2>&1; then
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 杀掉 roslaunch (PID=${slam_pid})..."
    kill "${slam_pid}" >/dev/null 2>&1 || true
    wait "${slam_pid}" 2>/dev/null || true
  fi

  if [[ ! -f "${est_cam_dst}" ]]; then
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] [ERROR] 估计轨迹不存在，跳过评估: ${est_cam_dst}"
    t_end=$(date +%s)
    ELAPSED_SEC=$((t_end - t_start))
    echo "${cfg_name},${seq_name},${run_id},${UW_TAG},,,,,,,,,${ELAPSED_SEC},,,,,,0" >> "${RAW_CSV}"
    return 1
  fi

  local perf_csv="${ROS_HOME_DIR}/${traj_tag}_perf.csv"
  local track_ms_mean="" ba_ms_mean="" enh_ms_mean="" sel_pts_mean="" num_kfs="" mem_mb=""
  if [[ -f "${perf_csv}" ]]; then
    IFS=',' read -r track_ms_mean ba_ms_mean enh_ms_mean sel_pts_mean num_kfs _ mem_mb < <(tail -n1 "${perf_csv}")
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 性能统计: ${perf_csv}"
  else
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] [WARN] 未找到性能统计: ${perf_csv}"
  fi

  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 开始评估 ATE/RPE..."
  cd "${work_dir}"

  # GT: CSV -> TUM（Tank: timestamp[ns], pos_x, pos_y, pos_z, qx, qy, qz, qw）
  awk -F',' '
    NR>1 && NF>=8 {
        ts = $1 / 1e9;
        printf("%.9f %s %s %s %s %s %s %s\n", ts, $2, $3, $4, $5, $6, $7, $8);
    }
  ' "${gt_file}" | sort -k1,1n > groundtruth.tum
  evo_traj tum groundtruth.tum --save_plot gt_traj_xz.pdf --plot_mode xz --no_warnings || true

  # EST -> TUM（自动识别时间单位）
  cp "${est_cam_dst}" est.raw
  local MAXTS SCALE
  MAXTS=$(awk 'NF>=8 && $1 !~ /^#/ {if($1>m)m=$1} END{print m+0}' est.raw)
  SCALE=$(awk -v m="${MAXTS}" 'BEGIN{ if (m>1e12) print 1e9; else if (m>1e10) print 1e3; else print 1; }')
  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] Detected est timestamp scale: divide by ${SCALE}"

  awk -v s="${SCALE}" '
    $1 ~ /^#/ {next}
    NF>=8 { printf("%.9f %s %s %s %s %s %s %s\n", $1/s, $2,$3,$4,$5,$6,$7,$8) }
  ' est.raw | sort -k1,1n > est.tum
  evo_traj tum est.tum --save_plot est_traj_xz.pdf --plot_mode xz --no_warnings || true

  # 时间归一化
  awk 'BEGIN{OFS=" "}
    $1!~/^#/ && NF>=8 { if(!seen++){t0=$1} printf("%.9f %s %s %s %s %s %s %s\n",$1-t0,$2,$3,$4,$5,$6,$7,$8) }
  ' est.tum > est_norm.tum

  awk 'BEGIN{OFS=" "}
    $1!~/^#/ && NF>=8 { if(!seen++){t0=$1} printf("%.9f %s %s %s %s %s %s %s\n",$1-t0,$2,$3,$4,$5,$6,$7,$8) }
  ' groundtruth.tum > groundtruth_norm.tum

  evo_traj tum est_norm.tum --save_plot est_norm_xz.pdf --plot_mode xz --no_warnings || true
  evo_traj tum groundtruth_norm.tum --save_plot gt_norm_xz.pdf --plot_mode xz --no_warnings || true

  # ★修复点：evo_ape/evo_rpe 失败不让脚本退出，用 completed 标记
  local completed=1

  local ATE_ZIP="ATE_${eval_tag}_${UW_TAG}.zip"
  if evo_ape tum est_norm.tum groundtruth_norm.tum \
      -a --align --correct_scale \
      --t_max_diff "${T_MAX_DIFF}" \
      -s -v \
      --save_results "${ATE_ZIP}" \
      --save_plot "ATE_${eval_tag}_${UW_TAG}_xz.pdf" --plot_mode xz \
      --no_warnings; then
    :
  else
    completed=0
  fi

  local RPE_ZIP=""
  if [[ "${RPE_ENABLED}" -eq 1 ]]; then
    echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] RPE enabled: unit=${RPE_UNIT}, delta=${RPE_DELTA}"
    RPE_ZIP="RPE_${eval_tag}_${UW_TAG}.zip"
    if evo_rpe tum est_norm.tum groundtruth_norm.tum \
        -a --align --correct_scale \
        --t_max_diff "${T_MAX_DIFF}" \
        -s -v \
        -r trans_part \
        -d "${RPE_DELTA}" -u "${RPE_UNIT}" \
        --save_results "${RPE_ZIP}" \
        --save_plot "RPE_${eval_tag}_${UW_TAG}_xz.pdf" --plot_mode xz \
        --no_warnings; then
      :
    else
      completed=0
    fi
  fi

  # 从 zip 读指标（stats.json）
  local ATE_RMSE="" ATE_MEAN="" ATE_MED="" ATE_STD=""
  local RPE_RMSE="" RPE_MEAN="" RPE_MED="" RPE_STD=""

  if [[ -f "${ATE_ZIP}" ]]; then
    ATE_RMSE=$(get_metric_from_zip "${ATE_ZIP}" "rmse")
    ATE_MEAN=$(get_metric_from_zip "${ATE_ZIP}" "mean")
    ATE_MED=$( get_metric_from_zip "${ATE_ZIP}" "median")
    ATE_STD=$( get_metric_from_zip "${ATE_ZIP}" "std")
  else
    completed=0
  fi

  if [[ -n "${RPE_ZIP}" && -f "${RPE_ZIP}" ]]; then
    RPE_RMSE=$(get_metric_from_zip "${RPE_ZIP}" "rmse")
    RPE_MEAN=$(get_metric_from_zip "${RPE_ZIP}" "mean")
    RPE_MED=$( get_metric_from_zip "${RPE_ZIP}" "median")
    RPE_STD=$( get_metric_from_zip "${RPE_ZIP}" "std")
  fi

  t_end=$(date +%s)
  ELAPSED_SEC=$((t_end - t_start))

  cd "${WS}"
  echo "${cfg_name},${seq_name},${run_id},${UW_TAG},${ATE_RMSE},${ATE_MEAN},${ATE_MED},${ATE_STD},${RPE_RMSE},${RPE_MEAN},${RPE_MED},${RPE_STD},${ELAPSED_SEC},${track_ms_mean},${ba_ms_mean},${enh_ms_mean},${sel_pts_mean},${num_kfs},${mem_mb},${completed}" >> "${RAW_CSV}"

  echo "[CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 完成，结果目录: ${work_dir}"
}

########################################
# 5. 主循环
########################################

echo "[INFO] 开始批量实验..."
echo "[INFO] WS=${WS}"
echo "[INFO] TANK_ROOT=${TANK_ROOT}"
echo "[INFO] CONFIG_DIR=${CONFIG_DIR}"
echo "[INFO] RESULTS_ROOT=${RESULTS_ROOT}"
echo "[INFO] RAW_CSV=${RAW_CSV}"
echo "[INFO] RUNS_PER_SEQ=${RUNS_PER_SEQ}"

for cfg_name in "${CONFIG_NAMES[@]}"; do
  cfg_yaml="${CONFIG_DIR}/${cfg_name}.yaml"
  for seq_name in "${SEQ_NAMES[@]}"; do
    for run_id in $(seq 1 "${RUNS_PER_SEQ}"); do
      run_one_sequence "${cfg_name}" "${cfg_yaml}" "${seq_name}" "${run_id}" || {
        echo "[WARN] [CFG ${cfg_name}] [SEQ ${seq_name}] [RUN ${run_id}] 运行失败，继续下一个。"
      }
    done
  done
done

########################################
# 6. 汇总中位数
########################################

echo "[INFO] 所有 run 完成，开始汇总中位数到: ${SUMMARY_CSV}"
echo "config,sequence,uw_tag,ate_rmse_med,ate_mean_med,ate_median_med,ate_std_med,rpe_rmse_med,rpe_mean_med,rpe_median_med,rpe_std_med,elapsed_sec_med,track_ms_med,ba_ms_med,enh_ms_med,sel_pts_med,num_kfs_med,mem_mb_med,completion_rate" > "${SUMMARY_CSV}"

for cfg_name in "${CONFIG_NAMES[@]}"; do
  for seq_name in "${SEQ_NAMES[@]}"; do

    uw_tag=$(awk -F',' -v cfg="$cfg_name" -v seq="$seq_name" 'NR>1 && $1==cfg && $2==seq && $4!="" {print $4; exit}' "${RAW_CSV}" || true)
    [[ -n "${uw_tag}" ]] || continue

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

