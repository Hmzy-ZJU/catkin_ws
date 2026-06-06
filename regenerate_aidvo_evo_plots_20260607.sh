#!/usr/bin/env bash
set -o pipefail

# Regenerate aligned evo plots for existing UW-AIDVO result folders.
#
# Usage:
#   cd ~/catkin_ws
#   bash regenerate_aidvo_evo_plots_20260607.sh
#   bash regenerate_aidvo_evo_plots_20260607.sh dataset_tank/results/aidvo_full_20260607_023205

if [ -z "$WS" ]; then WS="$HOME/catkin_ws"; fi
export MPLBACKEND=Agg

ROOTS=("$@")
if [ "${#ROOTS[@]}" -eq 0 ]; then
  ROOTS=(
    "$WS/dataset_EuRoc/results"
    "$WS/dataset_tank/results"
    "$WS/dataset_harbor/results"
    "$WS/results"
  )
fi

regen_one() {
  local evo_dir="$1"
  local gt_tum="$evo_dir/groundtruth_matched.tum"
  local est_tum="$evo_dir/estimated_matched.tum"
  local status_file="$evo_dir/evo_status.txt"

  [ -s "$gt_tum" ] || return 0
  [ -s "$est_tum" ] || return 0

  echo "[EVO] regenerating $evo_dir"
  python3 - "$gt_tum" "$est_tum" "$evo_dir" <<'PY' > "$evo_dir/regenerate_plots_stdout.txt" 2>&1
import math
import os
import sys
import numpy as np

gt_path, est_path, out_dir = sys.argv[1:4]

def read_tum(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 4:
                continue
            rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
    return rows

def umeyama_align(est_pts, gt_pts):
    p = np.asarray(est_pts, dtype=float)
    q = np.asarray(gt_pts, dtype=float)
    mu_p = p.mean(axis=0)
    mu_q = q.mean(axis=0)
    x = p - mu_p
    y = q - mu_q
    sigma = (y.T @ x) / len(p)
    u, d, vt = np.linalg.svd(sigma)
    s_mat = np.eye(3)
    if np.linalg.det(u @ vt) < 0:
        s_mat[2, 2] = -1.0
    r = u @ s_mat @ vt
    var_p = np.mean(np.sum(x * x, axis=1))
    scale = np.trace(np.diag(d) @ s_mat) / max(var_p, 1e-12)
    t = mu_q - scale * (r @ mu_p)
    return (scale * (r @ p.T)).T + t

gt = read_tum(gt_path)
est = read_tum(est_path)
n = min(len(gt), len(est))
if n < 3:
    raise SystemExit("not enough matched poses")
gt = gt[:n]
est = est[:n]

gt_xyz = np.asarray([(r[1], r[2], r[3]) for r in gt], dtype=float)
est_xyz = np.asarray([(r[1], r[2], r[3]) for r in est], dtype=float)
aligned = umeyama_align(est_xyz, gt_xyz)

aligned_tum = os.path.join(out_dir, "estimated_aligned.tum")
with open(aligned_tum, "w") as f:
    for i, row in enumerate(aligned):
        f.write(f"{gt[i][0]:.9f} {row[0]:.9f} {row[1]:.9f} {row[2]:.9f} 0 0 0 1\n")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

t = [r[0] for r in gt]
gx, gy, gz = gt_xyz[:, 0], gt_xyz[:, 1], gt_xyz[:, 2]
ex, ey, ez = aligned[:, 0], aligned[:, 1], aligned[:, 2]
err = np.linalg.norm(aligned - gt_xyz, axis=1)

def save_plane(name, a_gt, b_gt, a_est, b_est, xlabel, ylabel, title):
    plt.figure(figsize=(8, 6), dpi=160)
    plt.plot(a_gt, b_gt, label="groundtruth", linewidth=2)
    plt.plot(a_est, b_est, label="estimated aligned", linewidth=1.5)
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, name))
    plt.close()

save_plane("matched_trajectory_xy.png", gx, gy, ex, ey, "x [m]", "y [m]", "Aligned trajectory XY")
save_plane("matched_trajectory_xz.png", gx, gz, ex, ez, "x [m]", "z [m]", "Aligned trajectory XZ")
save_plane("matched_trajectory_yz.png", gy, gz, ey, ez, "y [m]", "z [m]", "Aligned trajectory YZ")

try:
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    fig = plt.figure(figsize=(8, 6), dpi=160)
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(gx, gy, gz, label="groundtruth", linewidth=2)
    ax.plot(ex, ey, ez, label="estimated aligned", linewidth=1.5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Aligned trajectory 3D")
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "matched_trajectory_3d.png"))
    plt.close()
except Exception as exc:
    with open(os.path.join(out_dir, "regenerate_plot_errors.txt"), "a") as f:
        f.write(f"3d plot failed: {exc}\n")

plt.figure(figsize=(9, 4.5), dpi=160)
plt.plot(t, err, linewidth=1.5)
plt.grid(True, alpha=0.3)
plt.xlabel("time [s]")
plt.ylabel("position error [m]")
plt.title("Aligned position error")
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "matched_position_error.png"))
plt.close()

plt.figure(figsize=(9, 5), dpi=160)
plt.plot(t, gx, label="gt x", linewidth=1.5)
plt.plot(t, ex, label="est x", linewidth=1.2)
plt.plot(t, gy, label="gt y", linewidth=1.5)
plt.plot(t, ey, label="est y", linewidth=1.2)
plt.plot(t, gz, label="gt z", linewidth=1.5)
plt.plot(t, ez, label="est z", linewidth=1.2)
plt.grid(True, alpha=0.3)
plt.xlabel("time [s]")
plt.ylabel("position [m]")
plt.title("Aligned position components")
plt.legend(ncol=3, fontsize=8)
plt.tight_layout()
plt.savefig(os.path.join(out_dir, "matched_position_components.png"))
plt.close()

with open(os.path.join(out_dir, "custom_plot_metrics.txt"), "w") as f:
    f.write(f"matched_pairs: {n}\n")
    f.write(f"mean_position_error_m: {float(np.mean(err)):.6f}\n")
    f.write(f"rmse_position_error_m: {math.sqrt(float(np.mean(err * err))):.6f}\n")
    f.write(f"max_position_error_m: {float(np.max(err)):.6f}\n")
PY

  if command -v evo_ape >/dev/null 2>&1 && command -v evo_rpe >/dev/null 2>&1 && command -v evo_traj >/dev/null 2>&1; then
    evo_ape tum "$gt_tum" "$est_tum" -a --align --correct_scale -s -v --save_results "$evo_dir/evo_ape.zip" --save_plot "$evo_dir/evo_ape_plot_xy.pdf" --plot_mode xy --no_warnings > "$evo_dir/evo_ape.txt" 2>&1 || echo "evo_ape failed" >> "$status_file"
    evo_rpe tum "$gt_tum" "$est_tum" -a --align --correct_scale -s -v -r trans_part -d 1 -u f --save_results "$evo_dir/evo_rpe.zip" --save_plot "$evo_dir/evo_rpe_plot_xy.pdf" --plot_mode xy --no_warnings > "$evo_dir/evo_rpe.txt" 2>&1 || echo "evo_rpe failed" >> "$status_file"
    evo_traj tum "$est_tum" --ref "$gt_tum" --align --correct_scale --save_plot "$evo_dir/evo_traj_plot_xy.pdf" --plot_mode xy --no_warnings > "$evo_dir/evo_traj.txt" 2>&1 || echo "evo_traj failed" >> "$status_file"
    echo "EVO_REGENERATED" > "$status_file"
  else
    echo "EVO_REGENERATED_CUSTOM_ONLY evo tools not found" > "$status_file"
  fi
}

for root in "${ROOTS[@]}"; do
  [ -d "$root" ] || continue
  while IFS= read -r evo_dir; do
    regen_one "$evo_dir"
  done < <(find "$root" -type f -name "groundtruth_matched.tum" -printf '%h\n' 2>/dev/null | sort -u)
done

echo "[DONE] regenerated AIDVO evo plots"
