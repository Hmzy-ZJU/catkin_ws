#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import os


def load_tum_trajectory(filename):
    """
    读取 TUM 轨迹文件:
    每行: timestamp tx ty tz qx qy qz qw
    返回: t (N,), xyz (N,3)
    """
    ts = []
    xyz = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line[0] == '#':
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            t = float(parts[0])
            x, y, z = map(float, parts[1:4])
            ts.append(t)
            xyz.append([x, y, z])
    if len(ts) == 0:
        raise RuntimeError(f"No valid trajectory data found in {filename}")
    ts = np.array(ts)
    xyz = np.array(xyz)
    return ts, xyz


def align_trajectory_umeyama(P, Q):
    """
    使用 Umeyama 方法计算相似变换 (scale + R + t)，
    将 P 对齐到 Q（P_est -> Q_gt）。

    P, Q: (N,3)
    返回: s, R, t, P_aligned
    """
    assert P.shape == Q.shape
    n = P.shape[0]
    if n < 2:
        raise RuntimeError("Not enough points for Umeyama alignment.")

    mu_P = P.mean(axis=0)
    mu_Q = Q.mean(axis=0)

    X = P - mu_P
    Y = Q - mu_Q

    # 协方差
    Sigma = (Y.T @ X) / n

    U, D, Vt = np.linalg.svd(Sigma)
    S = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        S[2, 2] = -1.0

    R = U @ S @ Vt

    var_P = np.sum(np.sum(X ** 2, axis=1)) / n
    if var_P < 1e-12:
        raise RuntimeError("Variance of P too small, cannot compute scale.")

    scale = np.trace(np.diag(D) @ S) / var_P
    t = mu_Q - scale * (R @ mu_P)

    P_aligned = (scale * (R @ P.T)).T + t

    return scale, R, t, P_aligned


def compute_ate_rmse(P_aligned, Q):
    """
    计算 ATE RMSE:
    P_aligned, Q: (N,3) 已对齐的估计轨迹 与 真值
    """
    assert P_aligned.shape == Q.shape
    err = P_aligned - Q
    sq = np.sum(err ** 2, axis=1)
    rmse = np.sqrt(np.mean(sq))
    return rmse


def main():
    gt_file = "groundtruth_norm.tum"
    est_file = "est_norm.tum"

    if not os.path.isfile(gt_file):
        raise FileNotFoundError(f"Cannot find {gt_file} in current directory.")
    if not os.path.isfile(est_file):
        raise FileNotFoundError(f"Cannot find {est_file} in current directory.")

    print(f"Loading ground truth from {gt_file}")
    t_gt, p_gt = load_tum_trajectory(gt_file)
    print(f"  Loaded {len(t_gt)} GT poses.")

    print(f"Loading estimated trajectory from {est_file}")
    t_est, p_est = load_tum_trajectory(est_file)
    print(f"  Loaded {len(t_est)} estimated poses.")

    # ===== 关键：不按时间对齐，直接按索引截断 =====
    n = min(len(p_gt), len(p_est))
    print(f"\nIndex-based association:")
    print(f"  Using first {n} poses from both GT and EST for alignment / ATE.")

    P_gt = p_gt[:n]
    P_est = p_est[:n]

    # 对齐 + 计算 ATE
    print("\nAligning estimated trajectory to ground truth (Umeyama)...")
    scale, R, t, P_est_aligned = align_trajectory_umeyama(P_est, P_gt)
    ate_rmse = compute_ate_rmse(P_est_aligned, P_gt)
    print(f"  Scale: {scale:.6f}")
    print(f"  ATE RMSE (index-based pairing): {ate_rmse:.6f} m")

    # 画图：XY 和 XZ
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # XY
    ax_xy = axes[0]
    ax_xy.plot(P_gt[:, 0], P_gt[:, 1], label="Ground Truth")
    ax_xy.plot(P_est_aligned[:, 0], P_est_aligned[:, 1], linestyle="--", label="Estimated (aligned)")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.set_title(f"XY view (ATE RMSE = {ate_rmse:.3f} m, index-based)")
    ax_xy.axis("equal")
    ax_xy.legend()

    # XZ
    ax_xz = axes[1]
    ax_xz.plot(P_gt[:, 0], P_gt[:, 2], label="Ground Truth")
    ax_xz.plot(P_est_aligned[:, 0], P_est_aligned[:, 2], linestyle="--", label="Estimated (aligned)")
    ax_xz.set_xlabel("X [m]")
    ax_xz.set_ylabel("Z [m]")
    ax_xz.set_title(f"XZ view (ATE RMSE = {ate_rmse:.3f} m, index-based)")
    ax_xz.axis("equal")
    ax_xz.legend()

    fig.suptitle("Trajectory Comparison (Ground Truth vs Estimated)", fontsize=14)

    out_file = "traj_xy_xz.png"
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(out_file, dpi=200)
    print(f"\nSaved figure to {out_file}")


if __name__ == "__main__":
    main()

