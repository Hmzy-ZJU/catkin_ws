#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Modified by Raul Mur-Artal
# Automatically compute the optimal scale factor for monocular VO/SLAM.

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements:
#   pip install numpy matplotlib
#   (depends on local module: associate.py)

"""
Compute Absolute Trajectory Error (ATE) between ground-truth and estimated
trajectories, with closed-form Horn alignment and automatic scale recovery.
"""

from __future__ import annotations

import sys
import argparse
from typing import Tuple, Sequence

import numpy as np
import associate


def align(model: np.ndarray, data: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Align two trajectories using the method of Horn (closed-form).

    Parameters
    ----------
    model : np.ndarray
        First trajectory, shape (3, N).
    data : np.ndarray
        Second trajectory, shape (3, N).

    Returns
    -------
    rot : np.ndarray
        Rotation matrix, shape (3, 3).
    transGT : np.ndarray
        Translation vector with scale, shape (3, 1).
    trans_errorGT : np.ndarray
        Per-point translational error with scale, shape (N,).
    trans : np.ndarray
        Translation vector without scale, shape (3, 1).
    trans_error : np.ndarray
        Per-point translational error without scale, shape (N,).
    s : float
        Optimal scale.
    """
    np.set_printoptions(precision=3, suppress=True)

    # zero-center
    model_mean = model.mean(axis=1, keepdims=True)
    data_mean = data.mean(axis=1, keepdims=True)
    model_c = model - model_mean
    data_c = data - data_mean

    # cross-covariance; follow original (svd on transpose)
    H = model_c @ data_c.T
    U, _, Vt = np.linalg.svd(H.T)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    rot = U @ S @ Vt

    # scale
    rot_model = rot @ model_c
    dots = np.sum(np.einsum("ij,ij->j", data_c, rot_model))
    norms = np.sum(np.linalg.norm(model_c, axis=0) ** 2)
    s = float(dots / norms) if norms > 0 else 1.0

    # translations (with and without applying scale)
    transGT = data_mean - s * rot @ model_mean
    trans = data_mean - rot @ model_mean

    # aligned trajectories
    model_alignedGT = s * rot @ model + transGT
    model_aligned = rot @ model + trans

    # per-point euclidean errors
    trans_errorGT = np.linalg.norm(model_alignedGT - data, axis=0)
    trans_error = np.linalg.norm(model_aligned - data, axis=0)

    return rot, transGT, trans_errorGT, trans, trans_error, s


# 修改 plot_traj：允许透传绘图参数（linewidth、zorder 等）
def plot_traj(ax, stamps, traj, style, color, label, **kwargs):
    if len(stamps) == 0:
        return
    if len(stamps) > 1:
        intervals = [s - t for s, t in zip(stamps[1:], stamps[:-1])]
        interval = float(np.median(intervals))
    else:
        interval = 0.0

    xs, ys = [], []
    last = stamps[0]
    for i, t in enumerate(stamps):
        if (t - last) < 2 * interval if interval > 0 else True:
            xs.append(float(traj[i, 0]))
            ys.append(float(traj[i, 1]))
        elif xs:
            ax.plot(xs, ys, style, color=color, label=label, **kwargs)
            label = ""
            xs, ys = [], []
        last = t
    if xs:
        ax.plot(xs, ys, style, color=color, label=label, **kwargs)



def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Compute the absolute trajectory error from the ground-truth "
            "trajectory and the estimated trajectory."
        )
    )
    parser.add_argument("first_file", help="ground truth trajectory (timestamp tx ty tz qx qy qz qw)")
    parser.add_argument("second_file", help="estimated trajectory (timestamp tx ty tz qx qy qz qw)")
    parser.add_argument("--offset", type=float, default=0.0,
                        help="time offset added to the timestamps of the second file (default: 0.0)")
    parser.add_argument("--scale", type=float, default=1.0,
                        help="scaling factor for the second trajectory (default: 1.0)")
    parser.add_argument("--max_difference", type=float, default=20_000_000.0,
                        help="max allowed time difference for matching entries (default: 20000000)")
    parser.add_argument("--save", type=str,
                        help="save aligned (ROT+TRANS) second trajectory to disk (format: stamp2 x2 y2 z2)")
    parser.add_argument("--save_associations", type=str,
                        help=("save associated first and aligned second trajectory "
                              "(format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)"))
    parser.add_argument("--plot", type=str,
                        help="plot the first and the aligned second trajectory to an image file (e.g., out.pdf)")
    parser.add_argument("--verbose", action="store_true",
                        help="print full evaluation data (default prints RMSE and scale only)")
    parser.add_argument("--verbose2", action="store_true",
                        help="print scale error and RMSE with/without scale correction")
    args = parser.parse_args()

    # read files
    first_list = associate.read_file_list(args.first_file, False)
    second_list = associate.read_file_list(args.second_file, False)

    # associate by timestamp
    matches = associate.associate(first_list, second_list, float(args.offset), float(args.max_difference))
    if len(matches) < 2:
        sys.exit(
            "Couldn't find matching timestamp pairs between ground-truth and estimated trajectory! "
            "Did you choose the correct sequence?"
        )

    # matched xyz (3 x N)
    first_xyz = np.array([[float(v) for v in first_list[a][0:3]] for a, _ in matches], dtype=float).T
    second_xyz = np.array([[float(v) * float(args.scale) for v in second_list[b][0:3]] for _, b in matches],
                          dtype=float).T

    # full (sorted by time) for saving/plotting
    sorted_second = sorted(second_list.items(), key=lambda kv: kv[0])
    second_xyz_full_unsorted = np.array([[float(v) * float(args.scale) for v in item[1][0:3]]
                                         for item in sorted_second], dtype=float).T

    # alignment
    rot, transGT, trans_errorGT, trans, trans_error, scale = align(second_xyz, first_xyz)

    # aligned/mapped sets
    second_xyz_aligned = scale * (rot @ second_xyz) + trans
    second_xyz_notscaled = (rot @ second_xyz) + trans
    second_xyz_notscaled_full = (rot @ second_xyz_full_unsorted) + trans

    first_stamps = sorted(first_list.keys())
    first_xyz_full = np.array([[float(v) for v in first_list[k][0:3]] for k in first_stamps], dtype=float)

    second_stamps = sorted(second_list.keys())
    second_xyz_full = np.array([[float(v) * float(args.scale) for v in second_list[k][0:3]]
                                for k in second_stamps], dtype=float)
    second_xyz_full_aligned = (scale * (rot @ second_xyz_full.T) + trans).T  # (N,3)

    # reporting
    rmse_no_scale = float(np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)))
    rmse_with_scale = float(np.sqrt(np.dot(trans_errorGT, trans_errorGT) / len(trans_errorGT)))

    if args.verbose:
        print(f"compared_pose_pairs {len(trans_error)} pairs")
        print(f"absolute_translational_error.rmse {rmse_no_scale:.6f} m")
        print(f"absolute_translational_error.mean {np.mean(trans_error):.6f} m")
        print(f"absolute_translational_error.median {np.median(trans_error):.6f} m")
        print(f"absolute_translational_error.std {np.std(trans_error):.6f} m")
        print(f"absolute_translational_error.min {np.min(trans_error):.6f} m")
        print(f"absolute_translational_error.max {np.max(trans_error):.6f} m")
        print(f"max idx: {int(np.argmax(trans_error))}")
    else:
        # Keep the concise CSV-like output: RMSE(no-scale), scale, RMSE(with-scale)
        print(f"{rmse_no_scale:.6f},{scale:.6f},{rmse_with_scale:.6f}")

    if args.verbose2:
        print(f"compared_pose_pairs {len(trans_error)} pairs")
        print(f"absolute_translational_error.rmse {rmse_no_scale:.6f} m")
        print(f"absolute_translational_errorGT.rmse {rmse_with_scale:.6f} m")

    # save associations (GT vs aligned estimate)
    if args.save_associations:
        with open(args.save_associations, "w", encoding="utf-8") as f:
            lines = []
            for (a, b), (x1, y1, z1), (x2, y2, z2) in zip(
                matches, first_xyz.T, second_xyz_aligned.T
            ):
                lines.append(f"{a:.6f} {x1:.6f} {y1:.6f} {z1:.6f} {b:.6f} {x2:.6f} {y2:.6f} {z2:.6f}")
            f.write("\n".join(lines))

    # save aligned (rot+trans) estimated trajectory (no per-match scale reapplication here)
    if args.save:
        with open(args.save, "w", encoding="utf-8") as f:
            for stamp, line in zip(second_stamps, second_xyz_notscaled_full.T):
                x, y, z = line.tolist()
                f.write(f"{stamp:.6f} {x:.6f} {y:.6f} {z:.6f}\n")

    # plot
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # (N,3) arrays expected by plot_traj
        plot_traj(ax, first_stamps, first_xyz_full, "-", "black", "ground truth")
        plot_traj(ax, second_stamps, second_xyz_full_aligned, "-", "blue", "estimated")

        label = "difference"
        for (_, _), (x1, y1, _), (x2, y2, _) in zip(matches, first_xyz.T, second_xyz_aligned.T):
            ax.plot([x1, x2], [y1, y2], "-", color="red", label=label)
            label = ""

        ax.legend()
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        fig.tight_layout()
        plt.savefig(args.plot)


if __name__ == "__main__":
    main()

