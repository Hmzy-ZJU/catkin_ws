# UW-IDVO Metrics Guide

This document defines the metrics that should be collected for each paper experiment run.
It is intentionally independent of old temporary test scripts.

## 1. Required Files Per Run

Each run directory should keep:

```text
settings.yaml
trajectory.txt
adaptive_frames.csv
runtime_perf.csv
evaluation_metrics.csv
roslaunch.log or offline_stdout.txt
evo/evo_ape.txt
evo/evo_rpe.txt
evo/evo_traj.txt
evo/evo_ape_plot_xy.pdf
evo/evo_rpe_plot_xy.pdf
evo/evo_traj_plot_xy.pdf
metadata.json
```

## 2. Main Metrics

| Category | Metric | Source | Meaning |
| --- | --- | --- | --- |
| Accuracy | `ate_rmse_m` | evo APE / `evaluation_metrics.csv` | Absolute trajectory error RMSE in meters. |
| Accuracy | `rpe_rmse_m` | evo RPE / `evaluation_metrics.csv` | Relative pose error RMSE in meters. |
| Stability | `completeness_percent` | summary / `evaluation_metrics.csv` | Percentage of the sequence with valid trajectory output. |
| Lightweight | `candidate_points_mean` | `adaptive_frames.csv` | Mean candidate map point matches. |
| Lightweight | `selected_points_mean` | `adaptive_frames.csv` | Mean selected informative points after IDPS. |
| Lightweight | `selection_ratio_mean` | `adaptive_frames.csv` | `selected_points / candidate_points`. |
| Map size | `keyframes_final` | `adaptive_frames.csv` / `runtime_perf.csv` | Final number of keyframes. |
| Map size | `map_points_final` | `adaptive_frames.csv` / `runtime_perf.csv` | Final number of map points. |
| Efficiency | `tracking_time_ms_mean` | `adaptive_frames.csv` / `runtime_perf.csv` | Mean per-frame tracking time. |
| Efficiency | `local_ba_time_ms_mean` | `adaptive_frames.csv` / `runtime_perf.csv` | Mean Local BA time. |
| Efficiency | `runtime_sec` | summary / run script | Wall-clock runtime of the case. |
| Resource | `memory_mb` | `runtime_perf.csv` | Process RSS memory in MB. |

## 3. Completeness

Completeness measures how much of a sequence is covered by valid trajectory output.

For ROS online runs:

```text
completeness_percent = 100 * trajectory_duration_sec / played_duration_sec
```

where:

- `trajectory_duration_sec` is the timestamp span of the saved trajectory.
- `played_duration_sec` is the rosbag playback duration.

For offline direct-reading runs:

```text
completeness_percent = 100 * trajectory_poses / input_frames
```

where:

- `trajectory_poses` is the number of valid pose rows in the output trajectory.
- `input_frames` is the number of image timestamps.

Interpretation:

- Near `100%`: trajectory is almost complete.
- Low value, for example `20%`: the system likely failed early, repeatedly reset, or did not save poses for most frames.

## 4. Tracking Time

Tracking time is the per-frame cost of the ORB-SLAM3 tracking pipeline.

Current code records it with `std::chrono::steady_clock` around the tracking call:

```text
tracking_time_ms = time_after_tracking - time_before_tracking
```

Sources:

- Per-frame: `adaptive_frames.csv`, field `tracking_time_ms`.
- Per-run mean: `runtime_perf.csv`, field `track_ms_mean`.

This is algorithm processing time. It is not the same as rosbag playback time or full case runtime.

## 5. Local BA Time

Local BA time is the cost of Local Bundle Adjustment in the local mapping thread.

Sources:

- Per-frame latest value: `adaptive_frames.csv`, field `recent_local_ba_time_ms`.
- Per-run mean: `runtime_perf.csv`, field `ba_ms_mean`.

If Local BA is not triggered in a short sequence, the value may be `0`.

## 6. Runtime

Runtime is wall-clock time for one case:

```text
runtime_sec = case_end_time - case_start_time
```

It may include:

- program startup;
- dataset playback or offline image reading;
- tracking and local mapping;
- trajectory saving;
- metric generation, depending on script placement.

Use runtime for whole-experiment cost, and tracking/local BA time for algorithm-level analysis.

## 7. Memory

Memory is recorded as process RSS:

```text
memory_mb = resident_pages * page_size / 1024 / 1024
```

The current implementation reads:

```text
/proc/self/statm
```

Meaning:

- It is ORB-SLAM3 process resident memory.
- It is not total system memory.
- It is not GPU memory.
- It is the RSS at export time, not necessarily the peak RSS.

For peak memory, use an external tool such as:

```text
/usr/bin/time -v
```

and read `Maximum resident set size`.

## 8. Recommended Unified Summary Fields

Every paper experiment should eventually export:

```text
experiment_id
dataset
sequence
sensor
method
run_id
status
ate_rmse_m
rpe_rmse_m
completeness_percent
candidate_points_mean
selected_points_mean
selection_ratio_mean
keyframes_final
map_points_final
tracking_time_ms_mean
local_ba_time_ms_mean
runtime_sec
memory_mb
trajectory_file
evo_dir
result_dir
notes
```
