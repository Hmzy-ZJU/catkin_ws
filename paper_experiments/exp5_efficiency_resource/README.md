# Exp. 5 Efficiency and Resource Usage

## Purpose

Show that UW-IDVO improves efficiency by reducing selected points, keyframes, map points, Local BA cost, runtime, and memory usage.

## Datasets and Modes

| Dataset | Sequences | Sensor Modes | Runtime Mode |
| --- | --- | --- | --- |
| AQUALOC Harbor | valid mono sequences | mono | ROS online |
| AquaticVision | all 9 sequences | mono, stereo | ROS online |
| EuRoC | MH01-MH05 | mono, stereo | ROS online |

## Methods

```text
ORB_SLAM3
IDPS
IDKD
IDVO
```

## Repeats

- Recommended: 3 runs per sequence/sensor/method.

## Metrics

| Category | Metrics |
| --- | --- |
| Lightweight mapping | candidate points, selected points, selection ratio |
| Map size | keyframes, map points |
| Computation | tracking time, local BA time, runtime |
| Resource | memory RSS |
| Auxiliary | ATE, RPE, completeness |

## Reduction Ratio

Use ORB-SLAM3 as the baseline:

```text
reduction_percent = (ORB_SLAM3 - method) / ORB_SLAM3 * 100
```

Report reduction for:

- selected points
- keyframes
- map points
- tracking time
- local BA time
- memory

## Expected Outputs

```text
raw_results/
processed_results/exp5_all_runtime_metrics.csv
processed_results/exp5_efficiency_mean_std.csv
processed_results/exp5_reduction_ratio.csv
processed_results/exp5_memory_stats.csv
tables/table_exp5_runtime.csv
tables/table_exp5_map_size.csv
tables/table_exp5_memory.csv
tables/table_exp5_reduction.tex
figures/exp5_selected_points_bar.pdf
figures/exp5_keyframes_bar.pdf
figures/exp5_tracking_time_bar.pdf
figures/exp5_local_ba_time_bar.pdf
figures/exp5_memory_bar.pdf
```

## Runner

Config:

```text
run_config/exp5_config.env
```

Run script:

```text
run_exp5_efficiency_resource.sh
```

Quick smoke test:

```bash
cd ~/catkin_ws; DATASETS="euroc" EUROC_SEQUENCES="MH_01" SENSORS="mono stereo" RUNS_PER_CASE=1 BAG_DURATION=30 UWIDVO_MODES=ORB_SLAM3,IDVO bash paper_experiments/exp5_efficiency_resource/run_exp5_efficiency_resource.sh
```

Full run:

```bash
cd ~/catkin_ws; DATASETS="harbor aquaticvision euroc" RUNS_PER_CASE=3 BAG_DURATION=0 UWIDVO_MODES=ORB_SLAM3,IDPS,IDKD,IDVO bash paper_experiments/exp5_efficiency_resource/run_exp5_efficiency_resource.sh
```
