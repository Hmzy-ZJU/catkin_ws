# Exp. 3 AquaticVision Generalization

## Purpose

Show that UW-IDVO generalizes to a new public underwater dataset, not only AQUALOC Harbor.

## Dataset

- AquaticVision

## Sequences

```text
01_Scan_with_board
02_Cross1_with_board
03_Cross2_no_board
04_Loop1_with_board
05_Loop2_no_board
06_Dark1_with_board
07_Dark2_with_board
08_HDR
09_Blur
```

## Sensor Modes

```text
mono
stereo
```

## Runtime Mode

- ROS online

## Methods

```text
ORB_SLAM3
IDPS
IDKD
IDVO
```

## Repeats

- Recommended: 3 runs per sequence/sensor/method.
- Quick check: 1 run per sequence/sensor/method.

## Main Metrics

- ATE RMSE
- RPE RMSE
- Completeness

## Supporting Metrics

- Selected points
- Keyframes
- Map points
- Tracking time
- Runtime
- Memory

## Expected Outputs

```text
raw_results/AquaticVision/
processed_results/exp3_all_runs.csv
processed_results/exp3_mean_std.csv
processed_results/exp3_failure_cases.csv
tables/table_exp3_aquaticvision.csv
tables/table_exp3_aquaticvision.tex
figures/exp3_aquaticvision_trajectory_examples.pdf
figures/exp3_aquaticvision_ate_boxplot.pdf
```

## Runner

Config:

```text
run_config/exp3_config.env
```

Run script:

```text
run_exp3_aquaticvision_generalization.sh
```

Quick smoke test:

```bash
cd ~/catkin_ws; RUNS_PER_CASE=1 BAG_DURATION=30 SEQUENCES="01" SENSORS="mono stereo" UWIDVO_MODES=ORB_SLAM3,IDVO bash paper_experiments/exp3_aquaticvision_generalization/run_exp3_aquaticvision_generalization.sh
```

Full run:

```bash
cd ~/catkin_ws; RUNS_PER_CASE=3 BAG_DURATION=0 bash paper_experiments/exp3_aquaticvision_generalization/run_exp3_aquaticvision_generalization.sh
```
