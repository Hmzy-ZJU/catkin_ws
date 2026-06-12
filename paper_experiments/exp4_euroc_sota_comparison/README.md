# Exp. 4 EuRoC SOTA Comparison

## Purpose

Compare UW-IDVO with ORB-SLAM3 and reported SOTA results such as PKS and MSJCA-KS on the standard EuRoC benchmark.

## Dataset

- EuRoC

## Sequences

```text
MH_01
MH_02
MH_03
MH_04
MH_05
```

## Sensor Mode

```text
stereo-inertial
```

## Runtime Mode

- Offline direct reading

## Methods

```text
ORB_SLAM3
IDPS
IDKD
IDVO
```

PKS and MSJCA-KS can be included as reported literature values if their code is not rerun locally.

## Repeats

- 10 runs per sequence/method.

## Main Metrics

- ATE RMSE mean/std
- RPE RMSE mean/std
- Completeness

## Expected Outputs

```text
raw_results/EuRoC/
processed_results/exp4_all_runs.csv
processed_results/exp4_mean_std.csv
processed_results/exp4_sota_reported_values.csv
tables/table_exp4_sota_ate.csv
tables/table_exp4_sota_ate.tex
figures/exp4_euroc_ate_bar.pdf
```

## Runner

Config:

```text
run_config/exp4_config.env
```

Run script:

```text
run_exp4_euroc_sota_comparison.sh
```

Quick smoke test:

```bash
cd ~/catkin_ws; RUNS_PER_CASE=1 MAX_FRAMES=300 SEQUENCES="MH_01" bash paper_experiments/exp4_euroc_sota_comparison/run_exp4_euroc_sota_comparison.sh
```

Full run:

```bash
cd ~/catkin_ws; RUNS_PER_CASE=10 MAX_FRAMES=0 SEQUENCES="MH_01 MH_02 MH_03 MH_04 MH_05" bash paper_experiments/exp4_euroc_sota_comparison/run_exp4_euroc_sota_comparison.sh
```
