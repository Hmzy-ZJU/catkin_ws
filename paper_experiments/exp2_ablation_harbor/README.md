# Exp. 2 Ablation on AQUALOC Harbor

## Purpose

Verify that IDPS and IDKD are individually useful, and that complete UW-IDVO performs best overall.

## Dataset

- AQUALOC Harbor

## Sensor Mode

- `mono`

## Methods

```text
ORB_SLAM3
IDPS
IDKD
IDVO
```

## Current Status

The previous paper results can still be used. If rerun, keep the four method names above.

## Expected Outputs

```text
raw_results/AQUALOC_Harbor/
processed_results/exp2_all_runs.csv
processed_results/exp2_mean_std.csv
tables/table_exp2_ablation.csv
tables/table_exp2_ablation.tex
figures/exp2_ablation_trajectory.pdf
```
