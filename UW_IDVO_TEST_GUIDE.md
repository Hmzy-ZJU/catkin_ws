# UW-IDVO Test Guide

This guide records the current project convention after removing the AIDVO/adaptive-policy line.

Metric definitions are documented in:

```text
UW_IDVO_METRICS_GUIDE.md
```

Paper experiment outputs are organized in:

```text
paper_experiments/
```

## 1. Method Modes

The project now uses only four deterministic modes:

| Mode | IDPS | IDKD | Meaning |
| --- | ---: | ---: | --- |
| `ORB_SLAM3` | 0 | 0 | Original ORB-SLAM3 baseline |
| `IDPS` | 1 | 0 | FIM-based informative point selection only |
| `IDKD` | 0 | 1 | Information-degradation keyframe decision only |
| `IDVO` | 1 | 1 | Complete UW-IDVO |

Use these names in commands, result folders, figures, and manuscript tables.

Do not use these names in the main paper workflow:

```text
AIDVO
adaptive
rule-based
learned policy
RL policy
```

## 2. Expected YAML Switches

Every generated experiment config should explicitly contain:

```yaml
UWIDVO.Mode: "IDVO"
InfoSelector.Enable: 1
InfoKF.Use: 1
EnableAdaptiveIDVO: 0
AdaptivePolicyType: "Fixed"
```

For each mode:

| Mode | `InfoSelector.Enable` | `InfoKF.Use` |
| --- | ---: | ---: |
| `ORB_SLAM3` | 0 | 0 |
| `IDPS` | 1 | 0 |
| `IDKD` | 0 | 1 |
| `IDVO` | 1 | 1 |

`EnableAdaptiveIDVO` should stay `0` in all paper experiments.

## 3. Current Available Script

The currently retained AquaticVision script is:

```text
test_idvo_aquaticvision_20260613.sh
```

Example quick run:

```bash
cd ~/catkin_ws; source /opt/ros/noetic/setup.bash; source devel/setup.bash; AQUATIC_ROOT=~/catkin_ws/dataset_AquaticVision AQUATIC_BASELINE=0.11601670 SEQUENCES="01" SENSORS="mono stereo" UWIDVO_MODES=ORB_SLAM3,IDPS,IDKD,IDVO MAX_FRAMES=300 RUNS_PER_CASE=1 DO_BUILD=0 bash test_idvo_aquaticvision_20260613.sh
```

The script should produce:

```text
summary_aquaticvision_uwidvo.csv
trajectory files
evo/evo_ape.txt
evo/evo_rpe.txt
evo/evo_traj.txt
evo/evo_ape_plot_xy.pdf
evo/evo_rpe_plot_xy.pdf
evo/evo_traj_plot_xy.pdf
```

## 4. Paper Experiment Plan

| Experiment | Dataset | Sensor Mode | Runtime Mode | Status |
| --- | --- | --- | --- | --- |
| Exp. 1 Parameter selection | AQUALOC Harbor | mono | previous results | keep existing paper data |
| Exp. 2 Ablation | AQUALOC Harbor | mono | previous results | keep existing paper data |
| Exp. 3 Generalization | AquaticVision | mono, stereo | ROS online | rerun |
| Exp. 4 SOTA comparison | EuRoC MH01-MH05 | stereo-inertial | offline direct reading | rerun |
| Exp. 5 Efficiency/resource | AQUALOC, AquaticVision, EuRoC | mono, stereo | ROS online | rerun or collect from logs |

Detailed output layout is in:

```text
paper_experiments/README.md
```

## 5. Per-Run Required Files

Each run should keep:

```text
settings.yaml
trajectory.txt
adaptive_frames.csv
runtime_perf.csv
evaluation_metrics.csv
roslaunch.log or offline_stdout.txt
evo/
metadata.json
```

## 6. Success Criteria

A run is usable for paper statistics when:

- The process exits normally.
- A trajectory file exists and has enough poses.
- `completeness_percent` is not abnormally low for that dataset/sequence.
- ATE/RPE can be computed with evo or the local evaluator.
- The config matches the intended mode.
- Runtime logs include enough data for selected points, keyframes, map points, tracking time, local BA time, and memory.

## 7. Recommended Workflow

1. Run a short sanity test for one sequence.
2. Check generated YAML switches.
3. Check `summary_*.csv`.
4. Check evo figures.
5. Run the full experiment.
6. Copy or aggregate outputs into `paper_experiments/exp*/raw_results/`.
7. Generate `processed_results/expX_all_runs.csv`.
8. Generate final paper tables and figures under `paper_experiments/tables/` and `paper_experiments/figures/`.
