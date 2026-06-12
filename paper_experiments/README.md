# UW-IDVO Paper Experiments

This directory is the canonical output workspace for the paper experiments.
Keep temporary debugging outputs outside this directory.

## Experiment Layout

```text
paper_experiments/
├── experiment_config/
├── exp1_parameter_selection/
├── exp2_ablation_harbor/
├── exp3_aquaticvision_generalization/
├── exp4_euroc_sota_comparison/
├── exp5_efficiency_resource/
├── scripts/
├── tables/
├── figures/
└── logs/
```

## Method Names

Only four deterministic modes are used in the paper:

| Method | IDPS | IDKD | Meaning |
| --- | ---: | ---: | --- |
| `ORB_SLAM3` | 0 | 0 | Original ORB-SLAM3 baseline |
| `IDPS` | 1 | 0 | FIM-based informative point selection only |
| `IDKD` | 0 | 1 | Information-degradation keyframe decision only |
| `IDVO` | 1 | 1 | Complete UW-IDVO |

Do not use AIDVO, adaptive policy, rule-based policy, learned policy, or RL names in the main paper experiments.

## Standard Run Directory

Each run should follow this structure:

```text
raw_results/
└── <dataset>/
    └── <sequence>/
        └── <sensor>/
            └── <method>/
                └── run_<N>/
                    ├── settings.yaml
                    ├── trajectory.txt
                    ├── adaptive_frames.csv
                    ├── runtime_perf.csv
                    ├── evaluation_metrics.csv
                    ├── roslaunch.log or offline_stdout.txt
                    ├── evo/
                    │   ├── evo_ape.txt
                    │   ├── evo_rpe.txt
                    │   ├── evo_traj.txt
                    │   ├── evo_ape_plot_xy.pdf
                    │   ├── evo_rpe_plot_xy.pdf
                    │   └── evo_traj_plot_xy.pdf
                    └── metadata.json
```

## Standard Aggregated CSV

Every experiment should eventually produce:

```text
processed_results/expX_all_runs.csv
processed_results/expX_mean_std.csv
```

Recommended fields:

```text
experiment_id,dataset,sequence,sensor,method,run_id,status,
ate_rmse_m,rpe_rmse_m,completeness_percent,
candidate_points_mean,selected_points_mean,selection_ratio_mean,
keyframes_final,map_points_final,
tracking_time_ms_mean,local_ba_time_ms_mean,runtime_sec,memory_mb,
trajectory_file,evo_dir,result_dir,notes
```

## Final Paper Assets

Only final publication-ready assets should be placed in:

```text
tables/
figures/
```

Raw outputs should stay under each experiment's `raw_results/` directory.
