# Paper Experiment Scripts

Place only paper-level orchestration and post-processing scripts here.

Recommended script types:

```text
run_exp3_aquaticvision_ros.sh
run_exp4_euroc_offline.sh
run_exp5_efficiency_ros.sh
collect_exp_metrics.py
make_paper_tables.py
make_paper_figures.py
```

The current repository may not contain all final scripts yet. Avoid referencing deleted historical scripts in paper documentation.

## Script Rules

- Use method names `ORB_SLAM3`, `IDPS`, `IDKD`, `IDVO`.
- Save outputs into `paper_experiments/exp*/raw_results/`.
- Generate one `metadata.json` per run.
- Generate one `processed_results/expX_all_runs.csv` per experiment.
- Do not write temporary debugging outputs into `tables/` or `figures/`.
