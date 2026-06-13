# Run Config

Exp. 4 uses the complete YAML file in this directory as the source of camera,
IMU, ORB, IDPS, and IDKD parameters.

- `exp4_config.env`: experiment matrix and runtime options.
- `euroc_mono_inertial.yaml`: EuRoC mono-inertial configuration.
- `euroc_stereo_inertial.yaml`: EuRoC stereo-inertial configuration.

The runner only injects the current method switch (`ORB_SLAM3`, `IDPS`, `IDKD`,
or `IDVO`) and the per-run log path. To change `kappa_top`, `alpha`, `tau0`, or
other algorithm parameters, edit the YAML file here.

For the paper SOTA comparison table, the default matrix is:

- `SENSORS="mono-inertial stereo-inertial"`
- `UWIDVO_MODES=ORB_SLAM3,IDVO`
- `SEQUENCES="MH_01 MH_02 MH_03 MH_04 MH_05"`
- `RUNS_PER_CASE=10`

After the run, `build_exp4_paper_table.py` creates:

- `processed_results/exp4_paper_sota_table_<RUN_TAG>.csv`
- `processed_results/exp4_paper_sota_table_<RUN_TAG>.md`
