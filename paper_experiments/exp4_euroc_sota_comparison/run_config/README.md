# Run Config

Exp. 4 uses the complete YAML file in this directory as the source of camera,
IMU, ORB, IDPS, and IDKD parameters.

- `exp4_config.env`: experiment matrix and runtime options.
- `euroc_stereo_inertial.yaml`: EuRoC stereo-inertial configuration.

The runner only injects the current method switch (`ORB_SLAM3`, `IDPS`, `IDKD`,
or `IDVO`) and the per-run log path. To change `kappa_top`, `alpha`, `tau0`, or
other algorithm parameters, edit the YAML file here.
