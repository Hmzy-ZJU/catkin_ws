# Run Config

Exp. 3 uses the complete YAML files in this directory as the source of camera,
ORB, IDPS, and IDKD parameters.

- `exp3_config.env`: experiment matrix and runtime options.
- `aquaticvision_mono.yaml`: AquaticVision mono configuration.
- `aquaticvision_stereo.yaml`: AquaticVision stereo configuration.

`AQUATIC_SOURCE=stereo_images` is the default and publishes the exported
AquaticVision image pairs as ROS topics. Set `AQUATIC_SOURCE=rosbag` only when
checking the original recorded ROS topics.

The runner only injects the current method switch (`ORB_SLAM3`, `IDPS`, `IDKD`,
or `IDVO`) and the per-run log path. To change `kappa_top`, `alpha`, `tau0`, or
other algorithm parameters, edit the YAML files here.
