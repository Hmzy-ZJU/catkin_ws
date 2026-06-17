# Run Config

Exp. 5 uses the complete YAML files in this directory as the source of camera,
ORB, IDPS, and IDKD parameters.

- `exp5_config.env`: experiment matrix and runtime options.
- `harbor_mono.yaml`: AQUALOC Harbor mono configuration.
- `euroc_mono.yaml`: EuRoC mono configuration.
- `euroc_stereo.yaml`: EuRoC stereo configuration.
- `aquaticvision_mono.yaml`: AquaticVision mono configuration.
- `aquaticvision_stereo.yaml`: AquaticVision stereo configuration.

The runner only injects the current method switch (`ORB_SLAM3`, `IDPS`, `IDKD`,
or `IDVO`) and the per-run log path. To change `kappa_top`, `alpha`, `tau0`, or
other algorithm parameters, edit the YAML files here.

Current UW-IDVO paper setting for Exp. 5:

- AQUALOC Harbor mono and AquaticVision mono: `InfoSelector.TopK=115`, `InfoKF.AllowBitsDrop=2.0`
- AquaticVision stereo: `InfoSelector.TopK=150`, `InfoKF.AllowBitsDrop=2.3`, `InfoKF.MaxFramesForce=120`
- EuRoC mono: `InfoSelector.TopK=350`, `InfoKF.AllowBitsDrop=2.8`, `InfoKF.MaxFramesForce=120`
- EuRoC stereo: `InfoSelector.TopK=220`, `InfoKF.AllowBitsDrop=7.5`, `InfoKF.MaxFramesForce=300`
