# Dataset Paths

This file records the dataset layout used by the paper experiment scripts.

## AquaticVision

Root:

```text
~/catkin_ws/dataset_AquaticVision
```

Expected layout:

```text
dataset_AquaticVision/
|-- data/
|   |-- 01_Scan_with_board/
|   |   |-- data rosbag/
|   |   |   |-- Scan_with_board_imu_1000hz_images_30hz.bag
|   |   |   `-- Scan_with_board_imu_200hz_images_20hz.bag
|   |   |-- groundtruth/
|   |   |   |-- gt.csv
|   |   |   `-- gt.tum
|   |   `-- Stereo images/
|   |       |-- l1/
|   |       |-- r1/
|   |       `-- timestamp_pairs.txt
|   |-- 02_Cross1_with_board/
|   |-- 03_Cross2_no_board/
|   |-- 04_Loop1_with_board/
|   |-- 05_Loop2_no_board/
|   |-- 06_Dark1_with_board/
|   |-- 07_Dark2_with_board/
|   |-- 08_HDR/
|   |-- 09_Blur/
|   `-- calibration/
|       |-- cam0_pinhole.yaml
|       |-- cam1_pinhole.yaml
|       `-- davis_imucam_underwater.yaml
|-- GT/
`-- results/
```

Exp.3 and Exp.5 use the `*_imu_200hz_images_20hz.bag` bag by default.

## EuRoC

Root:

```text
~/catkin_ws/dataset_EuRoc
```

Expected layout:

```text
dataset_EuRoc/
|-- data/
|   |-- MH_01.bag
|   |-- MH_02.bag
|   |-- MH_03.bag
|   |-- MH_04.bag
|   |-- MH_05.bag
|   |-- V1_01.bag
|   |-- V1_02.bag
|   |-- V2_01.bag
|   `-- V2_02.bag
|-- GT/
|   |-- MH_01.csv
|   |-- MH_02.csv
|   |-- MH_03.csv
|   |-- MH_04.csv
|   |-- MH_05.csv
|   |-- V1_01.csv
|   |-- V1_02.csv
|   |-- V2_01.csv
|   `-- V2_02.csv
`-- results/
```

Exp.4 uses `MH_01` to `MH_05` and converts `GT/*.csv` to TUM format before running evo.

## AQUALOC Harbor

Root:

```text
~/catkin_ws/dataset_harbor
```

Expected layout:

```text
dataset_harbor/
|-- data/
|   |-- harbor_sequence_1.bag
|   |-- harbor_sequence_2.bag
|   |-- harbor_sequence_3.bag
|   |-- harbor_sequence_4.bag
|   |-- harbor_sequence_5.bag
|   |-- harbor_sequence_6.bag
|   `-- harbor_sequence_7.bag
|-- GT/
|   |-- harbor_colmap_traj_sequence_01.txt
|   |-- harbor_colmap_traj_sequence_02.txt
|   |-- harbor_colmap_traj_sequence_03.txt
|   |-- harbor_colmap_traj_sequence_04.txt
|   |-- harbor_colmap_traj_sequence_05.txt
|   |-- harbor_colmap_traj_sequence_06.txt
|   `-- harbor_colmap_traj_sequence_07.txt
`-- results/
```

Harbor is treated as monocular for the main UW-IDVO paper experiments.
