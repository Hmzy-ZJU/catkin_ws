# Experiment Config

Store the exact configuration used by each paper experiment here.

Recommended files:

```text
git_commit.txt
environment.txt
dataset_paths.md
common_parameters.md
run_commands.md
```

Current dataset layout is recorded in `dataset_paths.md`.

Each experiment should copy or reference:

- YAML settings used by ORB-SLAM3/UW-IDVO.
- Shell command used to launch the experiment.
- Git commit hash.
- ROS version, OpenCV version, compiler version, and evo version.
- Dataset root paths on the experiment machine.

Do not edit these files after generating paper tables unless the experiment is intentionally rerun.
