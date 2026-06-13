# UW-IDVO Core Code and AQUA-SLAM Migration Guide

This document summarizes the current UW-IDVO implementation and gives a practical migration path to AQUA-SLAM. The current paper mainline is:

```text
UW-IDVO = IDPS + IDKD
```

The four supported experimental modes are:

| Mode | InfoSelector.Enable | InfoKF.Use | Meaning |
| --- | ---: | ---: | --- |
| ORB_SLAM3 | 0 | 0 | Original ORB-SLAM3 front-end and keyframe logic |
| IDPS | 1 | 0 | FIM-based informative point selection only |
| IDKD | 0 | 1 | FIM-based keyframe decision only |
| IDVO | 1 | 1 | Full UW-IDVO: IDPS + IDKD |

The old AIDVO / adaptive / rule-based / learned-policy modules are not part of the current paper mainline. Some filenames still contain `Adaptive` because they are reused for logging and metrics, but the migration target should focus on the fixed-parameter UW-IDVO path.

## Core Files

Minimum algorithm files to migrate:

| File | Role |
| --- | --- |
| `src/orb_slam3_ros/orb_slam3/include/InfoGain.h` | IDPS data structures and FIM feature-selection API |
| `src/orb_slam3_ros/orb_slam3/src/InfoGain.cc` | IDPS implementation: pose Jacobian, Fisher information, greedy selection |
| `src/orb_slam3_ros/orb_slam3/include/InfoKFPolicy.h` | IDKD parameters/state/API |
| `src/orb_slam3_ros/orb_slam3/src/InfoKFPolicy.cc` | IDKD implementation: information variation, adaptive threshold, cumulative degradation |
| `src/orb_slam3_ros/orb_slam3/include/Tracking.h` | Adds UW-IDVO state, parameters, and helper declarations to Tracking |
| `src/orb_slam3_ros/orb_slam3/src/Tracking.cc` | Main integration points for parameter loading, IDPS execution, IDKD gating, and FIM reference update |

Optional but useful files for experiments and metrics:

| File | Role |
| --- | --- |
| `src/orb_slam3_ros/orb_slam3/include/AdaptiveIDVO.h` | Defines CSV logging state/params and logger classes; policy parts are not required for fixed UW-IDVO |
| `src/orb_slam3_ros/orb_slam3/src/AdaptiveIDVO.cc` | CSV logger implementation and legacy adaptive policy implementation |
| `src/orb_slam3_ros/orb_slam3/include/ORBAdaptiveStateCollector.h` | Collects FIM/tracking/image metrics for `adaptive_frames.csv` |
| `src/orb_slam3_ros/orb_slam3/src/ORBAdaptiveStateCollector.cc` | Metrics collector implementation |

If the first AQUA-SLAM migration should be minimal, port only `InfoGain.*`, `InfoKFPolicy.*`, and the fixed-parameter parts of `Tracking.*`. Add logging after the tracking result is stable.

## Paper Parameters

Use the parameter set already used by the current paper experiments:

```yaml
UWIDVO.Mode: "IDVO"

InfoSelector.Enable: 1
InfoSelector.TopK: 95
InfoSelector.UseUniform: 1
InfoSelector.w_uniform: 0.12
InfoSelector.MinPxDist: 7
InfoSelector.LambdaInit: 1.0e-3

InfoKF.Use: 1
InfoKF.AllowBitsDrop: 2.2
InfoKF.LambdaMean: 1.0e-3
InfoKF.Dyn.Alpha: 0.25
InfoKF.Dyn.Beta: 0.0
InfoKF.Dyn.TauMin: 0.1
InfoKF.Dyn.TauMax: 5.0
InfoKF.Cum.Decay: 0.95
InfoKF.Cum.Thr: 0.5
InfoKF.MaxFramesForce: 300

EnableAdaptiveIDVO: 0
AdaptivePolicyType: "Fixed"
EnableAdaptiveLogging: 1
AdaptiveLogPath: "adaptive_frames.csv"
```

For ablation:

```text
ORB_SLAM3: InfoSelector.Enable=0, InfoKF.Use=0
IDPS:      InfoSelector.Enable=1, InfoKF.Use=0
IDKD:      InfoSelector.Enable=0, InfoKF.Use=1
IDVO:      InfoSelector.Enable=1, InfoKF.Use=1
```

## IDPS Algorithm

IDPS is implemented in `InfoGain`.

Input:

- Current `Frame`
- Candidate 2D-3D matches: keypoint index plus `MapPoint*`
- `InfoSelectParams`

Core logic:

1. Initialize the pose Fisher Information Matrix:

   ```text
   H = lambda * I_6
   ```

2. For each candidate map point, compute the visual reprojection Jacobian:

   ```text
   J_i = d[u, v] / d[se3]
   ```

3. Compute the incremental information gain:

   ```text
   delta_bits = 0.5 * log2 det(I + J_i H^-1 J_i^T)
   ```

4. Optionally compute spatial uniformity score from local keypoint density.

5. Score each point:

   ```text
   score = (1 - alpha) * delta_bits + alpha * uniform_score
   ```

6. Greedily select up to `TopK` points and update:

   ```text
   H <- H + J_i^T J_i
   ```

7. In `Tracking`, non-selected `mCurrentFrame.mvpMapPoints[i]` are set to `nullptr`, so downstream pose optimization/local-map logic only uses selected informative points.

Current ORB-SLAM3 integration point:

- `Tracking.cc`, around the block containing:

  ```cpp
  InfoGain::SelectByInformationGain(mCurrentFrame, cand, frameInfoSelParams);
  ```

Recommended AQUA-SLAM insertion point:

```text
After current frame has a valid pose and candidate MapPoint matches have been established,
before later tracking/keyframe logic consumes mCurrentFrame.mvpMapPoints.
```

In this repository the active implementation is after local-map tracking has produced stable matches. If AQUA-SLAM has a stronger underwater front-end or acoustic/visual fusion stage, place IDPS after that matching/fusion stage so the candidate set reflects AQUA-SLAM's best available observations.

## IDKD Algorithm

IDKD is implemented in `InfoKFPolicy`.

State:

- `H_ref`: FIM of reference keyframe
- `H_mean`: running mean FIM
- `refMatches`: number of valid matches in the reference keyframe
- `cumDrop`: cumulative information degradation
- `framesSinceRef`: frame interval since last reference keyframe

Decision logic:

1. Compute current frame FIM:

   ```text
   H_curr = sum_i J_i^T J_i + lambda * I
   ```

2. Compute information variation:

   ```text
   delta_E = 0.5 * log2(det(H_curr) / det(H_ref))
   ```

3. Compute tracking-quality ratio:

   ```text
   r = n_matches_curr / refMatches
   ```

4. Compute effective threshold:

   ```text
   T_eff = clip(tau0 * (1 - alpha * (1-r)), tau_min, tau_max)
   ```

5. Update cumulative degradation:

   ```text
   D_t = decay * D_{t-1} + min(0, delta_E)
   ```

6. Allow new keyframe if any condition is true:

   ```text
   abs(delta_E) > T_eff
   D_t < -theta_drop
   framesSinceRef >= MaxFramesForce
   ```

Current ORB-SLAM3 integration points:

- `Tracking::NeedNewKeyFrame()`:

  ```cpp
  bool infoAllow = InfoKFPolicy::AllowNewKF(
      H_curr, n_matches_curr, mInfoKFState, frameInfoKFParams);
  if(!infoAllow) return false;
  ```

- `Tracking::CreateNewKeyFrame()`:

  ```cpp
  InfoKFPolicy::OnKeyFrameCreated(H_new, mInfoKFState, frameInfoKFParams);
  ```

Recommended AQUA-SLAM insertion points:

```text
1. Keep AQUA-SLAM's original NeedNewKeyFrame() conditions.
2. Only when original logic says "insert keyframe", run IDKD as an additional information gate.
3. If IDKD rejects, return false.
4. After a keyframe is actually created and inserted, update H_ref by calling OnKeyFrameCreated().
```

This keeps IDKD conservative: it reduces redundant keyframes without forcing keyframes when AQUA-SLAM itself would not insert one.

## Inertial-Mode Protection

In visual-inertial modes, do not let UW-IDVO interfere before VIO is stable.

Current rule:

```cpp
bool Tracking::IsInfoModuleRuntimeReady() const
{
    if visual mode:
        return true;

    if inertial mode:
        return current_map->isImuInitialized() &&
               current_map->GetIniertialBA2();
}
```

Migration requirement for AQUA-SLAM:

- Mono/stereo visual modes: UW-IDVO can run once the current frame has pose and map-point matches.
- Mono-inertial/stereo-inertial modes: bypass IDPS and IDKD until IMU initialization and visual-inertial bootstrap are complete.
- If AQUA-SLAM uses a different flag name for VIO readiness, map it to this condition.
- Fixed IDVO must behave like ORB-SLAM3 before inertial readiness.

This protection is important because IDPS can remove points needed by early VIO bootstrap, and IDKD can disturb the first keyframe sequence required for inertial initialization.

## Tracking Class Changes to Port

Add includes:

```cpp
#include "InfoGain.h"
#include "InfoKFPolicy.h"
```

Add members to `Tracking`:

```cpp
InfoSelectParams mInfoSelParams;
bool mUseInfoSelector = false;

InfoKFParams mInfoKFParams;
InfoKFState mInfoKFState;
```

Add helpers:

```cpp
void LoadInfoParams(cv::FileStorage& fSettings);
bool IsInfoModuleRuntimeReady() const;
InfoSelectParams GetCurrentInfoSelectParams() const;
InfoKFParams GetCurrentInfoKFParams() const;
```

For the current fixed-parameter UW-IDVO, `GetCurrentInfoSelectParams()` and `GetCurrentInfoKFParams()` can simply return `mInfoSelParams` and `mInfoKFParams`. Do not port rule-based policy logic unless a future experiment explicitly needs it.

## Parameter Loading

Read these YAML fields using OpenCV `cv::FileStorage`:

```cpp
InfoSelector.Enable
InfoSelector.TopK
InfoSelector.UseUniform
InfoSelector.w_uniform
InfoSelector.MinPxDist
InfoSelector.LambdaInit

InfoKF.Use
InfoKF.AllowBitsDrop
InfoKF.LambdaMean
InfoKF.Dyn.Alpha
InfoKF.Dyn.Beta
InfoKF.Dyn.TauMin
InfoKF.Dyn.TauMax
InfoKF.Cum.Decay
InfoKF.Cum.Thr
InfoKF.MaxFramesForce
```

Mode mapping can be implemented in the launch/script layer by editing only two booleans:

```text
InfoSelector.Enable
InfoKF.Use
```

## Build-System Changes

For a CMake-based AQUA-SLAM fork, add these source files to the ORB-SLAM3/AQUA-SLAM library target:

```cmake
src/InfoGain.cc
src/InfoKFPolicy.cc
```

If logging is also ported:

```cmake
src/AdaptiveIDVO.cc
src/ORBAdaptiveStateCollector.cc
```

Make sure the target already links/includes:

- Eigen
- OpenCV
- Sophus
- ORB-SLAM3 core headers: `Frame.h`, `MapPoint.h`, `KeyFrame.h`

This repository can compile UW-IDVO conditionally with `ORB3_USE_INFOSEL`. AQUA-SLAM can either:

- keep the same macro and wrap all integration code with `#ifdef ORB3_USE_INFOSEL`, or
- remove the macro and always compile the code while controlling behavior through YAML flags.

For safer migration, keep the macro during the first port.

## Minimal Migration Steps

1. Copy `InfoGain.h/.cc` and `InfoKFPolicy.h/.cc` into AQUA-SLAM's ORB-SLAM3 core.
2. Add them to AQUA-SLAM's build target.
3. Add `InfoSelectParams`, `mUseInfoSelector`, `InfoKFParams`, and `InfoKFState` to AQUA-SLAM `Tracking`.
4. Add YAML parameter loading.
5. Add `IsInfoModuleRuntimeReady()`.
6. Insert IDPS after AQUA-SLAM produces current-frame map-point matches and pose.
7. Insert IDKD at the end of `NeedNewKeyFrame()` as an additional gate.
8. Update IDKD reference FIM in `CreateNewKeyFrame()` after the keyframe is actually created.
9. Add four mode configs: ORB_SLAM3, IDPS, IDKD, IDVO.
10. Run short tests on one sequence, then full experiments.

## IDPS Pseudocode for AQUA-SLAM

```cpp
if(mUseInfoSelector && mCurrentFrame.HasPose() && mState == OK && IsInfoModuleRuntimeReady())
{
    std::vector<MatchInfo> candidates;
    for(int i = 0; i < mCurrentFrame.N; ++i)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP && !pMP->isBad())
            candidates.emplace_back(i, pMP);
    }

    if(!candidates.empty())
    {
        std::vector<int> selected =
            InfoGain::SelectByInformationGain(mCurrentFrame, candidates, mInfoSelParams);

        std::unordered_set<int> keep(selected.begin(), selected.end());
        for(int i = 0; i < mCurrentFrame.N; ++i)
        {
            if(mCurrentFrame.mvpMapPoints[i] && keep.find(i) == keep.end())
                mCurrentFrame.mvpMapPoints[i] = nullptr;
        }
    }
}
```

Practical guardrails:

- If `selected.size()` is too small, keep a minimum fallback set.
- Stereo/stereo-inertial modes usually need a larger fallback than monocular.
- Do not run before IMU readiness in inertial modes.

## IDKD Pseudocode for AQUA-SLAM

```cpp
bool need = OriginalNeedNewKeyFrameLogic();
if(!need)
    return false;

if(mInfoKFParams.use && mCurrentFrame.HasPose() && IsInfoModuleRuntimeReady())
{
    std::vector<int> validIndices;
    for(int i = 0; i < mCurrentFrame.N; ++i)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if(pMP && !pMP->isBad())
            validIndices.push_back(i);
    }

    if(!validIndices.empty())
    {
        Eigen::Matrix<double,6,6> H_curr =
            InfoGain::ComputePoseInformation(mCurrentFrame, validIndices, mInfoKFParams.lambdaMean);

        mInfoKFState.framesSinceRef++;

        if(!InfoKFPolicy::AllowNewKF(H_curr, mnMatchesInliers, mInfoKFState, mInfoKFParams))
            return false;
    }
}

return true;
```

After actual keyframe creation:

```cpp
if(mInfoKFParams.use && mCurrentFrame.HasPose() && IsInfoModuleRuntimeReady())
{
    std::vector<int> validIndices = CollectValidMapPointIndices(mCurrentFrame);
    if(!validIndices.empty())
    {
        mInfoKFState.refMatches = mnMatchesInliers;
        Eigen::Matrix<double,6,6> H_new =
            InfoGain::ComputePoseInformation(mCurrentFrame, validIndices, mInfoKFParams.lambdaMean);
        InfoKFPolicy::OnKeyFrameCreated(H_new, mInfoKFState, mInfoKFParams);
    }
}
```

## Expected Outputs for Experiments

For paper experiments, each sequence/mode should output:

- trajectory file
- keyframe trajectory file if available
- `adaptive_frames.csv` or equivalent metrics CSV
- ATE/RPE from evo
- completeness
- selected point statistics
- keyframe count
- map-point count
- tracking time / Local BA time / runtime
- memory usage if measured by the shell runner

Even if the filename remains `adaptive_frames.csv`, interpret it as the UW-IDVO per-frame metrics log in the current paper mainline.

## Porting Risks

1. **AQUA-SLAM may change `Frame`, `MapPoint`, or camera model interfaces.**  
   `InfoGain` assumes access to `Frame::fx`, `Frame::fy`, `Frame::mvpMapPoints`, `Frame::mvKeysUn`, `Frame::GetPose()`, and `MapPoint::GetWorldPos()`.

2. **Underwater-specific front-end changes may alter candidate match timing.**  
   Insert IDPS after AQUA-SLAM's strongest matching/fusion stage, not before.

3. **Visual-inertial initialization is fragile.**  
   Keep the inertial readiness bypass until AQUA-SLAM's IMU initialization and visual-inertial BA are stable.

4. **IDKD should be conservative.**  
   Use it as an additional gate after original keyframe logic. Do not let it force keyframes in the first migration.

5. **Stereo and underwater datasets may need minimum-retention guards.**  
   If tracking becomes unstable, increase `TopK` or the fallback minimum keep count before changing the FIM logic.

## Recommended Validation Order

1. Compile AQUA-SLAM with `InfoGain` and `InfoKFPolicy` included, but both switches off.
2. Run ORB_SLAM3/AQUA-SLAM baseline and confirm identical behavior.
3. Enable IDPS only on a short visual sequence.
4. Enable IDKD only on the same sequence.
5. Enable full IDVO.
6. Repeat on a water sequence where AQUA-SLAM baseline is already stable.
7. Only after visual modes are stable, test inertial modes.

This sequence isolates integration bugs from dataset difficulty and avoids confusing AQUA-SLAM baseline limitations with UW-IDVO behavior.
