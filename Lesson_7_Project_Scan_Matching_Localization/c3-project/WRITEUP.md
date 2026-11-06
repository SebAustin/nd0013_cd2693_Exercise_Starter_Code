# Scan Matching Localization — Write-up

Localizes a manually driven CARLA car by matching each lidar **Scan** against the prebuilt **Map** (`map.pcd`) with PCL's Normal Distributions Transform (NDT). Terms follow [`GLOSSARY.md`](../../GLOSSARY.md).

## Pipeline (per Scan)

1. **Snapshot** the Scan from the lidar buffer and re-arm the listener (fixes a race in the starter loop where the lidar thread could overwrite the cloud mid-processing).
2. **Voxel filter** the Scan (`kScanLeafSize = 0.5 m`) to cut the point count NDT has to score.
3. **Initial guess** from a constant-velocity motion model: the last inter-Scan motion, expressed in the car frame, is applied once more. If that step exceeds 3 m it is treated as a bad match and the guess falls back to the current Pose estimate.
4. **NDT match** the filtered Scan to the Map (`NdtLocalizer`); the result is the new **Pose estimate** (`getPose`).
5. **Transform** the filtered Scan into the Map frame with that estimate and render it in red.

The **ground-truth pose** is read once, to seed the very first Pose estimate; after that, localization uses lidar only.

## Files

| File | Role |
|---|---|
| `c3-main.cpp` | Simulator loop; calls the three steps above |
| `localization.h/.cpp` | `voxelFilter`, `poseToMatrix`, `predictNextTransform`, `NdtLocalizer` |
| `CMakeLists.txt` | Adds `localization.cpp` to the build |

## NDT parameters (`NdtConfig`)

| Parameter | Value | Why |
|---|---|---|
| resolution | 1.0 m | Cell size fine enough for road-side structure, coarse enough to be robust |
| step size | 0.1 | Stable line search |
| epsilon | 1e-4 | Stop once the transform barely changes |
| max iterations | 60 | Bounds per-Scan latency at medium speed |

The Map's NDT grid is built once at startup (`setInputTarget`), not per Scan.

## Results

_To be filled in after the graded run: distance driven, max pose error, screenshot._
