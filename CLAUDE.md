# CLAUDE.md

This file gives repository-specific guidance for coding agents working in this workspace.

## What This Project Is

A single-package ROS 2 Jazzy LiDAR-camera fusion pipeline built around KITTI playback.

- ROS package: `perception_pipeline_cpp`
- Dataset source: KITTI raw sequences
- 2D detector runtime: ONNX Runtime with YOLOv8 ONNX export
- LiDAR preprocessing: PCL
- Projection + fusion: Eigen/OpenCV + custom C++ logic
- KITTI publisher: Python node installed by the same package

## Current Status

| Phase | Description | Status |
|---|---|---|
| 2 | KITTI publisher (`kitti_publisher`) | Done |
| 3 | LiDAR preprocessing (`lidar_processor_cpp`) | Done |
| 4 | Camera detection (`camera_detector_cpp`) | Done |
| 5 | Calibration + projection utilities | Done |
| 6 | Fusion node (`fusion_node_cpp`) | Done |
| 7 | RViz2 / KITTI validation polish | Planned |
| Extra | C++ profiling | Planned |

## Canonical Commands

```bash
pixi install
pixi run download-kitti
pixi run export-onnx
pixi run build

pixi run verify-env
pixi run verify-publisher
pixi run verify-lidar
pixi run verify-camera
pixi run verify-projections
pixi run verify-fusion

pixi run launch

# Optional explicit overrides (launch arguments)
pixi run launch --sequence_path:=/abs/path/to/2011_09_26_drive_0001_sync --model_path:=/abs/path/to/yolov8n.onnx

# Optional explicit overrides (environment variables)
KITTI_SEQ=/abs/path/to/2011_09_26_drive_0001_sync \
YOLO_ONNX=/abs/path/to/yolov8n.onnx \
pixi run launch
```

Optional:

```bash
pixi run play-bag
pixi run foxglove
```

## Launch Surface

The supported full-pipeline launch file is:

- `src/perception_pipeline_cpp/launch/fusion_pipeline_cpp.launch.py`

Environment variables used by launch:

- `KITTI_SEQ` (optional override)
- `FRAME_RATE`
- `LOOP`
- `YOLO_ONNX` (optional override)
- `CALIB_FILE` (optional override)

Default launch path behavior when overrides are not set:

- `sequence_path` uses `$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync` if it exists.
- `model_path` uses `$(pwd)/models/yolov8n.onnx` if it exists.
- Overrides can be passed as launch args (`--sequence_path:=... --model_path:=...`) or env vars (`KITTI_SEQ=... YOLO_ONNX=...`).

## Key Files

- `src/perception_pipeline_cpp/scripts/kitti_publisher.py`
  Python KITTI replay node, installed as executable `kitti_publisher`.
- `src/perception_pipeline_cpp/config/calibration.yaml`
  Canonical camera intrinsics and LiDAR-to-camera extrinsic.
- `src/perception_pipeline_cpp/include/perception_pipeline_cpp/lidar_preprocessor.hpp`
  LiDAR preprocessing public API.
- `src/perception_pipeline_cpp/src/lidar_preprocessor.cpp`
  CropBox -> VoxelGrid -> distance filter -> RANSAC implementation.
- `src/perception_pipeline_cpp/include/perception_pipeline_cpp/camera_detector.hpp`
  Camera detector public API with ONNX Runtime hidden behind pimpl.
- `src/perception_pipeline_cpp/src/camera_detector.cpp`
  Letterbox, tensor prep, inference, decode, and NMS.
- `src/perception_pipeline_cpp/include/perception_pipeline_cpp/calibration.hpp`
  YAML-backed calibration data loader.
- `src/perception_pipeline_cpp/include/perception_pipeline_cpp/projector.hpp`
  LiDAR-to-image projection utilities.
- `src/perception_pipeline_cpp/include/perception_pipeline_cpp/fusion_engine.hpp`
  Pure C++ frustum fusion API.
- `src/perception_pipeline_cpp/src/fusion_node.cpp`
  ROS 2 wrapper around synchronized fusion and debug outputs.

## Topic Graph

```text
/camera/image_raw -> camera_detector_cpp -> /detections_2d
/lidar/points -> lidar_processor_cpp -> /lidar/filtered
/detections_2d + /lidar/filtered -> fusion_node_cpp -> /detections_3d_fused
```

Additional debug outputs:

- `/lidar/ground_plane`
- `/camera/detections_viz`
- `/detections_3d_markers`
- `/fusion/debug_image`

## Verification Scripts

- `scripts/verify_env.py`
- `scripts/verify_publisher.py`
- `scripts/verify_lidar.py`
- `scripts/verify_camera.py`
- `scripts/verify_projections.py`
- `scripts/verify_fusion.py`

Shared PointCloud2 helpers for verification live in:

- `scripts/pc2_helpers.py`

`verify-camera` specifics:

- Default behavior uses normal provider auto-selection, matching full launch behavior.
- Set `VERIFY_CAMERA_FORCE_CPU=1` to force CPU-only smoke mode when debugging machine-specific GPU startup failures.
- Timing knobs are available via `VERIFY_CAMERA_DISCOVERY_SEC`, `VERIFY_CAMERA_TIMEOUT_SEC`, and `VERIFY_CAMERA_PUBLISH_HZ`.
- If DDS transport is restricted (socket/interface permissions), ROS dry-run runtime checks are reported as warnings instead of hard failures.

## Build / Packaging Notes

- `perception_pipeline_cpp` is an `ament_cmake` package.
- The package owns both the C++ binaries and the installed Python publisher script.
- `config/calibration.yaml` is installed into `share/perception_pipeline_cpp/config`.
- On macOS, CMake provides the `libatomic` stub and explicit `libpython3.12` preload required by RoboStack.
- ONNX Runtime headers are fetched at configure time to match the Python-installed runtime version.
- On macOS, the build prefers the official CoreML-enabled ONNX Runtime prebuilt.

## Important Assumptions

- There is no separate legacy Python ROS package anymore.
- There is no separate Python pipeline to keep in sync.
- Node behavior should stay stable unless a task explicitly asks for algorithmic changes.
- The dirty worktree may contain user-owned artifacts such as `models/yolov8n.onnx`; do not revert them.

## Next Work Worth Doing

- Add profiling around `pc2_to_floats`, PCL stages, ONNX inference, and postprocess.
- Document end-to-end KITTI validation workflow and expected outputs.
- Add more launch/runtime guidance for Linux CUDA vs macOS CoreML execution providers.
