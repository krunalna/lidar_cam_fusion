# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Project Is

A **ROS 2 LiDAR-Camera fusion perception pipeline** that runs entirely on KITTI dataset playback — no physical sensors or model training required. YOLOv8 runs pre-trained inference; fusion logic is pure projection geometry.

The pipeline exists in two implementations: a **Python** package (`perception_pipeline`) and a **C++** package (`perception_pipeline_cpp`).

## Current Status

### Python pipeline (`perception_pipeline`)

| Phase | Description | Status |
|---|---|---|
| 1 | Environment & workspace setup (Pixi + ROS 2 Jazzy) | Done |
| 2 | KITTI publisher node (`/camera/image_raw`, `/lidar/points`) | Done |
| 3 | LiDAR processor node (ROI → voxel → distance → RANSAC ground removal) | Done |
| 4 | Camera detector node (YOLOv8 → `/detections_2d`) | Done |
| 5 | Calibration utilities + projection math | Planned |
| 6 | Fusion node (frustum-based → `/detections_3d_fused`) | Planned |
| 7 | Visualization (RViz2) + KITTI validation | Planned |

### C++ pipeline (`perception_pipeline_cpp`)

| Phase | Description | Status |
|---|---|---|
| 2 | KITTI publisher | Reuses Python node |
| 3 | `lidar_processor_cpp` — PCL: CropBox → VoxelGrid → distance → RANSAC | Done |
| 4 | `camera_detector_cpp` — ONNX Runtime + custom NMS | Done |
| 5 | Calibration + projection (C++) | Planned |
| 6 | Fusion node (C++) | Planned |

## Node Graph

```
/camera/image_raw ──► [Camera Detector Node] ──► /detections_2d
                                                        │
                                                        ▼
                                                  [Fusion Node] ──► /detections_3d_fused
                                                        ▲
/lidar/points ──────► [LiDAR Processor Node] ──► /lidar/filtered
```

## Environment

Uses **Pixi** (Python 3.12 + ROS 2 Jazzy via conda). Prefix all commands with `pixi run` unless the environment is manually activated.

### First-time setup

```bash
# 1. Install Pixi (if not already installed)
curl -fsSL https://pixi.sh/install.sh | bash
# Restart terminal or: source ~/.bashrc

# 2. Enter the Pixi environment
pixi shell

# 3. Build the ROS 2 workspace
pixi run build

# 4. Activate ROS + colcon overlay
. scripts/activate_ros.sh
```

`config/cyclonedds.xml` suppresses DDS thread-affinity noise; `activate_ros.sh` exports `CYCLONEDDS_URI` automatically.

## Common Commands

### Python pipeline

```bash
pixi run build                            # colcon build --symlink-install
pixi run verify1                          # check Python 3.12+, ROS 2 Jazzy, all deps
pixi run verify2                          # KITTI publisher: imports, conversion, ROS dry-run
pixi run verify3                          # LiDAR processor: preprocessing stages, ROS dry-run
pixi run verify_camera_detector           # camera detector: YOLOv8 inference dry-run

pixi run download-kitti                   # default: 2011_09_26 seq 0001 (~380 MB)
pixi run download-kitti -- --list
pixi run download-kitti -- --sequence 0005

pixi run play-bag                         # requires BAG_PATH env var
```

### C++ pipeline

```bash
pixi run export-onnx                      # export yolov8n.pt → models/yolov8n.onnx (one-time)
pixi run build-cpp                        # build perception_pipeline_cpp only
pixi run build-all                        # build all packages

pixi run verify-cpp-lidar                 # lidar_processor_cpp: source + build + ROS dry-run
pixi run verify-cpp-camera                # camera_detector_cpp: source + build + model + ROS dry-run
```

## Launching the Pipeline

### Python pipeline (first time)

```bash
# 1. Download KITTI data (one-time, ~380 MB)
pixi run download-kitti

# 2. Build
pixi run build

# 3. Launch
KITTI_SEQ=$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync pixi run launch
```

### C++ pipeline (first time)

```bash
# 1. Download KITTI data (one-time, if not already done)
pixi run download-kitti

# 2. Export ONNX model (one-time)
pixi run export-onnx

# 3. Build
pixi run build-all

# 4. Launch
KITTI_SEQ=$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync \
  YOLO_ONNX=$(pwd)/models/yolov8n.onnx \
  pixi run launch-cpp
```

### Launch with options

```bash
# Custom frame rate, no loop (Python)
KITTI_SEQ=$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync \
  FRAME_RATE=20 LOOP=false pixi run launch

# Custom frame rate, no loop (C++)
KITTI_SEQ=$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync \
  YOLO_ONNX=$(pwd)/models/yolov8n.onnx \
  FRAME_RATE=20 LOOP=false pixi run launch-cpp
```

### Expected startup logs (healthy)

```
[kitti_publisher-1]   KittiPublisher ready — 114 frames @ 10.0 Hz  loop=True
[lidar_processor-2]   LidarProcessor ready — waiting for /lidar/points
[camera_detector-3]   ONNX model loaded.
[camera_detector-3]   CameraDetector ready — waiting for /camera/image_raw
```

> Note: CycloneDDS `Protocol family not supported` errors on macOS are harmless — thread affinity is a Linux-only feature.

## ROS 2 Topics

| Topic | Type | Node | QoS |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` (rgb8) | kitti_publisher | best-effort, depth 1 |
| `/lidar/points` | `sensor_msgs/PointCloud2` | kitti_publisher | best-effort, depth 1 |
| `/lidar/filtered` | `sensor_msgs/PointCloud2` | lidar_processor | reliable, depth 5 |
| `/lidar/ground_plane` | `sensor_msgs/PointCloud2` | lidar_processor | reliable, depth 5 |
| `/detections_2d` | `vision_msgs/Detection2DArray` | camera_detector | reliable, depth 5 |
| `/camera/detections_viz` | `sensor_msgs/Image` (rgb8) | camera_detector | reliable, depth 1 |
| `/detections_3d_fused` | `vision_msgs/Detection3DArray` | fusion_node | *(planned)* |

## Key Source Files

### Python (`src/perception_pipeline/perception_pipeline/`)
- `kitti_publisher_node.py` — reads KITTI `.bin`/`.png` files, publishes at configurable rate
- `lidar_processor_node.py` — `LidarPreprocessor` (numpy, no ROS) wrapped by `LidarProcessorNode`; intensity preserved through custom numpy voxel grid
- `camera_detector_node.py` — `CameraDetector` (ultralytics YOLOv8) wrapped by `CameraDetectorNode`; publishes viz to `/camera/detections_viz`
- `config/calibration.yaml` — KITTI camera intrinsics (K), 4×4 Velodyne→camera extrinsic (T)
- `launch/fusion_pipeline.launch.py` — controlled via `KITTI_SEQ`, `FRAME_RATE`, `LOOP` env vars

### C++ (`src/perception_pipeline_cpp/`)
- `include/perception_pipeline_cpp/lidar_preprocessor.hpp` — `LidarPreprocessorConfig`, `LidarPreprocessorResult`, `LidarPreprocessor`
- `src/lidar_preprocessor.cpp` — PCL pipeline: CropBox → VoxelGrid (intensity averaged) → distance filter → RANSAC
- `src/lidar_processor_node.cpp` — ROS wrapper; `pc2_to_floats()` / `floats_to_pc2()` handle arbitrary PointCloud2 field layouts
- `include/perception_pipeline_cpp/camera_detector.hpp` — `CameraDetectorConfig`, `Detection`, `LetterboxInfo`, `CameraDetector` (pimpl, no ONNX headers exposed)
- `src/camera_detector.cpp` — letterbox preprocess, ONNX Runtime inference, NMS postprocess; decodes `(1, 84, 8400)` output tensor
- `src/camera_detector_node.cpp` — ROS wrapper; publishes `Detection2DArray` + viz image
- `launch/fusion_pipeline_cpp.launch.py` — controlled via `KITTI_SEQ`, `FRAME_RATE`, `LOOP`, `YOLO_ONNX` env vars

## C++ Camera Detector — ONNX Output Format

YOLOv8n ONNX output tensor shape: `(1, 84, 8400)`
- **Rows 0–3**: `cx, cy, w, h` in letterboxed 640×640 space
- **Rows 4–83**: 80 COCO class scores (no sigmoid — raw logits from ultralytics export)
- **8400 anchors**: product of the three detection heads (80×80 + 40×40 + 20×20)

Postprocessing: confidence filter → per-class greedy NMS (IoU threshold 0.45) → invert letterbox → clip to image bounds.

Export with: `pixi run export-onnx` → `models/yolov8n.onnx`

## Projection Math (Phase 5)

Core equation: `p_image = K × [R|t] × P_lidar`

- **K** (3×3): camera intrinsics — focal length + principal point, from `calibration.yaml`
- **T** (4×4): extrinsic transform Velodyne → rectified camera frame, from `calibration.yaml`
- Use this to project LiDAR points onto the image plane; points inside a YOLO bbox are associated with that detection

## Fusion Algorithm (Phase 6)

Frustum-based association using `message_filters.ApproximateTimeSynchronizer`:
1. For each YOLO 2D bbox, back-project into a 3D frustum
2. Extract LiDAR points from `/lidar/filtered` that fall inside the frustum
3. Fit a 3D bounding box (min/max or cluster) over extracted points
4. Attach YOLO class label + confidence → publish `vision_msgs/Detection3DArray`

## LidarPreprocessor Pipeline Order

Both Python and C++ implement the same 4-stage pipeline:

1. ROI crop (forward-facing box filter) — PCL `CropBox` in C++
2. Voxel downsampling — custom numpy in Python (intensity averaged); PCL `VoxelGrid` in C++ (intensity averaged)
3. Distance filter (Euclidean norm ≤ `max_depth`)
4. RANSAC ground removal — PCL `SACSegmentation` in C++

Returns `filtered` (N×4) and `ground` (K×4), both with `x, y, z, intensity`.

## KITTI Data Layout

```
<sequence_path>/
  image_02/data/*.png            # left color camera frames
  velodyne_points/data/*.bin     # float32 x/y/z/intensity per point
```

Default path (gitignored): `data/kitti/2011_09_26/2011_09_26_drive_0001_sync/`

## Entry Points

### Python (`setup.py`)
- `kitti_publisher` → `perception_pipeline.kitti_publisher_node:main`
- `lidar_processor` → `perception_pipeline.lidar_processor_node:main`
- `camera_detector` → `perception_pipeline.camera_detector_node:main`
- `fusion_node` — defined, not yet implemented

### C++ (`CMakeLists.txt`)
- `lidar_processor_cpp` — Phase 3 C++ node
- `camera_detector_cpp` — Phase 4 C++ node (requires `YOLO_ONNX`)

## macOS Build Notes (C++)

Four macOS-specific workarounds are in `CMakeLists.txt`:
- **libatomic stub**: clang has atomic built-in; an empty `libatomic.a` stub satisfies robostack's `-latomic` flag
- **libpython3.12 preload**: `rosidl_generator_py` uses flat-namespace Python symbol lookup; linking libpython explicitly ensures `_PyExc_*` symbols are available before the dylib loads
- **numpy `_core/include` stub**: rclcpp's cmake config lists `numpy/_core/include` in its `INTERFACE_INCLUDE_DIRECTORIES`; after a fresh pixi install the directory may not exist — CMakeLists.txt creates it automatically
- **ONNX Runtime headers**: conda-forge's `onnxruntime` package for macOS is Python-only (no C++ headers). CMakeLists.txt uses `FetchContent` to download the matching official prebuilt (version queried from the Python package) on first configure; the `.dylib` is taken from `site-packages/onnxruntime/capi/`. Headers land flat in `include/` so the include is `#include <onnxruntime_cxx_api.h>`
