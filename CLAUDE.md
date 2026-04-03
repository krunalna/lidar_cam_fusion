# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Project Is

A **ROS 2 LiDAR-Camera fusion perception pipeline** that runs entirely on KITTI dataset playback — no physical sensors or model training required. YOLOv8 runs pre-trained inference; fusion logic is pure projection geometry.

## Current Status

| Phase | Description | Status |
|---|---|---|
| 1 | Environment & workspace setup (Pixi + ROS 2 Jazzy) | Done |
| 2 | KITTI publisher node (`/camera/image_raw`, `/lidar/points`) | Done |
| 3 | LiDAR processor node (ROI → voxel → distance → RANSAC ground removal) | Done |
| 4 | Camera detector node (YOLOv8 → `/detections_2d`) | Done |
| 5 | Calibration utilities + projection math | Planned |
| 6 | Fusion node (frustum-based → `/detections_3d_fused`) | Planned |
| 7 | Visualization (RViz2) + KITTI validation | Planned |

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

## Launching the Pipeline

### Step-by-step (first time)

```bash
# 1. Download KITTI data (one-time, ~380 MB)
pixi run download-kitti

# 2. Build the workspace
pixi run build

# 3. Launch all nodes
KITTI_SEQ=$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync pixi run launch
```

### Launch with options

```bash
# Default: seq 0001, 10 Hz, loop=true
KITTI_SEQ=$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync pixi run launch

# Custom frame rate, no loop
KITTI_SEQ=$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync \
  FRAME_RATE=20 LOOP=false pixi run launch

# Different sequence (download it first)
KITTI_SEQ=$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0005_sync pixi run launch
```

### Expected startup logs (healthy)

```
[kitti_publisher-1]  KittiPublisher ready — 114 frames @ 10.0 Hz  loop=True
[lidar_processor-2]  LidarProcessor ready — waiting for /lidar/points
[camera_detector-3]  YOLOv8 model loaded.
[camera_detector-3]  CameraDetector ready — waiting for /camera/image_raw
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
| `/detections_3d_fused` | `vision_msgs/Detection3DArray` | fusion_node | *(planned)* |

## Key Source Files

- `kitti_publisher_node.py` — reads KITTI `.bin`/`.png` files, publishes at configurable rate; `loop` and `frame_rate` are ROS parameters
- `lidar_processor_node.py` — `LidarPreprocessor` (pure numpy/Open3D, no ROS, unit-testable) wrapped by `LidarProcessorNode`; all pipeline params are ROS parameters
- `config/calibration.yaml` — KITTI camera intrinsics (K), 4×4 Velodyne→camera extrinsic (T), fusion params (`max_depth`, `min_cluster_points`)
- `launch/fusion_pipeline.launch.py` — controlled via `KITTI_SEQ`, `FRAME_RATE`, `LOOP` env vars
- `utils/projection.py`, `utils/calibration.py` — *(planned)* projection math and calibration loading

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

1. ROI crop (forward-facing box filter)
2. Voxel downsampling (Open3D — per-point intensity is lost, zeroed downstream)
3. Distance filter (Euclidean norm ≤ `max_depth`)
4. RANSAC ground removal (`segment_plane`)

Returns `dict`: `filtered` (N,4), `ground` (K,4), `stats`.

## KITTI Data Layout

```
<sequence_path>/
  image_02/data/*.png            # left color camera frames
  velodyne_points/data/*.bin     # float32 x/y/z/intensity per point
```

Default path (gitignored): `data/kitti/2011_09_26/2011_09_26_drive_0001_sync/`

## Entry Points (setup.py)

- `kitti_publisher` → `perception_pipeline.kitti_publisher_node:main`
- `lidar_processor` → `perception_pipeline.lidar_processor_node:main`
- `camera_detector` → `perception_pipeline.camera_detector_node:main`
- `fusion_node` — defined, not yet implemented
