# LiDAR-Camera Fusion Pipeline

A ROS 2 perception pipeline that replays [KITTI raw dataset](https://www.cvlibs.net/datasets/kitti/raw_data.php) sequences as simulated live sensor streams, and progressively adds 2D object detection, 3D point cloud processing, and sensor fusion.

## Architecture

```
KITTI Files on Disk
        │
        ▼
┌─────────────────────┐
│  kitti_publisher    │  ── /camera/image_raw  (sensor_msgs/Image)
│  (implemented)      │  ── /lidar/points      (sensor_msgs/PointCloud2)
└─────────────────────┘
        │
        ▼
┌─────────────────────┐   ┌──────────────────────┐
│  camera_detector    │   │  lidar_processor     │   (planned)
│  YOLOv8 2D boxes    │   │  Open3D clusters     │
└─────────────────────┘   └──────────────────────┘
        │                          │
        └──────────┬───────────────┘
                   ▼
        ┌─────────────────────┐
        │     fusion_node     │   (planned)
        │  3D bounding boxes  │
        └─────────────────────┘
```

| Node | Status | Description |
|------|--------|-------------|
| `kitti_publisher` | Done | Reads `.png` + `.bin` files, publishes as ROS 2 topics |
| `camera_detector` | Planned | YOLOv8-based 2D object detection on camera frames |
| `lidar_processor` | Planned | Open3D-based point cloud clustering |
| `fusion_node` | Planned | Time-synchronized fusion of 2D+3D detections |

## Prerequisites

- **OS:** Ubuntu 24.04 or macOS (Apple Silicon / osx-arm64)
- **Pixi:** reproducible environment manager

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

## Quick Start

```bash
# 1. Install all dependencies (ROS 2 Jazzy + Python stack)
pixi install

# 2. Download default KITTI sequence (~380 MB, 114 frames)
pixi run download-kitti

# 3. Build the ROS 2 workspace
pixi run build

# 4. Verify environment and node implementation
pixi run verify-env
pixi run verify-publisher

# 5. Launch the pipeline
pixi run launch
# or with a specific sequence:
ros2 launch perception_pipeline fusion_pipeline.launch.py \
  sequence_path:=data/kitti/2011_09_26/2011_09_26_drive_0001_sync
```

## Project Structure

```
lidar_cam_fusion/
├── pixi.toml                          # Dependency management & task definitions
├── scripts/
│   ├── activate_ros.sh                # Sourced by Pixi to activate ROS 2 environment
│   ├── download_kitti.py              # Downloads KITTI sequences from public S3
│   ├── verify_step1.py                # Validates environment setup (Step 1)
│   └── verify_step2.py                # Validates kitti_publisher node (Step 2)
├── data/
│   └── kitti/                         # Downloaded dataset (gitignored)
└── src/
    └── perception_pipeline/
        ├── package.xml
        ├── setup.py
        ├── config/
        │   └── calibration.yaml       # Camera intrinsics + LiDAR-camera extrinsics
        ├── launch/
        │   └── fusion_pipeline.launch.py
        └── perception_pipeline/
            ├── kitti_publisher_node.py
            ├── camera_detector_node.py  # (planned)
            ├── lidar_processor_node.py  # (planned)
            ├── fusion_node.py           # (planned)
            └── utils/
```

## Configuration

[src/perception_pipeline/config/calibration.yaml](src/perception_pipeline/config/calibration.yaml) stores KITTI calibration values:

| Section | Key Fields | Description |
|---------|-----------|-------------|
| `camera` | `K` (3×3), `P2` (3×4) | Intrinsic matrix + rectified projection matrix |
| `lidar_to_camera` | `T` (4×4) | Homogeneous transform: Velodyne frame → camera frame |
| `fusion` | `max_depth`, `min_cluster_points` | Runtime filtering thresholds |

The values are derived from KITTI's `calib_cam_to_cam.txt` and `calib_velo_to_cam.txt`. If you use a different sequence date, update this file with the matching calibration.

## Pixi Tasks

| Task | Command |
|------|---------|
| `pixi run build` | `colcon build --symlink-install` for `perception_pipeline` |
| `pixi run launch` | `ros2 launch perception_pipeline fusion_pipeline.launch.py` |
| `pixi run play-bag` | `ros2 bag play $BAG_PATH --loop` |
| `pixi run foxglove` | Launch Foxglove bridge on `FOXGLOVE_PORT` (default `8765`) |
| `pixi run download-kitti` | Download default KITTI sequence |
| `pixi run verify-env` | Check environment, ROS 2, and workspace scaffold |
| `pixi run verify-publisher` | Unit-test the KITTI publisher node |

## Downloading KITTI Data

```bash
# Default: 2011_09_26 drive 0001 (~380 MB)
pixi run download-kitti

# Specific sequence
python scripts/download_kitti.py --date 2011_09_26 --sequence 0002

# List all available sequences
python scripts/download_kitti.py --list

# Force re-download existing files
python scripts/download_kitti.py --force
```

Data is saved to `data/kitti/<date>/<date>_drive_<seq>_sync/` and is gitignored.

## Foxglove

Run the bridge with:

```bash
pixi run foxglove
```

If port `8765` is already in use, override it:

```bash
FOXGLOVE_PORT=8766 pixi run foxglove
```

## Verification Scripts

**`pixi run verify-env`** — Run after `pixi install`. Checks:
- Python ≥ 3.12 and ROS 2 Jazzy are active
- All required ROS message packages are importable
- ML/vision stack (NumPy, OpenCV, Open3D, YOLOv8, SciPy) is installed
- Workspace scaffold (launch files, config, scripts) is complete
- `calibration.yaml` has required sections

**`pixi run verify-publisher`** — Run after `pixi run build`. Checks:
- `kitti_publisher_node.py` exists and entry point is registered
- `_bin_to_pointcloud2` and `_png_to_image` conversion utilities are correct
- Node instantiation and frame publishing work without a live ROS daemon
- (Optional) Real KITTI sequence layout when `KITTI_SEQ=<path>` is set

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Left color camera frames (RGB8, 1242×375) |
| `/lidar/points` | `sensor_msgs/PointCloud2` | Velodyne HDL-64 point clouds (x, y, z, intensity) |

Both publishers use **BEST_EFFORT** QoS with history depth 5, matching real sensor driver conventions.

## Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| ROS 2 Jazzy | — | Middleware, message types, colcon build |
| Python | 3.12 | Runtime |
| OpenCV / cv2 | ≥ 4.7 | Image I/O and detector visualization |
| Open3D | ≥ 0.17 | 3D point cloud processing (future nodes) |
| ultralytics | ≥ 8.0 | YOLOv8 object detection (future nodes) |
| NumPy | ≥ 1.24, < 2.0 | Array operations |
| Pixi | — | Environment and task management |

## License

MIT
