# LiDAR-Camera Fusion Pipeline

A ROS 2 Jazzy perception pipeline that replays [KITTI raw dataset](https://www.cvlibs.net/datasets/kitti/raw_data.php) sequences as live sensor streams and runs LiDAR preprocessing, YOLOv8 2D detection, calibration-based projection, and frustum fusion.

The repo now supports a single ROS package: `perception_pipeline_cpp`.

## Architecture

```text
KITTI Files on Disk
        |
        v
+----------------------+      /camera/image_raw
|   kitti_publisher    |----> /lidar/points
| (Python node, owned  |
|  by perception_... ) |
+----------------------+
        |                         +----------------------+
        +-----------------------> |  camera_detector_cpp |
        |                         +----------------------+
        |                                    |
        |                                    v
        |                              /detections_2d
        |                                    |
        v                                    v
+----------------------+              +-----------------+
|  lidar_processor_cpp |------------> | fusion_node_cpp |
+----------------------+ /lidar/filtered +-----------------+
                                           |
                                           v
                                   /detections_3d_fused
```

## Status

| Phase | Component | Status |
|---|---|---|
| 2 | `kitti_publisher` | Done |
| 3 | `lidar_processor_cpp` | Done |
| 4 | `camera_detector_cpp` | Done |
| 5 | Calibration + projection utilities | Done |
| 6 | `fusion_node_cpp` | Done |
| 7 | RViz2 / KITTI validation polish | Remaining |
| Extra | C++ profiling | Remaining |

## Prerequisites

- Ubuntu 24.04 or macOS Apple Silicon
- [Pixi](https://pixi.sh/)

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

## Quick Start

```bash
# 1. Install the environment
pixi install

# 2. Download a KITTI sequence
pixi run download-kitti

# 3. Export the YOLO ONNX model once
pixi run export-onnx

# 4. Build the workspace
pixi run build

# 5. Run the checks you care about
pixi run verify-env
pixi run verify-publisher
pixi run verify-lidar
pixi run verify-camera
pixi run verify-projections
pixi run verify-fusion

# 6. Launch the full pipeline
pixi run launch

# Optional explicit overrides (launch arguments)
pixi run launch --sequence_path:=/abs/path/to/2011_09_26_drive_0001_sync --model_path:=/abs/path/to/yolov8n.onnx

# Optional explicit overrides (environment variables)
KITTI_SEQ=/abs/path/to/2011_09_26_drive_0001_sync \
YOLO_ONNX=/abs/path/to/yolov8n.onnx \
pixi run launch
```

## Pixi Tasks

| Task | Purpose |
|---|---|
| `pixi run build` | Build `perception_pipeline_cpp` in Release mode |
| `pixi run launch` | Launch the full KITTI + C++ fusion pipeline |
| `pixi run download-kitti` | Download the default KITTI sequence or list/select others |
| `pixi run export-onnx` | Export `yolov8n.pt` to `models/yolov8n.onnx` |
| `pixi run verify-env` | Validate workspace/tooling dependencies |
| `pixi run verify-publisher` | Dry-run the KITTI publisher |
| `pixi run verify-lidar` | Dry-run `lidar_processor_cpp` |
| `pixi run verify-camera` | Dry-run `camera_detector_cpp` (smoke test with configurable timing) |
| `pixi run verify-projections` | Check calibration/projection math and tests |
| `pixi run verify-fusion` | Check fusion binary, tests, and node startup |
| `pixi run play-bag` | Play a ROS bag if `BAG_PATH` is set |
| `pixi run foxglove` | Run Foxglove bridge |

## Configuration

The canonical calibration file lives at `src/perception_pipeline_cpp/config/calibration.yaml`.

`pixi run launch` defaults:
- `sequence_path` auto-resolves to `$(pwd)/data/kitti/2011_09_26/2011_09_26_drive_0001_sync` when present.
- `model_path` auto-resolves to `$(pwd)/models/yolov8n.onnx` when present.
- Override via launch args: `--sequence_path:=... --model_path:=...`.
- Override via env vars: `KITTI_SEQ=... YOLO_ONNX=... pixi run launch`.

It contains:

| Section | Key fields | Purpose |
|---|---|---|
| `camera` | `fx`, `fy`, `cx`, `cy`, `width`, `height`, `P` | Intrinsics and KITTI projection matrix |
| `lidar_to_camera` | `T` | Velodyne-to-camera transform |
| `fusion` | `max_depth`, `min_cluster_points` | Fusion defaults and thresholds |

If you change KITTI recording dates, update the calibration file to the matching sequence values.

## Project Structure

```text
lidar_cam_fusion/
├── pixi.toml
├── scripts/
│   ├── download_kitti.py
│   ├── export_yolo_onnx.py
│   ├── pc2_helpers.py
│   ├── verify_*.py
│   └── activate_ros.sh
├── src/
│   └── perception_pipeline_cpp/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   └── calibration.yaml
│       ├── launch/
│       │   └── fusion_pipeline_cpp.launch.py
│       ├── scripts/
│       │   └── kitti_publisher.py
│       ├── include/perception_pipeline_cpp/
│       ├── src/
│       ├── test/
│       └── docs/
└── data/kitti/
```

## Main Topics

| Topic | Type | Producer |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | `kitti_publisher` |
| `/lidar/points` | `sensor_msgs/PointCloud2` | `kitti_publisher` |
| `/lidar/filtered` | `sensor_msgs/PointCloud2` | `lidar_processor_cpp` |
| `/lidar/ground_plane` | `sensor_msgs/PointCloud2` | `lidar_processor_cpp` |
| `/detections_2d` | `vision_msgs/Detection2DArray` | `camera_detector_cpp` |
| `/camera/detections_viz` | `sensor_msgs/Image` | `camera_detector_cpp` |
| `/detections_3d_fused` | `vision_msgs/Detection3DArray` | `fusion_node_cpp` |
| `/detections_3d_markers` | `visualization_msgs/MarkerArray` | `fusion_node_cpp` |
| `/fusion/debug_image` | `sensor_msgs/Image` | `fusion_node_cpp` |

## Verification Notes

- `verify-publisher` imports the publisher source directly from `src/perception_pipeline_cpp/scripts/kitti_publisher.py`.
- `verify-lidar` and `verify-camera` exercise the built binaries from `install/perception_pipeline_cpp/lib/perception_pipeline_cpp/`.
- `verify-projections` and `verify-fusion` run `colcon test --packages-select perception_pipeline_cpp`.
- `verify-camera` defaults to normal provider auto-selection to match full launch behavior.
- Use `VERIFY_CAMERA_FORCE_CPU=1 pixi run verify-camera` to force CPU-only smoke mode when debugging machine-specific GPU startup issues.
- `verify-camera` timing can be tuned via `VERIFY_CAMERA_DISCOVERY_SEC`, `VERIFY_CAMERA_TIMEOUT_SEC`, and `VERIFY_CAMERA_PUBLISH_HZ`.
- In restricted environments (for example, DDS socket creation blocked), `verify-camera` downgrades ROS dry-run runtime failures to warnings and still reports static/build checks.

## Current Follow-up Work

- Add end-to-end RViz2 / KITTI validation documentation and screenshots.
- Profile per-stage latency in the C++ nodes.
- Continue polishing platform-specific startup guidance for macOS vs Linux GPU backends.

## License

MIT
