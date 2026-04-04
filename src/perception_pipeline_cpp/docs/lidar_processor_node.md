# LiDAR Processor Node (C++)

`lidar_processor_cpp` — Phase 3 of the C++ perception pipeline.

Subscribes to raw Velodyne point clouds, runs a 4-stage PCL preprocessing pipeline, and publishes filtered (non-ground) and ground-plane clouds. Mirrors the behaviour of the Python `lidar_processor` node.

---

## Topics

| Topic | Type | Role | QoS |
|---|---|---|---|
| `/lidar/points` | `sensor_msgs/PointCloud2` | **Subscribe** — raw scan from kitti_publisher | Reliable, depth 5 |
| `/lidar/filtered` | `sensor_msgs/PointCloud2` | **Publish** — non-ground points after full pipeline | Reliable, depth 5 |
| `/lidar/ground_plane` | `sensor_msgs/PointCloud2` | **Publish** — ground plane inliers (debug) | Reliable, depth 5 |

> **PointCloud2 layout:** Output clouds always use the canonical 4-field layout — `x, y, z, intensity` — each a FLOAT32, `point_step = 16`, `is_dense = true`. Intensity is averaged within each voxel by PCL VoxelGrid.

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `roi_x_min` | double | `0.0` | Forward ROI start (m) |
| `roi_x_max` | double | `50.0` | Forward ROI end (m) |
| `roi_y_min` | double | `-10.0` | Lateral ROI left bound (m) |
| `roi_y_max` | double | `10.0` | Lateral ROI right bound (m) |
| `roi_z_min` | double | `-3.0` | Vertical ROI floor (m) |
| `roi_z_max` | double | `2.0` | Vertical ROI ceiling (m) |
| `voxel_size` | double | `0.1` | Voxel leaf size for downsampling (m) |
| `ransac_dist` | double | `0.2` | RANSAC plane inlier distance threshold (m) |
| `ransac_iter` | int | `100` | Maximum RANSAC iterations |
| `max_depth` | double | `50.0` | Maximum Euclidean distance from origin to keep (m) |

All parameters match the Python node defaults to ensure identical output on the same data.

---

## Pipeline Overview

```
sensor_msgs/PointCloud2
        │
        ▼
  ┌─────────────────┐
  │  pc2_to_floats  │  arbitrary field layout → flat float32 [x,y,z,i] buffer
  └─────────────────┘
        │  float*  N×4
        ▼
  ┌──────────┐
  │ ROI Crop │  PCL CropBox — forward-facing box filter
  └──────────┘
        │  roi_cloud
        ▼
  ┌──────────────────┐
  │ Voxel Downsample │  PCL VoxelGrid 0.1 m — uniform density reduction
  └──────────────────┘
        │  voxel_cloud
        ▼
  ┌─────────────────┐
  │ Distance Filter │  Euclidean norm ≤ max_depth (50 m)
  └─────────────────┘
        │  dist_cloud
        ▼
  ┌──────────────────────┐
  │ RANSAC Ground Removal│  PCL SACSegmentation PLANE model
  └──────────────────────┘
        │
        ├──► filtered_cloud  (non-ground outliers)
        └──► ground_cloud    (plane inliers)
        │
        ▼
  ┌─────────────────┐
  │ floats_to_pc2   │  flat buffer → canonical PointCloud2 [x,y,z,intensity]
  └─────────────────┘
        │
        ▼
  /lidar/filtered  +  /lidar/ground_plane
```

---

## Stage Details

### 1 · ROI Crop

Uses PCL `CropBox` to discard all points outside a forward-facing axis-aligned box:

```
x ∈ [0.0,  50.0]   — forward direction only (behind-vehicle points removed)
y ∈ [-10.0, 10.0]  — lane-width lateral window
z ∈ [-3.0,   2.0]  — from below the vehicle to above rooftop height
```

If fewer than 10 points remain after cropping, the pipeline short-circuits and returns empty results.

### 2 · Voxel Downsampling

Uses PCL `VoxelGrid` with leaf size 0.1 m. Each voxel cell is replaced by the centroid of all points that fall within it. Intensity is averaged across all points in the voxel — this matches the Python node's custom numpy voxel grid behaviour.

Reduces point count by ~10× on a typical KITTI scan (~115,000 → ~10,000 points in the ROI).

If fewer than 10 points remain, the pipeline short-circuits.

### 3 · Distance Filter

Removes points beyond `max_depth` using the Euclidean 3D norm:

```cpp
std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) <= cfg_.max_depth
```

This is applied after voxel downsampling (more efficient than filtering before). With the ROI already limiting x ≤ 50 m, this filter is mostly a safety net for diagonal points that pass the box but exceed the spherical distance limit.

If fewer than 10 points remain, the pipeline short-circuits.

### 4 · RANSAC Ground Removal

Uses PCL `SACSegmentation` to fit a dominant plane model:

```cpp
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(0.2);   // inlier tolerance: 20 cm
seg.setMaxIterations(100);
seg.setOptimizeCoefficients(true);
```

`ExtractIndices` then splits the cloud into:
- **Ground inliers** (`setNegative(false)`) — points within 0.2 m of the fitted plane → `/lidar/ground_plane`
- **Non-ground outliers** (`setNegative(true)`) — everything else → `/lidar/filtered`

The dominant plane in a KITTI scene is always the road surface (z ≈ −1.7 m in sensor frame). RANSAC robustly finds it even when the vehicle is on a slope or there are a few non-ground points mixed in.

---

## PointCloud2 Serialisation

### Input: `pc2_to_floats()`

Reads the incoming PointCloud2 with arbitrary field layout by resolving byte offsets from `msg.fields` at runtime:

```cpp
for (const auto & f : msg.fields) {
    if (f.name == "x")         x_off = f.offset;
    else if (f.name == "y")    y_off = f.offset;
    else if (f.name == "z")    z_off = f.offset;
    else if (f.name == "intensity") { i_off = f.offset; has_intensity = true; }
}
```

If no `intensity` field is present, intensity is zeroed. This handles both the KITTI `.bin` format (x/y/z/intensity, 16 bytes/point) and any other source that may not include intensity.

### Output: `floats_to_pc2()`

Produces a canonical FLOAT32 PointCloud2 with fixed layout: `x@0, y@4, z@8, intensity@12`, `point_step = 16`. Header (`stamp` + `frame_id`) is copied from the incoming message, preserving the original sensor timestamp and coordinate frame.

---

## Output Messages

### `/lidar/filtered`

Non-ground points after all four pipeline stages. These are the points forwarded to the future fusion node for 3D bounding box fitting inside YOLO frustums.

Each point: `[x, y, z, intensity]` FLOAT32, in Velodyne sensor frame.

Typical counts for a KITTI frame:

| Stage | Count |
|---|---|
| Raw input | ~115,000 |
| After ROI crop | ~12,000 |
| After voxel (0.1 m) | ~9,000 |
| After distance filter | ~9,000 |
| After ground removal | ~7,500 |

### `/lidar/ground_plane`

Ground inlier points from RANSAC — published for debug visualisation in RViz2 / Foxglove. Not consumed by downstream nodes in the current pipeline.

---

## Diagnostic Log

The node logs a per-frame summary every 20 frames:

```
[lidar_processor_cpp-2] Frame 20 | in=115200  roi= 11843  voxel=  9102  ground= 1628  out=  7474
```

Fields: `in` = raw points, `roi` = after box crop, `voxel` = after downsampling, `ground` = RANSAC inliers, `out` = final filtered count.

---

## Source Files

| File | Role |
|---|---|
| `include/perception_pipeline_cpp/lidar_preprocessor.hpp` | Public API: `LidarPreprocessorConfig`, `LidarPreprocessorResult`, `LidarPreprocessor` class |
| `src/lidar_preprocessor.cpp` | Core logic: ROI crop, voxel downsample, distance filter, RANSAC ground removal |
| `src/lidar_processor_node.cpp` | ROS 2 wrapper: `pc2_to_floats()`, `floats_to_pc2()`, subscription, publishing, diagnostics |

---

## Startup Log (healthy)

```
[lidar_processor_cpp-2] LidarProcessor ready — waiting for /lidar/points
[lidar_processor_cpp-2] Frame 20 | in=115200  roi= 11843  voxel=  9102  ground= 1628  out=  7474
[lidar_processor_cpp-2] Frame 40 | in=115200  roi= 11801  voxel=  9088  ground= 1615  out=  7473
```
