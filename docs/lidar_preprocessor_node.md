# LiDAR Preprocessor Node (C++)

**Files:**
- `src/perception_pipeline_cpp/include/perception_pipeline_cpp/lidar_preprocessor.hpp`
- `src/perception_pipeline_cpp/src/lidar_preprocessor.cpp`
- `src/perception_pipeline_cpp/src/lidar_processor_node.cpp`

## Overview

Subscribes to raw Velodyne point clouds, runs a four-stage C++ preprocessing pipeline (ROI crop → voxel downsampling → distance filter → RANSAC ground removal), and republishes two cleaned point clouds for the downstream fusion node.

| | Topic | Message Type |
|---|---|---|
| Subscribes | `/lidar/points` | `sensor_msgs/PointCloud2` |
| Publishes | `/lidar/filtered` | `sensor_msgs/PointCloud2` (non-ground points) |
| Publishes | `/lidar/ground_plane` | `sensor_msgs/PointCloud2` (ground points, debug) |

---

## Architecture

The implementation uses a two-class split:

- **`LidarPreprocessor`** — pure C++, no ROS dependency, fully unit-testable. Accepts a flat `float32` buffer of `N * 4` values in `[x, y, z, intensity]` order and returns a `LidarPreprocessorResult` struct containing the filtered and ground buffers plus per-stage point counts. Configured entirely via `LidarPreprocessorConfig`.
- **`LidarProcessorNode`** — ROS 2 node (`rclcpp::Node` subclass). Handles subscription, `PointCloud2` parsing, calling `LidarPreprocessor::process()`, and publishing. All ROS parameter declarations and QoS setup live here; the core logic does not.

This separation means the preprocessing pipeline can be exercised in GTest unit tests without spinning up a ROS context.

---

## Data Flow

```
/lidar/points  (sensor_msgs/PointCloud2)
        │
        ▼
  pc2_to_floats()
    Resolve field byte offsets from msg.fields (handles any layout)
    Iterate points: reinterpret_cast each field to float32
    Output: std::vector<float>  N*4  [x, y, z, intensity]
        │
        ▼
  LidarPreprocessor::process()
        │
        ├─ Stage 1: ROI Crop  (pcl::CropBox)
        │     Discard points outside forward-facing 3-D box
        │     result.stats.n_roi
        │
        ├─ Stage 2: Voxel Downsampling  (pcl::VoxelGrid)
        │     Uniform density reduction — one representative point per voxel
        │     result.stats.n_voxel
        │
        ├─ Stage 3: Distance Filter  (Euclidean norm <= max_depth)
        │     Discard far points missed by axis-aligned ROI corners
        │
        ├─ Stage 4: RANSAC Ground Removal  (pcl::SACSegmentation)
        │     Fit dominant plane → split inliers (ground) / outliers (objects)
        │     result.stats.n_ground  /  result.stats.n_output
        │
        └─ Pack to flat float32 buffers (voxel-averaged intensity preserved)
               result.filtered   N_out * 4
               result.ground     N_gnd * 4
        │
        ▼
  floats_to_pc2()
    Build PointCloud2 with fields [x, y, z, intensity], point_step=16
    Copy input header stamp + frame_id
        │
        ├──► /lidar/filtered      (non-ground points)
        └──► /lidar/ground_plane  (ground points)
```

---

## Pipeline Stages

### Stage 1 — ROI Crop

**PCL filter:** `pcl::CropBox<pcl::PointXYZI>`

Removes all points that fall outside a configurable axis-aligned bounding box. The default box is forward-facing (positive X = forward, Y = lateral, Z = vertical):

```
x: [roi_x_min, roi_x_max]  =  [0.0,  50.0] m
y: [roi_y_min, roi_y_max]  =  [-10.0, 10.0] m
z: [roi_z_min, roi_z_max]  =  [-3.0,  2.0] m
```

**What gets discarded:** points behind the vehicle, ground returns far off to the sides, returns above the maximum sensor height, and the majority of 360-degree scan points that are irrelevant to the forward-facing camera frustum. If fewer than 10 points survive, the pipeline returns early with empty output buffers.

---

### Stage 2 — Voxel Downsampling

**PCL filter:** `pcl::VoxelGrid<pcl::PointXYZI>`

Divides the ROI cloud into a uniform 3-D grid of cubic voxels with side length `voxel_size` (default 0.1 m) and replaces all points in each occupied voxel with their centroid. This enforces a maximum spatial density and reduces the point count before the more expensive RANSAC step.

**What gets discarded:** redundant nearby points within the same voxel cell. Intensity values are averaged by PCL's centroid computation and preserved in the output buffers. If fewer than 10 points survive, the pipeline returns early.

---

### Stage 3 — Distance Filter

**Implementation:** manual Euclidean norm loop (no PCL filter class)

After voxel downsampling, a point-by-point loop computes `sqrt(x^2 + y^2 + z^2)` and keeps only points where the result is `<= max_depth` (default 50.0 m). This is complementary to the ROI crop: the box test catches axis-aligned ranges but the diagonal corners of the box can admit points farther than `max_depth` in Euclidean distance.

**What gets discarded:** points in the far corners of the ROI box whose true 3-D distance from the sensor exceeds the configured depth limit. If fewer than 10 points survive, the pipeline returns early.

---

### Stage 4 — RANSAC Ground Removal

**PCL filter:** `pcl::SACSegmentation<pcl::PointXYZI>` + `pcl::ExtractIndices<pcl::PointXYZI>`

Fits a dominant plane to the surviving cloud using RANSAC (`pcl::SACMODEL_PLANE`, `pcl::SAC_RANSAC`). Points whose distance to the fitted plane is within `ransac_dist` (default 0.2 m) are classified as ground (inliers); all remaining points are the object cloud (outliers). `setOptimizeCoefficients(true)` refines the plane model after the initial RANSAC fit.

Both sets are packed into flat float32 buffers and stored in `result.ground` and `result.filtered` respectively.

**What gets discarded from `/lidar/filtered`:** road surface, flat parking lots, and any large planar surface that RANSAC identifies as the dominant plane.

---

## Parameters

All parameters are declared as ROS 2 parameters in `LidarProcessorNode`'s constructor. The node name is `lidar_processor`. Parameters are read once at startup and used to construct a `LidarPreprocessorConfig`; there is no runtime reconfiguration.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `roi_x_min` | double | `0.0` | Forward ROI start (m) — typically set to 0 to exclude the vehicle body behind the sensor |
| `roi_x_max` | double | `50.0` | Forward ROI end (m) |
| `roi_y_min` | double | `-10.0` | Left ROI boundary (m) — negative is left in KITTI convention |
| `roi_y_max` | double | `10.0` | Right ROI boundary (m) |
| `roi_z_min` | double | `-3.0` | Minimum height (m) — captures ground returns below sensor mount |
| `roi_z_max` | double | `2.0` | Maximum height (m) — discards overhead returns (bridges, overpasses) |
| `voxel_size` | double | `0.1` | Voxel grid leaf size (m) — smaller values preserve more detail at the cost of speed |
| `ransac_dist` | double | `0.2` | RANSAC plane inlier distance threshold (m) — increase on rough terrain |
| `ransac_iter` | int | `100` | Maximum RANSAC iterations — higher values improve plane fit quality |
| `max_depth` | double | `50.0` | Maximum Euclidean distance from sensor (m) — applied after voxel downsampling |

Override from the command line:

```bash
ros2 run perception_pipeline_cpp lidar_processor_cpp \
  --ros-args -p voxel_size:=0.2 -p ransac_dist:=0.3 -p max_depth:=30.0
```

---

## QoS Profiles

| Direction | Topic | Reliability | History | Depth |
|---|---|---|---|---|
| Subscriber | `/lidar/points` | reliable | keep-last | 5 |
| Publisher | `/lidar/filtered` | reliable | keep-last | 5 |
| Publisher | `/lidar/ground_plane` | reliable | keep-last | 5 |

All three use the same `rclcpp::QoS(rclcpp::KeepLast(5)).reliable()` profile. This matches the reliable publisher used by the KITTI publisher for `/lidar/points` and ensures the fusion node does not miss point clouds.

---

## Key Implementation Details

### PointCloud2 manual parsing — field offset resolution

The `pc2_to_floats()` helper does not assume a fixed byte layout. It walks `msg.fields` to find the byte offsets of `x`, `y`, `z`, and `intensity` before entering the per-point loop:

```cpp
for (const auto & f : msg.fields) {
    if (f.name == "x")              x_off = f.offset;
    else if (f.name == "y")         y_off = f.offset;
    else if (f.name == "z")         z_off = f.offset;
    else if (f.name == "intensity") { i_off = f.offset; has_intensity = true; }
}
```

Each point is then accessed as `msg.data.data() + i * msg.point_step + field_offset`, cast with `reinterpret_cast<const float *>`. If no `intensity` field is found, the intensity channel defaults to `0.0f`. This makes the node compatible with any `PointCloud2` producer regardless of field ordering or the presence of extra fields.

### Intensity handling after voxel grid

PCL's `VoxelGrid` filter computes the centroid of all points in each voxel, which includes averaging the intensity channel. The implementation preserves that averaged intensity when packing both `result.filtered` and `result.ground`:

```cpp
result.filtered.push_back(pt.intensity);
```

This keeps the output schema aligned with the actual point cloud data emitted by the node while preserving compatibility with downstream subscribers that expect an intensity field.

### macOS libpython flat-namespace fix

RoboStack's `rosidl_generator_py` shared libraries use macOS flat-namespace dynamic linking and reference Python C API symbols (e.g. `_PyExc_RuntimeError`) without bundling them. On macOS the linker must find these symbols in a library that is loaded before those dylibs. `CMakeLists.txt` handles this with two measures:

1. **libpython preload** — `find_library(_python_lib python3.12 ...)` locates the Conda `libpython3.12.dylib` and links it directly into the `lidar_processor_cpp` binary via `$<$<PLATFORM_ID:Darwin>:${_python_lib}>`. Because the binary itself holds the strong reference, `libpython` is in the dynamic linker's flat namespace before any ROS dylib is opened.

2. **libatomic stub** — RoboStack's `rcutils` embeds a linker flag `-latomic` which does not exist on macOS/clang (atomic operations are built into the compiler runtime). A tiny one-function C file is compiled into a static `libatomic.a` stub and placed on the linker search path so the flag resolves without error.

Neither of these changes affects runtime behavior on Linux.

### Frame-count log throttling

The `on_pointcloud` callback increments a `uint32_t frame_count_` on every message and emits a diagnostic `RCLCPP_INFO` log line only when `frame_count_ % 20 == 0`:

```
Frame 20 | in= 28652  roi=  8431  voxel=  4102  ground=  612  out= 3490
```

The five counters — `n_input`, `n_roi`, `n_voxel`, `n_ground`, `n_output` — come directly from `LidarPreprocessorResult::stats` and make it straightforward to diagnose misconfigured ROI or RANSAC parameters at runtime without flooding the terminal.

---

## Output Schema

Both `/lidar/filtered` and `/lidar/ground_plane` are produced by `floats_to_pc2()` and share the same layout:

```
sensor_msgs/PointCloud2
  header.stamp      # copied from incoming /lidar/points message
  header.frame_id   # copied from incoming /lidar/points message
  height     = 1              # unorganized cloud
  width      = N              # number of points
  is_dense   = true           # no NaN/Inf points
  is_bigendian = false        # little-endian (x86 / arm64)
  point_step = 16             # bytes per point (4 fields * 4 bytes each)
  row_step   = point_step * width

  fields[0]  name="x"         offset=0   datatype=FLOAT32  count=1
  fields[1]  name="y"         offset=4   datatype=FLOAT32  count=1
  fields[2]  name="z"         offset=8   datatype=FLOAT32  count=1
  fields[3]  name="intensity" offset=12  datatype=FLOAT32  count=1  (always 0.0)
```

Points are packed as contiguous little-endian `float32` values with no padding between fields. The `intensity` field is always `0.0f` — it is present in the schema to keep the layout compatible with standard RViz2 point cloud visualizers and with any future node that re-introduces intensity data.
