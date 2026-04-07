# Calibration & Projection (C++)

`Calibration` + `Projector` — Phase 5 of the C++ perception pipeline.

Loads KITTI calibration parameters from YAML and implements the core equation that maps Velodyne LiDAR points onto the image plane. No ROS dependency — both classes are pure C++ utilities consumed by the fusion node (Phase 6).

---

## Files

| File | Role |
|---|---|
| `include/perception_pipeline_cpp/calibration.hpp` | `CalibrationData` struct, `Calibration` loader class |
| `src/calibration.cpp` | YAML parsing via OpenCV `FileStorage` |
| `include/perception_pipeline_cpp/projector.hpp` | `Projector` class, `PixelCoord` result type |
| `src/projector.cpp` | Single-point, cloud, and frustum-association projection |

---

## Calibration

### YAML Structure

`Calibration` reads `src/perception_pipeline/config/calibration.yaml` (path passed at construction). Expected structure:

```yaml
camera:
  fx: 721.5377
  fy: 721.5377
  cx: 609.5593
  cy: 172.8540
  width: 1242
  height: 375
  P: [12 floats, row-major]   # 3×4 KITTI P2 projection matrix

lidar_to_camera:
  T: [16 floats, row-major]   # 4×4 Velodyne → rectified camera extrinsic
```

KITTI `.txt` calibration files are plain YAML without the `%YAML:1.0` header required by OpenCV `FileStorage`. `calibration.cpp` prepends that header in memory before parsing — the file on disk is never modified.

### CalibrationData Fields

| Field | Type | Description |
|---|---|---|
| `fx`, `fy` | `float` | Focal lengths (pixels) |
| `cx`, `cy` | `float` | Principal point (pixels) |
| `width`, `height` | `int` | Image dimensions |
| `P` | `Eigen::Matrix<float,3,4>` | KITTI P2: projects rectified-camera points to pixels |
| `T` | `Eigen::Matrix4f` | Velodyne frame → rectified camera frame (extrinsic) |

Both `P` and `T` are read as flat row-major float vectors and mapped into Eigen matrices with `Eigen::Map`:

```cpp
data_.P = Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::RowMajor>>(P_vec.data());
data_.T = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(T_vec.data());
```

### KITTI Calibration Source

For a new KITTI sequence, extract values from:
- `calib_cam_to_cam.txt` → `P_rect_02` (12 values) → `camera.P`
- `calib_velo_to_cam.txt` → compose `R_rect_00 × [R|t]_velo_to_cam` into a 4×4 matrix → `lidar_to_camera.T`

The values in the repo correspond to KITTI sequence `2011_09_26_drive_0001_sync`.

---

## Projector

### Core Equation

```
p_image = P × (T × P_lidar)
```

Where:
- `P_lidar = [x, y, z, 1]ᵀ` — homogeneous point in Velodyne frame
- `T` (4×4) — extrinsic: Velodyne → rectified camera frame
- `P` (3×4) — KITTI P2: maps rectified-camera point to homogeneous pixel
- `p_image = [u·w, v·w, w]ᵀ` — homogeneous pixel coordinate
- Final pixel: `u = p[0]/p[2]`, `v = p[1]/p[2]`

### Projection Pipeline (single point)

```
Velodyne point (x, y, z)
        │
        ▼
  T × [x, y, z, 1]ᵀ  →  P_cam = [Xc, Yc, Zc, 1]ᵀ
        │
        ▼  depth check: Zc > 0 ?  (point must be in front of camera)
        │
        ▼
  P × P_cam  →  [u·w, v·w, w]ᵀ
        │
        ▼  homogeneous divide: u = p[0]/p[2], v = p[1]/p[2]
        │
        ▼  bounds check: 0 ≤ u < width  AND  0 ≤ v < height
        │
        ▼
  PixelCoord { u, v, valid=true }
```

Points behind the camera (`Zc ≤ 0`) or outside image bounds return `PixelCoord { valid=false }`.

### API

#### `project(x, y, z) → PixelCoord`

Projects a single Velodyne point. Returns `{u, v, valid}`.

#### `project_cloud(points_xyzi, n_points) → vector<PixelCoord>`

Projects all N points in a flat XYZI buffer. Returns one `PixelCoord` per point in the same order. Iterates the buffer in strides of 4 (`x@0, y@4, z@8, i@12`).

#### `points_in_bbox(points_xyzi, n_points, x1, y1, x2, y2) → vector<uint32_t>`

Projects every point and returns the **indices** of those whose pixel coordinate falls inside the supplied bounding box `[x1,x2] × [y1,y2]`. Only points with `valid=true` projections are included. This is the frustum association step used directly by `FusionEngine`.

---

## KITTI Example Values

For sequence `2011_09_26_drive_0001_sync`, left colour camera:

| Parameter | Value |
|---|---|
| `fx = fy` | 721.54 px |
| `cx` | 609.56 px |
| `cy` | 172.85 px |
| Image size | 1242 × 375 |
| P2 translation (x) | +44.86 mm (stereo baseline offset) |
| T rotation | ~180° around Y — Velodyne faces forward, camera faces forward, axes differ |
| T translation z | −0.272 m (Velodyne is ~27 cm above camera optical centre) |

---

## Usage (non-ROS)

```cpp
#include "perception_pipeline_cpp/calibration.hpp"
#include "perception_pipeline_cpp/projector.hpp"

using namespace perception_pipeline_cpp;

// Load once at startup
const Calibration calib("path/to/calibration.yaml");
const Projector   proj(calib.data());

// Project a single point
auto px = proj.project(10.5f, -1.2f, 0.3f);
if (px.valid) {
    // px.u, px.v are in original image pixel coordinates
}

// Project a whole cloud
auto pixels = proj.project_cloud(cloud_xyzi.data(), n_points);

// Frustum association for a YOLO bbox
auto indices = proj.points_in_bbox(
    cloud_xyzi.data(), n_points,
    bbox.x1, bbox.y1, bbox.x2, bbox.y2);
```

---

## Source Files

| File | Role |
|---|---|
| `include/perception_pipeline_cpp/calibration.hpp` | `CalibrationData` struct, `Calibration` class declaration |
| `src/calibration.cpp` | YAML loading, OpenCV `FileStorage` parsing, Eigen matrix mapping |
| `include/perception_pipeline_cpp/projector.hpp` | `Projector` class, `PixelCoord` struct, API declarations |
| `src/projector.cpp` | `project()`, `project_cloud()`, `points_in_bbox()` implementations |
| `src/perception_pipeline/config/calibration.yaml` | KITTI intrinsics + extrinsic for sequence `0001` |
