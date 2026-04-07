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

## Intrinsics — What the Numbers Mean

Intrinsics describe the **geometry of the camera lens/sensor itself**, independent of where the camera is physically placed.

### The K matrix (3×3)

```
K = [ fx    0   cx ]     [ 721.54    0      609.56 ]
    [  0   fy   cy ]  =  [   0     721.54   172.85 ]
    [  0    0    1 ]     [   0       0        1    ]
```

| Parameter | Value | Meaning |
|---|---|---|
| `fx` | 721.54 px | Pixels per metre horizontally at 1 m depth — how "zoomed in" the lens is |
| `fy` | 721.54 px | Same vertically. Equal to `fx` → square pixels |
| `cx` | 609.56 px | Horizontal pixel where the optical axis hits the sensor (~centre of 1242 px wide image) |
| `cy` | 172.85 px | Vertical pixel. **Not** at image centre (375/2 = 187.5) — the optical axis hits slightly above centre |

**Geometric meaning:** a real-world point at `(X, Y, Z)` in the camera frame projects to pixel:

```
u = fx × (X / Z) + cx
v = fy × (Y / Z) + cy
```

`X/Z` is the horizontal angle (small-angle approximation). `fx` scales that angle into pixels. `cx`/`cy` shift the origin from the image centre to the top-left corner (pixel `(0,0)`).

**Why `fx == fy`?** Square pixels. On fisheye or anamorphic lenses they differ.

---

### The P matrix (3×4)

```
P2 = [ 721.54    0      609.56   44.86  ]
     [   0     721.54   172.85    0.22  ]
     [   0       0        1.0    0.0027 ]
```

P2 is K extended to 3×4 to accept homogeneous 4-vectors, **plus a stereo baseline offset in the last column**:

```
P2 = K × [ I | t_stereo ]
```

The last column `[44.86, 0.22, 0.0027]ᵀ` encodes the **physical offset between the left grayscale camera (cam0, the rectification reference) and the left colour camera (cam2)** — approximately 44.86 mm to the right. KITTI's `P_rect_02` bakes this in so you can project directly from the rectified frame into the colour camera image without a separate translation.

At 10 m depth this shifts a projected pixel by `44.86 / 10 ≈ 4.5 px` — small but meaningful for precise depth.

---

## Extrinsics — Where the Camera Is Relative to the LiDAR

Extrinsics describe the **rigid body transform** between two physical sensors: where one sensor's origin and axes are, as seen from the other sensor's frame.

### The T matrix (4×4)

```
T = [ R (3×3) | t (3×1) ]

  = [  7.53e-3   -9.999e-1   -6.17e-4  |  -4.07e-3 ]
    [  1.48e-2    7.28e-4    -9.999e-1  |  -7.63e-2 ]
    [  9.999e-1   7.52e-3     1.48e-2   |  -0.2718  ]
    [  0          0           0         |   1.0     ]
```

**R (3×3 rotation):** the near-1 values on the anti-diagonal show this is approximately a **−90° rotation around X followed by −90° around Z**. This is physically correct: the Velodyne X-axis points forward and the camera Z-axis also points forward, but their Y/Z axes are oriented differently — a ~90° flip is needed to align them.

**t (3×1 translation):** `[-0.004, -0.076, -0.272]` metres in camera frame:
- `t_z = -0.272 m` → the camera optical centre is **27.2 cm below** the Velodyne origin
- `t_y = -0.076 m` → ~7.6 cm lateral offset
- `t_x = -0.004 m` → negligible fore-aft offset

**What T does:**

```
P_cam = T × P_lidar

[Xc]   [ R | t ] [Xl]
[Yc] = [       ] [Yl]
[Zc]   [ 0 | 1 ] [Zl]
[ 1]              [ 1]
```

It takes a 3D point measured in the Velodyne coordinate system and expresses it in the camera coordinate system (`Zc > 0` = in front of the camera).

**R_rect note:** KITTI raw data has a slight camera misalignment corrected by `R_rect_00`. In `calibration.yaml`, `R_rect_00` is **pre-multiplied into T** (`T = R_rect_00 × T_velo_to_cam`), so it never needs to be applied separately.

---

## Full Chain with Actual Numbers

```
Velodyne frame          Camera frame           Image plane
  P_lidar     →  T  →    P_cam      →  P2  →   pixel (u,v)

[Xl]   [  7.53e-3  -9.999e-1  -6.17e-4  -4.07e-3] [Xl]
[Yl] → [  1.48e-2   7.28e-4   -9.999e-1 -7.63e-2] [Yl]  →  [Xc, Yc, Zc]
[Zl]   [  9.999e-1  7.52e-3    1.48e-2  -0.2718 ] [Zl]
[ 1]   [  0         0          0         1.0    ] [ 1]

[u·w]   [721.54    0      609.56  44.86 ] [Xc]
[v·w] = [  0    721.54    172.85   0.22 ] [Yc]
[ w ]   [  0       0        1.0  0.0027] [Zc]
                                          [ 1]

u = u·w / w,   v = v·w / w
```

**In plain English:** rotate + translate the LiDAR point into camera-space, then use focal length and principal point to figure out which pixel it lands on.

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
