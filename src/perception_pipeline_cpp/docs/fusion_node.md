# Fusion Node (C++)

`fusion_node_cpp` — Phase 6 of the C++ perception pipeline.

Subscribes to `/detections_2d` and `/lidar/filtered`, synchronises them by timestamp, and runs frustum-based LiDAR-camera association to produce `vision_msgs/Detection3DArray` on `/detections_3d_fused`. Optionally publishes a `MarkerArray` for Foxglove/RViz2 and a debug image showing projected LiDAR points overlaid on the camera frame.

---

## Topics

| Topic | Type | Role | QoS |
|---|---|---|---|
| `/detections_2d` | `vision_msgs/Detection2DArray` | **Subscribe (sync)** — YOLO 2D bboxes from camera_detector | Reliable, depth 1 |
| `/lidar/filtered` | `sensor_msgs/PointCloud2` | **Subscribe (sync)** — preprocessed non-ground cloud | Reliable, depth 1 |
| `/camera/image_raw` | `sensor_msgs/Image` (rgb8) | **Subscribe (independent)** — latest frame cached for debug overlay | Reliable, depth 1 |
| `/detections_3d_fused` | `vision_msgs/Detection3DArray` | **Publish** — one array per sync'd pair (may be empty) | Reliable, depth 5 |
| `/detections_3d_markers` | `visualization_msgs/MarkerArray` | **Publish** — 3D box + label markers (if `publish_markers=true`) | Reliable, depth 1 |
| `/fusion/debug_image` | `sensor_msgs/Image` (rgb8) | **Publish** — projected LiDAR + 2D bbox overlay (if `publish_debug_image=true`) | Reliable, depth 1 |

> **Sync depth 1 rationale:** `KeepLast(1)` on both sync inputs prevents the `ApproximateTimeSynchronizer` buffer from accumulating stale messages that would cause burst-then-silence delivery at pipeline startup.

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `calibration_file` | string | `""` (required) | Absolute path to `calibration.yaml`. Node throws on startup if empty. |
| `min_cluster_points` | int | `5` | Minimum LiDAR points inside a frustum to accept as a 3D detection. |
| `depth_gate_min_m` | double | `4.0` | Minimum depth-slice thickness (metres) from the nearest in-frustum point. |
| `depth_gate_scale` | double | `0.20` | Additional slice thickness as a fraction of nearest depth (`nearest × 0.20`). |
| `sync_slop` | double | `0.5` | `ApproximateTimeSynchronizer` max allowed stamp difference (seconds). |
| `publish_markers` | bool | `true` | Publish `visualization_msgs/MarkerArray` for Foxglove 3D panel. |
| `publish_debug_image` | bool | `true` | Publish projected-LiDAR debug image on `/fusion/debug_image`. |

---

## Architecture

The fusion logic is split into two layers, following the same pattern as the LiDAR processor and camera detector:

- **`FusionEngine`** — pure C++, no ROS dependency, fully unit-testable. Takes a `vector<BBox2D>`, a flat XYZI buffer, and a `Projector`; returns a `FusionResult`.
- **`FusionNode`** — ROS 2 node. Handles time synchronisation, message deserialization, calling `FusionEngine`, and publishing results + optional debug outputs.

---

## Pipeline Overview

```
/detections_2d  (vision_msgs/Detection2DArray)
/lidar/filtered (sensor_msgs/PointCloud2)
        │                │
        └────────────────┘
     ApproximateTimeSynchronizer (slop = 0.5 s)
                │
                ▼
  ┌──────────────────────────────────┐
  │  Deserialise                     │
  │  Detection2DArray → vector<BBox2D│
  │  PointCloud2      → float[]      │
  └──────────────────────────────────┘
                │
                ▼
  ┌─────────────────────────────────────┐
  │  FusionEngine::fuse()               │
  │  for each bbox:                     │
  │    1. Projector::points_in_bbox()   │
  │    2. min_cluster_points check      │
  │    3. Depth gate (nearest slice)    │
  │    4. Axis-aligned min/max 3D box   │
  └─────────────────────────────────────┘
                │
                ▼
  /detections_3d_fused (vision_msgs/Detection3DArray)
                │
       ┌────────┴──────────┐
       ▼                   ▼
  MarkerArray         Debug image
  /detections_3d_markers  /fusion/debug_image
```

---

## FusionEngine Stage Details

### 1 · Frustum Association

For each `BBox2D`, calls `Projector::points_in_bbox()` which:
1. Projects every point in the filtered cloud onto the image plane (`P × T × P_lidar`)
2. Returns the indices of all points whose pixel `(u, v)` falls inside `[x1,x2] × [y1,y2]`

Only points with valid projections (in front of camera, within image bounds) are considered.

### 2 · Minimum Evidence Check

If `indices.size() < min_cluster_points` (default 5), the bbox is discarded — not enough LiDAR evidence to fit a reliable 3D box.

### 3 · Depth Gate

A bbox frustum can capture objects at very different depths (e.g. a car at 8 m and a wall at 40 m both projecting into the same YOLO box). The depth gate keeps only the **nearest depth slice**:

```
nearest_depth  = min Euclidean distance among all in-frustum points
depth_gate     = nearest_depth + max(depth_gate_min_m, nearest_depth × depth_gate_scale)
```

With defaults (`min=4.0 m`, `scale=0.20`):
- At 10 m: gate = 10 + max(4, 2) = **14 m** → keeps 10–14 m slice
- At 40 m: gate = 40 + max(4, 8) = **48 m** → keeps 40–48 m slice

After gating, the minimum evidence check is applied again on the gated subset.

### 4 · Axis-Aligned 3D Bounding Box

Min/max over `x`, `y`, `z` across all gated points (in Velodyne frame):

```
cx = (x_min + x_max) / 2
cy = (y_min + y_max) / 2
cz = (z_min + z_max) / 2
size_x = x_max - x_min
size_y = y_max - y_min
size_z = z_max - z_min
```

The YOLO class label (`class_id`) and confidence (`score`) from the originating 2D detection are carried through unchanged.

---

## Output Messages

### `/detections_3d_fused` — `vision_msgs/Detection3DArray`

One message published per sync'd pair, even if empty. Header stamp and `frame_id` are copied from the incoming `/lidar/filtered` message (Velodyne frame).

Each `Detection3D`:
```
bbox.center.position.{x,y,z}    # box centre in Velodyne frame (metres)
bbox.center.orientation.w = 1.0 # axis-aligned — no rotation
bbox.size.{x,y,z}               # full extents (not half-extents)
results[0].hypothesis.class_id  # COCO class name, e.g. "car"
results[0].hypothesis.score     # confidence in [0, 1]
```

### `/detections_3d_markers` — `visualization_msgs/MarkerArray`

Emits a `DELETEALL` marker followed by two markers per detection:
- **`CUBE`** (ns `fusion_boxes`) — the 3D bounding box, coloured by class:
  - Blue → `car`, `truck`, `bus`
  - Red → `person`, `pedestrian`
  - Orange → `bicycle`, `motorcycle`
  - Green → all other classes
- **`TEXT_VIEW_FACING`** (ns `fusion_labels`) — `"car 87%"` label, 0.3 m above the top face

The `DELETEALL` marker at the start of each frame wipes stale markers from the previous frame, so boxes disappear immediately when a detection is lost.

### `/fusion/debug_image` — `sensor_msgs/Image` (rgb8)

The latest cached `/camera/image_raw` frame annotated with:
1. **Projected LiDAR points** — 3×3 pixel splats coloured by depth (green=near → yellow=mid → red=far, mapped over 0–50 m)
2. **2D detection bboxes** — yellow rectangles with `"class conf%"` labels from `detections_2d`

The debug image is rendered using OpenCV and converts RGB↔BGR as needed. It is only published when `publish_debug_image=true` and a camera image has been received.

---

## Time Synchronisation

`ApproximateTimeSynchronizer` matches `/detections_2d` and `/lidar/filtered` messages by header timestamp within `sync_slop` seconds (default 0.5 s). The KITTI publisher publishes both at the same rate (default 10 Hz), so stamps are typically within a single frame period (~0.1 s).

The camera image subscription is **independent** — it uses a separate subscriber with `KeepLast(1)` and its latest message is cached under a mutex. This avoids adding a third input to the synchronizer (which would reduce match rate), while still providing an up-to-date background image for the debug overlay.

---

## Timing Log

Per-frame timing is measured with `std::chrono::steady_clock` and logged every 30 frames (or immediately for slow frames > 80 ms):

```
Frame   30 |  10.0 Hz | total=  4.2 ms (deser=  1.1  fuse=  2.8  pub/dbg=  0.3) | 2D=5 LiDAR=7500 → 3D=3
Frame   60 |  10.0 Hz | total=  5.1 ms (deser=  1.0  fuse=  3.5  pub/dbg=  0.6) | 2D=4 LiDAR=7432 → 3D=2
Frame   90 |  10.0 Hz | total= 82.3 ms (deser=  1.2  fuse=  2.9  pub/dbg= 78.2) [SLOW] | 2D=6 LiDAR=7511 → 3D=4
```

Fields: `deser` = deserialization, `fuse` = FusionEngine, `pub/dbg` = publishing + debug image render. `Hz` is an exponential moving average (α=0.2).

---

## Source Files

| File | Role |
|---|---|
| `include/perception_pipeline_cpp/fusion_engine.hpp` | `FusionConfig`, `BBox2D`, `Detection3D`, `FusionResult`, `FusionEngine` class |
| `src/fusion_engine.cpp` | Frustum association, depth gate, min/max 3D box fitting |
| `src/fusion_node.cpp` | ROS 2 wrapper: sync, deserialization, publishing, markers, debug image |

---

## Startup Log (healthy)

```
[fusion_node_cpp-4] Loading calibration: /path/to/calibration.yaml
[fusion_node_cpp-4] FusionNode ready — min_cluster_points=5  sync_slop=0.50 s  depth_gate_min_m=4.00  depth_gate_scale=0.20  publish_markers=true  publish_debug_image=true
[fusion_node_cpp-4] Frame    1 |   0.0 Hz | total=  5.3 ms (deser=  1.2  fuse=  3.4  pub/dbg=  0.7) | 2D=3 LiDAR=7480 → 3D=2
[fusion_node_cpp-4] Frame    2 |  10.1 Hz | total=  4.8 ms (deser=  1.1  fuse=  3.1  pub/dbg=  0.6) | 2D=5 LiDAR=7512 → 3D=3
[fusion_node_cpp-4] Frame    3 |  10.0 Hz | total=  4.7 ms (deser=  1.0  fuse=  3.0  pub/dbg=  0.7) | 2D=4 LiDAR=7491 → 3D=3
```
