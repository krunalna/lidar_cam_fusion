# Camera Detector Node (Phase 4)

**File:** `src/perception_pipeline/perception_pipeline/camera_detector_node.py`

## Overview

Subscribes to raw camera images, runs YOLOv8 pre-trained inference, and publishes 2D bounding-box detections for the downstream fusion node.

| | Topic | Message Type |
|---|---|---|
| Subscribes | `/camera/image_raw` | `sensor_msgs/Image` (rgb8) |
| Publishes | `/detections_2d` | `vision_msgs/Detection2DArray` |

---

## Architecture

The node is split into two classes, following the same pattern as the LiDAR processor:

- **`CameraDetector`** — pure Python, no ROS dependency, fully unit-testable. Wraps a YOLOv8 model and returns plain dicts.
- **`CameraDetectorNode`** — ROS 2 node. Handles subscription, message conversion, and publishing.

---

## Data Flow

```
/camera/image_raw  (sensor_msgs/Image, rgb8)
        │
        ▼
  _imgmsg_to_numpy()
    np.frombuffer(msg.data) → reshape (H, W, 3) uint8
        │
        ▼
  CameraDetector.detect(image_rgb)
    YOLOv8 inference → list of dicts:
      { x1, y1, x2, y2, class_id, class_name, confidence }
        │
        ▼
  Build Detection2DArray
    BoundingBox2D  → center (cx, cy) + size_x / size_y
    ObjectHypothesisWithPose → class_name (str) + confidence
        │
        ▼
/detections_2d  (vision_msgs/Detection2DArray)
```

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `model_name` | str | `yolov8n.pt` | YOLOv8 weights file or Ultralytics model name |
| `conf_threshold` | float | `0.5` | Minimum confidence to keep a detection |
| `device` | str | `cpu` | Inference device: `cpu`, `cuda`, or `mps` |

Set in `launch/fusion_pipeline.launch.py` or via `--ros-args`:

```bash
ros2 run perception_pipeline camera_detector \
  --ros-args -p model_name:=yolov8n.pt -p conf_threshold:=0.4 -p device:=cpu
```

---

## QoS Profiles

| Direction | Topic | Reliability | History | Depth |
|---|---|---|---|---|
| Subscriber | `/camera/image_raw` | best-effort | keep-last | 1 |
| Publisher | `/detections_2d` | reliable | keep-last | 5 |

The subscriber matches the KITTI publisher's best-effort QoS. The publisher uses reliable so the fusion node doesn't miss detections.

---

## Key Implementation Details

**Deferred model import**
`ultralytics` is imported inside `CameraDetector.__init__`, not at module level. This keeps `import camera_detector_node` fast and avoids loading the model during unit tests that don't need it.

**No cv_bridge**
Image conversion uses `np.frombuffer(bytes(msg.data))` directly, avoiding any `cv_bridge` version dependency. Assumes `rgb8` encoding (as published by the KITTI publisher).

**Bounding box format**
YOLO returns corner coordinates `(x1, y1, x2, y2)`. These are converted to `BoundingBox2D` center+size format required by `vision_msgs`:
```
cx = (x1 + x2) / 2
cy = (y1 + y2) / 2
size_x = x2 - x1
size_y = y2 - y1
```

**Class label as string**
`ObjectHypothesis.class_id` is a string field in `vision_msgs` — the COCO class name (e.g. `"car"`, `"person"`) is stored there, not the integer index.

**Header stamp passthrough**
The output `Detection2DArray` header stamp is copied from the incoming image message. This ensures the fusion node's `ApproximateTimeSynchronizer` can match detections to the correct `/lidar/filtered` point cloud.

**Log throttling**
A log line is emitted every 20 frames to avoid flooding the terminal:
```
Frame 20 | 3 detection(s)
```

---

## Detection Output Schema

Each `Detection2D` in the published array contains:

```
Detection2D
  header.stamp        # copied from /camera/image_raw
  header.frame_id     # copied from /camera/image_raw
  bbox
    center.x          # horizontal center (pixels)
    center.y          # vertical center (pixels)
    center.theta      # always 0.0 (YOLO gives axis-aligned boxes)
    size_x            # box width (pixels)
    size_y            # box height (pixels)
  results[0]
    hypothesis.class_id   # COCO class name string, e.g. "car"
    hypothesis.score      # confidence in [0, 1]
```
