# Camera Detector Node (C++)

`camera_detector_cpp` — Phase 4 of the C++ perception pipeline.

Subscribes to raw camera frames, runs YOLOv8n inference via ONNX Runtime, and publishes 2D bounding-box detections.

---

## Topics

| Topic | Type | Role | QoS |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` (rgb8) | **Subscribe** — raw frames from kitti_publisher | Reliable, depth 5 |
| `/detections_2d` | `vision_msgs/Detection2DArray` | **Publish** — one array per frame (may be empty) | Reliable, depth 5 |
| `/camera/detections_viz` | `sensor_msgs/Image` (rgb8) | **Publish** — annotated frame with bounding boxes | Reliable, depth 1 |

> **QoS note:** The subscription uses **RELIABLE** to match the kitti_publisher's publisher profile. A BEST_EFFORT subscription silently fails to connect under CycloneDDS on macOS even though the DDS spec allows it.

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `model_path` | string | `""` (required) | Absolute path to `yolov8n.onnx`. Node throws on startup if empty. |
| `conf_threshold` | double | `0.5` | Minimum class score to keep a detection candidate. |
| `iou_threshold` | double | `0.45` | IoU threshold for per-class greedy NMS. |

Export the model once before launching:
```bash
pixi run export-onnx          # → models/yolov8n.onnx
```

---

## Pipeline Overview

```
sensor_msgs/Image (rgb8)
        │
        ▼
  ┌─────────────┐
  │  Letterbox  │  uniform scale + grey pad → 640×640
  └─────────────┘
        │  cv::Mat (RGB, uint8)
        ▼
  ┌─────────────┐
  │  Normalise  │  HWC uint8 → CHW float32 [0, 1]
  └─────────────┘
        │  std::vector<float>  shape (1, 3, 640, 640)
        ▼
  ┌─────────────────────┐
  │  ONNX Runtime Infer │  YOLOv8n, single-threaded CPU session
  └─────────────────────┘
        │  float*  shape (1, 84, 8400)
        ▼
  ┌──────────────┐
  │  Postprocess │  decode → confidence filter → letterbox invert → clip
  └──────────────┘
        │  candidates
        ▼
  ┌──────────┐
  │  NMS     │  per-class greedy, IoU threshold 0.45
  └──────────┘
        │  std::vector<Detection>
        ▼
  vision_msgs/Detection2DArray  +  annotated sensor_msgs/Image
```

---

## Stage Details

### 1 · Letterbox Resize

Resizes the input image to fit inside 640×640 while preserving aspect ratio. Padding is added symmetrically with grey (value 114 — the YOLOv8 canonical pad colour).

For a KITTI frame (1242 × 375):
- Scale = min(640/375, 640/1242) = **0.515**
- Resized = 640 × 193
- Padding: pad_x = 0 px, pad_y = 223.5 px (each side)

`LetterboxInfo` stores `scale`, `pad_x`, `pad_y` for use in the inverse transform later.

### 2 · Normalise to CHW float

Converts the padded 640×640 RGB `cv::Mat` to a contiguous `float` buffer in channel-first (CHW) layout, normalised to [0.0, 1.0]:

```
tensor[c][y][x] = canvas_pixel[y][x][c] / 255.0
```

Channel order is preserved as RGB — no BGR swap is needed because the input is already RGB from the kitti_publisher.

### 3 · ONNX Runtime Inference

The `OrtImpl` pimpl struct owns the session and is created once in the constructor:

```cpp
opts.SetIntraOpNumThreads(1);
opts.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
session = make_unique<Ort::Session>(env, model_path, opts);
// I/O names queried from the model — not hardcoded
input_name  = session->GetInputNameAllocated(0, alloc).get();
output_name = session->GetOutputNameAllocated(0, alloc).get();
```

Input tensor shape: `(1, 3, 640, 640)` float32  
Output tensor shape: `(1, 84, 8400)` float32

### 4 · Decode Output Tensor

The output tensor has shape **(1, 84, 8400)** stored row-major:

```
output[feat * 8400 + anchor]   (batch dimension dropped, batch=1)

feat 0  → cx      ┐
feat 1  → cy      │  bounding box centre + size in 640-space
feat 2  → w       │
feat 3  → h       ┘
feat 4  → P(person)
feat 5  → P(bicycle)
...
feat 83 → P(toothbrush)   — 80 COCO classes, post-sigmoid probabilities
```

Per anchor:
1. Find `best_score = max(feat 4..83)` and its `best_cls`
2. Skip if `best_score < conf_threshold`
3. Convert `(cx, cy, w, h)` → `(x1, y1, x2, y2)` in 640-space
4. Invert letterbox: `x = (x - pad_x) / scale`, same for y
5. Clip to `[0, orig_w]` × `[0, orig_h]`

### 5 · Per-Class Greedy NMS

```
sort candidates by confidence (descending)
for each candidate (highest conf first):
    if not suppressed → keep
    suppress all lower-conf candidates of the same class with IoU > iou_threshold
```

IoU is computed as intersection / union of axis-aligned boxes. Suppression is per-class — a car and a person with high overlap are both kept.

---

## Output Messages

### `vision_msgs/Detection2DArray`

One message published per incoming frame, even if no detections pass the threshold (empty `detections` list).

Each `Detection2D` entry:
```
bbox.center.position.x  = (x1 + x2) / 2   # pixel coords, original image space
bbox.center.position.y  = (y1 + y2) / 2
bbox.size_x             = x2 - x1
bbox.size_y             = y2 - y1
results[0].hypothesis.class_id  = "car" / "person" / ...
results[0].hypothesis.score     = confidence in [0, 1]
```

### `sensor_msgs/Image` (viz)

A copy of the incoming frame with bounding boxes and labels rendered via OpenCV. Always published — not gated on subscriber count.

---

## ONNX Model

The node requires `yolov8n.onnx` — exported from the ultralytics `yolov8n.pt` checkpoint.

```bash
pixi run export-onnx           # writes models/yolov8n.onnx
```

Export settings: opset 12, static input shape (1, 3, 640, 640), simplified with onnxslim.

The C++ headers for ONNX Runtime are downloaded automatically by CMake on first configure via `FetchContent` (matching the version installed in the pixi environment). The runtime library is resolved from the active pixi environment, with the macOS build preferring the official CoreML-enabled prebuilt.

---

## Source Files

| File | Role |
|---|---|
| `include/perception_pipeline_cpp/camera_detector.hpp` | Public API: `CameraDetectorConfig`, `Detection`, `LetterboxInfo`, `CameraDetector` class |
| `src/camera_detector.cpp` | Core logic: letterbox, normalise, ONNX inference, decode, NMS |
| `src/camera_detector_node.cpp` | ROS 2 wrapper: subscription, publishing, viz rendering |

---

## Startup Log (healthy)

```
[camera_detector_cpp-3] Loading ONNX model: /path/to/models/yolov8n.onnx  (conf>=0.50  iou<=0.45)
[camera_detector_cpp-3] ONNX model loaded.
[camera_detector_cpp-3] CameraDetector ready — waiting for /camera/image_raw
[camera_detector_cpp-3] First image received: 1242x375  encoding=rgb8  step=3726  data=1397790 bytes
[camera_detector_cpp-3] 3 detection(s) on first frame (conf_threshold=0.50)
[camera_detector_cpp-3] Frame 20 | 5 detection(s)
```
