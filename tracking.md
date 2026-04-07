# Tracking Roadmap

Implementation plan for adding a dedicated ROS 2 C++ tracking phase with
launch-selectable tracker techniques and per-step verification, consistent with
the existing fusion pipeline workflow.

## Summary

- Add a new ROS 2 C++ package: `tracking`.
- Keep tracker selection runtime-configurable through launch.
- Implement in phases:
  - v1: `centroid3d`, `kalman3d`
  - v2: `deepsort2d` (all C++: ONNX ReID + DeepSORT-style association)
- Keep `/detections_3d_fused` unchanged as fusion output.
- Publish tracking output on `/detections_3d_tracked`.
- Add verification scripts after each phase with meaningful Pixi task names.

## Locked Interface Decisions

- Runtime selector:
  - `tracker_type:=off|centroid3d|kalman3d|deepsort2d`
- Tracking package output:
  - `/detections_3d_tracked` (`vision_msgs/Detection3DArray`)
- Track identity field:
  - `Detection3D.id` and `Detection2D.id`
- Launch structure:
  - separate `tracking.launch.py`, included by the main pipeline launch

## Step-by-Step Plan With Verification

| Step | Implementation Scope | Pixi Verification Task | Verification Goal |
|---|---|---|---|
| 1 | Scaffold `tracking` package (`ament_cmake`), add `tracking_node_cpp`, backend interface, params, topic plumbing, CMake/package install wiring. | `verify-tracking-foundation` | Package builds, node starts, topic IO is valid, empty input does not crash. |
| 2 | Implement `centroid3d` backend on `/detections_3d_fused` with class-aware nearest-neighbor matching and lifecycle (`min_hits`, `max_missed`). | `verify-tracking-centroid3d` | Stable IDs on smooth sequences, new IDs for new objects, stale tracks removed correctly. |
| 3 | Implement `kalman3d` backend (constant-velocity model + gating + Hungarian assignment). | `verify-tracking-kalman3d` | Fewer ID switches than centroid in scripted crossing/short-occlusion scenarios. |
| 4 | Add dedicated `tracking.launch.py` args (`enable_tracking`, `tracker_type`, thresholds), include from main fusion launch. | `verify-tracking-launch-config` | `off`, `centroid3d`, and `kalman3d` modes switch correctly via launch config. |
| 5 | Extend fusion path to preserve incoming 2D detection IDs and allow configurable 2D input topic for tracked detections. | `verify-tracking-id-propagation` | If 2D detections carry `id`, fused 3D detections retain the same `id`. |
| 6 | Add C++ ReID embedding extractor for DeepSORT (`ONNX Runtime + OpenCV` preprocess), model-path parameter, model load checks. | `verify-tracking-deepsort-model` | ReID model loads and embedding output shape/normalization checks are stable. |
| 7 | Implement `deepsort2d` backend (image-space Kalman + appearance cosine matching + IoU fallback + lifecycle), publish `/detections_2d_tracked`. | `verify-tracking-deepsort2d` | IDs remain stable through short occlusions and close object passes. |
| 8 | Wire `deepsort2d` end-to-end: route fusion 2D input to `/detections_2d_tracked`, output tracked 3D detections on `/detections_3d_tracked`. | `verify-tracking-deepsort-fusion` | End-to-end path preserves consistent IDs into 3D tracked output. |
| 9 | Add aggregate verification runner and documentation updates (`README`, `CLAUDE`, status/commands). | `verify-tracking` | Single command validates the complete tracking stack. |

## Verification Design (Per Step)

Each tracking verification script should follow the same structure used in the
current fusion pipeline verifiers:

1. `Source checks`:
   - required files exist
   - launch/CMake entries present
2. `Build artifact checks`:
   - compiled binaries/libraries exist
3. `Runtime dry-run`:
   - start node(s), publish synthetic message stream, assert expected outputs
4. `Graceful warning mode`:
   - DDS transport restrictions are warnings where appropriate, not false hard-fails

Suggested script naming:

- `scripts/verify_tracking_foundation.py`
- `scripts/verify_tracking_centroid3d.py`
- `scripts/verify_tracking_kalman3d.py`
- `scripts/verify_tracking_launch_config.py`
- `scripts/verify_tracking_id_propagation.py`
- `scripts/verify_tracking_deepsort_model.py`
- `scripts/verify_tracking_deepsort2d.py`
- `scripts/verify_tracking_deepsort_fusion.py`
- `scripts/verify_tracking.py` (aggregate)

## Pixi Task Names

- `verify-tracking-foundation`
- `verify-tracking-centroid3d`
- `verify-tracking-kalman3d`
- `verify-tracking-launch-config`
- `verify-tracking-id-propagation`
- `verify-tracking-deepsort-model`
- `verify-tracking-deepsort2d`
- `verify-tracking-deepsort-fusion`
- `verify-tracking` (aggregate runner)

## DeepSORT Plan (C++ Only)

### Mode

- `tracker_type:=deepsort2d`

### Inputs/Outputs

- Inputs:
  - `/camera/image_raw`
  - `/detections_2d`
- Intermediate output:
  - `/detections_2d_tracked` (`Detection2D.id` set)
- Final tracked output:
  - `/detections_3d_tracked` (`Detection3D.id` preserved via fusion path)

### Core Components

- ReID embedding extractor:
  - ONNX model loaded in C++ (ONNX Runtime)
  - OpenCV crop/resize/normalize pipeline
- Association logic:
  - motion gate -> appearance cosine cost -> Hungarian assignment -> IoU fallback
- Track lifecycle:
  - tentative -> confirmed -> deleted
  - configurable hit/miss thresholds

### Integration Path

- `deepsort2d` produces tracked 2D detections with stable IDs.
- Fusion consumes tracked 2D topic and carries IDs into fused detections.
- Tracking package publishes normalized final output on `/detections_3d_tracked`.

## Test and Acceptance Scenarios

- Single-object continuity over long frame sequences.
- Multi-object identity preservation in dense scenes.
- Short occlusion recovery without ID reset.
- Crossing trajectories with ID-switch comparison per method.
- Launch-based tracker mode switching without code changes.
- Restricted DDS environments handled like existing verifier behavior.

## Assumptions and Defaults

- `vision_msgs` `id` fields are available and used as canonical track IDs.
- Default operational mode after rollout: `tracker_type:=kalman3d`.
- DeepSORT uses a local ONNX ReID model path parameter.
- CPU fallback is supported for ReID inference.
- All tracking nodes and wrappers are C++ (no Python tracking wrappers).
