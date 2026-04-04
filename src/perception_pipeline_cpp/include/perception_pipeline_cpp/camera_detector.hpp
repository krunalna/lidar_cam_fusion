#pragma once

/**
 * camera_detector.hpp
 * ====================
 * Pure-logic YOLOv8 detector using ONNX Runtime — no ROS dependency.
 *
 * Pipeline:
 *   RGB image → letterbox resize (640×640) → CHW float normalise
 *     → ONNX inference → decode (1,84,8400) tensor
 *     → confidence filter → per-class greedy NMS
 *     → original-image pixel coordinates
 *
 * Headers are sourced from the official ORT prebuilt release (downloaded once by CMake FetchContent).
 * The ONNX model must be exported from the ultralytics YOLOv8 checkpoint:
 *   pixi run export-onnx
 */

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace perception_pipeline_cpp {

// ── COCO class names (80 classes) ─────────────────────────────────────────────
static constexpr std::array<const char *, 80> COCO_CLASSES = {{
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
    "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
}};

// ── Config ────────────────────────────────────────────────────────────────────
struct CameraDetectorConfig {
    std::string model_path;
    float       conf_threshold{0.5f};
    float       iou_threshold{0.45f};
    int         input_width{640};
    int         input_height{640};
};

// ── Single detection result ───────────────────────────────────────────────────
struct Detection {
    float x1, y1, x2, y2;   // pixel coords in original image space
    int   class_id;
    float confidence;
};

// ── Letterbox metadata (needed to invert the transform in postprocessing) ─────
struct LetterboxInfo {
    float scale;   // uniform scale applied to the original image
    float pad_x;   // horizontal padding added (one side, pixels in 640-space)
    float pad_y;   // vertical   padding added (one side, pixels in 640-space)
};

// ── Detector ─────────────────────────────────────────────────────────────────
class CameraDetector {
public:
    explicit CameraDetector(const CameraDetectorConfig & cfg);
    ~CameraDetector();

    /**
     * Run inference on an RGB image.
     *
     * @param rgb    H×W×3 uint8 row-major buffer in RGB channel order
     * @param width  Image width  in pixels
     * @param height Image height in pixels
     * @return       Detections in original-image pixel coordinates
     */
    std::vector<Detection> detect(
        const uint8_t * rgb, int width, int height) const;

private:
    // Letterbox resize + CHW float normalise → (3×640×640) tensor
    std::vector<float> preprocess(
        const uint8_t * rgb, int width, int height,
        LetterboxInfo & info) const;

    // Decode raw ONNX output, invert letterbox, clip to image bounds
    std::vector<Detection> postprocess(
        const float * output, int n_anchors,
        const LetterboxInfo & info, int orig_w, int orig_h) const;

    // Per-class greedy NMS
    static std::vector<Detection> nms(
        std::vector<Detection> dets, float iou_thresh);

    CameraDetectorConfig cfg_;

    // pimpl: ONNX Runtime types stay out of this header
    struct OrtImpl;
    std::unique_ptr<OrtImpl> ort_;
};

}  // namespace perception_pipeline_cpp
