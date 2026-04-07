#include "perception_pipeline_cpp/camera_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>

namespace perception_pipeline_cpp {

// ── ONNX Runtime pimpl ────────────────────────────────────────────────────────

struct CameraDetector::OrtImpl {
    Ort::Env            env{ORT_LOGGING_LEVEL_WARNING, "camera_detector"};
    Ort::SessionOptions opts;
    std::unique_ptr<Ort::Session> session;
    std::string input_name;
    std::string output_name;

    explicit OrtImpl(const std::string & model_path)
    {
        // Prefer CUDA; fall back to CPU if the provider is unavailable.
        try {
            OrtCUDAProviderOptions cuda_opts{};
            cuda_opts.device_id = 0;
            opts.AppendExecutionProvider_CUDA(cuda_opts);
        } catch (const Ort::Exception &) {
            // CUDA provider not available — running on CPU.
        }
        opts.SetIntraOpNumThreads(1);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = std::make_unique<Ort::Session>(env, model_path.c_str(), opts);

        // Query actual I/O names from the loaded model
        Ort::AllocatorWithDefaultOptions alloc;
        input_name  = session->GetInputNameAllocated(0, alloc).get();
        output_name = session->GetOutputNameAllocated(0, alloc).get();
    }
};

// ── Constructor / Destructor ──────────────────────────────────────────────────

CameraDetector::CameraDetector(const CameraDetectorConfig & cfg)
: cfg_(cfg)
{
    ort_ = std::make_unique<OrtImpl>(cfg_.model_path);
}

CameraDetector::~CameraDetector() = default;

// ── Preprocess ────────────────────────────────────────────────────────────────

std::vector<float> CameraDetector::preprocess(
    const uint8_t * rgb, int width, int height, LetterboxInfo & info) const
{
    // Compute uniform scale to fit within input_width × input_height
    const float scale_x = static_cast<float>(cfg_.input_width)  / static_cast<float>(width);
    const float scale_y = static_cast<float>(cfg_.input_height) / static_cast<float>(height);
    info.scale = std::min(scale_x, scale_y);

    const int new_w = static_cast<int>(std::round(static_cast<float>(width)  * info.scale));
    const int new_h = static_cast<int>(std::round(static_cast<float>(height) * info.scale));

    // Padding (one side) to reach the target square size
    info.pad_x = (static_cast<float>(cfg_.input_width)  - static_cast<float>(new_w)) / 2.0f;
    info.pad_y = (static_cast<float>(cfg_.input_height) - static_cast<float>(new_h)) / 2.0f;

    // Wrap caller's buffer as a cv::Mat (zero-copy) and resize
    cv::Mat src(height, width, CV_8UC3, const_cast<uint8_t *>(rgb));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h), 0.0, 0.0, cv::INTER_LINEAR);

    // Pad to target size with grey (YOLOv8 canonical value = 114)
    cv::Mat canvas(cfg_.input_height, cfg_.input_width, CV_8UC3,
                   cv::Scalar(114, 114, 114));
    const int x0 = static_cast<int>(std::round(info.pad_x));
    const int y0 = static_cast<int>(std::round(info.pad_y));
    resized.copyTo(canvas(cv::Rect(x0, y0, new_w, new_h)));

    // HWC uint8 [0,255] → CHW float32 [0.0, 1.0]
    const int H = cfg_.input_height;
    const int W = cfg_.input_width;
    std::vector<float> chw(3 * H * W);
    const float inv255 = 1.0f / 255.0f;
    for (int c = 0; c < 3; ++c) {
        float * dst = chw.data() + c * H * W;
        for (int y = 0; y < H; ++y) {
            const uint8_t * row = canvas.ptr<uint8_t>(y);
            for (int x = 0; x < W; ++x) {
                dst[y * W + x] = static_cast<float>(row[x * 3 + c]) * inv255;
            }
        }
    }
    return chw;
}

// ── NMS ───────────────────────────────────────────────────────────────────────

static float box_iou(const Detection & a, const Detection & b)
{
    const float ix1 = std::max(a.x1, b.x1);
    const float iy1 = std::max(a.y1, b.y1);
    const float ix2 = std::min(a.x2, b.x2);
    const float iy2 = std::min(a.y2, b.y2);
    const float iw  = std::max(0.0f, ix2 - ix1);
    const float ih  = std::max(0.0f, iy2 - iy1);
    const float inter = iw * ih;
    if (inter == 0.0f) return 0.0f;
    const float union_area = (a.x2 - a.x1) * (a.y2 - a.y1)
                           + (b.x2 - b.x1) * (b.y2 - b.y1)
                           - inter;
    return inter / union_area;
}

std::vector<Detection> CameraDetector::nms(
    std::vector<Detection> dets, float iou_thresh)
{
    // Sort by confidence descending
    std::sort(dets.begin(), dets.end(),
              [](const Detection & a, const Detection & b){
                  return a.confidence > b.confidence;
              });

    std::vector<bool> suppressed(dets.size(), false);
    std::vector<Detection> out;
    out.reserve(dets.size());

    for (size_t i = 0; i < dets.size(); ++i) {
        if (suppressed[i]) continue;
        out.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (!suppressed[j]
                && dets[i].class_id == dets[j].class_id
                && box_iou(dets[i], dets[j]) > iou_thresh)
            {
                suppressed[j] = true;
            }
        }
    }
    return out;
}

// ── Postprocess ───────────────────────────────────────────────────────────────

std::vector<Detection> CameraDetector::postprocess(
    const float * output, int n_anchors,
    const LetterboxInfo & info, int orig_w, int orig_h) const
{
    // Output layout: (84, n_anchors) stored row-major
    //   rows 0..3  = cx, cy, w, h  (in 640×640 letterboxed space)
    //   rows 4..83 = per-class scores (80 COCO classes)
    // Access: output[feat * n_anchors + anchor_idx]

    std::vector<Detection> candidates;
    candidates.reserve(256);

    for (int i = 0; i < n_anchors; ++i) {
        // Find best-scoring class
        int   best_cls   = 0;
        float best_score = 0.0f;
        for (int c = 0; c < 80; ++c) {
            const float s = output[(4 + c) * n_anchors + i];
            if (s > best_score) { best_score = s; best_cls = c; }
        }
        if (best_score < cfg_.conf_threshold) continue;

        // Decode bbox (cx,cy,w,h) → (x1,y1,x2,y2) in 640-space
        const float cx = output[0 * n_anchors + i];
        const float cy = output[1 * n_anchors + i];
        const float bw = output[2 * n_anchors + i];
        const float bh = output[3 * n_anchors + i];

        float x1 = cx - bw * 0.5f;
        float y1 = cy - bh * 0.5f;
        float x2 = cx + bw * 0.5f;
        float y2 = cy + bh * 0.5f;

        // Invert letterbox: remove padding then undo scale
        x1 = (x1 - info.pad_x) / info.scale;
        y1 = (y1 - info.pad_y) / info.scale;
        x2 = (x2 - info.pad_x) / info.scale;
        y2 = (y2 - info.pad_y) / info.scale;

        // Clip to original image bounds
        const float fw = static_cast<float>(orig_w);
        const float fh = static_cast<float>(orig_h);
        x1 = std::max(0.0f, std::min(x1, fw));
        y1 = std::max(0.0f, std::min(y1, fh));
        x2 = std::max(0.0f, std::min(x2, fw));
        y2 = std::max(0.0f, std::min(y2, fh));

        if (x2 <= x1 || y2 <= y1) continue;

        candidates.push_back({x1, y1, x2, y2, best_cls, best_score});
    }

    return nms(std::move(candidates), cfg_.iou_threshold);
}

// ── Detect ────────────────────────────────────────────────────────────────────

std::vector<Detection> CameraDetector::detect(
    const uint8_t * rgb, int width, int height) const
{
    LetterboxInfo info;
    const auto chw = preprocess(rgb, width, height, info);

    // Build input tensor (1 × 3 × H × W)
    const std::array<int64_t, 4> in_shape{
        1, 3,
        static_cast<int64_t>(cfg_.input_height),
        static_cast<int64_t>(cfg_.input_width)
    };
    Ort::MemoryInfo mem_info =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
        mem_info,
        const_cast<float *>(chw.data()), chw.size(),
        in_shape.data(), in_shape.size());

    // Run inference
    const char * in_name  = ort_->input_name.c_str();
    const char * out_name = ort_->output_name.c_str();
    auto outputs = ort_->session->Run(
        Ort::RunOptions{nullptr},
        &in_name,  &in_tensor, 1,
        &out_name, 1);

    // Decode output: expected shape (1, 84, n_anchors)
    const auto & out_tensor = outputs[0];
    const auto   out_shape  = out_tensor.GetTensorTypeAndShapeInfo().GetShape();
    if (out_shape.size() < 3) {
        throw std::runtime_error("Unexpected ONNX output rank: " +
                                 std::to_string(out_shape.size()));
    }
    const int n_anchors = static_cast<int>(out_shape[2]);

    return postprocess(out_tensor.GetTensorData<float>(), n_anchors,
                       info, width, height);
}

}  // namespace perception_pipeline_cpp
