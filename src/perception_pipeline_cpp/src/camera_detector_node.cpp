/**
 * camera_detector_node.cpp
 * =========================
 * ROS 2 node — subscribes to raw camera images, runs YOLOv8 ONNX inference,
 * and publishes 2D bounding-box detections.
 *
 * Subscribed topics:
 *   /camera/image_raw       sensor_msgs/Image (rgb8)          raw frame
 *
 * Published topics:
 *   /detections_2d          vision_msgs/Detection2DArray      per-frame detections
 *   /camera/detections_viz  sensor_msgs/Image (rgb8)          annotated frame (debug)
 *
 * Parameters:
 *   model_path      (string) Absolute path to yolov8n.onnx  [required]
 *   conf_threshold  (double) Minimum confidence to keep     [default: 0.5]
 *   iou_threshold   (double) NMS IoU threshold              [default: 0.45]
 *
 * Export the ONNX model first:
 *   pixi run export-onnx
 *   YOLO_ONNX=$(pwd)/models/yolov8n.onnx pixi run launch-cpp
 */

#include <memory>
#include <string>
#include <vector>

#include "perception_pipeline_cpp/camera_detector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <opencv2/imgproc.hpp>

namespace perception_pipeline_cpp {

// ── Visualisation helper ──────────────────────────────────────────────────────

static void draw_detections(cv::Mat & canvas, const std::vector<Detection> & dets)
{
    for (const auto & d : dets) {
        const cv::Point pt1(static_cast<int>(d.x1), static_cast<int>(d.y1));
        const cv::Point pt2(static_cast<int>(d.x2), static_cast<int>(d.y2));
        cv::rectangle(canvas, pt1, pt2, cv::Scalar(0, 255, 0), 2);

        const std::string label =
            std::string(COCO_CLASSES[d.class_id]) + " "
            + std::to_string(static_cast<int>(d.confidence * 100)) + "%";

        int baseline = 0;
        const auto text_sz =
            cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        const int tag_y1 =
            std::max(static_cast<int>(d.y1) - text_sz.height - 4, 0);

        cv::rectangle(canvas,
            cv::Point(static_cast<int>(d.x1), tag_y1),
            cv::Point(static_cast<int>(d.x1) + text_sz.width + 4,
                      static_cast<int>(d.y1)),
            cv::Scalar(0, 255, 0), cv::FILLED);
        cv::putText(canvas, label,
            cv::Point(static_cast<int>(d.x1) + 2, static_cast<int>(d.y1) - 3),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
}

// ── Node ─────────────────────────────────────────────────────────────────────

class CameraDetectorNode : public rclcpp::Node
{
public:
    explicit CameraDetectorNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("camera_detector", options)
    {
        declare_parameter<std::string>("model_path",      "");
        declare_parameter<double>     ("conf_threshold",  0.5);
        declare_parameter<double>     ("iou_threshold",   0.45);

        CameraDetectorConfig cfg;
        cfg.model_path     = get_parameter("model_path").as_string();
        cfg.conf_threshold = static_cast<float>(get_parameter("conf_threshold").as_double());
        cfg.iou_threshold  = static_cast<float>(get_parameter("iou_threshold").as_double());

        if (cfg.model_path.empty()) {
            RCLCPP_FATAL(get_logger(),
                "model_path parameter is empty. Export the model first:\n"
                "  pixi run export-onnx\n"
                "Then launch with:\n"
                "  YOLO_ONNX=$(pwd)/models/yolov8n.onnx pixi run launch-cpp");
            throw std::runtime_error("model_path not set");
        }

        RCLCPP_INFO(get_logger(),
            "Loading ONNX model: %s  (conf>=%.2f  iou<=%.2f)",
            cfg.model_path.c_str(), cfg.conf_threshold, cfg.iou_threshold);

        detector_ = std::make_unique<CameraDetector>(cfg);
        RCLCPP_INFO(get_logger(), "ONNX model loaded.");

        // ── QoS ─────────────────────────────────────────────────────────────
        rclcpp::QoS sub_qos(rclcpp::KeepLast(1));
        sub_qos.best_effort();

        rclcpp::QoS pub_qos(rclcpp::KeepLast(5));
        pub_qos.reliable();

        rclcpp::QoS viz_qos(rclcpp::KeepLast(1));
        viz_qos.reliable();

        // ── Sub / Pub ────────────────────────────────────────────────────────
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", sub_qos,
            [this](sensor_msgs::msg::Image::SharedPtr msg){ on_image(msg); });

        pub_det_ = create_publisher<vision_msgs::msg::Detection2DArray>(
            "/detections_2d", pub_qos);
        pub_viz_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/detections_viz", viz_qos);

        RCLCPP_INFO(get_logger(), "CameraDetector ready — waiting for /camera/image_raw");
    }

private:
    void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        const int W = static_cast<int>(msg->width);
        const int H = static_cast<int>(msg->height);

        if (msg->data.size() != static_cast<size_t>(H * W * 3)) {
            RCLCPP_ERROR(get_logger(), "Unexpected image data size %zu (expected %d)",
                         msg->data.size(), H * W * 3);
            return;
        }

        std::vector<Detection> dets;
        try {
            dets = detector_->detect(msg->data.data(), W, H);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Inference failed on frame %u: %s",
                         frame_count_, e.what());
            return;
        }

        // ── Publish Detection2DArray ─────────────────────────────────────────
        vision_msgs::msg::Detection2DArray out;
        out.header = msg->header;

        for (const auto & d : dets) {
            vision_msgs::msg::Detection2D det;
            det.header = msg->header;

            det.bbox.center.position.x = static_cast<double>((d.x1 + d.x2) * 0.5f);
            det.bbox.center.position.y = static_cast<double>((d.y1 + d.y2) * 0.5f);
            det.bbox.center.theta      = 0.0;
            det.bbox.size_x = static_cast<double>(d.x2 - d.x1);
            det.bbox.size_y = static_cast<double>(d.y2 - d.y1);

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = COCO_CLASSES[d.class_id];
            hyp.hypothesis.score    = static_cast<double>(d.confidence);
            det.results.push_back(hyp);

            out.detections.push_back(det);
        }
        pub_det_->publish(out);

        // ── Visualisation (only when subscribed) ─────────────────────────────
        if (pub_viz_->get_subscription_count() > 0) {
            cv::Mat canvas(H, W, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
            cv::Mat annotated = canvas.clone();
            draw_detections(annotated, dets);

            sensor_msgs::msg::Image viz;
            viz.header       = msg->header;
            viz.height       = static_cast<uint32_t>(H);
            viz.width        = static_cast<uint32_t>(W);
            viz.encoding     = "rgb8";
            viz.is_bigendian = false;
            viz.step         = static_cast<uint32_t>(W * 3);
            viz.data.assign(annotated.data,
                            annotated.data + static_cast<size_t>(H * W * 3));
            pub_viz_->publish(viz);
        }

        ++frame_count_;
        if (frame_count_ % 20 == 0) {
            RCLCPP_INFO(get_logger(), "Frame %u | %zu detection(s)",
                        frame_count_, dets.size());
        }
    }

    std::unique_ptr<CameraDetector>                                  detector_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_det_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr            pub_viz_;
    uint32_t frame_count_{0};
};

}  // namespace perception_pipeline_cpp

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<perception_pipeline_cpp::CameraDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
