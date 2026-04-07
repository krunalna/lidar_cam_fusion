/**
 * fusion_node.cpp
 * ===============
 * ROS 2 node — synchronises 2D detections with filtered LiDAR, runs
 * frustum-based fusion, and publishes 3D detections.
 *
 * Subscribed topics (ApproximateTimeSynchronizer):
 *   /detections_2d        vision_msgs/Detection2DArray   YOLO bboxes
 *   /lidar/filtered       sensor_msgs/PointCloud2         preprocessed cloud
 *
 * Subscribed topics (independent, latest cached):
 *   /camera/image_raw     sensor_msgs/Image               used for debug overlay
 *
 * Published topics (always):
 *   /detections_3d_fused  vision_msgs/Detection3DArray   fused 3D detections
 *
 * Published topics (debug flags):
 *   /detections_3d_markers  visualization_msgs/MarkerArray  3D boxes + labels (publish_markers=true)
 *   /fusion/debug_image     sensor_msgs/Image                projected LiDAR + 2D bbox overlay (publish_debug_image=true)
 *
 * Parameters:
 *   calibration_file       (string) Path to calibration.yaml           [required]
 *   min_cluster_points     (int)    Min LiDAR points per cluster        [default: 5]
 *   depth_gate_min_m       (double) Minimum nearest-depth slice width m [default: 4.0]
 *   depth_gate_scale       (double) Additional slice width ratio        [default: 0.20]
 *   sync_slop              (double) ApproximateTimeSynchronizer slop s  [default: 0.1]
 *   publish_markers        (bool)   Publish MarkerArray for Foxglove    [default: true]
 *   publish_debug_image    (bool)   Publish projected-LiDAR debug image [default: true]
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>

#include "perception_pipeline_cpp/calibration.hpp"
#include "perception_pipeline_cpp/projector.hpp"
#include "perception_pipeline_cpp/fusion_engine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <opencv2/imgproc.hpp>

namespace perception_pipeline_cpp {

// ── PointCloud2 helper ────────────────────────────────────────────────────────

static std::vector<float> pc2_to_floats(const sensor_msgs::msg::PointCloud2 & msg)
{
    uint32_t x_off = 0, y_off = 4, z_off = 8, i_off = 12;
    bool has_intensity = false;
    for (const auto & f : msg.fields) {
        if      (f.name == "x")         x_off = f.offset;
        else if (f.name == "y")         y_off = f.offset;
        else if (f.name == "z")         z_off = f.offset;
        else if (f.name == "intensity") { i_off = f.offset; has_intensity = true; }
    }
    const uint32_t n = msg.width * msg.height;
    std::vector<float> out(n * 4, 0.f);
    for (uint32_t i = 0; i < n; ++i) {
        const uint8_t * p = msg.data.data() + i * msg.point_step;
        out[i*4+0] = *reinterpret_cast<const float *>(p + x_off);
        out[i*4+1] = *reinterpret_cast<const float *>(p + y_off);
        out[i*4+2] = *reinterpret_cast<const float *>(p + z_off);
        out[i*4+3] = has_intensity ? *reinterpret_cast<const float *>(p + i_off) : 0.f;
    }
    return out;
}

// ── Colour helpers ────────────────────────────────────────────────────────────

// Per-class colour for MarkerArray (RGBA, a=0.6).
static std_msgs::msg::ColorRGBA class_colour(const std::string & cls)
{
    std_msgs::msg::ColorRGBA c;
    c.a = 0.6f;
    if      (cls == "car" || cls == "truck" || cls == "bus")
        { c.r = 0.2f; c.g = 0.5f; c.b = 1.0f; }   // blue
    else if (cls == "person" || cls == "pedestrian")
        { c.r = 1.0f; c.g = 0.2f; c.b = 0.2f; }   // red
    else if (cls == "bicycle" || cls == "motorcycle")
        { c.r = 1.0f; c.g = 0.6f; c.b = 0.0f; }   // orange
    else
        { c.r = 0.2f; c.g = 1.0f; c.b = 0.3f; }   // green (default)
    return c;
}

// BGR color for a projected LiDAR point based on Velodyne-frame distance.
// green (near) → yellow (mid) → red (far), no heap allocation.
static cv::Scalar depth_to_bgr(float dist_m)
{
    const float t = std::min(std::max(dist_m / 50.f, 0.f), 1.f);
    uint8_t r, g;
    if (t < 0.5f) {
        r = static_cast<uint8_t>(t * 2.f * 255.f);   // green → yellow
        g = 255;
    } else {
        r = 255;                                        // yellow → red
        g = static_cast<uint8_t>((1.f - (t - 0.5f) * 2.f) * 255.f);
    }
    return cv::Scalar(0, g, r);  // BGR
}

// ── Node ─────────────────────────────────────────────────────────────────────

using Detection2DArray = vision_msgs::msg::Detection2DArray;
using PointCloud2      = sensor_msgs::msg::PointCloud2;
using SyncPolicy       = message_filters::sync_policies::ApproximateTime<
                            Detection2DArray, PointCloud2>;

class FusionNode : public rclcpp::Node
{
public:
    explicit FusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("fusion_node", options)
    {
        // ── Parameters ──────────────────────────────────────────────────────
        declare_parameter<std::string>("calibration_file",    "");
        declare_parameter<int>        ("min_cluster_points",  5);
        declare_parameter<double>     ("depth_gate_min_m",    4.0);
        declare_parameter<double>     ("depth_gate_scale",    0.20);
        declare_parameter<double>     ("sync_slop",           0.2);
        declare_parameter<bool>       ("publish_markers",     true);
        declare_parameter<bool>       ("publish_debug_image", true);

        const std::string calib_path = get_parameter("calibration_file").as_string();
        if (calib_path.empty()) {
            RCLCPP_FATAL(get_logger(),
                "calibration_file parameter is required.");
            throw std::runtime_error("calibration_file not set");
        }

        FusionConfig cfg;
        cfg.min_cluster_points = get_parameter("min_cluster_points").as_int();
        cfg.depth_gate_min_m   = static_cast<float>(get_parameter("depth_gate_min_m").as_double());
        cfg.depth_gate_scale   = static_cast<float>(get_parameter("depth_gate_scale").as_double());
        const double slop      = get_parameter("sync_slop").as_double();
        publish_markers_       = get_parameter("publish_markers").as_bool();
        publish_debug_image_   = get_parameter("publish_debug_image").as_bool();

        // ── Calibration + projection ────────────────────────────────────────
        RCLCPP_INFO(get_logger(), "Loading calibration: %s", calib_path.c_str());
        const Calibration calib(calib_path);
        projector_ = std::make_unique<Projector>(calib.data());
        engine_    = std::make_unique<FusionEngine>(cfg);

        // ── QoS ─────────────────────────────────────────────────────────────
        rclcpp::QoS qos5(rclcpp::KeepLast(5));
        qos5.reliable();

        // KeepLast(1) for sync inputs: prevents old messages piling up in the
        // synchronizer buffer and causing burst-then-silence behaviour.
        rclcpp::QoS qos1(rclcpp::KeepLast(1));
        qos1.reliable();

        // ── Synchronized subscribers (detections + LiDAR) ───────────────────
        sub_det_.subscribe(this, "/detections_2d",  qos1.get_rmw_qos_profile());
        sub_pts_.subscribe(this, "/lidar/filtered", qos1.get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_det_, sub_pts_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop));
        sync_->registerCallback(
            std::bind(&FusionNode::on_sync, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ── Camera image subscriber (independent — latest cached for debug) ──
        if (publish_debug_image_) {
            sub_image_ = create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
                [this](sensor_msgs::msg::Image::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(image_mutex_);
                    latest_image_ = msg;
                });
        }

        // ── Publishers ───────────────────────────────────────────────────────
        pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
            "/detections_3d_fused", qos5);

        if (publish_markers_) {
            pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                "/detections_3d_markers",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        }

        if (publish_debug_image_) {
            pub_debug_image_ = create_publisher<sensor_msgs::msg::Image>(
                "/fusion/debug_image",
                rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        }

        RCLCPP_INFO(get_logger(),
            "FusionNode ready — min_cluster_points=%d  sync_slop=%.2f s"
            "  depth_gate_min_m=%.2f  depth_gate_scale=%.2f"
            "  publish_markers=%s  publish_debug_image=%s",
            cfg.min_cluster_points, slop,
            cfg.depth_gate_min_m, cfg.depth_gate_scale,
            publish_markers_     ? "true" : "false",
            publish_debug_image_ ? "true" : "false");
    }

private:
    // ── Synchronized callback ─────────────────────────────────────────────────

    using Clock = std::chrono::steady_clock;

    void on_sync(
        const Detection2DArray::ConstSharedPtr & det_msg,
        const PointCloud2::ConstSharedPtr      & pts_msg)
    {
        const auto t0 = Clock::now();

        // ── Convert Detection2DArray → vector<BBox2D> ────────────────────────
        std::vector<BBox2D> bboxes;
        bboxes.reserve(det_msg->detections.size());
        for (const auto & d : det_msg->detections) {
            BBox2D b;
            b.x1 = static_cast<float>(d.bbox.center.position.x - d.bbox.size_x * 0.5);
            b.y1 = static_cast<float>(d.bbox.center.position.y - d.bbox.size_y * 0.5);
            b.x2 = static_cast<float>(d.bbox.center.position.x + d.bbox.size_x * 0.5);
            b.y2 = static_cast<float>(d.bbox.center.position.y + d.bbox.size_y * 0.5);
            if (!d.results.empty()) {
                b.class_id = d.results[0].hypothesis.class_id;
                b.score    = static_cast<float>(d.results[0].hypothesis.score);
            }
            bboxes.push_back(b);
        }

        // ── Convert PointCloud2 → flat float buffer ──────────────────────────
        const auto pts   = pc2_to_floats(*pts_msg);
        const uint32_t n = pts_msg->width * pts_msg->height;
        const auto t1 = Clock::now();

        // ── Fuse ─────────────────────────────────────────────────────────────
        const auto result = engine_->fuse(bboxes, pts.data(), n, *projector_);
        const auto t2 = Clock::now();

        // ── Publish Detection3DArray (always) ────────────────────────────────
        {
            vision_msgs::msg::Detection3DArray out;
            out.header = pts_msg->header;
            for (const auto & det : result.detections) {
                vision_msgs::msg::Detection3D d3;
                d3.header = out.header;
                d3.bbox.center.position.x    = static_cast<double>(det.cx);
                d3.bbox.center.position.y    = static_cast<double>(det.cy);
                d3.bbox.center.position.z    = static_cast<double>(det.cz);
                d3.bbox.center.orientation.w = 1.0;
                d3.bbox.size.x = static_cast<double>(det.size_x);
                d3.bbox.size.y = static_cast<double>(det.size_y);
                d3.bbox.size.z = static_cast<double>(det.size_z);
                vision_msgs::msg::ObjectHypothesisWithPose hyp;
                hyp.hypothesis.class_id = det.class_id;
                hyp.hypothesis.score    = static_cast<double>(det.score);
                d3.results.push_back(hyp);
                out.detections.push_back(d3);
            }
            pub_->publish(out);
        }

        // ── Publish MarkerArray (if enabled) ─────────────────────────────────
        if (publish_markers_) {
            publish_markers(pts_msg->header, result.detections);
        }

        // ── Publish debug image (if enabled) ─────────────────────────────────
        if (publish_debug_image_) {
            publish_debug_image(det_msg->header, pts.data(), n, bboxes);
        }
        const auto t3 = Clock::now();

        // ── Timing ───────────────────────────────────────────────────────────
        const double deser_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double fuse_ms  = std::chrono::duration<double, std::milli>(t2 - t1).count();
        const double pub_ms   = std::chrono::duration<double, std::milli>(t3 - t2).count();
        const double total_ms = std::chrono::duration<double, std::milli>(t3 - t0).count();

        if (last_cb_time_.time_since_epoch().count() > 0) {
            const double dt_s = std::chrono::duration<double>(t0 - last_cb_time_).count();
            const double hz = (dt_s > 0.0) ? 1.0 / dt_s : 0.0;
            rolling_hz_    = kAlpha * hz    + (1.0 - kAlpha) * rolling_hz_;
            rolling_pub_ms_ = kAlpha * pub_ms + (1.0 - kAlpha) * rolling_pub_ms_;
        } else {
            rolling_pub_ms_ = pub_ms;
        }
        last_cb_time_ = t0;

        ++frame_count_;
        const bool slow = total_ms > 80.0;
        if (frame_count_ <= 3 || frame_count_ % 30 == 0 || slow) {
            if (slow) {
                RCLCPP_WARN(get_logger(),
                    "Frame %4u | %5.1f Hz | total=%6.1f ms "
                    "(deser=%5.1f  fuse=%5.1f  pub/dbg=%5.1f) [SLOW] "
                    "| 2D=%zu LiDAR=%u → 3D=%zu",
                    frame_count_, rolling_hz_, total_ms,
                    deser_ms, fuse_ms, pub_ms,
                    det_msg->detections.size(), n, result.detections.size());
            } else {
                RCLCPP_INFO(get_logger(),
                    "Frame %4u | %5.1f Hz | total=%6.1f ms "
                    "(deser=%5.1f  fuse=%5.1f  pub/dbg=%5.1f) "
                    "| 2D=%zu LiDAR=%u → 3D=%zu",
                    frame_count_, rolling_hz_, total_ms,
                    deser_ms, fuse_ms, pub_ms,
                    det_msg->detections.size(), n, result.detections.size());
            }
        }
    }

    // ── MarkerArray publisher ─────────────────────────────────────────────────

    void publish_markers(
        const std_msgs::msg::Header       & header,
        const std::vector<Detection3D>    & detections)
    {
        visualization_msgs::msg::MarkerArray markers;

        // DELETEALL wipes stale markers from the previous frame
        {
            visualization_msgs::msg::Marker del;
            del.header = header;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            markers.markers.push_back(del);
        }

        int id = 0;
        for (const auto & det : detections) {
            const auto colour = class_colour(det.class_id);

            // CUBE — the 3D bounding box
            visualization_msgs::msg::Marker cube;
            cube.header    = header;
            cube.ns        = "fusion_boxes";
            cube.id        = id++;
            cube.type      = visualization_msgs::msg::Marker::CUBE;
            cube.action    = visualization_msgs::msg::Marker::ADD;
            cube.pose.position.x    = static_cast<double>(det.cx);
            cube.pose.position.y    = static_cast<double>(det.cy);
            cube.pose.position.z    = static_cast<double>(det.cz);
            cube.pose.orientation.w = 1.0;
            cube.scale.x = std::max(static_cast<double>(det.size_x), 0.1);
            cube.scale.y = std::max(static_cast<double>(det.size_y), 0.1);
            cube.scale.z = std::max(static_cast<double>(det.size_z), 0.1);
            cube.color   = colour;
            markers.markers.push_back(cube);

            // TEXT — class label + confidence, 0.3 m above the top face
            visualization_msgs::msg::Marker label;
            label.header    = header;
            label.ns        = "fusion_labels";
            label.id        = id++;
            label.type      = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            label.action    = visualization_msgs::msg::Marker::ADD;
            label.pose.position.x    = static_cast<double>(det.cx);
            label.pose.position.y    = static_cast<double>(det.cy);
            label.pose.position.z    = static_cast<double>(det.cz)
                                       + std::max(static_cast<double>(det.size_z), 0.1) * 0.5
                                       + 0.3;
            label.pose.orientation.w = 1.0;
            label.scale.z  = 0.4;
            label.color.r  = 1.f; label.color.g = 1.f;
            label.color.b  = 1.f; label.color.a = 1.f;
            label.text     = det.class_id + " "
                             + std::to_string(static_cast<int>(det.score * 100)) + "%";
            markers.markers.push_back(label);
        }

        pub_markers_->publish(markers);
    }

    // ── Debug image publisher ─────────────────────────────────────────────────

    void publish_debug_image(
        const std_msgs::msg::Header  & header,
        const float *                  points_xyzi,
        uint32_t                       n_points,
        const std::vector<BBox2D>    & bboxes)
    {
        // Grab cached camera image
        sensor_msgs::msg::Image::SharedPtr img_msg;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            img_msg = latest_image_;
        }
        if (!img_msg) return;   // no image received yet

        const int W = static_cast<int>(img_msg->width);
        const int H = static_cast<int>(img_msg->height);

        // Wrap image data into cv::Mat (RGB → BGR for OpenCV drawing)
        cv::Mat rgb(H, W, CV_8UC3,
            const_cast<uint8_t *>(img_msg->data.data()));
        cv::Mat canvas;
        cv::cvtColor(rgb, canvas, cv::COLOR_RGB2BGR);

        // ── Draw projected LiDAR points (depth-coloured dots) ────────────────
        // Direct pixel write (3×3 splat) avoids per-point cv::circle overhead.
        for (uint32_t i = 0; i < n_points; ++i) {
            const float * p  = points_xyzi + i * 4;
            const auto    px = projector_->project(p[0], p[1], p[2]);
            if (!px.valid) continue;

            const int u = static_cast<int>(px.u);
            const int v = static_cast<int>(px.v);
            if (u < 1 || u >= W - 1 || v < 1 || v >= H - 1) continue;

            const float dist = std::sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
            const cv::Scalar c = depth_to_bgr(dist);
            const cv::Vec3b  pix(
                static_cast<uint8_t>(c[0]),
                static_cast<uint8_t>(c[1]),
                static_cast<uint8_t>(c[2]));

            // 3×3 splat so points are visible at normal zoom
            for (int dv = -1; dv <= 1; ++dv)
                for (int du = -1; du <= 1; ++du)
                    canvas.at<cv::Vec3b>(v + dv, u + du) = pix;
        }

        // ── Draw 2D detection bboxes + labels ────────────────────────────────
        for (const auto & b : bboxes) {
            const cv::Point pt1(static_cast<int>(b.x1), static_cast<int>(b.y1));
            const cv::Point pt2(static_cast<int>(b.x2), static_cast<int>(b.y2));
            cv::rectangle(canvas, pt1, pt2, cv::Scalar(0, 255, 255), 2);  // yellow

            const std::string txt = b.class_id + " "
                + std::to_string(static_cast<int>(b.score * 100)) + "%";
            int baseline = 0;
            const auto ts = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            const int ty  = std::max(static_cast<int>(b.y1) - 4, ts.height);
            cv::rectangle(canvas,
                cv::Point(static_cast<int>(b.x1), ty - ts.height - 2),
                cv::Point(static_cast<int>(b.x1) + ts.width + 4, ty + 2),
                cv::Scalar(0, 255, 255), cv::FILLED);
            cv::putText(canvas, txt,
                cv::Point(static_cast<int>(b.x1) + 2, ty),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        }

        // ── Convert BGR → RGB and publish ────────────────────────────────────
        cv::Mat out_rgb;
        cv::cvtColor(canvas, out_rgb, cv::COLOR_BGR2RGB);

        sensor_msgs::msg::Image out_msg;
        out_msg.header       = header;
        out_msg.height       = static_cast<uint32_t>(H);
        out_msg.width        = static_cast<uint32_t>(W);
        out_msg.encoding     = "rgb8";
        out_msg.is_bigendian = false;
        out_msg.step         = static_cast<uint32_t>(W * 3);
        out_msg.data.assign(out_rgb.data, out_rgb.data + H * W * 3);

        pub_debug_image_->publish(out_msg);
    }

    // ── Members ───────────────────────────────────────────────────────────────

    std::unique_ptr<Projector>    projector_;
    std::unique_ptr<FusionEngine> engine_;

    // Synchronized subs
    message_filters::Subscriber<Detection2DArray> sub_det_;
    message_filters::Subscriber<PointCloud2>      sub_pts_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Camera image cache (for debug overlay)
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    std::mutex image_mutex_;

    // Publishers
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr  pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              pub_debug_image_;

    // Debug flags
    bool     publish_markers_{true};
    bool     publish_debug_image_{true};
    uint32_t frame_count_{0};

    // Timing
    Clock::time_point last_cb_time_{};
    double rolling_hz_{0.0};
    double rolling_pub_ms_{0.0};
    static constexpr double kAlpha = 0.2;
};

}  // namespace perception_pipeline_cpp

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<perception_pipeline_cpp::FusionNode>());
    rclcpp::shutdown();
    return 0;
}
