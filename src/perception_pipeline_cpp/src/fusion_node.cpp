/**
 * fusion_node.cpp
 * ===============
 * ROS 2 node — synchronises 2D detections with filtered LiDAR, runs
 * frustum-based fusion, and publishes 3D detections.
 *
 * Subscribed topics (ApproximateTimeSynchronizer):
 *   /detections_2d       vision_msgs/Detection2DArray   YOLO bboxes
 *   /lidar/filtered      sensor_msgs/PointCloud2         preprocessed cloud
 *
 * Published topics:
 *   /detections_3d_fused vision_msgs/Detection3DArray   fused 3D detections
 *
 * Parameters:
 *   calibration_file     (string) Path to calibration.yaml  [required]
 *   min_cluster_points   (int)    Min LiDAR points per cluster  [default: 5]
 *   sync_slop            (double) ApproximateTimeSynchronizer slop in s  [default: 0.1]
 */

#include <memory>
#include <string>
#include <vector>

#include "perception_pipeline_cpp/calibration.hpp"
#include "perception_pipeline_cpp/projector.hpp"
#include "perception_pipeline_cpp/fusion_engine.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace perception_pipeline_cpp {

// ── PointCloud2 helper (mirrors lidar_processor_node.cpp) ────────────────────

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
        declare_parameter<std::string>("calibration_file",   "");
        declare_parameter<int>        ("min_cluster_points", 5);
        declare_parameter<double>     ("sync_slop",          0.1);

        const std::string calib_path = get_parameter("calibration_file").as_string();
        if (calib_path.empty()) {
            RCLCPP_FATAL(get_logger(),
                "calibration_file parameter is required. Add to launch:\n"
                "  parameters=[{'calibration_file': '/path/to/calibration.yaml'}]");
            throw std::runtime_error("calibration_file not set");
        }

        FusionConfig cfg;
        cfg.min_cluster_points = get_parameter("min_cluster_points").as_int();
        const double slop      = get_parameter("sync_slop").as_double();

        // ── Calibration + projection ────────────────────────────────────────
        RCLCPP_INFO(get_logger(), "Loading calibration: %s", calib_path.c_str());
        const Calibration calib(calib_path);
        projector_ = std::make_unique<Projector>(calib.data());

        engine_ = std::make_unique<FusionEngine>(cfg);

        // ── QoS ─────────────────────────────────────────────────────────────
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.reliable();

        // ── Subscribers (message_filters) ────────────────────────────────────
        sub_det_.subscribe(this, "/detections_2d",  qos.get_rmw_qos_profile());
        sub_pts_.subscribe(this, "/lidar/filtered", qos.get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_det_, sub_pts_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop));
        sync_->registerCallback(
            std::bind(&FusionNode::on_sync, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ── Publisher ────────────────────────────────────────────────────────
        pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
            "/detections_3d_fused", qos);

        RCLCPP_INFO(get_logger(),
            "FusionNode ready — min_cluster_points=%d  sync_slop=%.2f s",
            cfg.min_cluster_points, slop);
    }

private:
    void on_sync(
        const Detection2DArray::ConstSharedPtr & det_msg,
        const PointCloud2::ConstSharedPtr      & pts_msg)
    {
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
        const auto pts    = pc2_to_floats(*pts_msg);
        const uint32_t n  = pts_msg->width * pts_msg->height;

        // ── Fuse ─────────────────────────────────────────────────────────────
        const auto result = engine_->fuse(bboxes, pts.data(), n, *projector_);

        // ── Build + publish Detection3DArray ────────────────────────────────
        vision_msgs::msg::Detection3DArray out;
        out.header = pts_msg->header;   // Velodyne frame

        for (const auto & det : result.detections) {
            vision_msgs::msg::Detection3D d3;
            d3.header = out.header;

            // Bounding box centre
            d3.bbox.center.position.x = static_cast<double>(det.cx);
            d3.bbox.center.position.y = static_cast<double>(det.cy);
            d3.bbox.center.position.z = static_cast<double>(det.cz);
            // No rotation — axis-aligned box
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

        ++frame_count_;
        if (frame_count_ <= 3 || frame_count_ % 20 == 0) {
            RCLCPP_INFO(get_logger(),
                "Frame %u | 2D=%zu  LiDAR=%u pts  → 3D=%zu detection(s)",
                frame_count_,
                det_msg->detections.size(), n,
                result.detections.size());
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────
    std::unique_ptr<Projector>     projector_;
    std::unique_ptr<FusionEngine>  engine_;

    message_filters::Subscriber<Detection2DArray> sub_det_;
    message_filters::Subscriber<PointCloud2>      sub_pts_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_;
    uint32_t frame_count_{0};
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
