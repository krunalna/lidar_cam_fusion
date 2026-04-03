/**
 * kitti_publisher_node.cpp
 * ========================
 * ROS 2 node that reads a KITTI raw sync sequence from disk and publishes
 * camera images and LiDAR point clouds at a configurable frame rate.
 *
 * Published topics:
 *   /camera/image_raw   sensor_msgs/Image        rgb8, frame_id=camera_left
 *   /lidar/points       sensor_msgs/PointCloud2  float32 x/y/z/intensity, frame_id=velodyne
 *
 * Parameters:
 *   sequence_path  (string) Path to KITTI raw sync sequence dir  [required]
 *   frame_rate     (double) Playback rate in Hz                  [default: 10.0]
 *   loop           (bool)   Restart after last frame             [default: true]
 *   camera_id      (string) Camera subfolder name                [default: image_02]
 */

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "perception_pipeline_cpp/kitti_reader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

namespace perception_pipeline_cpp {

// ── Message builders (no ROS dependency in KittiReader) ───────────────────────

static sensor_msgs::msg::Image make_image_msg(
  const cv::Mat & rgb, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::Image msg;
  msg.header       = header;
  msg.height       = static_cast<uint32_t>(rgb.rows);
  msg.width        = static_cast<uint32_t>(rgb.cols);
  msg.encoding     = "rgb8";
  msg.is_bigendian = false;
  msg.step         = static_cast<uint32_t>(rgb.cols * 3);
  const auto * data = rgb.data;
  msg.data.assign(data, data + rgb.total() * rgb.elemSize());
  return msg;
}

static sensor_msgs::msg::PointCloud2 make_pc2_msg(
  const std::vector<float> & points_xyzi, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header       = header;
  msg.height       = 1;
  msg.width        = static_cast<uint32_t>(points_xyzi.size() / 4);
  msg.is_bigendian = false;
  msg.is_dense     = true;
  msg.point_step   = 16;  // 4 × float32
  msg.row_step     = msg.point_step * msg.width;

  const std::vector<std::pair<std::string, uint32_t>> fields = {
    {"x", 0}, {"y", 4}, {"z", 8}, {"intensity", 12}};
  for (const auto & [name, offset] : fields) {
    sensor_msgs::msg::PointField f;
    f.name     = name;
    f.offset   = offset;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count    = 1;
    msg.fields.push_back(f);
  }

  const auto * bytes = reinterpret_cast<const uint8_t *>(points_xyzi.data());
  msg.data.assign(bytes, bytes + points_xyzi.size() * sizeof(float));
  return msg;
}

// ── Node ──────────────────────────────────────────────────────────────────────

class KittiPublisherNode : public rclcpp::Node
{
public:
  explicit KittiPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("kitti_publisher", options)
  {
    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter<std::string>("sequence_path", "");
    declare_parameter<double>("frame_rate", 10.0);
    declare_parameter<bool>("loop", true);
    declare_parameter<std::string>("camera_id", "image_02");

    const auto seq_path   = get_parameter("sequence_path").as_string();
    frame_rate_            = get_parameter("frame_rate").as_double();
    loop_                  = get_parameter("loop").as_bool();
    const auto camera_id  = get_parameter("camera_id").as_string();

    if (seq_path.empty()) {
      RCLCPP_FATAL(get_logger(),
        "Parameter 'sequence_path' is required. "
        "Pass --ros-args -p sequence_path:=/path/to/kitti/sequence");
      throw std::runtime_error("sequence_path not set");
    }

    // ── KittiReader ──────────────────────────────────────────────────────────
    reader_ = std::make_unique<KittiReader>(seq_path, camera_id);
    n_frames_ = reader_->frame_count();

    // ── QoS — reliable, small queue ─────────────────────────────────────────
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();

    img_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", qos);
    pc_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", qos);

    // ── Timer ────────────────────────────────────────────────────────────────
    const auto period = std::chrono::duration<double>(1.0 / frame_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_frame(); });

    RCLCPP_INFO(get_logger(),
      "KittiPublisher ready — %zu frames @ %.1f Hz  loop=%s  seq=%s",
      n_frames_, frame_rate_, loop_ ? "true" : "false",
      std::filesystem::path(seq_path).filename().string().c_str());
  }

private:
  void publish_frame()
  {
    if (frame_idx_ >= n_frames_) {
      if (loop_) {
        frame_idx_ = 0;
        RCLCPP_INFO(get_logger(), "Looping back to frame 0");
      } else {
        RCLCPP_INFO(get_logger(), "Sequence finished. Shutting down.");
        timer_->cancel();
        rclcpp::shutdown();
        return;
      }
    }

    const size_t idx = frame_idx_;

    // Timestamp: KITTI or wall-clock
    builtin_interfaces::msg::Time stamp;
    const int64_t ts_ns = reader_->timestamp_ns(idx);
    if (ts_ns >= 0) {
      stamp.sec     = static_cast<int32_t>(ts_ns / 1'000'000'000LL);
      stamp.nanosec = static_cast<uint32_t>(ts_ns % 1'000'000'000LL);
    } else {
      stamp = now().operator builtin_interfaces::msg::Time();
    }

    // Image
    try {
      std_msgs::msg::Header hdr;
      hdr.stamp    = stamp;
      hdr.frame_id = "camera_left";
      img_pub_->publish(make_image_msg(reader_->read_image(idx), hdr));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Image frame %zu: %s", idx, e.what());
    }

    // LiDAR
    try {
      std_msgs::msg::Header hdr;
      hdr.stamp    = stamp;  // same stamp → time-sync downstream
      hdr.frame_id = "velodyne";
      pc_pub_->publish(make_pc2_msg(reader_->read_lidar(idx), hdr));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "LiDAR frame %zu: %s", idx, e.what());
    }

    if (idx % 50 == 0) {
      RCLCPP_INFO(get_logger(), "Publishing frame %zu/%zu", idx, n_frames_ - 1);
    }

    ++frame_idx_;
  }

  std::unique_ptr<KittiReader>                                    reader_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     pc_pub_;
  rclcpp::TimerBase::SharedPtr                                    timer_;

  double  frame_rate_{10.0};
  bool    loop_{true};
  size_t  n_frames_{0};
  size_t  frame_idx_{0};
};

}  // namespace perception_pipeline_cpp

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<perception_pipeline_cpp::KittiPublisherNode>());
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(rclcpp::get_logger("kitti_publisher"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
