/**
 * lidar_processor_node.cpp
 * ========================
 * ROS 2 node — subscribes to raw Velodyne point clouds, runs the C++
 * preprocessing pipeline, and republishes filtered + ground clouds.
 *
 * Subscribed topics:
 *   /lidar/points          sensor_msgs/PointCloud2   raw scan
 *
 * Published topics:
 *   /lidar/filtered        sensor_msgs/PointCloud2   preprocessed cloud
 *   /lidar/ground_plane    sensor_msgs/PointCloud2   ground points (debug)
 *
 * Parameters: roi_x_min/max, roi_y_min/max, roi_z_min/max, voxel_size,
 *             ransac_dist, ransac_iter, max_depth  (same defaults as Python node)
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "perception_pipeline_cpp/lidar_preprocessor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

namespace perception_pipeline_cpp {

// ── PointCloud2 helpers ───────────────────────────────────────────────────────

static std::vector<float> pc2_to_floats(const sensor_msgs::msg::PointCloud2 & msg)
{
  // Resolve field byte offsets from msg.fields (handles any layout)
  uint32_t x_off = 0, y_off = 4, z_off = 8, i_off = 12;
  bool has_intensity = false;
  for (const auto & f : msg.fields) {
    if (f.name == "x")         x_off = f.offset;
    else if (f.name == "y")    y_off = f.offset;
    else if (f.name == "z")    z_off = f.offset;
    else if (f.name == "intensity") { i_off = f.offset; has_intensity = true; }
  }

  const uint32_t n = msg.width * msg.height;
  std::vector<float> out(n * 4, 0.0f);
  for (uint32_t i = 0; i < n; ++i) {
    const uint8_t * p = msg.data.data() + i * msg.point_step;
    out[i * 4 + 0] = *reinterpret_cast<const float *>(p + x_off);
    out[i * 4 + 1] = *reinterpret_cast<const float *>(p + y_off);
    out[i * 4 + 2] = *reinterpret_cast<const float *>(p + z_off);
    out[i * 4 + 3] = has_intensity ? *reinterpret_cast<const float *>(p + i_off) : 0.0f;
  }
  return out;
}

static sensor_msgs::msg::PointCloud2 floats_to_pc2(
  const std::vector<float> & pts, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header       = header;
  msg.height       = 1;
  msg.width        = static_cast<uint32_t>(pts.size() / 4);
  msg.is_bigendian = false;
  msg.is_dense     = true;
  msg.point_step   = 16;
  msg.row_step     = msg.point_step * msg.width;

  for (const auto & [name, offset] :
    std::initializer_list<std::pair<const char *, uint32_t>>{
      {"x", 0}, {"y", 4}, {"z", 8}, {"intensity", 12}})
  {
    sensor_msgs::msg::PointField f;
    f.name     = name;
    f.offset   = offset;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count    = 1;
    msg.fields.push_back(f);
  }

  const auto * bytes = reinterpret_cast<const uint8_t *>(pts.data());
  msg.data.assign(bytes, bytes + pts.size() * sizeof(float));
  return msg;
}

// ── Node ─────────────────────────────────────────────────────────────────────

class LidarProcessorNode : public rclcpp::Node
{
public:
  explicit LidarProcessorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lidar_processor", options)
  {
    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter<double>("roi_x_min",    0.0);
    declare_parameter<double>("roi_x_max",   50.0);
    declare_parameter<double>("roi_y_min",  -10.0);
    declare_parameter<double>("roi_y_max",   10.0);
    declare_parameter<double>("roi_z_min",   -3.0);
    declare_parameter<double>("roi_z_max",    2.0);
    declare_parameter<double>("voxel_size",   0.1);
    declare_parameter<double>("ransac_dist",  0.2);
    declare_parameter<int>   ("ransac_iter",  100);
    declare_parameter<double>("max_depth",   50.0);

    LidarPreprocessorConfig cfg;
    cfg.roi_x_min   = static_cast<float>(get_parameter("roi_x_min").as_double());
    cfg.roi_x_max   = static_cast<float>(get_parameter("roi_x_max").as_double());
    cfg.roi_y_min   = static_cast<float>(get_parameter("roi_y_min").as_double());
    cfg.roi_y_max   = static_cast<float>(get_parameter("roi_y_max").as_double());
    cfg.roi_z_min   = static_cast<float>(get_parameter("roi_z_min").as_double());
    cfg.roi_z_max   = static_cast<float>(get_parameter("roi_z_max").as_double());
    cfg.voxel_size  = static_cast<float>(get_parameter("voxel_size").as_double());
    cfg.ransac_dist = static_cast<float>(get_parameter("ransac_dist").as_double());
    cfg.ransac_iter = get_parameter("ransac_iter").as_int();
    cfg.max_depth   = static_cast<float>(get_parameter("max_depth").as_double());

    preprocessor_ = std::make_unique<LidarPreprocessor>(cfg);

    // ── QoS ─────────────────────────────────────────────────────────────────
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.reliable();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_pointcloud(msg); });

    pub_filtered_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/filtered", qos);
    pub_ground_   = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/ground_plane", qos);

    RCLCPP_INFO(get_logger(), "LidarProcessor ready — waiting for /lidar/points");
  }

private:
  using Clock = std::chrono::steady_clock;

  void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto cb_start = Clock::now();

    const uint32_t n_points = msg->width * msg->height;
    if (n_points == 0) return;

    const auto pts = pc2_to_floats(*msg);
    const auto result = preprocessor_->process(pts.data(), n_points);
    const auto & s = result.stats;

    std_msgs::msg::Header hdr;
    hdr.stamp    = msg->header.stamp;
    hdr.frame_id = msg->header.frame_id;

    pub_filtered_->publish(floats_to_pc2(result.filtered, hdr));
    pub_ground_->publish(floats_to_pc2(result.ground, hdr));

    // ── Timing ──────────────────────────────────────────────────────────────
    const double proc_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - cb_start).count();

    if (last_cb_time_.time_since_epoch().count() > 0) {
      const double dt_s =
        std::chrono::duration<double>(cb_start - last_cb_time_).count();
      const double hz = (dt_s > 0.0) ? 1.0 / dt_s : 0.0;
      rolling_hz_      = kAlpha * hz      + (1.0 - kAlpha) * rolling_hz_;
      rolling_proc_ms_ = kAlpha * proc_ms + (1.0 - kAlpha) * rolling_proc_ms_;
    } else {
      rolling_proc_ms_ = proc_ms;
    }
    last_cb_time_ = cb_start;

    ++frame_count_;
    // Log first 3 frames, then every 30; always warn if processing is slow
    const bool slow = proc_ms > 80.0;  // >80 ms risks dropping frames at 10 Hz
    if (frame_count_ <= 3 || frame_count_ % 30 == 0 || slow) {
      if (slow) {
        RCLCPP_WARN(get_logger(),
          "Frame %4u | %5.1f Hz | proc=%6.1f ms [SLOW] | in=%6u roi=%6u voxel=%6u out=%6u",
          frame_count_, rolling_hz_, proc_ms,
          s.n_input, s.n_roi, s.n_voxel, s.n_output);
      } else {
        RCLCPP_INFO(get_logger(),
          "Frame %4u | %5.1f Hz | proc=%6.1f ms | in=%6u roi=%6u voxel=%6u out=%6u",
          frame_count_, rolling_hz_, proc_ms,
          s.n_input, s.n_roi, s.n_voxel, s.n_output);
      }
    }
  }

  std::unique_ptr<LidarPreprocessor>                                preprocessor_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr       pub_filtered_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr       pub_ground_;
  uint32_t   frame_count_{0};
  Clock::time_point last_cb_time_{};
  double rolling_hz_{0.0};
  double rolling_proc_ms_{0.0};
  static constexpr double kAlpha = 0.2;  // EMA smoothing factor
};

}  // namespace perception_pipeline_cpp

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<perception_pipeline_cpp::LidarProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
