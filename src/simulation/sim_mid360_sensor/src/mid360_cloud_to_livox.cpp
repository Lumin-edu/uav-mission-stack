#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{

constexpr std::size_t kSectorCount = 4;
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<double, kSectorCount> kSectorYaw = {
  0.0, kPi / 2.0, kPi, -kPi / 2.0};

struct FieldOffsets
{
  int x{-1};
  int y{-1};
  int z{-1};
  int intensity{-1};
};

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

bool is_float32_field(const sensor_msgs::msg::PointField * field)
{
  return field != nullptr && field->datatype == sensor_msgs::msg::PointField::FLOAT32;
}

bool field_offsets(const sensor_msgs::msg::PointCloud2 & cloud, FieldOffsets & offsets)
{
  const auto * x = find_field(cloud, "x");
  const auto * y = find_field(cloud, "y");
  const auto * z = find_field(cloud, "z");
  const auto * intensity = find_field(cloud, "intensity");
  if (!is_float32_field(x) || !is_float32_field(y) || !is_float32_field(z)) {
    return false;
  }
  offsets.x = static_cast<int>(x->offset);
  offsets.y = static_cast<int>(y->offset);
  offsets.z = static_cast<int>(z->offset);
  offsets.intensity = is_float32_field(intensity) ? static_cast<int>(intensity->offset) : -1;
  return true;
}

float read_float(const uint8_t * data, int offset)
{
  float value{};
  std::memcpy(&value, data + offset, sizeof(value));
  return value;
}

uint8_t reflectivity(float intensity)
{
  if (!std::isfinite(intensity)) {
    return 100;
  }
  if (intensity >= 0.0F && intensity <= 1.0F) {
    intensity *= 255.0F;
  }
  return static_cast<uint8_t>(std::clamp(intensity, 0.0F, 255.0F));
}

int64_t stamp_ns(const std_msgs::msg::Header & header)
{
  return static_cast<int64_t>(header.stamp.sec) * 1000000000LL + header.stamp.nanosec;
}

}  // namespace

class Mid360CloudToLivox : public rclcpp::Node
{
public:
  Mid360CloudToLivox()
  : Node("mid360_cloud_to_livox")
  {
    input_topics_ = {
      declare_parameter<std::string>("sector_front_topic", "/sim/mid360/sector_front"),
      declare_parameter<std::string>("sector_left_topic", "/sim/mid360/sector_left"),
      declare_parameter<std::string>("sector_rear_topic", "/sim/mid360/sector_rear"),
      declare_parameter<std::string>("sector_right_topic", "/sim/mid360/sector_right")};
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar");
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    scan_period_sec_ = declare_parameter<double>("scan_period_sec", 0.1);
    sync_tolerance_sec_ = declare_parameter<double>("sync_tolerance_sec", 0.06);
    min_publish_interval_sec_ = declare_parameter<double>("min_publish_interval_sec", 0.05);
    min_range_m_ = declare_parameter<double>("min_range_m", 0.5);
    max_range_m_ = declare_parameter<double>("max_range_m", 70.0);
    max_points_per_scan_ = declare_parameter<int>("max_points_per_scan", 24000);
    // Gazebo publishes completed scan snapshots. These switches let the
    // simulation use that completed-scan clock while retaining Livox-style
    // timing for any caller that provides a real scan-start timestamp.
    stamp_at_scan_end_ = declare_parameter<bool>("stamp_at_scan_end", false);
    zero_point_offsets_ = declare_parameter<bool>("zero_point_offsets", false);

    if (scan_period_sec_ <= 0.0 || sync_tolerance_sec_ <= 0.0 ||
      min_publish_interval_sec_ < 0.0 || min_publish_interval_sec_ > scan_period_sec_ ||
      min_range_m_ <= 0.0 ||
      max_range_m_ <= min_range_m_ || max_points_per_scan_ < 100) {
      throw std::runtime_error("Invalid MID-360 adapter parameters.");
    }

    publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_topic_, 10);
    const auto qos = rclcpp::SensorDataQoS();
    for (std::size_t index = 0; index < kSectorCount; ++index) {
      subscriptions_[index] = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[index], qos,
        [this, index](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
          latest_[index] = std::move(cloud);
          publish_if_synchronized();
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "GPU MID-360 adapter: %s -> %s, scan=%.1f Hz, cap=%d points/scan, timing=%s",
      input_topics_[0].c_str(), output_topic_.c_str(), 1.0 / scan_period_sec_,
      max_points_per_scan_, stamp_at_scan_end_ ? "snapshot-end" : "livox-start");
  }

private:
  void publish_if_synchronized()
  {
    if (std::any_of(latest_.begin(), latest_.end(), [](const auto & message) {return !message;})) {
      return;
    }

    std::array<int64_t, kSectorCount> stamps{};
    for (std::size_t index = 0; index < kSectorCount; ++index) {
      stamps[index] = stamp_ns(latest_[index]->header);
      if (stamps[index] == 0) {
        return;
      }
    }
    const auto [min_stamp, max_stamp] = std::minmax_element(stamps.begin(), stamps.end());
    const int64_t tolerance_ns = static_cast<int64_t>(sync_tolerance_sec_ * 1e9);
    const int64_t min_publish_interval_ns =
      static_cast<int64_t>(min_publish_interval_sec_ * 1e9);
    if (
      *max_stamp - *min_stamp > tolerance_ns || *max_stamp <= last_publish_stamp_ns_ ||
      (last_publish_stamp_ns_ != std::numeric_limits<int64_t>::min() &&
      *max_stamp - last_publish_stamp_ns_ < min_publish_interval_ns)) {
      return;
    }

    std::vector<livox_ros_driver2::msg::CustomPoint> points;
    points.reserve(static_cast<std::size_t>(max_points_per_scan_));
    const std::size_t max_points_per_sector =
      std::max<std::size_t>(1, static_cast<std::size_t>(max_points_per_scan_) / kSectorCount);

    for (std::size_t sector = 0; sector < kSectorCount; ++sector) {
      append_sector(*latest_[sector], sector, max_points_per_sector, points);
    }
    if (points.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "MID-360 sectors were synchronized but contained no valid points.");
      return;
    }

    const int64_t scan_period_ns = static_cast<int64_t>(scan_period_sec_ * 1e9);
    if (zero_point_offsets_) {
      for (auto & point : points) {
        point.offset_time = 0;
      }
    } else {
      const auto count = points.size();
      for (std::size_t index = 0; index < count; ++index) {
        points[index].offset_time = static_cast<uint32_t>(
          std::llround(scan_period_ns * static_cast<double>(index) /
          static_cast<double>(std::max<std::size_t>(1, count - 1))));
      }
    }

    // Gazebo timestamps a completed ray scan. In that mode the CustomMsg
    // header/timebase must match the completed scan and point offsets are zero.
    // The original Livox-style start-time behaviour remains available when
    // stamp_at_scan_end is disabled.
    const int64_t base_stamp_ns = stamp_at_scan_end_
      ? *max_stamp : std::max<int64_t>(0, *max_stamp - scan_period_ns);
    livox_ros_driver2::msg::CustomMsg output;
    output.header.stamp.sec = static_cast<int32_t>(base_stamp_ns / 1000000000LL);
    output.header.stamp.nanosec = static_cast<uint32_t>(base_stamp_ns % 1000000000LL);
    output.header.frame_id = frame_id_;
    output.timebase = static_cast<uint64_t>(base_stamp_ns);
    output.point_num = static_cast<uint32_t>(points.size());
    output.lidar_id = 0;
    output.points = std::move(points);
    publisher_->publish(output);
    last_publish_stamp_ns_ = *max_stamp;
  }

  void append_sector(
    const sensor_msgs::msg::PointCloud2 & cloud, std::size_t sector,
    std::size_t max_points, std::vector<livox_ros_driver2::msg::CustomPoint> & output)
  {
    if (cloud.is_bigendian) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring big-endian PointCloud2 data from %s.",
        input_topics_[sector].c_str());
      return;
    }
    FieldOffsets offsets;
    if (!field_offsets(cloud, offsets) || cloud.point_step == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Expected float32 x/y/z PointCloud2 fields on %s.", input_topics_[sector].c_str());
      return;
    }
    const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
    if (point_count == 0 || cloud.data.size() < point_count * cloud.point_step) {
      return;
    }
    const std::size_t stride = std::max<std::size_t>(1, (point_count + max_points - 1) / max_points);
    const float cos_yaw = static_cast<float>(std::cos(kSectorYaw[sector]));
    const float sin_yaw = static_cast<float>(std::sin(kSectorYaw[sector]));
    const float min_squared = static_cast<float>(min_range_m_ * min_range_m_);
    const float max_squared = static_cast<float>(max_range_m_ * max_range_m_);

    for (std::size_t index = 0; index < point_count; index += stride) {
      const auto * data = cloud.data.data() + index * cloud.point_step;
      const float x = read_float(data, offsets.x);
      const float y = read_float(data, offsets.y);
      const float z = read_float(data, offsets.z);
      const float squared_range = x * x + y * y + z * z;
      if (!std::isfinite(squared_range) || squared_range < min_squared || squared_range > max_squared) {
        continue;
      }
      livox_ros_driver2::msg::CustomPoint point;
      point.x = cos_yaw * x - sin_yaw * y;
      point.y = sin_yaw * x + cos_yaw * y;
      point.z = z;
      point.reflectivity = offsets.intensity >= 0 ? reflectivity(read_float(data, offsets.intensity)) : 100;
      point.tag = 0;
      point.line = static_cast<uint8_t>(index % 4);
      output.push_back(point);
    }
  }

  std::array<std::string, kSectorCount> input_topics_;
  std::array<sensor_msgs::msg::PointCloud2::ConstSharedPtr, kSectorCount> latest_{};
  std::array<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr, kSectorCount>
    subscriptions_{};
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
  std::string output_topic_;
  std::string frame_id_;
  double scan_period_sec_{0.1};
  double sync_tolerance_sec_{0.06};
  double min_publish_interval_sec_{0.05};
  double min_range_m_{0.5};
  double max_range_m_{70.0};
  int max_points_per_scan_{24000};
  bool stamp_at_scan_end_{false};
  bool zero_point_offsets_{false};
  int64_t last_publish_stamp_ns_{std::numeric_limits<int64_t>::min()};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360CloudToLivox>());
  rclcpp::shutdown();
  return 0;
}
