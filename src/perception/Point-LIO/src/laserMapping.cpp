// #include <so3_math.h>
#include <malloc.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "li_initialization.h"

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;
string root_dir = ROOT_DIR;

int time_log_counter = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool flg_reset = false, flg_exit = false;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMapPublish;
pcl::VoxelGrid<PointType> downSizeFilterMapSave;

V3D euler_cur;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

auto LOGGER = rclcpp::get_logger("laserMapping");

struct MapFrameSnapshot
{
  double stamp = 0.0;
  PointCloudXYZI::Ptr world_cloud;
  PointCloudXYZI::Ptr body_cloud;
  geometry_msgs::msg::PoseStamped pose;
};

std::string resolve_output_path(const std::string & output_path)
{
  if (output_path.empty() || output_path.front() == '/') {
    return output_path;
  }
  return std::string(ROOT_DIR) + output_path;
}

void ensure_parent_directory(const std::string & output_path)
{
  const auto parent_path = std::filesystem::path(output_path).parent_path();
  if (parent_path.empty()) {
    return;
  }
  try {
    std::filesystem::create_directories(parent_path);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      LOGGER, "Failed to create PCD output directory '%s': %s", parent_path.string().c_str(),
      e.what());
  }
}

bool save_cloud_to_pcd(
  const PointCloudXYZI::Ptr & cloud, pcl::VoxelGrid<PointType> & save_filter,
  const std::string & output_path)
{
  if (cloud == nullptr || cloud->empty()) {
    RCLCPP_WARN(LOGGER, "Skip PCD save because the map cloud is empty.");
    return false;
  }

  PointCloudXYZI::Ptr downsampled_map(new PointCloudXYZI());
  save_filter.setInputCloud(cloud);
  save_filter.filter(*downsampled_map);

  const std::string full_path = resolve_output_path(output_path);
  ensure_parent_directory(full_path);
  pcl::PCDWriter pcd_writer;
  const int ret = pcd_writer.writeBinary(full_path, *downsampled_map);
  if (ret == 0) {
    RCLCPP_INFO(
      LOGGER, "PCD saved successfully: %s (points=%zu)", full_path.c_str(),
      downsampled_map->size());
    return true;
  }

  RCLCPP_ERROR(LOGGER, "Failed to save PCD: %s", full_path.c_str());
  return false;
}

class AsyncMapWorker
{
public:
  AsyncMapWorker(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_pub,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub)
  : cloud_pub_(std::move(cloud_pub)),
    body_cloud_pub_(std::move(body_cloud_pub)),
    map_pub_(std::move(map_pub)),
    path_pub_(std::move(path_pub)),
    queue_depth_(std::max(1, async_map_queue_depth)),
    accumulated_map_(new PointCloudXYZI())
  {
    map_filter_.setLeafSize(
      filter_size_map_publish, filter_size_map_publish, filter_size_map_publish);
    save_filter_.setLeafSize(filter_size_map_save, filter_size_map_save, filter_size_map_save);
    path_.header.frame_id = odom_frame_id;
  }

  ~AsyncMapWorker() { stop(); }

  void start()
  {
    running_ = true;
    worker_ = std::thread(&AsyncMapWorker::run, this);
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) {
        return;
      }
      running_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (pcd_save_en && save_on_shutdown) {
      save_cloud_to_pcd(accumulated_map_, save_filter_, pcd_save_path);
    }
  }

  void enqueue(MapFrameSnapshot && snapshot)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= static_cast<size_t>(queue_depth_)) {
      queue_.pop_front();
      dropped_jobs_++;
      RCLCPP_WARN(
        LOGGER,
        "Point-LIO async map warning: dropped old map job(s). dropped_async=%llu async_q=%zu "
        "queue_depth=%d",
        static_cast<unsigned long long>(dropped_jobs_), queue_.size(), queue_depth_);
    }
    queue_.emplace_back(std::move(snapshot));
    cv_.notify_one();
  }

  size_t queue_size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  uint64_t dropped_jobs() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_jobs_;
  }

private:
  void run()
  {
    while (true) {
      MapFrameSnapshot snapshot;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return !running_ || !queue_.empty(); });
        if (!running_ && queue_.empty()) {
          break;
        }
        snapshot = std::move(queue_.front());
        queue_.pop_front();
      }
      process(snapshot);
    }
  }

  void process(const MapFrameSnapshot & snapshot)
  {
    if (path_en) {
      path_.header.stamp = get_ros_time(snapshot.stamp);
      path_.poses.emplace_back(snapshot.pose);
      path_pub_->publish(path_);
    }

    if (scan_pub_en && snapshot.world_cloud && !snapshot.world_cloud->empty()) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*snapshot.world_cloud, msg);
      msg.header.stamp = get_ros_time(snapshot.stamp);
      msg.header.frame_id = cloud_registered_frame_id;
      cloud_pub_->publish(msg);
    }

    if (scan_pub_en && scan_body_pub_en && snapshot.body_cloud && !snapshot.body_cloud->empty()) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*snapshot.body_cloud, msg);
      msg.header.stamp = get_ros_time(snapshot.stamp);
      msg.header.frame_id = cloud_registered_body_frame_id;
      body_cloud_pub_->publish(msg);
    }

    if (map_pub_en && snapshot.world_cloud && !snapshot.world_cloud->empty()) {
      PointCloudXYZI::Ptr downsampled_frame(new PointCloudXYZI());
      map_filter_.setInputCloud(snapshot.world_cloud);
      map_filter_.filter(*downsampled_frame);
      *accumulated_map_ += *downsampled_frame;

      PointCloudXYZI::Ptr downsampled_map(new PointCloudXYZI());
      map_filter_.setInputCloud(accumulated_map_);
      map_filter_.filter(*downsampled_map);
      accumulated_map_ = downsampled_map;

      map_publish_count_++;
      if (map_publish_count_ >= map_publish_interval) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*accumulated_map_, msg);
        msg.header.stamp = get_ros_time(snapshot.stamp);
        msg.header.frame_id = map_frame_id;
        map_pub_->publish(msg);
        map_publish_count_ = 0;
      }
    }

    if (pcd_save_en && pcd_save_period_sec > 0.0) {
      save_period_count_++;
      const int frames_per_save =
        std::max(1, static_cast<int>(std::round(pcd_save_period_sec / lidar_time_inte)));
      if (save_period_count_ >= frames_per_save) {
        save_cloud_to_pcd(accumulated_map_, save_filter_, pcd_save_path);
        save_period_count_ = 0;
      }
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<MapFrameSnapshot> queue_;
  std::thread worker_;
  bool running_ = false;
  int queue_depth_ = 1;
  uint64_t dropped_jobs_ = 0;
  int map_publish_count_ = 0;
  int save_period_count_ = 0;
  nav_msgs::msg::Path path_;
  PointCloudXYZI::Ptr accumulated_map_;
  pcl::VoxelGrid<PointType> map_filter_;
  pcl::VoxelGrid<PointType> save_filter_;
};

std::string normalize_topic_name(const std::string & topic)
{
  size_t first_non_slash = topic.find_first_not_of('/');
  if (first_non_slash == std::string::npos) {
    return "";
  }
  return topic.substr(first_non_slash);
}

bool topic_matches(const std::string & configured_topic, const std::string & bag_topic)
{
  return normalize_topic_name(configured_topic) == normalize_topic_name(bag_topic);
}

template <typename MessageT>
std::shared_ptr<MessageT> deserialize_bag_message(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  rclcpp::Serialization<MessageT> serializer;
  auto message = std::make_shared<MessageT>();
  rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
  serializer.deserialize_message(&serialized_message, message.get());
  return message;
}

class OfflineBagFeeder
{
public:
  bool open(const std::string & bag_path)
  {
    if (bag_path.empty()) {
      RCLCPP_ERROR(LOGGER, "lio.offline.bag_path is empty.");
      return false;
    }
    try {
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_path;
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      reader_->open(storage_options);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(LOGGER, "Failed to open rosbag2 '%s': %s", bag_path.c_str(), e.what());
      return false;
    }
    RCLCPP_INFO(LOGGER, "Reading offline rosbag2: %s", bag_path.c_str());
    return true;
  }

  bool has_next() { return reader_ && reader_->has_next(); }

  bool feed_next()
  {
    if (!reader_ || !reader_->has_next()) {
      return false;
    }

    auto bag_message = reader_->read_next();
    if (topic_matches(imu_topic, bag_message->topic_name)) {
      imu_cbk(deserialize_bag_message<sensor_msgs::msg::Imu>(bag_message));
      return true;
    }

    if (topic_matches(lid_topic, bag_message->topic_name)) {
      if (p_pre->lidar_type == AVIA) {
        livox_pcl_cbk(deserialize_bag_message<livox_ros_driver2::msg::CustomMsg>(bag_message));
      } else {
        standard_pcl_cbk(deserialize_bag_message<sensor_msgs::msg::PointCloud2>(bag_message));
      }
      return true;
    }

    return true;
  }

private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

M3D BuildYawOnlyRotation(const double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

void Apply2DConstraint(state_input & state)
{
  if (!enable_2d_mode) {
    return;
  }
  const V3D euler = SO3ToEuler(state.rot);
  state.rot = SO3(BuildYawOnlyRotation(euler(2)));
  state.pos(2) = 0.0;
  state.vel(2) = 0.0;
}

void Apply2DConstraint(state_output & state)
{
  if (!enable_2d_mode) {
    return;
  }
  const V3D euler = SO3ToEuler(state.rot);
  state.rot = SO3(BuildYawOnlyRotation(euler(2)));
  state.pos(2) = 0.0;
  state.vel(2) = 0.0;
  state.omg(0) = 0.0;
  state.omg(1) = 0.0;
}

void Apply2DConstraintToCurrentState()
{
  if (!enable_2d_mode) {
    return;
  }
  if (use_imu_as_input) {
    Apply2DConstraint(kf_input.x_);
  } else {
    Apply2DConstraint(kf_output.x_);
  }
}

void SigHandle(int sig)
{
  flg_exit = true;
  RCLCPP_WARN(LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE * fp)
{
  V3D rot_ang;
  if (!use_imu_as_input) {
    rot_ang = SO3ToEuler(kf_output.x_.rot);
  } else {
    rot_ang = SO3ToEuler(kf_input.x_.rot);
  }

  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
  if (use_imu_as_input) {
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // Acc
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));  // Bias_g
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
      kf_input.x_.gravity(2));  // Bias_a
  } else {
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // omega
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // Acc
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));  // Bias_g
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
      kf_output.x_.gravity(2));  // Bias_a
  }
  fprintf(fp, "\r\n");
  fflush(fp);
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void MapIncremental()
{
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType & point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector & points_near = Nearest_Points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_internal).array().floor() + 0.5) *
        filter_size_map_internal;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (
          fabs(dis_2_center.x()) < 0.5 * filter_size_map_internal &&
          fabs(dis_2_center.y()) < 0.5 * filter_size_map_internal &&
          fabs(dis_2_center.z()) < 0.5 * filter_size_map_internal) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  ivox_->AddPoints(points_to_add);
}

void publish_init_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  int size_init_map = init_feats_world->size();

  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*init_feats_world, laserCloudmsg);

  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = map_frame_id;
  pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));

void publish_global_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudMap)
{
  if (pcl_wait_pub->empty()) {
    return;
  }
  if (!map_pub_en) {
    return;
  }
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = map_frame_id;
  pubLaserCloudMap->publish(laserCloudmsg);
}

void reset_map_publish_cache(const PointCloudXYZI::Ptr & source_cloud)
{
  if (!map_pub_en || source_cloud == nullptr || source_cloud->empty()) {
    return;
  }

  PointCloudXYZI::Ptr downsampled_map(new PointCloudXYZI());
  downSizeFilterMapPublish.setInputCloud(source_cloud);
  downSizeFilterMapPublish.filter(*downsampled_map);
  *pcl_wait_pub = *downsampled_map;
}

void update_map_publish_cache()
{
  if (!map_pub_en) {
    return;
  }

  PointCloudXYZI::Ptr downsampled_frame(new PointCloudXYZI());
  downSizeFilterMapPublish.setInputCloud(feats_down_world);
  downSizeFilterMapPublish.filter(*downsampled_frame);

  *pcl_wait_pub += *downsampled_frame;

  PointCloudXYZI::Ptr downsampled_map(new PointCloudXYZI());
  downSizeFilterMapPublish.setInputCloud(pcl_wait_pub);
  downSizeFilterMapPublish.filter(*downsampled_map);
  *pcl_wait_pub = *downsampled_map;
}

PointCloudXYZI::Ptr export_internal_map_cloud()
{
  PointVector internal_points;
  ivox_->CollectPoints(internal_points);
  auto map_cloud = std::make_shared<PointCloudXYZI>();
  map_cloud->reserve(internal_points.size());
  for (const auto & pt : internal_points) {
    map_cloud->push_back(pt);
  }
  return map_cloud;
}

bool save_internal_map_to_pcd(const std::string & output_path)
{
  auto internal_map = export_internal_map_cloud();
  if (internal_map->empty()) {
    RCLCPP_WARN(LOGGER, "Skip PCD save because the internal map is empty.");
    return false;
  }

  PointCloudXYZI::Ptr downsampled_map(new PointCloudXYZI());
  downSizeFilterMapSave.setInputCloud(internal_map);
  downSizeFilterMapSave.filter(*downsampled_map);

  std::string full_path = output_path;
  if (!output_path.empty() && output_path.front() != '/') {
    full_path = std::string(ROOT_DIR) + output_path;
  }
  ensure_parent_directory(full_path);

  pcl::PCDWriter pcd_writer;
  const int ret = pcd_writer.writeBinary(full_path, *downsampled_map);
  if (ret == 0) {
    RCLCPP_INFO(
      LOGGER, "PCD saved successfully: %s (points=%zu)", full_path.c_str(), downsampled_map->size());
    return true;
  }

  RCLCPP_ERROR(LOGGER, "Failed to save PCD: %s", full_path.c_str());
  return false;
}

void publish_frame_world(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  if (scan_pub_en) {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_down_world, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = cloud_registered_frame_id;
    pubLaserCloudFullRes->publish(laserCloudmsg);

  }
}

void publish_frame_body(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = cloud_registered_body_frame_id;
  pubLaserCloudFull_body->publish(laserCloudmsg);
}

template <typename T>
void set_posestamp(T & out)
{
  auto set_output_from_kf = [&](const auto & kf) {
    out.position.x = kf.x_.pos(0);
    out.position.y = kf.x_.pos(1);
    out.position.z = kf.x_.pos(2);
    Eigen::Quaterniond q(kf.x_.rot);
    if (enable_2d_mode) {
      const V3D euler = SO3ToEuler(kf.x_.rot);
      q = Eigen::Quaterniond(BuildYawOnlyRotation(euler(2)));
      out.position.z = 0.0;
    }
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  };

  if (!use_imu_as_input) {
    set_output_from_kf(kf_output);
  } else {
    set_output_from_kf(kf_input);
  }
}

void publish_odometry(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped,
  std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
  Apply2DConstraintToCurrentState();
  odomAftMapped.header.frame_id = odom_frame_id;
  odomAftMapped.child_frame_id = base_frame_id;
  if (publish_odometry_without_downsample) {
    odomAftMapped.header.stamp = get_ros_time(time_current);
  } else {
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
  }
  set_posestamp(odomAftMapped.pose.pose);
  if (enable_2d_mode) {
    odomAftMapped.pose.pose.position.z = 0.0;
    odomAftMapped.twist.twist.linear.z = 0.0;
    odomAftMapped.twist.twist.angular.x = 0.0;
    odomAftMapped.twist.twist.angular.y = 0.0;
  }

  pubOdomAftMapped->publish(odomAftMapped);

  if (tf_send_en) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = odom_frame_id;
    transform.child_frame_id = base_frame_id;
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transform.header.stamp = odomAftMapped.header.stamp;
    tf_br->sendTransform(transform);
  }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  set_posestamp(msg_body_pose.pose);
  // msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  msg_body_pose.header.frame_id = odom_frame_id;
  static int jjj = 0;
  jjj++;
  // if (jjj % 2 == 0) // if path is too large, the rvis will crash
  {
    path.poses.emplace_back(msg_body_pose);
    pubPath->publish(path);
  }
}

MapFrameSnapshot make_map_frame_snapshot(const double stamp, const bool include_body_cloud)
{
  MapFrameSnapshot snapshot;
  snapshot.stamp = stamp;
  snapshot.world_cloud.reset(new PointCloudXYZI(*feats_down_world));
  snapshot.pose.header.stamp = get_ros_time(stamp);
  snapshot.pose.header.frame_id = odom_frame_id;
  set_posestamp(snapshot.pose.pose);

  if (include_body_cloud) {
    snapshot.body_cloud.reset(new PointCloudXYZI(feats_undistort->points.size(), 1));
    for (size_t i = 0; i < feats_undistort->points.size(); i++) {
      pointBodyLidarToIMU(&feats_undistort->points[i], &snapshot.body_cloud->points[i]);
    }
  }

  return snapshot;
}

MapFrameSnapshot make_init_map_snapshot(const PointCloudXYZI::Ptr & init_map_cloud)
{
  MapFrameSnapshot snapshot;
  snapshot.stamp = lidar_end_time;
  snapshot.world_cloud.reset(new PointCloudXYZI(*init_map_cloud));
  snapshot.pose.header.stamp = get_ros_time(lidar_end_time);
  snapshot.pose.header.frame_id = odom_frame_id;
  set_posestamp(snapshot.pose.pose);
  return snapshot;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);

  readParameters(nh);
  std::cout << "lidar_type: " << lidar_type << '\n';
  ivox_ = std::make_shared<IVoxType>(ivox_options_);

  path.header.stamp = get_ros_time(lidar_end_time);
  path.header.frame_id = odom_frame_id;

  /*** variables definition for counting ***/
  int frame_num = 0;
  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
         aver_time_solve = 0, aver_time_propag = 0;

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMapPublish.setLeafSize(
    filter_size_map_publish, filter_size_map_publish, filter_size_map_publish);
  downSizeFilterMapSave.setLeafSize(filter_size_map_save, filter_size_map_save, filter_size_map_save);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    } else {
      kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    }
  }

  p_imu->lidar_type = p_pre->lidar_type = lidar_type;
  p_imu->imu_en = imu_en;

  kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
  kf_output.init_dyn_share_modified_3h(
    get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
  Eigen::Matrix<double, 24, 24> P_init;  // = MD(18, 18)::Identity() * 0.1;
  reset_cov(P_init);
  kf_input.change_P(P_init);
  Eigen::Matrix<double, 30, 30> P_init_output;  // = MD(24, 24)::Identity() * 0.01;
  reset_cov_output(P_init_output);
  kf_output.change_P(P_init_output);
  Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
  Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
  /*** debug record ***/
  FILE * fp;
  string pos_log_dir = root_dir + "/Log/pos_log.txt";
  fp = fopen(pos_log_dir.c_str(), "w");
  open_file();

  /*** ROS subscribe initialization ***/
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  const bool offline_mode = lio_operation_mode == "offline_map";
  if (!offline_mode) {
    if (p_pre->lidar_type == AVIA) {
      sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic, rclcpp::SensorDataQoS(),
        [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livox_pcl_cbk(msg); });
    } else {
      sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(),
        [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { standard_pcl_cbk(msg); });
    }
    sub_imu =
      nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);
  }
  auto pub_laser_cloud_full_res =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_registered_topic, 20);
  auto pub_laser_cloud_full_res_body =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_registered_body_topic, 20);
  auto pub_laser_cloud_effect =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_effected", 20);
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  auto pub_laser_cloud_map =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, map_qos);
  auto pub_odom_laser_link =
    nh->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 20);
  auto pub_path = nh->create_publisher<nav_msgs::msg::Path>(path_topic, 20);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
  std::unique_ptr<AsyncMapWorker> async_map_worker;
  if (lio_operation_mode == "online_odom_async_map") {
    async_map_worker = std::make_unique<AsyncMapWorker>(
      pub_laser_cloud_full_res, pub_laser_cloud_full_res_body, pub_laser_cloud_map, pub_path);
    async_map_worker->start();
  }
  std::unique_ptr<OfflineBagFeeder> offline_feeder;
  if (offline_mode) {
    offline_feeder = std::make_unique<OfflineBagFeeder>();
    if (!offline_feeder->open(offline_bag_path)) {
      return 2;
    }
  }

  //------------------------------------------------------------------------------------------------------
  signal(SIGINT, SigHandle);
  rclcpp::Rate rate(500);
  int map_publish_frame_count = 0;
  int save_period_frame_count = 0;
  uint64_t odom_overrun_count = 0;
  bool initial_map_prompt_logged = false;
  while (rclcpp::ok()) {
    if (flg_exit) break;
    bool synced_measure = false;
    if (offline_mode) {
      while (!(synced_measure = sync_packages(Measures)) && offline_feeder->has_next()) {
        offline_feeder->feed_next();
      }
      if (!synced_measure && !offline_feeder->has_next()) {
        break;
      }
    } else {
      executor.spin_some();
      synced_measure = sync_packages(Measures);
    }

    if (synced_measure) {
      if (flg_reset) {
        RCLCPP_WARN(LOGGER, "reset when rosbag play back");
        p_imu->Reset();
        feats_undistort.reset(new PointCloudXYZI());
        if (use_imu_as_input) {
          // state_in = kf_input.get_x();
          state_in = state_input();
          kf_input.change_P(P_init);
        } else {
          // state_out = kf_output.get_x();
          state_out = state_output();
          kf_output.change_P(P_init_output);
        }
        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;

        {
          ivox_.reset(new IVoxType(ivox_options_));
        }
      }

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if (imu_en) {
          // imu_next = *(imu_deque.front());
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc *= -1;

          {
            while (Measures.lidar_beg_time >
                   get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_deque.pop();
            }
          }
        } else {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);   // _init);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);  //_init);
          kf_output.x_.acc << VEC_FROM_ARRAY(gravity);      //_init);
          kf_output.x_.acc *= -1;
          p_imu->imu_need_init_ = false;
          // p_imu->after_imu_init_ = true;
        }
        G_m_s2 =
          std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      }

      double t0, t1, t2, t3, t4, t5, match_start, solve_start;
      match_time = 0;
      solve_time = 0;
      propag_time = 0;
      update_time = 0;
      t0 = omp_get_wtime();

      /*** downsample the feature points in a scan ***/
      t1 = omp_get_wtime();
      p_imu->Process(Measures, feats_undistort);
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      } else {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      }
      {
        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();
      }

      if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
      {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          if (imu_en) {
            tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
          } else {
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->after_imu_init_ = true;
          }
          // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          kf_input.x_.rot = rot_init;
          kf_output.x_.rot = rot_init;
          Apply2DConstraint(kf_input.x_);
          Apply2DConstraint(kf_output.x_);
          // kf_input.x_.rot; //.normalize();
          // kf_output.x_.rot; //.normalize();
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      /*** initialize the map ***/
      if (!init_map) {
        if (!initial_map_prompt_logged) {
          RCLCPP_INFO(
            LOGGER,
            "Building Point-LIO initial local map. Keep the LiDAR-IMU device stationary until "
            "odometry tracking starts.");
          initial_map_prompt_logged = true;
        }
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
          }
        }
        for (const auto & point : *feats_down_world) {
          init_feats_world->points.emplace_back(point);
        }

        if (init_feats_world->size() >= init_map_size) {
          ivox_->AddPoints(init_feats_world->points);
          if (async_map_worker && map_pub_en) {
            async_map_worker->enqueue(make_init_map_snapshot(init_feats_world));
          } else if (lio_operation_mode != "online_odom" && lio_operation_mode != "offline_map" &&
                     map_pub_en) {
            reset_map_publish_cache(init_feats_world);
            publish_global_map(pub_laser_cloud_map);
          }
          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
          RCLCPP_INFO(
            LOGGER,
            "Point-LIO initial local map is ready. Odometry tracking has started; normal motion is "
            "now allowed.");
        } else {
          init_map = false;
        }
        continue;
      }

      /*** ICP and Kalman filter update ***/
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);

      t2 = omp_get_wtime();

      /*** iterated state estimation ***/
      crossmat_list.resize(feats_down_size);
      pbody_list.resize(feats_down_size);
      // pbody_ext_list.reserve(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(
          feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        if (!extrinsic_est_en)
        // {
        //     if (!use_imu_as_input)
        //     {
        //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
        //     }
        //     else
        //     {
        //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
        //     }
        // }
        // else
        {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          crossmat_list[i] = point_crossmat;
        }
      }
      if (!use_imu_as_input) {
        bool imu_upda_cov = false;
        effct_feat_num = 0;
        /**** point by point update ****/
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];

            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            if (is_first_frame) {
              if (imu_en) {
                while (time_current > get_time_sec(imu_next.header.stamp)) {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
              }
              is_first_frame = false;
              imu_upda_cov = true;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            if (imu_en && !imu_deque.empty()) {
              bool last_imu = get_time_sec(imu_next.header.stamp) ==
                              get_time_sec(imu_deque.front()->header.stamp);
              while (get_time_sec(imu_next.header.stamp) < time_predict_last_const &&
                     !imu_deque.empty()) {
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                imu_upda_cov = true;
                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                /*** covariance update ***/
                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                kf_output.predict(dt, Q_output, input_in, true, false);
                Apply2DConstraint(kf_output.x_);
                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);
                    double propag_imu_start = omp_get_wtime();

                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                    Apply2DConstraint(kf_output.x_);

                    propag_time += omp_get_wtime() - propag_imu_start;
                    double solve_imu_start = omp_get_wtime();
                    kf_output.update_iterated_dyn_share_IMU();
                    Apply2DConstraint(kf_output.x_);
                    solve_time += omp_get_wtime() - solve_imu_start;
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            if (flg_reset) {
              break;
            }

            double dt = time_current - time_predict_last_const;
            double propag_state_start = omp_get_wtime();
            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                Apply2DConstraint(kf_output.x_);
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output, input_in, true, false);
            Apply2DConstraint(kf_output.x_);
            propag_time += omp_get_wtime() - propag_state_start;
            time_predict_last_const = time_current;
            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            if (!kf_output.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }
            Apply2DConstraint(kf_output.x_);
            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_laser_link, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_output.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                         << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose()
                         << " " << kf_output.x_.acc.transpose() << " "
                         << kf_output.x_.gravity.transpose() << " " << kf_output.x_.bg.transpose()
                         << " " << kf_output.x_.ba.transpose() << " "
                         << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }

            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx += time_seq[k];
            // std::cout << "pbp output effect feat num:" << effct_feat_num << '\n';
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }
                  break;
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;

                imu_upda_cov = true;
                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - time_predict_last_const;
                {
                  double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  kf_output.predict(dt_cov, Q_output, input_in, false, true);
                  Apply2DConstraint(kf_output.x_);
                  time_update_last = time_current;
                }
                kf_output.predict(dt, Q_output, input_in, true, false);
                Apply2DConstraint(kf_output.x_);
              }

                time_predict_last_const = time_current;

                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                kf_output.update_iterated_dyn_share_IMU();
                Apply2DConstraint(kf_output.x_);
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      } else {
        bool imu_prop_cov = false;
        effct_feat_num = 0;
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];
            time_current = point_body.curvature / 1000.0 + pcl_beg_time;
            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
              imu_prop_cov = true;

              is_first_frame = false;
              t_last = time_current;
              time_update_last = time_current;
              {
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              }
            }

            while (time_current > get_time_sec(imu_next.header.stamp))  // && !imu_deque.empty())
            {
              imu_deque.pop_front();

              input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                imu_last.angular_velocity.z;
              input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                imu_last.linear_acceleration.z;
              input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              double dt = get_time_sec(imu_last.header.stamp) - t_last;

              double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                Apply2DConstraint(kf_input.x_);
                time_update_last = get_time_sec(imu_last.header.stamp);  //time_current;
              }
              kf_input.predict(dt, Q_input, input_in, true, false);
              Apply2DConstraint(kf_input.x_);
              t_last = get_time_sec(imu_last.header.stamp);
              imu_prop_cov = true;

              if (imu_deque.empty()) break;
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_upda_cov = true;
            }
            if (flg_reset) {
              break;
            }
            double dt = time_current - t_last;
            t_last = time_current;
            double propag_start = omp_get_wtime();

            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                Apply2DConstraint(kf_input.x_);
                time_update_last = time_current;
              }
            }
            kf_input.predict(dt, Q_input, input_in, true, false);
            Apply2DConstraint(kf_input.x_);

            propag_time += omp_get_wtime() - propag_start;

            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");

              idx += time_seq[k];
              continue;
            }
            if (!kf_input.update_iterated_dyn_share_modified()) {
              idx = idx + time_seq[k];
              continue;
            }
            Apply2DConstraint(kf_input.x_);

            solve_start = omp_get_wtime();

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_laser_link, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_input.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                         << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                         << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose()
                         << " " << feats_undistort->points.size() << '\n';
              }
            }

            for (int j = 0; j < time_seq[k]; j++) {
              PointType & point_body_j = feats_down_body->points[idx + j + 1];
              PointType & point_world_j = feats_down_world->points[idx + j + 1];
              pointBodyToWorld(&point_body_j, &point_world_j);
            }
            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx = idx + time_seq[k];
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }

                  break;
                }
                imu_prop_cov = true;

                t_last = time_current;
                time_update_last = time_current;
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - t_last;

                double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  // kf_input.predict(dt_cov, Q_input, input_in, false, true);
                  time_update_last = get_time_sec(imu_next.header.stamp);  //time_current;
                }
                // kf_input.predict(dt, Q_input, input_in, true, false);

                t_last = get_time_sec(imu_next.header.stamp);

                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }
      // M3D rot_cur_lidar;
      // {
      //     rot_cur_lidar = state.rot_end;
      // }
      // euler_cur = RotMtoEuler(rot_cur_lidar);
      // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      //                     (euler_cur(0), euler_cur(1), euler_cur(2));
      /******* Publish odometry downsample *******/
      Apply2DConstraintToCurrentState();
      if (!publish_odometry_without_downsample) {
        publish_odometry(pub_odom_laser_link, tf_broadcaster);
      }

      /*** add the feature points to map ***/
      t3 = omp_get_wtime();
      if (feats_down_size > 4) {
        MapIncremental();
        if (async_map_worker) {
          async_map_worker->enqueue(make_map_frame_snapshot(lidar_end_time, scan_body_pub_en));
        } else if (
          lio_operation_mode != "online_odom" && lio_operation_mode != "offline_map" && map_pub_en) {
          update_map_publish_cache();
          map_publish_frame_count++;
          if (map_pub_en && map_publish_frame_count >= map_publish_interval) {
            publish_global_map(pub_laser_cloud_map);
            map_publish_frame_count = 0;
          }
        }
        if (
          pcd_save_en && pcd_save_period_sec > 0.0 && !async_map_worker &&
          lio_operation_mode != "online_odom") {
          save_period_frame_count++;
          const int frames_per_save =
            std::max(1, static_cast<int>(std::round(pcd_save_period_sec / lidar_time_inte)));
          if (save_period_frame_count >= frames_per_save) {
            save_internal_map_to_pcd(pcd_save_path);
            save_period_frame_count = 0;
          }
        }
      }
      t5 = omp_get_wtime();
      /******* Publish points *******/
      if (!async_map_worker && lio_operation_mode != "online_odom" && lio_operation_mode != "offline_map") {
        if (path_en) publish_path(pub_path);
        if (scan_pub_en) publish_frame_world(pub_laser_cloud_full_res);
        if (scan_pub_en && scan_body_pub_en) publish_frame_body(pub_laser_cloud_full_res_body);
      }

      if ((t5 - t0) > lidar_time_inte) {
        odom_overrun_count++;
        RCLCPP_WARN(
          LOGGER,
          "Point-LIO realtime warning: odometry loop overrun. mode=%s loop=%.4f s "
          "lidar_interval=%.4f s map_update=%.4f s lidar_q=%zu imu_q=%zu odom_overrun=%llu",
          lio_operation_mode.c_str(), t5 - t0, lidar_time_inte, t5 - t3, lidar_buffer.size(),
          imu_deque.size(),
          static_cast<unsigned long long>(odom_overrun_count));
      }

      /*** Debug variables Logging ***/
      if (runtime_pos_log) {
        frame_num++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        {
          aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
        }
        aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
        aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
        aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = t5 - t0;
        s_plot2[time_log_counter] = feats_undistort->points.size();
        s_plot3[time_log_counter] = aver_time_consu;
        time_log_counter++;
        printf(
          "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: "
          "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
          t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu,
          aver_time_icp, aver_time_propag);
        if (!publish_odometry_without_downsample) {
          if (!use_imu_as_input) {
            euler_cur = SO3ToEuler(kf_output.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                     << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                     << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose()
                     << " " << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose()
                     << " " << feats_undistort->points.size() << '\n';
          } else {
            euler_cur = SO3ToEuler(kf_input.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                     << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                     << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                     << feats_undistort->points.size() << '\n';
          }
        }
        dump_lio_state_to_log(fp);
      }
    }
    if (!offline_mode) {
      rate.sleep();
    }
  }
  //--------------------------save map-----------------------------------
  // 1. make sure you have enough memories
  // 2. noted that pcd save will influence the real-time performances
  if (async_map_worker) {
    async_map_worker->stop();
  }
  if (
    offline_mode && offline_map_save_on_finish &&
    (pcd_save_path.empty() || pcd_save_path != offline_output_pcd_path)) {
    pcd_save_path = offline_output_pcd_path;
  }
  if (
    ((pcd_save_en && save_on_shutdown) || (offline_mode && offline_map_save_on_finish)) &&
    lio_operation_mode != "online_odom_async_map") {
    save_internal_map_to_pcd(pcd_save_path);
  }
  fout_out.close();
  fout_imu_pbp.close();
  return 0;
}
