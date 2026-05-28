#include "li_initialization.h"

#include <algorithm>
#include <cmath>

bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false,
     refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0,
       online_calib_starts_time = 0.0;  //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::msg::Imu imu_last, imu_next;
// sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

condition_variable sig_buffer;
int scan_count = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<double> time_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;
uint64_t lidar_buffer_drop_count = 0;
uint64_t imu_buffer_drop_count = 0;

namespace
{

bool imu_wait_warned_for_current_lidar = false;
bool imu_front_warned_for_current_lidar = false;

double lidar_duration_warn_threshold()
{
  if (lidar_time_inte <= 0.0) {
    return 0.02;
  }
  return std::max(0.01, 0.2 * lidar_time_inte);
}

double imu_sync_warn_threshold()
{
  if (imu_time_inte <= 0.0) {
    return 0.01;
  }
  return std::max(0.01, 2.0 * imu_time_inte);
}

void warn_lidar_duration_if_needed(const double lidar_beg_time, const double lidar_last_time)
{
  const double duration = lidar_last_time - lidar_beg_time;
  const double threshold = lidar_duration_warn_threshold();
  if (duration <= 0.0 || std::fabs(duration - lidar_time_inte) > threshold) {
    RCLCPP_WARN(
      rclcpp::get_logger("li_initialization"),
      "Point-LIO sync warning: abnormal LiDAR frame duration. duration=%.6f s expected=%.6f "
      "s threshold=%.6f s lidar_beg=%.6f lidar_end=%.6f",
      duration, lidar_time_inte, threshold, lidar_beg_time, lidar_last_time);
  }
}

void warn_imu_wait_if_needed(const double required_imu_time)
{
  if (imu_wait_warned_for_current_lidar) {
    return;
  }
  const double missing = required_imu_time - last_timestamp_imu;
  if (missing > imu_sync_warn_threshold()) {
    RCLCPP_WARN(
      rclcpp::get_logger("li_initialization"),
      "Point-LIO sync warning: waiting for IMU to cover LiDAR frame end. missing=%.6f s "
      "lidar_required=%.6f last_imu=%.6f",
      missing, required_imu_time, last_timestamp_imu);
    imu_wait_warned_for_current_lidar = true;
  }
}

void warn_imu_front_gap_if_needed(const double lidar_beg_time)
{
  if (imu_front_warned_for_current_lidar || imu_deque.empty()) {
    return;
  }
  const double imu_front_time = get_time_sec(imu_deque.front()->header.stamp);
  const double front_gap = imu_front_time - lidar_beg_time;
  if (front_gap > imu_sync_warn_threshold()) {
    RCLCPP_WARN(
      rclcpp::get_logger("li_initialization"),
      "Point-LIO sync warning: IMU queue starts after LiDAR frame begin. gap=%.6f s "
      "lidar_beg=%.6f imu_front=%.6f. LiDAR-IMU propagation may be degraded.",
      front_gap, lidar_beg_time, imu_front_time);
    imu_front_warned_for_current_lidar = true;
  }
}

void trim_lidar_buffer_if_needed()
{
  if (lio_operation_mode == "offline_map") {
    return;
  }
  bool dropped = false;
  while (
    realtime_max_lidar_queue > 0 &&
    lidar_buffer.size() > static_cast<size_t>(realtime_max_lidar_queue) &&
    !time_buffer.empty()) {
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_buffer_drop_count++;
    lidar_pushed = false;
    dropped = true;
  }
  if (dropped) {
    RCLCPP_WARN(
      rclcpp::get_logger("li_initialization"),
      "Point-LIO realtime warning: dropped old LiDAR frame(s). dropped_lidar=%llu "
      "lidar_queue=%zu max_lidar_queue=%d",
      static_cast<unsigned long long>(lidar_buffer_drop_count), lidar_buffer.size(),
      realtime_max_lidar_queue);
  }
}

void trim_imu_buffer_if_needed()
{
  if (lio_operation_mode == "offline_map") {
    return;
  }
  bool dropped = false;
  while (
    realtime_max_imu_queue > 0 &&
    imu_deque.size() > static_cast<size_t>(realtime_max_imu_queue)) {
    imu_deque.pop_front();
    imu_buffer_drop_count++;
    imu_pushed = false;
    dropped = true;
  }
  if (dropped) {
    RCLCPP_WARN(
      rclcpp::get_logger("li_initialization"),
      "Point-LIO realtime warning: dropped old IMU sample(s). dropped_imu=%llu imu_queue=%zu "
      "max_imu_queue=%d",
      static_cast<unsigned long long>(imu_buffer_drop_count), imu_deque.size(),
      realtime_max_imu_queue);
  }
}

}  // namespace

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  // mtx_buffer.lock();
  scan_count++;
  double preprocess_start_time = omp_get_wtime();
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");
    return;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
  // printf("check lidar time %f\n", last_timestamp_lidar);
  // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
  //     timediff_set_flg = true;
  //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
  //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
  // }

  if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
    deque<PointCloudXYZI::Ptr> ptr;
    deque<double> timestamp_lidar;
    p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
    while (!ptr.empty() && !timestamp_lidar.empty()) {
      lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      time_buffer.push_back(timestamp_lidar.front() / double(1000));  //unit:s
      timestamp_lidar.pop_front();
    }
  } else {
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(20000, 1));
    p_pre->process(msg, ptr);
    if (con_frame) {
      if (frame_ct == 0) {
        time_con = last_timestamp_lidar;  //msg->header.stamp.toSec();
      }
      if (frame_ct < 10) {
        for (int i = 0; i < ptr->size(); i++) {
          ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
          ptr_con->push_back(ptr->points[i]);
        }
        frame_ct++;
      } else {
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        // std::cout << "ptr div num:" << ptr_div->size() << '\n';
        *ptr_con_i = *ptr_con;
        lidar_buffer.push_back(ptr_con_i);
        double time_con_i = time_con;
        time_buffer.push_back(time_con_i);
        ptr_con->clear();
        frame_ct = 0;
      }
    } else {
      if (!ptr->points.empty()) {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
      }
    }
  }
  trim_lidar_buffer_if_needed();
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  // mtx_buffer.unlock();
  // sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg)
{
  // mtx_buffer.lock();
  double preprocess_start_time = omp_get_wtime();
  scan_count++;
  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "lidar loop back, clear buffer");
    return;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
  // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
  //     timediff_set_flg = true;
  //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
  //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
  // }

  if (cut_frame_init) {
    deque<PointCloudXYZI::Ptr> ptr;
    deque<double> timestamp_lidar;
    p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

    while (!ptr.empty() && !timestamp_lidar.empty()) {
      lidar_buffer.push_back(ptr.front());
      ptr.pop_front();
      time_buffer.push_back(timestamp_lidar.front() / double(1000));  //unit:s
      timestamp_lidar.pop_front();
    }
  } else {
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI(10000, 1));
    p_pre->process(msg, ptr);
    if (con_frame) {
      if (frame_ct == 0) {
        time_con = last_timestamp_lidar;  //msg->header.stamp.toSec();
      }
      if (frame_ct < 10) {
        for (int i = 0; i < ptr->size(); i++) {
          ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
          ptr_con->push_back(ptr->points[i]);
        }
        frame_ct++;
      } else {
        PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI(10000, 1));
        // std::cout << "ptr div num:" << ptr_div->size() << '\n';
        *ptr_con_i = *ptr_con;
        double time_con_i = time_con;
        lidar_buffer.push_back(ptr_con_i);
        time_buffer.push_back(time_con_i);
        ptr_con->clear();
        frame_ct = 0;
      }
    } else {
      if (!ptr->points.empty()) {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
      }
    }
  }
  trim_lidar_buffer_if_needed();
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  // mtx_buffer.unlock();
  // sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr & msg_in)
{
  // mtx_buffer.lock();

  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  // publish_count ++;

  msg->header.stamp = get_ros_time(
    get_time_sec(msg_in->header.stamp) - timediff_imu_wrt_lidar - time_diff_lidar_to_imu -
    time_lag_IMU_wtr_lidar);

  double timestamp = get_time_sec(msg->header.stamp);
  // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

  if (timestamp < last_timestamp_imu) {
    RCLCPP_ERROR(rclcpp::get_logger("li_initialization"), "imu loop back, clear deque");
    // imu_deque.shrink_to_fit();
    // std::cout << "check time:" << timestamp << ";" << last_timestamp_imu << '\n';
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
    return;
  }
  imu_deque.emplace_back(msg);
  last_timestamp_imu = timestamp;
  trim_imu_buffer_if_needed();
  // mtx_buffer.unlock();
  // sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup & meas)
{
  {
    if (!imu_en) {
      if (!lidar_buffer.empty()) {
        if (!lidar_pushed) {
          meas.lidar = lidar_buffer.front();
          meas.lidar_beg_time = time_buffer.front();
          lose_lid = false;
          if (meas.lidar->points.empty()) {
            std::cout << "lose lidar" << '\n';
            // return false;
            lose_lid = true;
          } else {
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt : meas.lidar->points) {
              if (pt.curvature > end_time) {
                end_time = pt.curvature;
              }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
            warn_lidar_duration_if_needed(meas.lidar_beg_time, meas.lidar_last_time);
          }
          lidar_pushed = true;
        }

        time_buffer.pop_front();
        lidar_buffer.pop_front();
        lidar_pushed = false;
        if (!lose_lid) {
          return true;
        } else {
          return false;
        }
      }
      return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty()) {
      return false;
    }
    /*** push a lidar scan ***/
    if (!lidar_pushed) {
      lose_lid = false;
      meas.lidar = lidar_buffer.front();
      meas.lidar_beg_time = time_buffer.front();
      if (meas.lidar->points.size() < 1) {
        std::cout << "lose lidar" << '\n';
        lose_lid = true;
        // lidar_buffer.pop_front();
        // time_buffer.pop_front();
        // return false;
      } else {
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt : meas.lidar->points) {
          if (pt.curvature > end_time) {
            end_time = pt.curvature;
          }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
        // std::cout << "check time lidar:" << end_time << '\n';
        meas.lidar_last_time = lidar_end_time;
        warn_lidar_duration_if_needed(meas.lidar_beg_time, meas.lidar_last_time);
      }
      imu_wait_warned_for_current_lidar = false;
      imu_front_warned_for_current_lidar = false;
      lidar_pushed = true;
    }

    if (!lose_lid && (last_timestamp_imu < lidar_end_time)) {
      warn_imu_wait_if_needed(lidar_end_time);
      return false;
    }
    if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte) {
      warn_imu_wait_if_needed(meas.lidar_beg_time + lidar_time_inte);
      return false;
    }
    if (!lose_lid) {
      warn_imu_front_gap_if_needed(meas.lidar_beg_time);
    }

    if (!lose_lid && !imu_pushed) {
      /*** push imu data, and pop from imu buffer ***/
      if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        imu_next = *(imu_deque.front());
        meas.imu.shrink_to_fit();
        while (imu_time < lidar_end_time) {
          meas.imu.emplace_back(imu_deque.front());
          imu_last = imu_next;
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_time = get_time_sec(imu_deque.front()->header.stamp);  // can be changed
          imu_next = *(imu_deque.front());
        }
      }
      imu_pushed = true;
    }

    if (lose_lid && !imu_pushed) {
      /*** push imu data, and pop from imu buffer ***/
      if (p_imu->imu_need_init_) {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();

        imu_next = *(imu_deque.front());
        while (imu_time < meas.lidar_beg_time + lidar_time_inte) {
          meas.imu.emplace_back(imu_deque.front());
          imu_last = imu_next;
          imu_deque.pop_front();
          if (imu_deque.empty()) break;
          imu_time = get_time_sec(imu_deque.front()->header.stamp);  // can be changed
          imu_next = *(imu_deque.front());
        }
      }
      imu_pushed = true;
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    imu_pushed = false;
    imu_wait_warned_for_current_lidar = false;
    imu_front_warned_for_current_lidar = false;
    return true;
  }
}
