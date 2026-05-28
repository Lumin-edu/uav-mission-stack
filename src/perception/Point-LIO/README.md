# Point-LIO
## Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry
 
## Upstream

- This project directly comes from: https://github.com/SMBU-PolarBear-Robotics-Team/point_lio/tree/RM2025_SMBU_auto_sentry
- Special thanks to: https://github.com/hku-mars/Point-LIO
- Current remote: https://github.com/HY-LiYihan/Point-LIO.git

## 1. Introduction

<div align="center">
    <div align="center">
        <img src="https://github.com/hku-mars/Point-LIO/raw/master/image/toc4.png" width = 75% >
    </div>
    <font color=#a0a0a0 size=2>The framework and key points of the Point-LIO.</font>
</div>

**New features:**
1. would not fly under degeneration.
2. high odometry output frequency, 4k-8kHz.
3. robust to IMU saturation and severe vibration, and other aggressive motions (75 rad/s in our test).
4. no motion distortion.
5. computationally efficient, robust, versatile on public datasets with general motions. 
6. As an odometry, Point-LIO could be used in various autonomous tasks, such as trajectory planning, control, and perception, especially in cases involving very fast ego-motions (e.g., in the presence of severe vibration and high angular or linear velocity) or requiring high-rate odometry output and mapping (e.g., for high-rate feedback control and perception).

**Important notes:**

A. Please make sure the IMU and LiDAR are **Synchronized**, that's important.

B. Please obtain the saturation values of your used IMU (i.e., accelerator and gyroscope), and the units of the accelerator of your used IMU, then modify the .yaml file according to those settings, including values of 'satu_acc', 'satu_gyro', 'acc_norm'. That's improtant.

C. The warning message "Failed to find match for field 'time'." means the timestamps of each LiDAR points are missed in the rosbag file. That is important because Point-LIO processes at the sampling time of each LiDAR point.

D. We recommend to set the **extrinsic_est_en** to false if the extrinsic is given. As for the extrinsic initiallization, please refer to our recent work: [**Robust and Online LiDAR-inertial Initialization**](https://github.com/hku-mars/LiDAR_IMU_Init).

E. If a high odometry output frequency without downsample is required, set ``` publish_odometry_without_downsample ``` as true. Then the warning message of tf "TF_REPEATED_DATA" will pop up in the terminal window, because the time interval between two publish odometery is too small. The following command could be used to suppress this warning to a smaller frequency:

in your catkin_ws/src,

git clone --branch throttle-tf-repeated-data-error git@github.com:BadgerTechnologies/geometry2.git

Then rebuild, source setup.bash, run and then it should be reduced down to once every 10 seconds. If 10 seconds is still too much log output then change the ros::Duration(10.0) to 10000 seconds or whatever you like.

F. If you want to use Point-LIO without imu, set the "imu_en" as false, and provide a predefined value of gavity in "gravity_init" as true as possible in the yaml file, and keep the "use_imu_as_input" as 0.

## **1.1. Developers:**
The codes of this repo are contributed by:
[Dongjiao He (贺东娇)](https://github.com/Joanna-HE) and [Wei Xu (徐威)](https://github.com/XW-HKU)


## **1.2. Related paper**
Our paper is published on Advanced Intelligent Systems(AIS). [Point-LIO](https://onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459), DOI: 10.1002/aisy.202200459


## **1.3. Related video**
Our accompany video is available on **YouTube**.
<div align="center">
    <a href="https://youtu.be/oS83xUs42Uw" target="_blank"><img src="https://github.com/hku-mars/Point-LIO/raw/master/image/final.png" width=60% /></a>
</div>

## 2. What can Point-LIO do?
### 2.1 Simultaneous LiDAR localization and mapping (SLAM) without motion distortion

### 2.2 Produce high odometry output frequence and high bandwidth

### 2.3 SLAM with aggressive motions even the IMU is saturated

# **3. Prerequisites**

## **3.1 Ubuntu and [ROS](https://www.ros.org/)**
We tested our code on Ubuntu20.04 with noetic. Ubuntu18.04 and lower versions have problems of environments to support the Point-LIO, try to avoid using Point-LIO in those systems. Additional ROS package is required:
```
sudo apt-get install ros-xxx-pcl-conversions
```

## **3.2 Eigen**
Following the official [Eigen installation](eigen.tuxfamily.org/index.php?title=Main_Page), or directly install Eigen by:
```
sudo apt-get install libeigen3-dev
```

## **3.3 livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

*Remarks:*
- Since the Point-LIO supports Livox serials LiDAR, so the **livox_ros_driver** must be installed and **sourced** before run any Point-LIO luanch file.
- How to source? The easiest way is add the line ``` source $Licox_ros_driver_dir$/devel/setup.bash ``` to the end of file ``` ~/.bashrc ```, where ``` $Licox_ros_driver_dir$ ``` is the directory of the livox ros driver workspace (should be the ``` ws_livox ``` directory if you completely followed the livox official document).

## 4. Build
Clone the repository and build with colcon:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/HY-LiYihan/Point-LIO.git
    cd ..
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    colcon build --symlink-install -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 3.3 **livox_ros_driver**)
- If you want to use a custom build of PCL, add the following line to ~/.bashrc
```export PCL_ROOT={CUSTOM_PCL_PATH}```

## 5. Directly run

This repository is configured for Livox MID360 only.

Connect the MID360 and start its ROS 2 driver first. Then run Point-LIO with one of the built-in MID360 launch files:

```
    ros2 launch point_lio point_lio.launch.py
```

or

```
    ros2 launch point_lio mapping.launch.py
```

The MID360 parameters are maintained in:

1. `config/mid360.yaml`
2. `config/mid360_mapping.yaml`

The main fields to adjust before deployment are:

1. LiDAR point cloud topic name: `common.lid_topic`
2. IMU topic name: `common.imu_topic`
3. Translational extrinsic: `mapping.extrinsic_T`
4. Rotational extrinsic: `mapping.extrinsic_R`
5. Saturation value of IMU's accelerator and gyroscope: `mapping.satu_acc`, `mapping.satu_gyro`
6. The norm of IMU's acceleration according to unit of acceleration messages: `mapping.acc_norm`
7. Input point sampling: `point_filter_num`
8. Online scan/map downsampling: `filter_size_surf`, `filter_size_map_internal`, `filter_size_map_publish`, `filter_size_map_save`

### 5.4 PCD file save

Use the YAML parameters under `pcd_save`:

1. `pcd_save.pcd_save_en`: enable or disable PCD export
2. `pcd_save.save_on_shutdown`: save once on normal exit / Ctrl+C
3. `pcd_save.save_period_sec`: periodic save interval in seconds, `0.0` disables periodic save
4. `pcd_save.save_path`: output path of the exported PCD

The exported map comes from the internal online map rather than the low-frequency visualization topic. `pcl_viewer scans.pcd` can visualize the point clouds.

*Tips for pcl_viewer:*
- change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running. 
```
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity
```

# **6. Examples**

## **6.1. Example-1: SLAM on datasets with aggressive motions where IMU is saturated**
<div align="center">
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example1.gif"  width="40%" />
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example2.gif"  width="54%" />
</div>

## **6.2. Example-2: Application on FPV and PULSAR**
<div align="center">
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example3.gif"  width="58%" />
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example4.gif"  width="35%" />
</div>

PULSAR is a self-rotating UAV actuated by only one motor, [PULSAR](https://github.com/hku-mars/PULSAR)

## 7. Contact us
If you have any questions about this work, please feel free to contact me <hdj65822ATconnect.hku.hk> and Dr. Fu Zhang <fuzhangAThku.hk> via email.
