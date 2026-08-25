cmake_minimum_required(VERSION 3.0.2)
project(bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  mavros_msgs
  nav_msgs
  apriltag_ros
  cv_bridge
  sensor_msgs
  rospy
  std_srvs
  std_msgs
  tf
)

add_service_files(
  FILES
  SetServoAngle.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  scripts/fastlio_to_mavros_visual_odom.py
  scripts/fixed_point_hover.py
  scripts/fixed_point_offboard.py
  scripts/up.py
  scripts/px4_dds_monitor.py
  scripts/px4_control_watchdog.py
  scripts/px4_fastlio_position_compare.py
  scripts/camera_node.py
  scripts/tag_tracker.py
  scripts/bringup_servo.py
  scripts/tag_drop_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
