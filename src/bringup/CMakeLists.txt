cmake_minimum_required(VERSION 3.0.2)
project(bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/fastlio_to_mavros_visual_odom.py
  scripts/fixed_point_hover.py
  scripts/fixed_point_offboard.py
  scripts/px4_dds_monitor.py
  scripts/px4_control_watchdog.py
  scripts/px4_fastlio_position_compare.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
