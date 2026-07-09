cmake_minimum_required(VERSION 3.8)
project(hx_bringup_full_mission)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pointlio_to_px4_visual_odom.py
  scripts/full_mission_controller.py
  scripts/px4_control_watchdog.py
  scripts/px4_dds_monitor.py
  scripts/px4_pointlio_position_compare.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
