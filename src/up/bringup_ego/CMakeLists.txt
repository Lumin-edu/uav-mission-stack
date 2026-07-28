cmake_minimum_required(VERSION 3.8)
project(hx_bringup_ego)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ego_px4_bridge.py
  scripts/ego_hw_monitor.py
  scripts/startup_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
