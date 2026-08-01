cmake_minimum_required(VERSION 3.8)
project(sim_ego_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/ego_px4_bridge.py
  scripts/sim_odom_guard.py
  scripts/planner_altitude_fusion.py
  scripts/pointlio_to_px4_visual_odom.py
  scripts/path_visualizer.py
  scripts/px4_odom_bridge.py
  scripts/scenario_map.py
  scripts/sim_health_monitor.py
  scripts/startup_goal.py
  scripts/waypoint_manager.py
  scripts/send_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
