cmake_minimum_required(VERSION 3.8)
project(bringup_ego_multi_mission)

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
  scripts/planner_altitude_fusion.py
  scripts/pointlio_to_px4_visual_odom.py
  scripts/px4_dds_monitor.py
  scripts/px4_control_watchdog.py
  scripts/px4_pointlio_position_compare.py
  scripts/multi_waypoint_runner.py
  scripts/startup_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/rigid_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_multi_waypoint_runner
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_multi_waypoint_runner.py
  )
  ament_add_test(test_planner_geometry
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_planner_geometry.py
  )
endif()

ament_package()
