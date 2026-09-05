cmake_minimum_required(VERSION 3.8)
project(hx_fastlio_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/fastlio_to_px4_visual_odom.py
  scripts/fixed_point_hover.py
  scripts/px4_dds_monitor.py
  scripts/px4_control_watchdog.py
  scripts/px4_fastlio_position_compare.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/rigid_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_rigid_transform
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_rigid_transform.py
  )
endif()

ament_package()
