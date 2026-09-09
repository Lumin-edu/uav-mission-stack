cmake_minimum_required(VERSION 3.8)
project(bringup_ego)

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
  scripts/auto_land_after_goal.py
  scripts/ego_hw_monitor.py
  scripts/rigid_odom_transform.py
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
  ament_add_test(test_planner_geometry
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_planner_geometry.py
  )
  ament_add_test(test_raw_pointlio_contract
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_raw_pointlio_contract.py
  )
  ament_add_test(test_package_names
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_package_names.py
  )
endif()

ament_package()
