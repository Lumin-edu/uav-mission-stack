cmake_minimum_required(VERSION 3.8)
project(bringup_ego_mpc)

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
  scripts/ego_mpc_controller.py
  scripts/reference_sampler.py
  scripts/ego_hw_monitor.py
  scripts/rigid_odom_transform.py
  scripts/pointlio_to_px4_visual_odom.py
  scripts/px4_dds_monitor.py
  scripts/px4_control_watchdog.py
  scripts/px4_pointlio_position_compare.py
  scripts/startup_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/constrained_mpc.py
  scripts/rigid_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_ego_mpc_controller
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ego_mpc_controller.py
  )
  ament_add_test(test_mpc_config
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mpc_config.py
  )
endif()

ament_package()
