cmake_minimum_required(VERSION 3.8)
project(bringup_ego_mavros)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fixed_point_mavros.py
  scripts/pointlio_to_mavros_vision_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/mavros_setpoint_contract.py
  scripts/rigid_transform.py
  scripts/vision_pose_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_mavros_up_python
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${CMAKE_COMMAND} -E env
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:$ENV{PYTHONPATH}
      ROS_LOG_DIR=/tmp/bringup_ego_mavros_test_logs
      ${Python3_EXECUTABLE} -m unittest discover
      -s ${CMAKE_CURRENT_SOURCE_DIR}/test -v
  )
endif()

ament_package()
