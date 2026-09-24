cmake_minimum_required(VERSION 3.8)
project(fastlio_up)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fixed_point_mavros.py
  scripts/fastlio_to_mavros_vision_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/mavros_setpoint_contract.py
  scripts/rigid_transform.py
  scripts/vision_pose_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
