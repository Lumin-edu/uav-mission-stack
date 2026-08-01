cmake_minimum_required(VERSION 3.16)
project(sim_mid360_sensor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(mid360_cloud_to_livox src/mid360_cloud_to_livox.cpp)
target_compile_features(mid360_cloud_to_livox PUBLIC cxx_std_17)
ament_target_dependencies(mid360_cloud_to_livox
  rclcpp
  sensor_msgs
  livox_ros_driver2
)

install(TARGETS mid360_cloud_to_livox DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})

ament_package()
