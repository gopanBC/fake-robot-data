cmake_minimum_required(VERSION 3.8)
project(fake_robot_data)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(include)


add_executable(fake_diffdrive_node src/fake_diffdrive_node.cpp)
ament_target_dependencies(fake_robot_data rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs)


install(TARGETS
fake_robot_data
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch config include
DESTINATION share/${PROJECT_NAME}/)


ament_package()