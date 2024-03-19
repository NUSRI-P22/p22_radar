cmake_minimum_required(VERSION 3.5)
project(p22_radar)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(serial REQUIRED)

add_executable(p22_radar src/p22_radar.cpp)
ament_target_dependencies(p22_radar rclcpp radar_msgs serial)


## Include headers
target_include_directories(${PROJECT_NAME} PRIVATE include)

install(TARGETS
  p22_radar
  DESTINATION lib/${PROJECT_NAME})

ament_package()
