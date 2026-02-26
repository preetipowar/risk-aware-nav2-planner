cmake_minimum_required(VERSION 3.5)
project(risk_aware_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(include)

add_library(risk_aware_planner SHARED
 src/risk_aware_planner.cpp
)

ament_target_dependencies(
 risk_aware_planner
 rclcpp
 nav2_core
 nav_msgs
 pluginlib
 nav2_costmap_2d
)

pluginlib_export_plugin_description_file(nav2_core risk_aware_planner_plugin.xml)

install(
 TARGETS risk_aware_planner
 LIBRARY DESTINATION lib
)

install(
 DIRECTORY include/
 DESTINATION include/
)

install(
 DIRECTORY config
 DESTINATION share/${PROJECT_NAME}
)

ament_package()
