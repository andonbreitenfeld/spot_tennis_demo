cmake_minimum_required(VERSION 3.8)
project(spot_behaviors)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(spot_bt_nodes SHARED
  src/bin_detected.cpp
  src/ball_detected.cpp
)

ament_target_dependencies(spot_bt_nodes
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  std_msgs
)

install(TARGETS spot_bt_nodes
  LIBRARY DESTINATION lib
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(FILES bt_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
