cmake_minimum_required(VERSION 3.8)
project(spot_behaviors)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

include_directories(
  include
)

# Library with our custom BT nodes
add_library(spot_bt_nodes SHARED
  src/bin_detected.cpp
)

ament_target_dependencies(spot_bt_nodes
  rclcpp
  behaviortree_cpp_v3
  pluginlib
  geometry_msgs
)

# Export plugin description for our custom nodes
pluginlib_export_plugin_description_file(behaviortree_cpp_v3 bt_plugin.xml)

# BT executor executable
add_executable(bt_executor src/bt_executor.cpp)
target_link_libraries(bt_executor
  spot_bt_nodes
)
ament_target_dependencies(bt_executor
  rclcpp
  behaviortree_cpp_v3
  ament_index_cpp
  nav2_behavior_tree
)

# Install library and executable
install(TARGETS
  spot_bt_nodes
  bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install behavior tree XML
install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# Install plugin description file
install(FILES bt_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
