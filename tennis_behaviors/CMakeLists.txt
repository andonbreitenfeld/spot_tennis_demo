cmake_minimum_required(VERSION 3.8)
project(tennis_behaviors)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

# ------------------------------------------------
#  Build BT Node Plugin Library
# ------------------------------------------------
add_library(tennis_bt_nodes SHARED
  src/ball_detected.cpp
  src/bin_detected.cpp
  src/check_startup_done.cpp 
  src/check_ball_mode_active.cpp
)

ament_target_dependencies(tennis_bt_nodes
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  std_msgs
  pluginlib
)

# Make this a pluginlib plugin
pluginlib_export_plugin_description_file(nav2_behavior_tree bt_plugin.xml)

# Install plugin library
install(TARGETS tennis_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# ------------------------------------------------
# Install plugin description + XML trees
# ------------------------------------------------
# Install XML tree(s)
install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin XML into Nav2's plugin directory (REQUIRED for BT to load)
# install(FILES bt_plugin.xml
#   DESTINATION share/nav2_behavior_tree/plugins
# )

# ------------------------------------------------
#  Build BT executor node
# ------------------------------------------------
add_executable(bt_executor src/bt_executor.cpp)

ament_target_dependencies(bt_executor
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  ament_index_cpp
)

# Link against your custom BT nodes library
target_link_libraries(bt_executor tennis_bt_nodes)

install(TARGETS bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------
# CRITICAL: Export the library so it can be found at runtime
# ------------------------------------------------
ament_export_libraries(tennis_bt_nodes)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  pluginlib
)

ament_package()