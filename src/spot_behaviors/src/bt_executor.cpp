#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "spot_behaviors/bin_detected.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("spot_bt_executor");
  RCLCPP_INFO(node->get_logger(), "Starting Spot BT Executor");

  BT::BehaviorTreeFactory factory;

  // 1) Register our custom node
  factory.registerNodeType<BinDetected>("BinDetected");

  // 2) Load Nav2 BT plugin library (gives us TriggerService, NavigateToPose, etc.)
  try
  {
    std::string nav2_prefix = ament_index_cpp::get_package_prefix("nav2_behavior_tree");
    std::string nav2_bt_lib = nav2_prefix + "/lib/libnav2_behavior_tree_nodes.so";

    RCLCPP_INFO(node->get_logger(), "Loading Nav2 BT plugins from: %s", nav2_bt_lib.c_str());
    factory.registerFromPlugin(nav2_bt_lib); //loads Nav2’s BT nodes (including TriggerService)
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to load Nav2 BT plugin library: %s", e.what());
    return 1;
  }

  // 3) Load our behavior tree XML file
  std::string pkg_share = ament_index_cpp::get_package_share_directory("spot_behaviors");
  std::string tree_file = pkg_share + "/behavior_trees/tennis_demo.xml";

  RCLCPP_INFO(node->get_logger(), "Loading BT from: %s", tree_file.c_str());

  BT::Tree tree = factory.createTreeFromFile(tree_file);

  // Put the rclcpp node into the blackboard so BT nodes can use it
  tree.rootBlackboard()->set("node", node);

  rclcpp::Rate rate(10.0);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    BT::NodeStatus status = tree.tickRoot();
    // For now, we just keep ticking, even if SUCCESS.
    (void)status;

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
