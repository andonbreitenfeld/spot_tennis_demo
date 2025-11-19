#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <nav2_behavior_tree/bt_action_node.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("spot_bt_executor");

  BT::BehaviorTreeFactory factory;

  // Load Nav2 standard nodes
  nav2_behavior_tree::registerNodes(factory);

  // Load our custom nodes
  factory.registerFromPlugin(
    ament_index_cpp::get_library_path("spot_bt_nodes"));

  // Set node on blackboard
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  // Load XML tree
  std::string tree_path = 
    ament_index_cpp::get_package_share_directory("spot_behaviors") +
    "/behavior_trees/tennis_demo.xml";

  auto tree = factory.createTreeFromFile(tree_path, blackboard);

  rclcpp::Rate rate(10);

  while(rclcpp::ok())
  {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
