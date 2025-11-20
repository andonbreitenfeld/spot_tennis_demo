#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "tennis_behaviors/ball_detected.hpp"
#include "tennis_behaviors/bin_detected.hpp"
#include "tennis_behaviors/check_startup_done.hpp"
#include "tennis_behaviors/check_ball_mode_active.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_executor");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    BT::BehaviorTreeFactory factory;

    // Load Nav2 Behavior Tree node plugins
    {
        std::string nav2_prefix =
            ament_index_cpp::get_package_prefix("nav2_behavior_tree");
        std::string nav2_plugin = nav2_prefix + "/lib/libnav2_behavior_tree_nodes.so";
        factory.registerFromPlugin(nav2_plugin);
    }

    // Load your BT plugin library
    {
        std::string pkg_prefix =
            ament_index_cpp::get_package_prefix("tennis_behaviors");
        std::string plugin_path = pkg_prefix + "/lib/libtennis_bt_nodes.so";
        factory.registerFromPlugin(plugin_path);
    }

    // Locate BT XML file
    const std::string pkg_share =
        ament_index_cpp::get_package_share_directory("tennis_behaviors");
    const std::string tree_file = pkg_share + "/behavior_trees/tennis_demo.xml";

    // Blackboard
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    // Build tree
    BT::Tree tree = factory.createTreeFromFile(tree_file, blackboard);

    rclcpp::Rate rate(10.0);
    RCLCPP_INFO(node->get_logger(), "BT executor started");

    while (rclcpp::ok())
    {
        exec.spin_some();
        tree.tickRoot();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
