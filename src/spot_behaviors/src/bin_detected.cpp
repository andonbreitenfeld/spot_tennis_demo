#include "spot_behaviors/bin_detected.hpp"

BinDetected::BinDetected(const std::string& name, const BT::NodeConfig& config)
: BT::ConditionNode(name, config),
  detected_(false)
{
  // Get the shared rclcpp::Node from the BT blackboard
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/bin_pose",
    10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr /*msg*/) {
      detected_ = true;
    }
  );
}

BT::NodeStatus BinDetected::tick()
{
  // Return SUCCESS once the bin has been detected at least once
  return detected_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
