#include "tennis_behaviors/bin_detected.hpp"
#include <std_msgs/msg/bool.hpp>

BinDetected::BinDetected(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  detected_ = false;

  auto callback = [this](std_msgs::msg::Bool::SharedPtr msg)
  {
    detected_ = msg->data;
  };

  node_->create_subscription<std_msgs::msg::Bool>(
    "/bin_detected", 10, callback);
}

BT::NodeStatus BinDetected::tick()
{
  return detected_ ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
}
