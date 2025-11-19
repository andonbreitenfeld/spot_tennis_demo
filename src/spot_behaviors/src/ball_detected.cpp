#include "spot_behaviors/ball_detected.hpp"
#include <std_msgs/msg/bool.hpp>

BallDetected::BallDetected(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  detected_ = false;

  auto cb = [this](std_msgs::msg::Bool::SharedPtr msg){
    detected_ = msg->data;
  };

  auto sub = node_->create_subscription<std_msgs::msg::Bool>(
    "/ball_detected", 10, cb);
}

BT::NodeStatus BallDetected::tick()
{
  return detected_ ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
}
