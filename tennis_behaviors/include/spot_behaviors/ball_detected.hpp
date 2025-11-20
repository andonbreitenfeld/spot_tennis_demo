#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>

class BallDetected : public BT::ConditionNode
{
public:
  BallDetected(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  bool detected_;
};
