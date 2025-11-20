#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>

class BinDetected : public BT::ConditionNode
{
public:
  BinDetected(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  bool detected_;
};
