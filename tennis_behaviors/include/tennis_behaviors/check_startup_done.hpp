#pragma once
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>

class CheckStartupDone : public BT::ConditionNode
{
public:
  CheckStartupDone(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts() 
  { 
    return { BT::InputPort<bool>("startup_done") }; 
  }
  BT::NodeStatus tick() override;
};