#pragma once
#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>

class CheckBallModeActive : public BT::ConditionNode
{
public:
  CheckBallModeActive(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts() 
  { 
    return { BT::InputPort<bool>("ball_mode_active") }; 
  }
  BT::NodeStatus tick() override;
};