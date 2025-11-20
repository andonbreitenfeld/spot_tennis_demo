#include "tennis_behaviors/check_ball_mode_active.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

CheckBallModeActive::CheckBallModeActive(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus CheckBallModeActive::tick()
{
  bool active = false;
  getInput("ball_mode_active", active);
  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CheckBallModeActive>("CheckBallModeActive");
}