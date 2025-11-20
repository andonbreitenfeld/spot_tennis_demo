#include <behaviortree_cpp_v3/bt_factory.h>
#include "tennis_behaviors/check_startup_done.hpp"

BT::NodeStatus CheckStartupDone::tick()
{
    bool done = false;
    getInput("startup_done", done);
    return done ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<CheckStartupDone>("CheckStartupDone");
}
