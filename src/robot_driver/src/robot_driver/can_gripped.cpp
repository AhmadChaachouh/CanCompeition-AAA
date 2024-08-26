#include "can_gripped.hpp"

CanGripped::CanGripped(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config), is_can_gripped_(false)
{
    node_ = rclcpp::Node::make_shared("can_gripped_node");

    // Initialize the is_can_gripped_ flag, if needed, from a blackboard or other source
    // Example: Check blackboard for the gripped status
    if (config.blackboard->get("can_gripped", is_can_gripped_))
    {
        // Successfully retrieved the gripped status
    }
}

BT::PortsList CanGripped::providedPorts()
{
    return {};
}

BT::NodeStatus CanGripped::tick()
{
    // Check the gripped status
    // Update this logic according to how you track the gripped state
    // if (config.blackboard->get("can_gripped", is_can_gripped_))
    // {
    //     if (is_can_gripped_)
    //     {
    //         return BT::NodeStatus::SUCCESS;
    //     }
    //     else
    //     {
    //         return BT::NodeStatus::FAILURE;
    //     }
    // }
    // else
    // {
    //     // Blackboard key not found
    //     return BT::NodeStatus::FAILURE;
    // }

    return BT::NodeStatus::SUCCESS;
}

// Register the condition node with BehaviorTree.CPP
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<CanGripped>("CanGripped");
}
