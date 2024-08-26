#include "grip_can.hpp"

GripCan::GripCan(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config)
{
    node_ = rclcpp::Node::make_shared("grip_can_node");

    // Create a publisher for the custom message
    grip_publisher_ = node_->create_publisher<your_package_name::msg::GripCommand>(
        "grip_command", 10);
}

BT::PortsList GripCan::providedPorts()
{
    return {};
}

BT::NodeStatus GripCan::tick()
{
    // Publish a message to control the gripper
    auto message = your_package_name::msg::GripCommand();
    message.grip = true; // Set to true to grip, false to release
    grip_publisher_->publish(message);

    // Return SUCCESS after publishing the message
    return BT::NodeStatus::SUCCESS;
}

// Register the action node with BehaviorTree.CPP
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<GripCan>("GripCan");
}
