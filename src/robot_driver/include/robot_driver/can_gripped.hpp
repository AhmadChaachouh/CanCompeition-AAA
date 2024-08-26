#ifndef CAN_GRIPPED_HPP_
#define CAN_GRIPPED_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

class CanGripped : public BT::ConditionNode
{
public:
    CanGripped(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    bool is_can_gripped_;
};

#endif // CAN_GRIPPED_HPP_
