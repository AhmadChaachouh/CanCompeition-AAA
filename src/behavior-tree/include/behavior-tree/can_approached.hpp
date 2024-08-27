#ifndef CAN_APPROACHED_HPP_
#define CAN_APPROACHED_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>

class CanApproached : public BT::ConditionNode
{
public:
    CanApproached(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

#endif // CAN_APPROACHED_HPP_
