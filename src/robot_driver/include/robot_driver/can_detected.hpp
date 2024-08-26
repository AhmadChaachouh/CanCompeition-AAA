#ifndef CAN_DETECTED_HPP_
#define CAN_DETECTED_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>

class CanDetected : public BT::ConditionNode
{
public:
    CanDetected(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    bool can_detected;
};

#endif // CAN_DETECTED_HPP_
