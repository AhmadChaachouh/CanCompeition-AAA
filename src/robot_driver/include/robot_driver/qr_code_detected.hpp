#ifndef QR_CODE_DETECTED_HPP_
#define QR_CODE_DETECTED_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

class QrCodeDetected : public BT::ConditionNode
{
public:
    QrCodeDetected(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

#endif // QR_CODE_DETECTED_HPP_
