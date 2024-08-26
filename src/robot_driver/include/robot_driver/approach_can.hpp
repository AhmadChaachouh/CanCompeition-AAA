#ifndef APPROACH_CAN_HPP_
#define APPROACH_CAN_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ApproachCan : public BT::SyncActionNode
{
public:
    ApproachCan(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg);
    double desired_distance_;
    double current_distance_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

#endif // APPROACH_CAN_HPP_
