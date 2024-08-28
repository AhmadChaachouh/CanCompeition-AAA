#ifndef GRIP_CAN_HPP_
#define GRIP_CAN_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <robot_hardware_interfaces/action/gripper_action.hpp>

class GripCan : public BT::SyncActionNode
{
public:
    using GripperAction = robot_hardware_interfaces::action::GripperAction;
    using GoalHandleGripperAction = rclcpp_action::ServerGoalHandle<GripperAction>;
    
    GripCan(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    // rclcpp::Publisher<robot_hardware_interfaces::action::GripperAction>::SharedPtr grip_publisher_;
    rclcpp::Node::SharedPtr node_;
    // void GripCan::send_goal();
    // void GripCan::goal_response_callback(const GoalHandleGripperAction::SharedPtr & goal_handle);
    // void GripCan::feedback_callback(GoalHandleGripperAction::SharedPtr, const std::shared_ptr<const GripperAction::Feedback> feedback);
    // void GripCan::get_result_callback(const GoalHandleGripperAction::WrappedResult & result);
    // rclcpp_action::Client<GripperAction>::SharedPtr grip_client_;

};

#endif // GRIP_CAN_HPP_
