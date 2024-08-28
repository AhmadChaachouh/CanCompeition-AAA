#include "behavior-tree/grip_can.hpp"

GripCan::GripCan(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("grip_can_node");

    // Create a publisher for the custom message
    // grip_publisher_ = node_->create_publisher<robot_hardware_interfaces::action::GripperAction>(
    //     "gripper_action", 10);

    // grip_client_ = node_->create_client<GripperAction>(this, "GripperAction");
}


// void GripCan::send_goal(){
//     using namespace std::placeholders;

//     RCLCPP_INFO(node_->get_logger(), "Waiting for action server...");

//     if (!action_client->wait_for_action_server()){
//         RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
//         rclcpp::shutdown();
//     }

//     auto goal_msg = GripperAction::Goal();
//     RCLCPP_INFO(node_->get_logger(), "Sending goal to GripperAction action server...");

//     auto send_goal_options = rclcpp_action::Client<GripperAction>::SendGoalOptions();
//     send_goal_options.goal_response_callback = std::bind(
//         &GripCan::goal_response_callback, this, _1);
//     send_goal_options.feedback_callback = std::bind(
//         &GripCan::feedback_callback, this, _1, _2);
//     send_goal_options.result_callback = std::bind(
//         &GripCan::get_result_callback, this, _1);

//     action_client->async_send_goal(goal_msg, send_goal_options);

// }

// void GripCan::goal_response_callback(const GoalHandleGripperAction::SharedPtr & goal_handle){

//     if (!goal_handle){
//         RCLCPP_INFO(node_->get_logger(), "Goal rejected by action server");
//     }
//     else{
//         RCLCPP_INFO(node_->get_logger(), "Goal accepted by action server");
//     }
// }


// void GripCan::feedback_callback(GoalHandleGripperAction::SharedPtr, const std::shared_ptr<const GripperAction::Feedback> feedback){
//     RCLCPP_INFO(node_->get_logger(), "Received feedback: left time = %.2f seconds.", feedback->feedback);
   
// }

// void GripCan::get_result_callback(const GoalHandleGripperAction::WrappedResult & result){
//     switch (result.code){
//         case rclcpp_action::ResultCode::SUCCEEDED:
//             break;
//         case rclcpp_action::ResultCode::ABORTED:
//             RCLCPP_ERROR(node_->get_logger(), "Goal was aborted...");
//             return;
//         case rclcpp_action::ResultCode::CANCELED:
//             RCLCPP_ERROR(node_->get_logger(), "Goal was canceled...");
//             return;
//         default:
//             RCLCPP_ERROR(node_->get_logger(), "Unkown result code...");
//             return;
//     }

//     // RCLCPP_INFO(node_->get_logger(), "Received result: total_time = %.2f seconds.", result.result->total_time);
//     rclcpp::shutdown();   

// }


BT::PortsList GripCan::providedPorts()
{
    return {};
}

BT::NodeStatus GripCan::tick()
{
    // grip_client_->send_goal();
    // rclcpp::spin(node_);
    // Publish a message to control the gripper
    // auto message = your_package_name::msg::GripCommand();
    // message.grip = true; // Set to true to grip, false to release
    // grip_publisher_->publish(message);

    RCLCPP_INFO(node_->get_logger(), "Gripped can...");
   

    // Return SUCCESS after publishing the message
    return BT::NodeStatus::SUCCESS;
}

// Register the action node with BehaviorTree.CPP
// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<GripCan>("GripCan");
// }
