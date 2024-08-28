#include "lap_time_action_client.hpp"

using namespace std::chrono_literals;

LapTimeActionClient::LapTimeActionClient():Node("lap_time_action_client"){
    action_client = rclcpp_action::create_client<MeasureLapTime>(
        this, 
        "MeasureLapTime"
    );
    
}


void LapTimeActionClient::send_goal(){
    using namespace std::placeholders;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for action server...");

    if (!action_client->wait_for_action_server()){
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = MeasureLapTime::Goal();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending goal to MeasureLapTime action server...");

    auto send_goal_options = rclcpp_action::Client<MeasureLapTime>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &LapTimeActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(
        &LapTimeActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(
        &LapTimeActionClient::get_result_callback, this, _1);

    action_client->async_send_goal(goal_msg, send_goal_options);

}

void LapTimeActionClient::goal_response_callback(const GoalHandleMeasureLapTime::SharedPtr & goal_handle){

    if (!goal_handle){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal rejected by action server");
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted by action server");
    }
}


void LapTimeActionClient::feedback_callback(GoalHandleMeasureLapTime::SharedPtr, const std::shared_ptr<const MeasureLapTime::Feedback> feedback){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received feedback: elapsed_time = %.2f seconds.", feedback->elapsed_time);
   
}


void LapTimeActionClient::get_result_callback(const GoalHandleMeasureLapTime::WrappedResult & result){
    switch (result.code){
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted...");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled...");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unkown result code...");
            return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received result: total_time = %.2f seconds.", result.result->total_time);
    rclcpp::shutdown();   

}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto lap_time_action_client = std::make_shared<LapTimeActionClient>();
  lap_time_action_client->send_goal();
  rclcpp::spin(lap_time_action_client);
  return 0;
}
