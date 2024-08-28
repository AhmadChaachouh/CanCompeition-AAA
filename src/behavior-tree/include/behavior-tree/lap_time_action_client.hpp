#ifndef LAP_TIME_ACTION_CLIENT_HPP
#define LAP_TIME_ACTION_CLIENT_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_interface/action/measure_lap_time.hpp"

using namespace std::chrono_literals;


class LapTimeActionClient : public rclcpp::Node
{
  public:
    using MeasureLapTime = my_interface::action::MeasureLapTime;
    using GoalHandleMeasureLapTime = rclcpp_action::ClientGoalHandle<MeasureLapTime>;
    LapTimeActionClient();
    void send_goal();

  private:
    void goal_response_callback(const GoalHandleMeasureLapTime::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleMeasureLapTime::SharedPtr, const std::shared_ptr<const MeasureLapTime::Feedback> feedback);
    void get_result_callback(const GoalHandleMeasureLapTime::WrappedResult & result);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<MeasureLapTime>::SharedPtr action_client;
        
};


#endif
