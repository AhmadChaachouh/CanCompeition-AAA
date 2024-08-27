#include "rclcpp/rclcpp.hpp"
#include "pepsi_detection_interface/action/detect_pepsi.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        using namespace std::placeholders;

        // Create a subscription to LaserScan
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&RobotController::laser_scan_callback, this, _1));

        // Create an action client
        action_client_ = rclcpp_action::create_client<pepsi_detection_interface::action::DetectPepsi>(
            this, "detect_pepsi");

        // Create a publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Send goal to the action server
        this->send_goal(1.0); // min_distance of 1 meter
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Client<pepsi_detection_interface::action::DetectPepsi>::SharedPtr action_client_;

    // LaserScan variables
    std::vector<float> laser_scan_ranges_;
    float angle_min_;
    float angle_max_;
    float angle_increment_;

    void send_goal(float min_distance)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = pepsi_detection_interface::action::DetectPepsi::Goal();
        goal_msg.min_distance = min_distance;

        auto goal_handle_future = action_client_->async_send_goal(goal_msg,
            std::bind(&RobotController::goal_response_callback, this, _1),
            std::bind(&RobotController::feedback_callback, this, _1, _2),
            std::bind(&RobotController::result_callback, this, _1));
    }

    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<pepsi_detection_interface::action::DetectPepsi>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server");
        }
    }

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<pepsi_detection_interface::action::DetectPepsi>::SharedPtr,
        const std::shared_ptr<const pepsi_detection_interface::action::DetectPepsi::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Feedback: x_center = %f", feedback->x_center);
        this->control_robot(feedback->x_center);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<pepsi_detection_interface::action::DetectPepsi>::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Action succeeded");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Action failed");
        }
    }

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        laser_scan_ranges_ = msg->ranges;
        angle_min_ = msg->angle_min;
        angle_max_ = msg->angle_max;
        angle_increment_ = msg->angle_increment;

        // Optional: Process laser scan data if needed
        // Example: find minimum distance
        if (!laser_scan_ranges_.empty()) {
            float min_distance = *std::min_element(laser_scan_ranges_.begin(), laser_scan_ranges_.end());
            RCLCPP_INFO(this->get_logger(), "Minimum distance from LaserScan: %.2f meters", min_distance);
        }
    }

    void control_robot(float x_center)
    {
        auto cmd = geometry_msgs::msg::Twist();
        float center_x = 1920 / 2;
        float rotation_speed = 0.3;
        float min_distance_threshold = 1.0;  // 1 meter threshold for stopping

        // Check if the robot is too close to any obstacle
        if (!laser_scan_ranges_.empty()) {
            float min_distance = *std::min_element(laser_scan_ranges_.begin(), laser_scan_ranges_.end());

            if (min_distance < min_distance_threshold) {
                // Stop the robot if it's too close to an obstacle
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            } else {
                // Rotate towards the Pepsi can
                if (x_center < center_x - 200) {
                    cmd.angular.z = rotation_speed;
                } else if (x_center > center_x + 200) {
                    cmd.angular.z = -rotation_speed;
                } else {
                    cmd.angular.z = 0.0;
                }

                // Move forward if aligned
                cmd.linear.x = 0.5;  // Adjust speed as necessary
            }
        }

        cmd_vel_pub_->publish(cmd);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
