#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
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

        // Create a subscription to the Pepsi coordinates
        pepsi_coord_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "pepsi_coordinates", 10, std::bind(&RobotController::pepsi_coordinates_callback, this, _1));

        // Create a subscription to the Pepsi status
        pepsi_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "pepsi_detected", 10, std::bind(&RobotController::pepsi_status_callback, this, _1));

        // Create a publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pepsi_coord_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pepsi_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // LaserScan variables
    std::vector<float> laser_scan_ranges_;
    float angle_min_;
    float angle_max_;
    float angle_increment_;

    float x_center_;
    bool pepsi_found_;

    void pepsi_coordinates_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        x_center_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received x_center: %f", x_center_);
        control_robot();
    }

    void pepsi_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        pepsi_found_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Pepsi found status: %s", pepsi_found_ ? "true" : "false");
        control_robot();
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

    void control_robot()
{
    auto cmd = geometry_msgs::msg::Twist();
    float center_x = 1920 / 2;  // Adjust based on your camera resolution
    float rotation_speed = 0.3;
    float min_distance_threshold = 0.6;  // 1 meter threshold for stopping

    // Check if the robot is too close to any obstacle
    if (!laser_scan_ranges_.empty()) {
        float min_distance = *std::min_element(laser_scan_ranges_.begin(), laser_scan_ranges_.end());

        if (min_distance < min_distance_threshold) {
            // Stop the robot if it's too close to an obstacle
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        } else {
            // If Pepsi can is found
            if (pepsi_found_) {
                if (x_center_ < center_x - 100) {
                    cmd.angular.z = rotation_speed - 0.2;
                } else if (x_center_ > center_x + 100) {
                    cmd.angular.z = -rotation_speed + 0.2;
                } else {
                    cmd.angular.z = 0.0;
                }

                // Move forward if aligned
                cmd.linear.x = 0.5;  // Adjust speed as necessary
            } else {
                // If Pepsi can is not found, rotate continuously
                cmd.angular.z = rotation_speed;  // Rotate in one direction
                cmd.linear.x = 0.0;  // Stop moving forward
            }
        }
    } else {
        // If no laser scan data available, stop all movement
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
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
