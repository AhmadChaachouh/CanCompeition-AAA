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

        //Create a subscription to LaserScan
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",10, std::bind(&RobotController::laser_scan_callback, this, _1)); //"scan"

        // subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        //     "/distanceToObstacle", 10, 
        //     std::bind(&RobotController::distance_callback, this, std::placeholders::_1));

        // Create a subscription to the Pepsi coordinates
        pepsi_coord_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "pepsi_coordinates", 10, std::bind(&RobotController::pepsi_coordinates_callback, this, _1));

        // Create a subscription to the Pepsi status
        pepsi_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "pepsi_detected", 10, std::bind(&RobotController::pepsi_status_callback, this, _1));

        // Create a publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Declare parameters with default values
        this->declare_parameter<float>("center_x", 960.0);
        this->declare_parameter<float>("rotation_speed", 0.3);
        this->declare_parameter<float>("min_distance_threshold", 0.6);

        // Get the parameter values
        this->get_parameter("center_x", center_x);
        this->get_parameter("rotation_speed", rotation_speed);
        this->get_parameter("min_distance_threshold", min_distance_threshold);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pepsi_coord_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pepsi_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

    float center_x ;
    float rotation_speed ;
    float min_distance_threshold ;
    float distance_to_obstacle = -1;

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
    // void distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    // {
    //     distance_to_obstacle = msg->data;
    //     RCLCPP_INFO(this->get_logger(), "Distance to obstacle: %f meters", distance_to_obstacle);
    // }

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        laser_scan_ranges_ = msg->ranges;
        angle_min_ = msg->angle_min;
        angle_max_ = msg->angle_max;
        angle_increment_ = msg->angle_increment;

        
    }

    void control_robot()
{
    auto cmd = geometry_msgs::msg::Twist();
    
    //distance_to_obstacle != -1)
    // Check if the robot is too close to any obstacle
    if (!laser_scan_ranges_.empty()) {
        //float min_distance = distance_to_obstacle;
        float min_distance = *std::min_element(laser_scan_ranges_.begin(), laser_scan_ranges_.end());

        if (min_distance < min_distance_threshold) {
            // Stop the robot if it's too close to an obstacle
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            if (x_center_ < center_x - 40) {
                    cmd.angular.z = 0.01;
                } else if (x_center_ > center_x + 40) {
                    cmd.angular.z = -0.01;
                } else {
                    cmd.angular.z = 0.0;
            }
        } else {
            // If Pepsi can is found
            if (pepsi_found_) {
                if (x_center_ < center_x - 50) {
                    cmd.angular.z = 0.1;
                } else if (x_center_ > center_x + 50) {
                    cmd.angular.z = -0.1;
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
