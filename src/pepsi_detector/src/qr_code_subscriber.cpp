#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        using namespace std::placeholders;

        // Create a subscription to the QR code coordinates
        qr_coord_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "qr_code_coordinates", 10, std::bind(&RobotController::qr_coordinates_callback, this, _1));

        // Create a subscription to the QR code detection status
        qr_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "qr_detected", 10, std::bind(&RobotController::qr_status_callback, this, _1));

        // Create a subscription to LaserScan data
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&RobotController::laser_scan_callback, this, _1));

        // Create a publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Declare parameters with default values
        this->declare_parameter<float>("center_x", 960.0);
        this->declare_parameter<float>("rotation_speed", 0.3);
        this->declare_parameter<float>("min_distance_threshold", 1);

        // Get the parameter values
        this->get_parameter("center_x", center_x);
        this->get_parameter("rotation_speed", rotation_speed);
        this->get_parameter("min_distance_threshold", min_distance_threshold);

        // Initialize variables
        qr_found_ = false;
        x_center_ = 0.0;
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr qr_coord_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr qr_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    float x_center_;
    bool qr_found_;
    std::vector<float> laser_scan_ranges_;

    float center_x;  // Adjust based on your camera resolution
    float rotation_speed;
    float min_distance_threshold;  // Threshold distance for stopping
    float min_distance;
    void qr_coordinates_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        x_center_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received x_center: %f", x_center_);
        control_robot();
    }

    void qr_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        qr_found_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "QR code found status: %s", qr_found_ ? "true" : "false");
        control_robot();
    }

    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        laser_scan_ranges_ = msg->ranges;
        float min_distance = *std::min_element(laser_scan_ranges_.begin(), laser_scan_ranges_.end());
        RCLCPP_INFO(this->get_logger(), "min_distance: %f", min_distance);

        
    }

    void control_robot()
    {
        auto cmd = geometry_msgs::msg::Twist();
        // float center_x = 1920 / 2;  // Adjust based on your camera resolution
        // float rotation_speed = 0.3;
        // float min_distance_threshold = 0.6;  // Threshold distance for stopping

        // Check if the robot is too close to any obstacle
        if (!laser_scan_ranges_.empty()) {
            float min_distance = laser_scan_ranges_[0];

            if (min_distance < min_distance_threshold && qr_found_  == true) {
                // Stop the robot if it's too close to an obstacle
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            } else {
                // If QR code is found
                if (qr_found_) {
                    if (x_center_ < center_x - 50) {
                        // QR code is to the left; rotate left
                        cmd.angular.z = 0.1;
                    } else if (x_center_ > center_x + 50) {
                        // QR code is to the right; rotate right
                        cmd.angular.z = -0.1;
                    } else {
                        // QR code is centered; stop rotating
                        cmd.angular.z = 0.0;
                    }

                    // Move forward if aligned
                    cmd.linear.x = 0.5;  // Adjust speed as necessary
                } else {
                    // If QR code is not found, keep rotating to find it
                    cmd.angular.z = rotation_speed;  // Rotate in one direction
                    cmd.linear.x = 0.0;  // Stop moving forward
                }
            }
        } else {
            // If no laser scan data available, stop all movement
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        // Publish the command to move the robot
        cmd_vel_pub_->publish(cmd);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
