#ifndef DETECT_QR_CODE_HPP_
#define DETECT_QR_CODE_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DetectQrCode : public BT::ActionNodeBase
{
public:
    DetectQrCode(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
    void halt() override;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    bool qr_code_detected_;
};

#endif // DETECT_QR_CODE_HPP_
