#ifndef DETECT_CAN_HPP_
#define DETECT_CAN_HPP_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/bool.hpp"
#include <cstdlib>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/opencv.hpp>

class DetectCan : public BT::SyncActionNode
{
public:
    DetectCan(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    // void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // bool simulateCanDetection(const cv::Mat &image);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tree_sub_;
    std::string input_msg_;
    bool can_detected;
    BT::NodeStatus treeCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

#endif // DETECT_CAN_HPP_
