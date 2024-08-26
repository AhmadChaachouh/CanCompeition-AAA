#include "detect_qr_code.hpp"

DetectQrCode::DetectQrCode(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config), qr_code_detected_(false)
{
    node_ = rclcpp::Node::make_shared("detect_qr_code_node");

    // Subscribe to the robot's camera image topic
    image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10, std::bind(&DetectQrCode::imageCallback, this, std::placeholders::_1));

    // Create a publisher for the velocity commands
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

BT::PortsList DetectQrCode::providedPorts()
{
    return {};
}

BT::NodeStatus DetectQrCode::tick()
{
    rclcpp::spin_some(node_);

    if (qr_code_detected_)
    {
        // Approach the QR code
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.2; // Move forward with a fixed speed
        cmd_vel_publisher_->publish(cmd_vel_msg);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

void DetectQrCode::halt()
{
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    cmd_vel_publisher_->publish(stop_msg);
    setStatus(BT::NodeStatus::IDLE);
}

void DetectQrCode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert ROS Image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Placeholder for QR code detection logic
    // Simulate QR code detection
    qr_code_detected_ = false;  // Reset detection flag

    // Assuming you have QR code detection logic here
    // For demonstration purposes, let's say we detected a QR code
    // Example:
    // qr_code_detected_ = your_qr_detection_function(cv_ptr->image);

    if (qr_code_detected_)
    {
        RCLCPP_INFO(node_->get_logger(), "QR code detected");
        // Update the blackboard
        config().blackboard->set("qr_code_detected", true);
    }
    else
    {
        // Reset the blackboard value
        config().blackboard->set("qr_code_detected", false);
    }
}


BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<DetectQrCode>("DetectQrCode");
}
