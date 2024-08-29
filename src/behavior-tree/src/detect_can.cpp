#include "../include/behavior-tree/detect_can.hpp"

DetectCan::DetectCan(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("detect_can");

    tree_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
         "can_status", 10, std::bind(&DetectCan::treeCallback, this, std::placeholders::_1));

    //Subscribe to the camera image topic
    //  image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
    //      "/camera/image_raw", 10, std::bind(&DetectCan::imageCallback, this, std::placeholders::_1));

    //  Publisher for robot velocity commands
    // cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    can_detected = false;
}

void DetectCan::treeCallback(const std_msgs::msg::Bool::SharedPtr msg){
    bool ccan_detected = msg->data;
    can_detected = ccan_detected;
        
}

BT::PortsList DetectCan::providedPorts()
{
    return {};
}

// void DetectCan::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
// {
//     cv_bridge::CvImagePtr cv_ptr;

//     try
//     {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//         RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
//         return;
//     }

//     // Simulate can detection (replace with actual object detection)
//     this->can_detected = simulateCanDetection(cv_ptr->image);

//     // Publish velocity command to approach the can if detected
//     geometry_msgs::msg::Twist cmd_vel_msg;
//     if (can_detected)
//     {
//         RCLCPP_INFO(node_->get_logger(), "Can detected! Approaching...");
//         cmd_vel_msg.linear.x = 0.2; // Move forward
//     }
//     else
//     {
//         cmd_vel_msg.linear.x = 0.0; // Stop
//     }
//     cmd_vel_publisher_->publish(cmd_vel_msg);
// }

BT::NodeStatus DetectCan::tick()
{
    std::string package_name, launch_file;
    package_name = "pepsi_detector";
    launch_file = "pepsi_detection_launch.py";

    // Construct the ROS2 launch command
    std::string command = "ros2 launch " + package_name + " " + launch_file;

    RCLCPP_INFO(node_->get_logger(), "Launching file: ...");
    // std::cout << "Launching file: " << command << std::endl;

    // Launch the file using system call
   int result = std::system(command.c_str());

    RCLCPP_INFO(node_->get_logger(), "After launch......................................");

    // Check if the launch was successful
    // if (result != 0)
    // {
    //     RCLCPP_INFO(node_->get_logger(), "Failed to launch file: ...");
    //     // std::cerr << "Failed to launch file: " << command << std::endl;
    //     return BT::NodeStatus::FAILURE;
    // }

    // if (result == 0){
    //     return BT::NodeStatus::SUCCESS;
    // }

    return BT::NodeStatus::SUCCESS;
    // while(!can_detected){
    //     RCLCPP_INFO(node_->get_logger(), "Waiting for can detection...");
    //     continue;
    // }
    
    // if (can_detected){
    //     return BT::NodeStatus::SUCCESS;
    // }
    // else{
    //     return BT::NodeStatus::FAILURE;
    // }

    // // Spin the ROS node to handle image processing
    // rclcpp::spin_some(node_);
    // RCLCPP_INFO(node_->get_logger(), "Can detected by detector");
    // BT::TreeNode::setOutput("is_can_detected", this->can_detected);
}

// bool DetectCan::simulateCanDetection(const cv::Mat &image)
// {
//     // Here, you would normally process the image to detect the can.
//     // For this example, let's assume the can is always detected.
//     return true; // Simulate can detection success
// }

// Register node with BehaviorTree.CPP
// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<DetectCan>("DetectCan");
// }

// int main(int argc, char **argv)
// {
//     // Initialize ROS 2
//     rclcpp::init(argc, argv);

//     // Create behavior tree factory
//     BT::BehaviorTreeFactory factory;

//     // Register custom nodes
//     factory.registerNodeType<DetectCan>("DetectCan");

//     // Define the tree from XML or script
//     auto tree = factory.createTreeFromText(R"(
//         <root main_tree_to_execute="MainTree">
//             <BehaviorTree ID="MainTree">
//                 <DetectCan/>
//             </BehaviorTree>
//         </root>
//     )");

//     // Run the behavior tree and ROS2 node
//     auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
//     executor->add_node(factory.registeredNodes()["DetectCan"]->node());

//     while (rclcpp::ok())
//     {
//         tree.tickRoot();
//         executor->spin_some();
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }

//     rclcpp::shutdown();
//     return 0;
// }
