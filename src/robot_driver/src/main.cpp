#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/xml_parsing.h>
#include "./robot_driver/detect_can_node.cpp"
#include "./robot_driver/can_detected.cpp"
#include "./robot_driver/approach_can.cpp"
#include "./robot_driver/can_approached.cpp"
#include "./robot_driver/grip_can.cpp"
#include "./robot_driver/can_gripped.cpp"
#include "./robot_driver/detect_qr_code.cpp"
#include "./robot_driver/qr_code_detected.cpp"

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a ROS2 node
    auto node = rclcpp::Node::make_shared("bt_main");

    // Create a BehaviorTreeFactory
    BT::BehaviorTreeFactory factory;

    // Register custom nodes
    factory.registerNodeType<DetectCan>("DetectCan");
    factory.registerNodeType<CanDetected>("CanDetected");
    // factory.registerNodeType<ApproachCan>("ApproachCan");
    // factory.registerNodeType<CanApproached>("CanApproached");
    // factory.registerNodeType<GripCan>("GripCan");
    // factory.registerNodeType<CanGripped>("CanGripped");
    // factory.registerNodeType<DetectQrCode>("DetectQrCode");
    // factory.registerNodeType<QrCodeDetected>("QrCodeDetected");

    // Load the behavior tree from XML file
    auto tree = factory.createTreeFromFile("./bt_tree.xml");

    // Create a ROS2 executor to spin the node
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Execute the behavior tree in a loop
    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        tree.tickRoot();
        executor.spin_some();  // Process ROS2 callbacks
        rate.sleep();          // Sleep to control the loop rate
    }

    // Shutdown ROS2
    rclcpp::shutdown();

    return 0;
}
