// #include "behavior-tree/approach_can.hpp"

// ApproachCan::ApproachCan(const std::string &name, const BT::NodeConfiguration &config)
//     : BT::SyncActionNode(name, config),
//       desired_distance_(6), // default value, will be overridden by parameter
//       current_distance_(std::numeric_limits<double>::infinity())
// {
//     node_ = rclcpp::Node::make_shared("approach_can_node");

//     // Get parameter from config file
//     node_->declare_parameter<double>("desired_distance", 6);
//     node_->get_parameter("desired_distance", desired_distance_);

//     // Subscribe to the ultrasonic sensor's range topic
//     range_subscription_ = node_->create_subscription<example_interfaces::msg::Float32>(
//         "/distanceToObstacle", 10, std::bind(&ApproachCan::rangeCallback, this, std::placeholders::_1));

//     // Publisher for robot velocity commands
//     cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
// }

// BT::PortsList ApproachCan::providedPorts()
// {
//     return {};
// }

// void ApproachCan::rangeCallback(const example_interfaces::msg::Float32::SharedPtr msg)
// {
//     current_distance_ = msg->distance;
// }

// BT::NodeStatus ApproachCan::tick()
// {
//     rclcpp::spin_some(node_); // Process incoming sensor data

//     geometry_msgs::msg::Twist cmd_vel_msg;

//     if (current_distance_ > desired_distance_)
//     {
//         RCLCPP_INFO(node_->get_logger(), "Approaching can, distance: %.2f", current_distance_);
//         cmd_vel_msg.linear.x = 0.1; // Move forward slowly
//         cmd_vel_publisher_->publish(cmd_vel_msg);
//         return BT::NodeStatus::SUCCESS;
//     }
//     else
//     {
//         RCLCPP_INFO(node_->get_logger(), "Can is within the desired distance.");
//         cmd_vel_msg.linear.x = 0.0; // Stop
//         cmd_vel_publisher_->publish(cmd_vel_msg);
//         return BT::NodeStatus::SUCCESS;
//     }
// }

// // Register node with BehaviorTree.CPP
// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<ApproachCan>("ApproachCan");
// }
