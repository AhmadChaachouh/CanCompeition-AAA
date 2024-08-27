// #include "behavior-tree/qr_code_detected.hpp"

// QrCodeDetected::QrCodeDetected(const std::string &name, const BT::NodeConfiguration &config)
//     : BT::ConditionNode(name, config)
// {
// }

// BT::PortsList QrCodeDetected::providedPorts()
// {
//     return {};
// }

// BT::NodeStatus QrCodeDetected::tick()
// {
//     bool qr_code_detected = false;

//     // Check the blackboard for the qr_code_detected key
//     if (config().blackboard->get("qr_code_detected", qr_code_detected))
//     {
//         if (qr_code_detected)
//         {
//             return BT::NodeStatus::SUCCESS;
//         }
//         else
//         {
//             return BT::NodeStatus::FAILURE;
//         }
//     }
//     else
//     {
//         // Blackboard key not found
//         return BT::NodeStatus::FAILURE;
//     }
// }

// // Register the condition node with BehaviorTree.CPP
// // BT_REGISTER_NODES(factory)
// // {
// //     factory.registerNodeType<QrCodeDetected>("QrCodeDetected");
// // }
