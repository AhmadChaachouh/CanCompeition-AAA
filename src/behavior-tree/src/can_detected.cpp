// #include "behavior-tree/can_detected.hpp"

// CanDetected::CanDetected(const std::string &name, const BT::NodeConfiguration &config)
//     : BT::ConditionNode(name, config)
// {
//     this->can_detected = false;
// }

// BT::PortsList CanDetected::providedPorts()
// {
//     return {BT::InputPort<bool>("is_can_detected")};
// }

// BT::NodeStatus CanDetected::tick()
// {
    
//     if(!getInput<bool>("is_can_detected", this->can_detected)){
//         throw BT::RuntimeError("Missing required input [threshold]");
//     }

//     if(this->can_detected){
//         return BT::NodeStatus::SUCCESS;
//     }else{
//         return BT::NodeStatus::FAILURE;
//     }

// }

// // Register the condition node with BehaviorTree.CPP
// // BT_REGISTER_NODES(factory)
// // {
// //     factory.registerNodeType<CanDetected>("CanDetected");
// // }
