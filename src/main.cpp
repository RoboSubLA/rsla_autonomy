#include "behaviortree_cpp/bt_factory.h"

#include "nodes.h"

#include "autonomy_node.hpp"

// Main
int main(int argc, char **argv)
{
    // Setup ROS2 node
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RSLA::AutonomyNode>("rsla_autonomy");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    // Setup tree factory
    BT::BehaviorTreeFactory factory;

    // Register all nodes
    factory.registerNodeType<RSLA::Ping>("Ping");
    factory.registerNodeType<RSLA::PrintToLog>("PrintToLog");
    factory.registerNodeType<RSLA::CheckForTrigger>("CheckForTrigger", node);
    factory.registerNodeType<RSLA::SetArmedState>("SetArmedState", node);

    // Create and run tree
    auto tree = factory.createTreeFromFile(strcat(getenv("HOME"), "/test_tree.xml"));

    tree.tickWhileRunning();

    // Exit
    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}
