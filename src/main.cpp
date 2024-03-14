#include <fstream>

#include "behaviortree_cpp/xml_parsing.h"
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
    factory.registerNodeType<RSLA::CheckForHWArm>("CheckForHWArm", node);

    // Get home directory
    std::string home = getenv("HOME");

    // Write node schema to file, create if it doesn't exist
    std::string nodesModel = BT::writeTreeNodesModelXML(factory);
    std::ofstream file;
    file.open(home + "/rsla_autonomy_nodes_model.xml");
    file << nodesModel;
    file.close();

    // Create and run tree
    auto tree = factory.createTreeFromFile(home + "/rsla_autonomy_tree.xml");

    tree.tickWhileRunning();

    // Exit
    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}
