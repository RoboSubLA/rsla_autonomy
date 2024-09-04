#include <fstream>

#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/bt_factory.h"

#include "conversions.hpp"
#include "nodes.hpp"

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
    factory.registerNodeType<RSLA::PrintToLog>("PrintToLog");
    factory.registerNodeType<RSLA::SetArmedState>("SetArmedState", node);
    factory.registerNodeType<RSLA::GoToPose>("GoToPose", node);
    factory.registerNodeType<RSLA::GoAtWrench>("GoAtWrench", node);
    factory.registerNodeType<RSLA::HoldPosition>("HoldPosition", node);
    factory.registerNodeType<RSLA::CalibrateSurface>("CalibrateSurface", node);
    factory.registerNodeType<RSLA::WaitForPose>("WaitForPose", node);
    factory.registerNodeType<RSLA::WaitForVision>("WaitForVision", node);
    factory.registerNodeType<RSLA::TurnTowardsObject>("TurnTowardsObject", node);

    factory.registerNodeType<RSLA::CanSeeObject>("CanSeeObject", node);
    factory.registerNodeType<RSLA::HaveSeenObject>("HaveSeenObject", node);
    factory.registerNodeType<RSLA::HaveSeenObjectSince>("HaveSeenObjectSince", node);
    factory.registerNodeType<RSLA::ObjectCloserThan>("ObjectCloserThan", node);

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

    tree.tickOnce();
    while(tree.rootNode()->status() == BT::NodeStatus::RUNNING && rclcpp::ok())
    {
        tree.tickOnce();
        tree.sleep(std::chrono::milliseconds(50));
    }

    // Exit
    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}
