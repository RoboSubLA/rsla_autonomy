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
    factory.registerNodeType<RSLA::UpdateMarker>("UpdateMarker", node);
    factory.registerNodeType<RSLA::LoadMarker>("LoadMarker", node);
    factory.registerNodeType<RSLA::GoToPose>("GoToPose", node);
    factory.registerNodeType<RSLA::DropBall>("DropBall", node);
    factory.registerNodeType<RSLA::GoAtWrench>("GoAtWrench", node);
    factory.registerNodeType<RSLA::HoldPosition>("HoldPosition", node);
    factory.registerNodeType<RSLA::CalibrateSurface>("CalibrateSurface", node);
    factory.registerNodeType<RSLA::WaitForPose>("WaitForPose", node);
    factory.registerNodeType<RSLA::WaitForFrontVision>("WaitForFrontVision", node);
    factory.registerNodeType<RSLA::WaitForDownVision>("WaitForDownVision", node);
    factory.registerNodeType<RSLA::TurnTowardsObject>("TurnTowardsObject", node);
    factory.registerNodeType<RSLA::OrientTowardsSlalom>("OrientTowardsSlalom", node);

    factory.registerNodeType<RSLA::FrontCanSeeObject>("FrontCanSeeObject", node);
    factory.registerNodeType<RSLA::FrontHaveSeenObject>("FrontHaveSeenObject", node);
    factory.registerNodeType<RSLA::FrontHaveSeenObjectSince>("FrontHaveSeenObjectSince", node);
    factory.registerNodeType<RSLA::FrontObjectCloserThan>("FrontObjectCloserThan", node);
    factory.registerNodeType<RSLA::DownCanSeeObject>("DownCanSeeObject", node);
    factory.registerNodeType<RSLA::DownHaveSeenObject>("DownHaveSeenObject", node);
    factory.registerNodeType<RSLA::DownHaveSeenObjectSince>("DownHaveSeenObjectSince", node);
    factory.registerNodeType<RSLA::DownObjectCloserThan>("DownObjectCloserThan", node);
    factory.registerNodeType<RSLA::PoseWithinTolerance>("PoseWithinTolerance", node);

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
