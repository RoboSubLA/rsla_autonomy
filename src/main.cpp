#include "behaviortree_cpp/bt_factory.h"

#include "nodes.h"

int main()
{
    // Setup tree factory
    BT::BehaviorTreeFactory factory;

    // Register all nodes
    factory.registerNodeType<RSLA::PrintToLog>("PrintToLog");

    // Create and run tree
    auto tree = factory.createTreeFromFile("./test_tree.xml");

    BT::NodeStatus treeStatus = BT::NodeStatus::RUNNING;

    while(treeStatus == BT::NodeStatus::RUNNING)
    {
        treeStatus = tree.tickOnce();

        // Other stuff here
        // like heartbeat etc
    }

    // Exit
    return 0;
}
