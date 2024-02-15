#include "behaviortree_cpp/action_node.h"

namespace RSLA
{

    class PrintToLog : public BT::SyncActionNode
    {
    public:
        PrintToLog(const std::string& name) : 
            BT::SyncActionNode(name, {})
        {}

        BT::NodeStatus tick() override
        {
            std::cout << "[LOG]: " << "TODO" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    };

}