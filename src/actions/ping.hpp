#include "behaviortree_cpp/action_node.h"

namespace RSLA
{

    class Ping : public BT::SyncActionNode
    {
    public:
        Ping(const std::string& name) : 
            BT::SyncActionNode(name, {})
        {}

        BT::NodeStatus tick() override
        {
            std::cout << "[LOG]: " << "Ping!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    };

}