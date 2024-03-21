#include "behaviortree_cpp/action_node.h"

namespace RSLA
{

    class PrintToLog : public BT::SyncActionNode
    {
    public:
        PrintToLog(const std::string& name, const BT::NodeConfig& config) : 
            BT::SyncActionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("message") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<std::string> msg = getInput<std::string>("message");

            if(!msg)
            {
                throw BT::RuntimeError("missing required input [message]: ", msg.error());
            }

            std::cout << "[LOG]: " << msg.value() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    };

}