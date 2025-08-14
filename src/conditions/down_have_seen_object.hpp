#include "behaviortree_cpp/condition_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{

    class DownHaveSeenObject : public BT::ConditionNode
    {
    public:
        DownHaveSeenObject(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) : 
            BT::ConditionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<uint8_t>("class") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<uint8_t> class_id = getInput<uint8_t>("class");

            if(!class_id)
            {
                throw BT::RuntimeError("missing required input [class_id]: ", class_id.error());
            }

            if(node_->downDetections[class_id.value()].detected_ever)
            {
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::FAILURE;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}