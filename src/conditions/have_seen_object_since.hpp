#include "behaviortree_cpp/condition_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{

    class HaveSeenObjectSince : public BT::ConditionNode
    {
    public:
        HaveSeenObjectSince(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) : 
            BT::ConditionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<uint8_t>("class"),
                     BT::InputPort<uint32_t>("timeout") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<uint8_t> class_id = getInput<uint8_t>("class");
            BT::Expected<uint32_t> timeout = getInput<uint32_t>("timeout");

            if(!class_id)
            {
                throw BT::RuntimeError("missing required input [class_id]: ", class_id.error());
            }

            if(!timeout)
            {
                throw BT::RuntimeError("missing required input [timeout]: ", timeout.error());
            }

            if(node_->detections[class_id.value()].millis_since_seen < timeout.value())
            {
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::FAILURE;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}