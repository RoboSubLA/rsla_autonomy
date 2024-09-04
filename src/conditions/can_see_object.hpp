#include "behaviortree_cpp/condition_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{

    class CanSeeObject : public BT::ConditionNode
    {
    public:
        CanSeeObject(const std::string& name, const BT::NodeConfig& config,
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

            if(node_->detections[class_id.value()].detected_now)
            {
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::FAILURE;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}