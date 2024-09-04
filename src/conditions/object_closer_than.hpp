#include "behaviortree_cpp/condition_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{

    class ObjectCloserThan : public BT::ConditionNode
    {
    public:
        ObjectCloserThan(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) : 
            BT::ConditionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<uint8_t>("class"),
                     BT::InputPort<float>("distance") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<uint8_t> class_id = getInput<uint8_t>("class");
            BT::Expected<float> distance = getInput<float>("distance");

            if(!class_id)
            {
                throw BT::RuntimeError("missing required input [class_id]: ", class_id.error());
            }

            if(!distance)
            {
                throw BT::RuntimeError("missing required input [distance]: ", distance.error());
            }

            if(node_->detections[class_id.value()].distance < distance.value())
            {
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::FAILURE;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}