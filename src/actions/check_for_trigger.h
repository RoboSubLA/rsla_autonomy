#include "behaviortree_cpp/condition_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{

    class CheckForTrigger : public BT::ConditionNode
    {
    public:
        CheckForTrigger(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) : 
            BT::ConditionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<int>("trigger") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<int> trigger = getInput<int>("trigger");

            if(!trigger)
            {
                throw BT::RuntimeError("missing required input [trigger]: ", trigger.error());
            }

            if(node_->new_trigger_data)
            {
                RCLCPP_INFO(node_->get_logger(), "New trigger data");

                node_->new_trigger_data = false;
                if(node_->trigger_value == trigger.value())
                {
                    RCLCPP_INFO(node_->get_logger(), "Trigger match!");

                    return BT::NodeStatus::SUCCESS;
                }
            }

            return BT::NodeStatus::FAILURE;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}