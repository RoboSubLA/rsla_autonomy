#include "behaviortree_cpp/condition_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{

    class CheckForHWArm : public BT::ConditionNode
    {
    public:
        CheckForHWArm(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) : 
            BT::ConditionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return {};
        }

        BT::NodeStatus tick() override
        {

            if(node_->new_hw_arm_data)
            {
                RCLCPP_INFO(node_->get_logger(), "New hw arm data");

                node_->new_hw_arm_data = false;
                if(node_->hw_arm_value == 1)
                {
                    RCLCPP_INFO(node_->get_logger(), "Hardware armed!");

                    return BT::NodeStatus::SUCCESS;
                }

                return BT::NodeStatus::FAILURE;
            }

            return BT::NodeStatus::RUNNING;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}