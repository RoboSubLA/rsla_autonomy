#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{

    class SetArmedState : public BT::SyncActionNode
    {
    public:
        SetArmedState(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::SyncActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<bool>("flag") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<bool> flag = getInput<bool>("flag");

            if(!flag)
            {
                throw BT::RuntimeError("missing required input [flag]: ", flag.error());
            }

            // Actually send the armed message
            RCLCPP_INFO(node_->get_logger(), "Sending arm message...");

            node_->set_armed(flag.value());

            return BT::NodeStatus::SUCCESS;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}