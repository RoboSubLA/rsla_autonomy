#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

#include "../conversions.hpp"

namespace RSLA
{

    class DropBall : public BT::SyncActionNode
    {
    public:
        DropBall(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::SyncActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<int8_t>("data") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<int8_t> data = getInput<int8_t>("data");

            if(!data)
            {
                throw BT::RuntimeError("missing required data [int8_t]: ", data.error());
            }

            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Sending drop ball command message...");

            node_->set_cmd_ball_drop(data.value());

            return BT::NodeStatus::SUCCESS;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}