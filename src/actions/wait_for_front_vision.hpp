#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{

    class WaitForFrontVision : public BT::StatefulActionNode
    {
    public:
        WaitForFrontVision(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::StatefulActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return {};
        }

        BT::NodeStatus onStart() override
        {
            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Waiting for vision message...");

            node_->new_front_vision_data = false;

            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            if(node_->new_front_vision_data)
            {
                return BT::NodeStatus::SUCCESS;
            }

            return BT::NodeStatus::RUNNING;
        }

        void onHalted() override
        {
            RCLCPP_INFO(node_->get_logger(), "WaitForFrontVision halted");
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}