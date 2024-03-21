#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

#include <chrono>

namespace RSLA
{

    class SetArmedState : public BT::StatefulActionNode
    {
    public:
        SetArmedState(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::StatefulActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<bool>("flag") };
        }

        BT::NodeStatus onStart() override
        {
            BT::Expected<bool> flag = getInput<bool>("flag");

            if(!flag)
            {
                throw BT::RuntimeError("missing required input [flag]: ", flag.error());
            }

            // Actually send the armed message
            RCLCPP_INFO(node_->get_logger(), "Sending arm message...");

            node_->set_armed(flag.value());

            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            BT::Expected<bool> flag = getInput<bool>("flag");

            if(!flag)
            {
                throw BT::RuntimeError("missing required input [flag]: ", flag.error());
            }

            // Check if the arm echo has been received
            if(node_->new_arm_echo_data)
            {
                RCLCPP_INFO(node_->get_logger(), "New arm echo data");
                node_->new_arm_echo_data = false;

                if(node_->arm_echo_value == flag.value())
                {
                    RCLCPP_INFO(node_->get_logger(), "Arm echo match!");

                    // Arm echo received and validated
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "Arm echo invalid!");

                    // Arm echo received but not what we expected
                    return BT::NodeStatus::FAILURE;
                }
            }

            return BT::NodeStatus::RUNNING;
        }

        void onHalted() override
        {
            RCLCPP_INFO(node_->get_logger(), "Arm halted");
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}