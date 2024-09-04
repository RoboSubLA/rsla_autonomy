#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

#include "../conversions.hpp"

namespace RSLA
{

    class GoAtWrench : public BT::SyncActionNode
    {
    public:
        GoAtWrench(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::SyncActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<geometry_msgs::msg::Vector3>("force"),
                     BT::InputPort<geometry_msgs::msg::Vector3>("torque"),
                     BT::InputPort<uint8_t>("mask") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<geometry_msgs::msg::Vector3> force = getInput<geometry_msgs::msg::Vector3>("force");
            BT::Expected<geometry_msgs::msg::Vector3> torque = getInput<geometry_msgs::msg::Vector3>("torque");
            BT::Expected<uint8_t> mask = getInput<uint8_t>("mask");

            if(!force)
            {
                throw BT::RuntimeError("missing required input [linear]: ", force.error());
            }

            if(!torque)
            {
                throw BT::RuntimeError("missing required input [euler]: ", torque.error());
            }

            if(!mask)
            {
                throw BT::RuntimeError("missing required input [mask]: ", mask.error());
            }

            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Sending pose command message...");

            node_->set_cmd_wrench(force.value(), torque.value(), mask.value());

            return BT::NodeStatus::SUCCESS;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}