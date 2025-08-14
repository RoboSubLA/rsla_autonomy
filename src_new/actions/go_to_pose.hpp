#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

#include "../conversions.hpp"

namespace RSLA
{

    class GoToPose : public BT::SyncActionNode
    {
    public:
        GoToPose(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::SyncActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<geometry_msgs::msg::Vector3>("position"),
                     BT::InputPort<geometry_msgs::msg::Vector3>("eulers"),
                     BT::InputPort<uint8_t>("mask"),
                     BT::InputPort<bool>("rel") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<geometry_msgs::msg::Vector3> pos = getInput<geometry_msgs::msg::Vector3>("position");
            BT::Expected<geometry_msgs::msg::Vector3> euler = getInput<geometry_msgs::msg::Vector3>("eulers");
            BT::Expected<uint8_t> mask = getInput<uint8_t>("mask");
            BT::Expected<bool> rel = getInput<bool>("rel");

            if(!pos)
            {
                throw BT::RuntimeError("missing required input [pos]: ", pos.error());
            }

            if(!euler)
            {
                throw BT::RuntimeError("missing required input [euler]: ", euler.error());
            }

            if(!mask)
            {
                throw BT::RuntimeError("missing required input [mask]: ", mask.error());
            }

            if(!rel)
            {
                throw BT::RuntimeError("missing required input [rel]: ", rel.error());
            }

            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Sending pose command message...");

            if(rel.value())
            {
                node_->set_cmd_pose_rel(RSLA::convertToPoint(pos.value()), RSLA::convertToEuler(euler.value()), mask.value());
            }
            else
            {
                node_->set_cmd_pose(RSLA::convertToPoint(pos.value()), RSLA::convertToEuler(euler.value()), mask.value());
            }

            return BT::NodeStatus::SUCCESS;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}