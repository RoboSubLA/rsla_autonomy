#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

#include "../conversions.hpp"

namespace RSLA
{

    class GoToPose : public BT::StatefulActionNode
    {
    public:
        GoToPose(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::StatefulActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<geometry_msgs::msg::Vector3>("position"),
                     BT::InputPort<geometry_msgs::msg::Vector3>("eulers") };
        }

        BT::NodeStatus onStart() override
        {
            BT::Expected<geometry_msgs::msg::Vector3> pos = getInput<geometry_msgs::msg::Vector3>("position");
            BT::Expected<geometry_msgs::msg::Vector3> euler = getInput<geometry_msgs::msg::Vector3>("eulers");

            if(!pos)
            {
                throw BT::RuntimeError("missing required input [pos]: ", pos.error());
            }

            if(!euler)
            {
                throw BT::RuntimeError("missing required input [euler]: ", euler.error());
            }

            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Sending pose command message...");

            node_->set_cmd_pose(RSLA::convertToPoint(pos.value()), RSLA::convertFromEulerAngles(euler.value()));

            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            BT::Expected<geometry_msgs::msg::Vector3> pos = getInput<geometry_msgs::msg::Vector3>("position");
            BT::Expected<geometry_msgs::msg::Vector3> euler = getInput<geometry_msgs::msg::Vector3>("eulers");

            if(!pos)
            {
                throw BT::RuntimeError("missing required input [pos]: ", pos.error());
            }

            if(!euler)
            {
                throw BT::RuntimeError("missing required input [euler]: ", euler.error());
            }

            // Check if the commanded pose echo has been received
            if(node_->new_cmd_pose_echo_data)
            {
                RCLCPP_INFO(node_->get_logger(), "New commanded pose echo data");
                node_->new_cmd_pose_echo_data = false;

                if(node_->cmd_pose_echo_value.position == RSLA::convertToPoint(pos.value()) &&
                   node_->cmd_pose_echo_value.orientation == RSLA::convertFromEulerAngles(euler.value()))
                {
                    RCLCPP_INFO(node_->get_logger(), "Commanded pose echo match!");

                    // Arm echo received and validated
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "Commanded pose echo invalid!");

                    // Arm echo received but not what we expected
                    return BT::NodeStatus::FAILURE;
                }
            }

            return BT::NodeStatus::RUNNING;
        }

        void onHalted() override
        {
            RCLCPP_INFO(node_->get_logger(), "Pose command halted");
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}