#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

#include "../conversions.hpp"

namespace RSLA
{

    class GoAtTwist : public BT::StatefulActionNode
    {
    public:
        GoAtTwist(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::StatefulActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<geometry_msgs::msg::Vector3>("linear"),
                     BT::InputPort<geometry_msgs::msg::Vector3>("angular") };
        }

        BT::NodeStatus onStart() override
        {
            BT::Expected<geometry_msgs::msg::Vector3> linear = getInput<geometry_msgs::msg::Vector3>("linear");
            BT::Expected<geometry_msgs::msg::Vector3> angular = getInput<geometry_msgs::msg::Vector3>("angular");

            if(!linear)
            {
                throw BT::RuntimeError("missing required input [linear]: ", linear.error());
            }

            if(!angular)
            {
                throw BT::RuntimeError("missing required input [angular]: ", angular.error());
            }

            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Sending twist command message...");

            node_->set_cmd_twist(linear.value(), angular.value());

            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            BT::Expected<geometry_msgs::msg::Vector3> linear = getInput<geometry_msgs::msg::Vector3>("position");
            BT::Expected<geometry_msgs::msg::Vector3> angular = getInput<geometry_msgs::msg::Vector3>("eulers");

            if(!linear)
            {
                throw BT::RuntimeError("missing required input [linear]: ", linear.error());
            }

            if(!angular)
            {
                throw BT::RuntimeError("missing required input [angular]: ", angular.error());
            }

            // Check if the commanded pose echo has been received
            if(node_->new_cmd_twist_echo_data)
            {
                RCLCPP_INFO(node_->get_logger(), "New commanded twist echo data");
                node_->new_cmd_twist_echo_data = false;

                if(node_->cmd_twist_echo_value.linear == linear.value() &&
                   node_->cmd_twist_echo_value.angular == angular.value())
                {
                    RCLCPP_INFO(node_->get_logger(), "Commanded twist echo match!");

                    // Arm echo received and validated
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "Commanded twist echo invalid!");

                    // Arm echo received but not what we expected
                    return BT::NodeStatus::FAILURE;
                }
            }

            return BT::NodeStatus::RUNNING;
        }

        void onHalted() override
        {
            RCLCPP_INFO(node_->get_logger(), "Twist command halted");
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}