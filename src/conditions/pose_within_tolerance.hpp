#include "behaviortree_cpp/condition_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{
    class PoseWithinTolerance : public BT::ConditionNode
    {
    public:
        PoseWithinTolerance(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::ConditionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<geometry_msgs::msg::Vector3>("target"),
                     BT::InputPort<float>("tolerance") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<geometry_msgs::msg::Vector3> target_exp = getInput<geometry_msgs::msg::Vector3>("target");
            if (!target_exp)
            {
                throw BT::RuntimeError("missing required input [target]: ", target_exp.error());
            }
            const geometry_msgs::msg::Vector3& target = target_exp.value();

            BT::Expected<float> tol_exp = getInput<float>("tolerance");
            if (!tol_exp)
            {
                throw BT::RuntimeError("missing required input [tolerance]: ", tol_exp.error());
}
            const float tolerance = tol_exp.value();
            if (tolerance < 0.0f)
            {
                throw BT::RuntimeError("tolerance must be non-negative");
            }

            const PoseEulerData& current_pose = node_->current_pose;
            float dx = current_pose.x - target.x;
            float dy = current_pose.y - target.y;
            float dz = current_pose.z - target.z;
            float distance = std::sqrt(dx*dx + dy*dy + dz*dz);

      	    RCLCPP_INFO(node_->get_logger(), "Distance: %f", distance);

            return (distance <= tolerance) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}

