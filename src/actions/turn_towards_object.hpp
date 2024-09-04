#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

#include "../conversions.hpp"

namespace RSLA
{

    class TurnTowardsObject : public BT::SyncActionNode
    {
    public:
        TurnTowardsObject(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::SyncActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<uint8_t>("class"),
                     BT::InputPort<float>("fraction") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<uint8_t> class_id = getInput<uint8_t>("class");
            BT::Expected<float> fraction = getInput<float>("fraction");

            if(!class_id)
            {
                throw BT::RuntimeError("missing required input [class_id]: ", class_id.error());
            }

            if(!fraction)
            {
                throw BT::RuntimeError("missing required input [fraction]: ", fraction.error());
            }

            if(!node_->detections[class_id.value()].detected_ever)
            {
                return BT::NodeStatus::SUCCESS;
            }

            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Turning towards object...");

            float turn_alpha = fraction.value();
            float detected_object_yaw = node_->detections[class_id.value()].yaw_abs_approx;
            float new_yaw_command = (detected_object_yaw * turn_alpha) + (node_->current_pose.yaw * (1 - turn_alpha));

            node_->set_cmd_pose(0, 0, 0, 0, 0, new_yaw_command, 32);

            return BT::NodeStatus::SUCCESS;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}