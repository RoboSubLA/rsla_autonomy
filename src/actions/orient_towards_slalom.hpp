#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

#include "../conversions.hpp"

namespace RSLA
{

    class OrientTowardsSlalom : public BT::SyncActionNode
    {
    public:
        OrientTowardsSlalom(const std::string& name, const BT::NodeConfig& config,
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
            BT::Expected<uint8_t> white_id = getInput<uint8_t>("white_id");
            BT::Expected<uint8_t> red_id = getInput<uint8_t>("red_id");
            // gate types
            //  0 => white to the left of red
            //  1 => white to the right of red
            //  2 => either is accepted
            BT::Expected<uint8_t> gate_type = getInput<uint8_t>("gate_type");
            // amount the robot shifts its position when its not looking at the right side of slalom
            BT::Expected<float> shift_amount = getInput<float>("shift_amount");
            BT::Expected<float> fraction = getInput<float>("fraction");

            if(!white_id)
            {
                throw BT::RuntimeError("missing required input [white_id]: ", white_id.error());
            }

            if(!red_id)
            {
                throw BT::RuntimeError("missing required input [red_id]: ", red_id.error());
            }

            if(!fraction)
            {
                throw BT::RuntimeError("missing required input [fraction]: ", fraction.error());
            }

            if(!node_->frontDetections[white_id.value()].detected_ever || !node_->frontDetections[red_id.value()].detected_ever)
            {
                return BT::NodeStatus::SUCCESS;
            }

            // visionActually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Turning towards object...");

            float turn_alpha = fraction.value();
            float white_yaw = node_->frontDetections[white_id.value()].yaw_abs_approx;
            float red_yaw = node_->frontDetections[red_id.value()].yaw_abs_approx;

            if(gate_type.value() < 2 && gate_type.value() == white_yaw > red_yaw)
            {
                node_->set_cmd_pose(0, node_->current_pose.y + (2*gate_type.value() - 1)*shift_amount, 0, 0, 0, 0, 2);
                return BT::NodeStatus::SUCCESS;
            }

            float slalom_yaw = (white_yaw + red_yaw) / 2;
            float new_yaw_command = (slalom_yaw * turn_alpha) + (node_->current_pose.yaw * (1 - turn_alpha));

            node_->set_cmd_pose(0, 0, 0, 0, 0, new_yaw_command, 32);

            return BT::NodeStatus::SUCCESS;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}