#include <cmath>
#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

#include "../conversions.hpp"

namespace RSLA
{

    class AlignAboveObject : public BT::SyncActionNode
    {
    public:
        AlignAboveObject(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::SyncActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<uint8_t>("class"),
                     BT::InputPort<uint8_t>("x_offset"),
                     BT::InputPort<uint8_t>("y_offset"),
                     BT::InputPort<uint8_t>("k_p") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<uint8_t> class_id = getInput<uint8_t>("class");
            BT::Expected<uint8_t> x_offset = getInput<uint8_t>("x_offset");
            BT::Expected<uint8_t> y_offset = getInput<uint8_t>("y_offset");
            BT::Expected<uint8_t> k_p = getInput<uint8_t>("k_p");

            if(!class_id)
            {
                throw BT::RuntimeError("missing required input [class_id]: ", class_id.error());
            }

            if(!x_offset)
            {
                throw BT::RuntimeError("missing required input [x_offset]: ", x_offset.error());
            }

            if(!y_offset)
            {
                throw BT::RuntimeError("missing required input [y_offset]: ", y_offset.error());
            }

            if(!k_p)
            {
                throw BT::RuntimeError("missing required input [k_p]: ", k_p.error());
            }

            if(!node_->frontDetections[class_id.value()].detected_ever)
            {
                return BT::NodeStatus::SUCCESS;
            }

            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Turning towards object...");

            float detected_object_yaw = node_->downDetections[class_id.value()].yaw_abs_approx;
            float detected_object_pitch = node_->downDetections[class_id.value()].pitch_abs_approx;
            float detected_object_distance = node_->downDetections[class_id.value()].distance;
            float detected_x = detected_object_distance * std::sin(detected_object_pitch);
            float detected_y = detected_object_distance * std::sin(detected_object_yaw);

            geometry_msgs::msg::Vector3 force;
            force.x = k_p.value() * (x_offset.value() - detected_x);
            force.y = k_p.value() * (y_offset.value() - detected_y);
            force.z = 0;
            geometry_msgs::msg::Vector3 torque;
            torque.x = 0;
            torque.y = 0;
            torque.z = 0;
            node_->set_cmd_wrench(force, torque, 3);

            return BT::NodeStatus::SUCCESS;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}
