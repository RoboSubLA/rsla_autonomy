#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"
#include "math.h"

#define CAMERA_OFFSET_X 0.33

namespace RSLA
{

    class UpdateMarker : public BT::SyncActionNode
    {
    public:
        UpdateMarker(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::SyncActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("id"),
                     BT::InputPort<uint8_t>("class"),
                     BT::InputPort<uint32_t>("timeout"),
                     BT::InputPort<bool>("recalibrate")};
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<std::string> id = getInput<std::string>("id");
            BT::Expected<uint8_t> class_id = getInput<uint8_t>("class");
            BT::Expected<uint32_t> timeout = getInput<uint32_t>("timeout");
            BT::Expected<bool> recalibrate = getInput<bool>("recalibrate");

            if(!id)
            {
                throw BT::RuntimeError("missing required input [id]: ", id.error());
            }

            if(!class_id)
            {
                throw BT::RuntimeError("missing required input [class]: ", class_id.error());
            }

            if(!timeout)
            {
                throw BT::RuntimeError("missing required input [timeout]: ", timeout.error());
            }

            if(!recalibrate)
            {
                throw BT::RuntimeError("missing required input [recalibrate]: ", recalibrate.error());
            }

            const auto det = node_->downDetections[class_id.value()];
            if (det.millis_since_seen > timeout.value()) return BT::NodeStatus::SUCCESS;

            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Updated marker \"%s\"...", id.value().c_str());

            const auto pos = node_->current_pose;

            const float x_offset = det.distance * sin(M_PI * det.pitch_abs_approx / 180) + CAMERA_OFFSET_X;
            const float y_offset = det.distance * sin(M_PI * det.yaw_abs_approx / 180);

            float x = pos.x + x_offset * cos(M_PI * pos.yaw / 180) - y_offset * sin(M_PI * pos.yaw / 180);
            float y = pos.y + x_offset * sin(M_PI * pos.yaw / 180) + y_offset * cos(M_PI * pos.yaw / 180);

            if(recalibrate.value()){
                const auto marker = node_->markers[id.value()];
                node_->offset = MarkerPosition{marker.x - x, marker.y - y};
            } else {
                node_->markers[id.value()] = MarkerPosition{x, y};
            }
            
            return BT::NodeStatus::SUCCESS;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}
