#include "behaviortree_cpp/action_node.h"
#include "../autonomy_node.hpp"

namespace RSLA
{

    class LoadMarker : public BT::SyncActionNode
    {
    public:
        LoadMarker(const std::string& name, const BT::NodeConfig& config,
            std::shared_ptr<RSLA::AutonomyNode> node) :
            BT::SyncActionNode(name, config),
            node_(node)
        {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<std::string>("id"),
                     BT::InputPort<geometry_msgs::msg::Vector3>("offset"),
                     BT::OutputPort<geometry_msgs::msg::Vector3>("coords") };
        }

        BT::NodeStatus tick() override
        {
            BT::Expected<std::string> id = getInput<std::string>("id");
            BT::Expected<geometry_msgs::msg::Vector3> offset = getInput<geometry_msgs::msg::Vector3>("offset");

            if(!id)
            {
                throw BT::RuntimeError("missing required input [id]: ", id.error());
            }

            if(!offset)
            {
                throw BT::RuntimeError("missing required input [offset]: ", offset.error());
            }

            // Actually send the pose message
            RCLCPP_INFO(node_->get_logger(), "Fetched marker \"%s\"...", id.value().c_str());

            geometry_msgs::msg::Vector3 coords = offset.value();
            auto marker = node_->markers[id.value()];
            coords.x += marker.x;
            coords.y += marker.y;

            setOutput("coords", coords);
            
            return BT::NodeStatus::SUCCESS;
        }
    private:
        std::shared_ptr<RSLA::AutonomyNode> node_;
    };

}
