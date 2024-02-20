#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "nodes.h"

// ROS Node definition
class AutonomyNode : public rclcpp::Node
{
public:
    AutonomyNode(const char* name) : Node(name)
    {
        hb_message = std_msgs::msg::Empty();
        hb_publisher_ = this->create_publisher<std_msgs::msg::Empty>("heartbeat", 10);

        hb_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&AutonomyNode::hbCallback, this));
    }

private:
    void hbCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Heartbeat!");
        hb_publisher_->publish(hb_message);
    }

    std_msgs::msg::Empty hb_message;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hb_publisher_;
    rclcpp::TimerBase::SharedPtr hb_timer_;
};

// Executor
int main(int argc, char **argv)
{
    // Setup ROS2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomyNode>("rsla_autonomy");

    // Setup tree factory
    BT::BehaviorTreeFactory factory;

    // Register all nodes
    factory.registerNodeType<RSLA::Ping>("Ping");
    factory.registerNodeType<RSLA::PrintToLog>("PrintToLog");

    // Create and run tree
    auto tree = factory.createTreeFromFile("/home/pererry/rsla/test_tree.xml");

    BT::NodeStatus treeStatus = BT::NodeStatus::RUNNING;

    while(treeStatus == BT::NodeStatus::RUNNING)
    {
        treeStatus = tree.tickOnce();

        // Other stuff here
        // like heartbeat etc
        rclcpp::spin_some(node);
    }

    // Exit
    rclcpp::shutdown();
    return 0;
}
