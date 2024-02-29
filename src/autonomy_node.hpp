#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

namespace RSLA
{

    // ROS Node definition
    class AutonomyNode : public rclcpp::Node
    {
    public:
        AutonomyNode(const char* name) : Node(name)
        {
            // Setup heartbeat
            hb_message = std_msgs::msg::Empty();
            hb_publisher_ = this->create_publisher<std_msgs::msg::Empty>("rsla/autonomy/heartbeat", 1);

            hb_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&AutonomyNode::hb_callback, this));

            // Setup arm publisher
            arm_message = std_msgs::msg::Bool();
            arm_publisher_ = this->create_publisher<std_msgs::msg::Bool>("rsla/autonomy/armed", 1);

            // Setup trigger subscriber
            trigger_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
                "rsla/autonomy/trigger",
                1,
                std::bind(&AutonomyNode::trigger_callback, this, std::placeholders::_1));

            // Setup arm echo subscriber
            arm_echo_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
                "rsla/controls/armed_echo",
                1,
                std::bind(&AutonomyNode::arm_echo_callback, this, std::placeholders::_1));
        }

        bool new_trigger_data = false;
        int trigger_value = 0;

        bool new_arm_echo_data = false;
        bool arm_echo_value = false;

        void set_armed(bool flag)
        {
            arm_message.data = flag;
            arm_publisher_->publish(arm_message);
        }
    private:
        void hb_callback()
        {
            RCLCPP_INFO(this->get_logger(), "Heartbeat!");
            hb_publisher_->publish(hb_message);
        }

        void trigger_callback(const std_msgs::msg::Int32::SharedPtr msg)
        {
            new_trigger_data = true;
            trigger_value = msg->data;
            RCLCPP_INFO(this->get_logger(), "Received trigger: %u", msg->data);
        }

        void arm_echo_callback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            new_arm_echo_data = true;
            arm_echo_value = msg->data;
            RCLCPP_INFO(this->get_logger(), "Received armed echo: %u", msg->data);
        }

        std_msgs::msg::Empty hb_message;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hb_publisher_;
        rclcpp::TimerBase::SharedPtr hb_timer_;

        std_msgs::msg::Bool arm_message;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_publisher_;

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_subscription_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_echo_subscription_;
    };

}