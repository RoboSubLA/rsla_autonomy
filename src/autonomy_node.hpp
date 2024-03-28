#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

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

            // Setup arm echo subscriber
            arm_echo_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
                "rsla/controls/armed_echo",
                1,
                std::bind(&AutonomyNode::arm_echo_callback, this, std::placeholders::_1));

            // Setup trigger subscriber
            trigger_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
                "rsla/autonomy/trigger",
                1,
                std::bind(&AutonomyNode::trigger_callback, this, std::placeholders::_1));

            // Setup hardware kill switch subscriber
            hw_arm_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
                "rsla/controls/hw_armed",   
                1,
                std::bind(&AutonomyNode::hw_armed_callback, this, std::placeholders::_1));

            // Setup commanded pose publisher
            cmd_pose_message = geometry_msgs::msg::Pose();
            cmd_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("rsla/controls/cmdPose", 1);

            // Setup commanded pose echo subscriber
            cmd_pose_echo_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
                "rsla/controls/cmdPoseEcho",
                1,
                std::bind(&AutonomyNode::cmd_pose_echo_callback, this, std::placeholders::_1));

            // Setup commanded twist publisher
            cmd_twist_message = geometry_msgs::msg::Twist();
            cmd_twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("rsla/controls/cmdTwist", 1);

            // Setup commanded twist echo subscriber
            cmd_twist_echo_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "rsla/controls/cmdTwistEcho",
                1,
                std::bind(&AutonomyNode::cmd_twist_echo_callback, this, std::placeholders::_1));
        }

        bool new_trigger_data = false;
        int trigger_value = 0;

        bool new_arm_echo_data = false;
        bool arm_echo_value = false;

        bool new_hw_arm_data = false;
        bool hw_arm_value = false;

        bool new_cmd_pose_echo_data = false;
        geometry_msgs::msg::Pose cmd_pose_echo_value;

        bool new_cmd_twist_echo_data = false;
        geometry_msgs::msg::Twist cmd_twist_echo_value;

        void set_armed(bool flag)
        {
            arm_message.data = flag;
            arm_publisher_->publish(arm_message);
        }

        void set_cmd_pose(geometry_msgs::msg::Pose pose)
        {
            cmd_pose_message = pose;
            cmd_pose_publisher_->publish(cmd_pose_message);
        }

        void set_cmd_pose(geometry_msgs::msg::Point pos, geometry_msgs::msg::Quaternion rot)
        {
            cmd_pose_message.position = pos;
            cmd_pose_message.orientation = rot;
            cmd_pose_publisher_->publish(cmd_pose_message);
        }

        void set_cmd_twist(geometry_msgs::msg::Twist twist)
        {
            cmd_twist_message = twist;
            cmd_twist_publisher_->publish(cmd_twist_message);
        }

        void set_cmd_twist(geometry_msgs::msg::Vector3 linear, geometry_msgs::msg::Vector3 angular)
        {
            cmd_twist_message.linear = linear;
            cmd_twist_message.angular = angular;
            cmd_twist_publisher_->publish(cmd_twist_message);
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

        void hw_armed_callback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            new_hw_arm_data = true;
            hw_arm_value = msg->data;
        }

        void cmd_pose_echo_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            new_cmd_pose_echo_data = true;
            cmd_pose_echo_value = *msg;
            RCLCPP_INFO(this->get_logger(), "Received pose command echo");
        }

        void cmd_twist_echo_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            new_cmd_twist_echo_data = true;
            cmd_twist_echo_value = *msg;
            RCLCPP_INFO(this->get_logger(), "Received twist command echo");
        }

        std_msgs::msg::Empty hb_message;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hb_publisher_;
        rclcpp::TimerBase::SharedPtr hb_timer_;

        std_msgs::msg::Bool arm_message;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_echo_subscription_;

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_subscription_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hw_arm_subscription_;

        geometry_msgs::msg::Pose cmd_pose_message;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_echo_subscription_;

        geometry_msgs::msg::Twist cmd_twist_message;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_twist_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_twist_echo_subscription_;
    };

}