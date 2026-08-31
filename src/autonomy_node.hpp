#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rsla_interfaces/msg/euler_angles.hpp"
#include "rsla_interfaces/msg/pose_euler.hpp"
#include "rsla_interfaces/msg/pose_with_mask.hpp"
#include "rsla_interfaces/msg/wrench_with_mask.hpp"
#include "rsla_interfaces/msg/detection.hpp"
#include "rsla_interfaces/msg/detection_array.hpp"

namespace RSLA
{

    constexpr uint8_t NUM_CLASSES = 8;

    struct PoseEulerData
    {
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;
    };

    struct DetectionData
    {
        uint8_t class_id = 0;
        float confidence = 0.0;

        bool detected_now = false;
        bool detected_ever = false;
        uint32_t millis_since_seen = 0;

        float yaw_abs_approx = 0.0;
        float pitch_abs_approx = 0.0;

        float distance = 0.0;
    };

    struct MarkerPosition
    {
        float x;
        float y;
    };

    // ROS Node definition
    class AutonomyNode : public rclcpp::Node
    {
    public:
        AutonomyNode(const char* name) : Node(name)
        {
            std::string home = getenv("HOME");
            this->load_csv(home + "/markers.csv");

            // Setup heartbeat
            hb_message = std_msgs::msg::Empty();
            hb_publisher_ = this->create_publisher<std_msgs::msg::Empty>("rsla/autonomy/heartbeat", 1);

            hb_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&AutonomyNode::hb_callback, this));

            // Setup arm publisher
            arm_message = std_msgs::msg::Bool();
            arm_publisher_ = this->create_publisher<std_msgs::msg::Bool>("rsla/controls/sw_arm", 1);

            // Setup commanded pose publisher
            cmd_pose_message = rsla_interfaces::msg::PoseWithMask();
            cmd_pose_publisher_ = this->create_publisher<rsla_interfaces::msg::PoseWithMask>("rsla/autonomy/pose_setpoint_with_mask", 1);

            // Setup commanded pose publisher
            cmd_wrench_message = rsla_interfaces::msg::WrenchWithMask();
            cmd_wrench_publisher_ = this->create_publisher<rsla_interfaces::msg::WrenchWithMask>("rsla/autonomy/wrench_setpoint_with_mask", 1);

            // Setup commanded ball drop publisher
            cmd_ball_drop_message = std_msgs::msg::Int8();
            cmd_ball_drop_publisher_ = this->create_publisher<std_msgs::msg::Int8>("rsla/controls/ball_dropper", 1);

            // Setup diagnostic command publisher
            diagnostic_command_message = std_msgs::msg::Int8();
            diagnostic_command_publisher_ = this->create_publisher<std_msgs::msg::Int8>("rsla/controls/diagnostic_command", 1);

            // Setup vehicle pose subscriber    
            vehicle_pose_subscription_ = this->create_subscription<rsla_interfaces::msg::PoseEuler>("rsla/controls/pose_euler", 1, std::bind(&AutonomyNode::vehicle_pose_callback, this, std::placeholders::_1));

            // Setup computer vision data subscribers
            vision_front_detections_message_subscription_ = this->create_subscription<rsla_interfaces::msg::DetectionArray>("rsla/vision/front_detections", 1, std::bind(&AutonomyNode::vision_front_detections_callback, this, std::placeholders::_1));
            vision_down_detections_message_subscription_ = this->create_subscription<rsla_interfaces::msg::DetectionArray>("rsla/vision/down_detections", 1, std::bind(&AutonomyNode::vision_down_detections_callback, this, std::placeholders::_1));
        }

        void load_csv(const std::string &path) {
          std::ifstream file(path);
          std::string line;
     
          while (std::getline(file, line)) {
            if (line.empty()) continue;
     
            std::stringstream ss(line);
            std::string name, xs, ys;
     
            if (!std::getline(ss, name, ',')) continue;
            if (!std::getline(ss, xs, ',')) continue;
            if (!std::getline(ss, ys, ',')) continue;
     
	    RCLCPP_INFO(this->get_logger(), "Loaded Marker \"%s\" (%s, %s)", name.c_str(), xs.c_str(), ys.c_str());
            markers[name] = MarkerPosition{std::stof(xs), std::stof(ys)};
          }
        }
     
        void set_armed(bool flag)
        {
            arm_message.data = flag;
            arm_publisher_->publish(arm_message);
        }

        void set_cmd_pose(rsla_interfaces::msg::PoseEuler pose, uint8_t mask)
        {
            cmd_pose_message.cmd = pose;
            cmd_pose_message.mask = mask;
            cmd_pose_publisher_->publish(cmd_pose_message);
        }

        void set_cmd_pose(geometry_msgs::msg::Point position, rsla_interfaces::msg::EulerAngles orientation, uint8_t mask)
        {
            cmd_pose_message.cmd.position = position;
            cmd_pose_message.cmd.position.x -= offset.x;
            cmd_pose_message.cmd.position.y -= offset.y;
            cmd_pose_message.cmd.orientation = orientation;
            cmd_pose_message.mask = mask;
            cmd_pose_publisher_->publish(cmd_pose_message);
        }

        void set_cmd_pose_rel(geometry_msgs::msg::Point position, rsla_interfaces::msg::EulerAngles orientation, uint8_t mask)
        {
            cmd_pose_message.cmd.position.x += position.x;
            cmd_pose_message.cmd.position.y += position.y;
            cmd_pose_message.cmd.position.z += position.z;

            cmd_pose_message.cmd.orientation.yaw += orientation.yaw;
            cmd_pose_message.cmd.orientation.yaw = fmodf(cmd_pose_message.cmd.orientation.yaw + 360, 360);

            cmd_pose_message.cmd.orientation.pitch += orientation.pitch;
            cmd_pose_message.cmd.orientation.roll += orientation.roll;

            cmd_pose_message.mask = mask;
            cmd_pose_publisher_->publish(cmd_pose_message);
        }

        void set_cmd_pose(float x, float y, float z, float roll, float pitch, float yaw, uint8_t mask)
        {
            cmd_pose_message.cmd.position.x = x;
            cmd_pose_message.cmd.position.y = y;
            cmd_pose_message.cmd.position.z = z;
            cmd_pose_message.cmd.orientation.roll = roll;
            cmd_pose_message.cmd.orientation.pitch = pitch;
            cmd_pose_message.cmd.orientation.yaw = yaw;
            cmd_pose_message.mask = mask;
            cmd_pose_publisher_->publish(cmd_pose_message);
        }

        void set_cmd_wrench(geometry_msgs::msg::Wrench wrench, uint8_t mask)
        {
            cmd_wrench_message.cmd = wrench;
            cmd_wrench_message.mask = mask;
            cmd_wrench_publisher_->publish(cmd_wrench_message);
        }

        void set_cmd_wrench(geometry_msgs::msg::Vector3 force, geometry_msgs::msg::Vector3 angular, uint8_t mask)
        {
            cmd_wrench_message.cmd.force = force;
            cmd_wrench_message.cmd.torque = angular;
            cmd_wrench_message.mask = mask;
            cmd_wrench_publisher_->publish(cmd_wrench_message);
        }

        void set_cmd_wrench(float fx, float fy, float fz, float tx, float ty, float tz, uint8_t mask)
        {
            cmd_wrench_message.cmd.force.x = fx;
            cmd_wrench_message.cmd.force.y = fy;
            cmd_wrench_message.cmd.force.z = fz;
            cmd_wrench_message.cmd.torque.x = tx;
            cmd_wrench_message.cmd.torque.y = ty;
            cmd_wrench_message.cmd.torque.z = tz;
            cmd_wrench_message.mask = mask;
            cmd_wrench_publisher_->publish(cmd_wrench_message);
        }

        void set_cmd_ball_drop(int8_t data)
        {
            cmd_ball_drop_message.data = data;
            cmd_ball_drop_publisher_->publish(cmd_ball_drop_message);
        }

        void set_hold_position()
        {
            set_cmd_pose(current_pose.x, current_pose.y, current_pose.z, current_pose.roll, current_pose.pitch, current_pose.yaw, 0b111111);
        }

        void send_diagnostic_command(int8_t command)
        {
            diagnostic_command_message.data = command;
            diagnostic_command_publisher_->publish(diagnostic_command_message);
        }

        std::unordered_map<std::string, MarkerPosition> markers;
        MarkerPosition offset{0.0f, 0.0f};

        PoseEulerData current_pose;
        bool new_pose_data = false;

        DetectionData frontDetections[NUM_CLASSES];
        DetectionData downDetections[NUM_CLASSES];
        bool new_front_vision_data = false;
        bool new_down_vision_data = false;
    private:
        void hb_callback()
        {
            hb_publisher_->publish(hb_message);
        }

        void vehicle_pose_callback(const rsla_interfaces::msg::PoseEuler::SharedPtr msg)
        {
            current_pose.x = msg->position.x + offset.x;
            current_pose.y = msg->position.y + offset.y;
            current_pose.z = msg->position.z;
            current_pose.roll = msg->orientation.roll;
            current_pose.pitch = msg->orientation.pitch;
            current_pose.yaw = msg->orientation.yaw;
            new_pose_data = true;
        }

        void vision_front_detections_callback(const rsla_interfaces::msg::DetectionArray::SharedPtr msg)
        {
            for(int i = 0; i < NUM_CLASSES; i++)
            {
                rsla_interfaces::msg::Detection det = msg->detections[i]; 
                frontDetections[i].class_id = det.id;
                frontDetections[i].detected_now = det.detected;
                if(det.detected)
                {
                    frontDetections[i].detected_ever = true;
                }
                frontDetections[i].confidence = det.confidence;
                frontDetections[i].millis_since_seen = det.millis_since_last_detected;
                if(det.detected)
                {
                    frontDetections[i].yaw_abs_approx = current_pose.yaw + det.ang_x;
                    frontDetections[i].pitch_abs_approx = current_pose.pitch + det.ang_y;
                }
                frontDetections[i].distance = det.distance;
            }
            new_front_vision_data = true;
        }

        void vision_down_detections_callback(const rsla_interfaces::msg::DetectionArray::SharedPtr msg)
        {
            for(int i = 0; i < NUM_CLASSES; i++)
            {
                rsla_interfaces::msg::Detection det = msg->detections[i]; 
                downDetections[i].class_id = det.id;
                downDetections[i].detected_now = det.detected;
                if(det.detected)
                {
                    downDetections[i].detected_ever = true;
                }
                downDetections[i].confidence = det.confidence;
                downDetections[i].millis_since_seen = det.millis_since_last_detected;
                if(det.detected)
                {
                    downDetections[i].yaw_abs_approx = det.ang_x - current_pose.roll;
                    downDetections[i].pitch_abs_approx = current_pose.pitch - det.ang_y;
                }
                downDetections[i].distance = det.distance;
            }
            new_down_vision_data = true;
        }

        // Publishers
        std_msgs::msg::Empty hb_message;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr hb_publisher_;
        rclcpp::TimerBase::SharedPtr hb_timer_;

        std_msgs::msg::Bool arm_message;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_publisher_;

        rsla_interfaces::msg::PoseWithMask cmd_pose_message;
        rclcpp::Publisher<rsla_interfaces::msg::PoseWithMask>::SharedPtr cmd_pose_publisher_;

        rsla_interfaces::msg::WrenchWithMask cmd_wrench_message;
        rclcpp::Publisher<rsla_interfaces::msg::WrenchWithMask>::SharedPtr cmd_wrench_publisher_;

        std_msgs::msg::Int8 cmd_ball_drop_message;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr cmd_ball_drop_publisher_;

        std_msgs::msg::Int8 diagnostic_command_message;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr diagnostic_command_publisher_;

        // Subscribers
        rclcpp::Subscription<rsla_interfaces::msg::PoseEuler>::SharedPtr vehicle_pose_subscription_;
        rclcpp::Subscription<rsla_interfaces::msg::DetectionArray>::SharedPtr vision_front_detections_message_subscription_;
        rclcpp::Subscription<rsla_interfaces::msg::DetectionArray>::SharedPtr vision_down_detections_message_subscription_;
    };
}
