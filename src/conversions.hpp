#pragma once

#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace BT
{
    template <> inline geometry_msgs::msg::Vector3 convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');
        if(parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }

        geometry_msgs::msg::Vector3 output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.z = convertFromString<double>(parts[2]);

        return output;
    }
}

namespace RSLA
{
    inline geometry_msgs::msg::Quaternion convertFromEulerAngles(geometry_msgs::msg::Vector3 euler)
    {
        // Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        double roll = euler.x;
        double pitch = euler.y;
        double yaw = euler.z;

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        geometry_msgs::msg::Quaternion output;
        output.w = cr * cp * cy + sr * sp * sy;
        output.x = sr * cp * cy - cr * sp * sy;
        output.y = cr * sp * cy + sr * cp * sy;
        output.z = cr * cp * sy - sr * sp * cy;

        return output;
    }

    inline geometry_msgs::msg::Point convertToPoint(geometry_msgs::msg::Vector3 vec)
    {
        geometry_msgs::msg::Point output;
        output.x = vec.x;
        output.y = vec.y;
        output.z = vec.z;

        return output;
    }
}