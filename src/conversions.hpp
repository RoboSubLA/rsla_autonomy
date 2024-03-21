#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

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