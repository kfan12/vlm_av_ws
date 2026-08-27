#pragma once
// C++ port of vlm_planner_py.debugtap — wire-compatible with the debugkit
// recorder: ONE flat JSON object per tick on /debug/<node_name>, gated by the
// bool parameter 'debug_tap' (default false, toggle live with `ros2 param set`).
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "av_common/json_io.hpp"

namespace av
{

    class DebugTap
    {
    public:
        explicit DebugTap(rclcpp::Node *node, const std::string &topic = "")
            : node_(node)
        {
            if (!node->has_parameter("debug_tap"))
                node->declare_parameter("debug_tap", false);
            pub_ = node->create_publisher<std_msgs::msg::String>(
                topic.empty() ? std::string("/debug/") + node->get_name() : topic, 10);
        }

        bool enabled() const
        {
            return node_->get_parameter("debug_tap").as_bool();
        }

        void put(const std::string &key, const json &value)
        {
            if (enabled())
                data_[key] = value;
        }

        void flush()
        {
            if (!enabled() || data_.empty())
            {
                data_.clear();
                return;
            }
            data_["t"] = node_->now().seconds();
            pub_->publish(to_msg(data_));
            data_.clear();
        }

    private:
        rclcpp::Node *node_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        json data_ = json::object();
    };

} // namespace av
