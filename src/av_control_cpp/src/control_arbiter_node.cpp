#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <av_feature_msgs/msg/aeb_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

using av_feature_msgs::msg::AebStatus;

//   1. manual e-stop        -> immediate stop
//   2. AEB emergency        -> /control/aeb_cmd, while fresh
//   3. system fault         -> stop (AEB itself in FAULT counts as a fault
//                              source - a stale/missing safety channel must
//                              not silently fall through to "drive normally")
//   4. MPC nominal command  -> /control/mpc_cmd, while fresh
//   5. fallback             -> built-in stop

class ControlArbiter : public rclcpp::Node
{
public:
    ControlArbiter()
        : Node("control_arbiter")
    {
        // Parameters

        mpc_timeout_ = declare_parameter("mpc_timeout_sec", 0.15);
        aeb_status_timeout_ = declare_parameter("aeb_status_timeout_sec", 0.15);
        aeb_cmd_timeout_ = declare_parameter("aeb_cmd_timeout_sec", 0.15);
        estop_timeout_ = declare_parameter("estop_timeout_sec", 0.5); // no publisher yet (Pitfalls); absence-of-message = not estopped, a STALE true message still latches the stop below
        max_speed_ = declare_parameter("max_speed_mps", 4.0);         // clamp ceiling, independent of any one feature's own cap
        max_yaw_rate_ = declare_parameter("max_yaw_rate_radps", 2.0);
        rate_hz_ = declare_parameter("arbiter_rate_hz", 40.0);

        sub_mpc_ = create_subscription<geometry_msgs::msg::Twist>(
            "control/mpc_cmd", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg)
            {
                mpc_cmd_ = *msg;
                mpc_t_ = now();
            });

        sub_aeb_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
            "control/aeb_cmd", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg)
            {
                aeb_cmd_ = *msg;
                aeb_cmd_t_ = now();
            });

        sub_aeb_status_ = create_subscription<AebStatus>(
            "/safety/aeb_status", 10,
            [this](AebStatus::ConstSharedPtr m)
            { aeb_status_ = *m; aeb_status_t_ = now(); });

        sub_estop_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/manual_estop", 10,
            [this](std_msgs::msg::Bool::ConstSharedPtr m)
            { estop_active_ = m->data; estop_t_ = now(); });

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_source_ = create_publisher<std_msgs::msg::String>("/control/active_source", 10);
        // RViz2 has no String display - same text as a marker riding above the car
        pub_source_marker_ = create_publisher<visualization_msgs::msg::Marker>("/control/active_source_marker", 10);

        auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]()
                                   { tick(); });
    }

private:
    static bool finite_twist(const geometry_msgs::msg::Twist &t)
    {
        return std::isfinite(t.linear.x) && std::isfinite(t.angular.z);
    }

    geometry_msgs::msg::Twist clamp(geometry_msgs::msg::Twist t) const
    {
        t.linear.x = std::clamp(t.linear.x, 0.0, max_speed_);
        t.angular.z = std::clamp(t.angular.z, -max_yaw_rate_, max_yaw_rate_);
        return t;
    }

    bool mpc_fresh() const
    {
        return mpc_t_.nanoseconds() > 0 && (now() - mpc_t_).seconds() < mpc_timeout_ && finite_twist(mpc_cmd_);
    }

    bool aeb_cmd_fresh() const
    {
        return aeb_cmd_t_.nanoseconds() > 0 && (now() - aeb_cmd_t_).seconds() < aeb_cmd_timeout_ && finite_twist(aeb_cmd_);
    }

    bool aeb_status_fresh() const
    {
        return aeb_status_t_.nanoseconds() > 0 && (now() - aeb_status_t_).seconds() < aeb_status_timeout_;
    }

    bool estop_fresh_and_active() const
    {
        return estop_t_.nanoseconds() > 0 && (now() - estop_t_).seconds() < estop_timeout_ && estop_active_;
    }

    void publish(const geometry_msgs::msg::Twist &cmd, const std::string &source)
    {
        pub_cmd_->publish(clamp(cmd));
        std_msgs::msg::String src;
        src.data = source;
        pub_source_->publish(src);

        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = "base_link"; // rides with the car, no pose bookkeeping
        mk.header.stamp = now();
        mk.ns = "active_source";
        mk.id = 0;
        mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        mk.pose.position.z = 3.0; // above lane_node's state text (2.5)
        mk.pose.orientation.w = 1.0;
        mk.scale.z = 0.7;
        mk.color.a = 1.0;
        if (source == "mpc")
        {
            mk.color.g = 1.0; // green: nominal
        }
        else if (source == "aeb")
        {
            mk.color.r = 1.0; // orange: safety override
            mk.color.g = 0.6;
        }
        else
        {
            mk.color.r = 1.0; // red: estop / fault / fallback
        }
        mk.text = "ctrl: " + source;
        pub_source_marker_->publish(mk);

        if (source != last_source_)
        {
            RCLCPP_INFO(get_logger(), "control source: %s -> %s", last_source_.c_str(), source.c_str());
            last_source_ = source;
        }
    }

    void tick()
    {
        // 1. manual e-stop        -> immediate stop
        if (estop_fresh_and_active())
        {
            publish(geometry_msgs::msg::Twist(), "estop");
            return;
        }

        // 2. AEB emergency: requires BOTH a fresh status claiming emergency_active
        // AND a fresh command - if either channel goes stale while the other
        // claims BRAKING, worth treating as a fault (fail toward stop, never toward "drive normally").

        bool aeb_emergency = aeb_status_fresh() && aeb_status_.emergency_active;
        if (aeb_emergency)
        {
            if (aeb_cmd_fresh())
            {
                publish(aeb_cmd_, "aeb");
                return;
            }

            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "AebStatus demands emergency but /control/aeb_cmd is stale - stopping");
            publish(geometry_msgs::msg::Twist(), "fault_aeb_cmd_stale");
            return;
        }

        // 3. system fault: no fresh AEB status, or AEB itself in FAULT -> stop
        bool safety_channel_healthy = aeb_status_fresh() && aeb_status_.state != AebStatus::FAULT;
        if (!safety_channel_healthy)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "AEB status missing or FAULT - stopping (no emergency channel = no nominal driving)");
            publish(geometry_msgs::msg::Twist(), "fault_aeb_status");
            return;
        }

        // 4. MPC nominal command
        if (mpc_fresh())
        {
            publish(mpc_cmd_, "mpc");
            return;
        }

        // 5. fallback: built-in stop
        publish(geometry_msgs::msg::Twist(), "stop_no_fresh_source");
    }

    double mpc_timeout_;
    double aeb_status_timeout_;
    double aeb_cmd_timeout_;
    double estop_timeout_;
    double max_speed_;
    double max_yaw_rate_;
    double rate_hz_;

    geometry_msgs::msg::Twist mpc_cmd_, aeb_cmd_;
    AebStatus aeb_status_;
    bool estop_active_ = false;
    std::string last_source_ = "none";

    rclcpp::Time mpc_t_{0, 0, RCL_ROS_TIME}, aeb_cmd_t_{0, 0, RCL_ROS_TIME},
        aeb_status_t_{0, 0, RCL_ROS_TIME}, estop_t_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_mpc_, sub_aeb_cmd_;
    rclcpp::Subscription<AebStatus>::SharedPtr sub_aeb_status_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_source_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_source_marker_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlArbiter>());
    rclcpp::shutdown();
    return 0;
}