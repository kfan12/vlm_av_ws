// constraint_from_maneuver - republishes the speed target on
// /maneuver/target_speed (+ /maneuver/state) as a VelocityConstraint, so the
// coordinator can merge it with other features' caps.
//
// The constraint is valid for validity_window_sec after the last target
// message, the same 1.0 s window mpc_tracker_v2 applies to the target itself:
// if the publisher stops, the constraint goes stale and a coordinator that
// lists it as required holds the car.
// Publishes nothing until the first target arrives.
#include <chrono>
#include <memory>
#include <string>

#include <av_feature_msgs/msg/velocity_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

using av_feature_msgs::msg::VelocityConstraint;

class ConstraintFromManeuver : public rclcpp::Node
{
public:
    ConstraintFromManeuver() : Node("constraint_from_maneuver")
    {
        source_ = declare_parameter("source", std::string("vlm_sign"));
        validity_window_ = declare_parameter("validity_window_sec", 1.0);
        const double rate_hz = declare_parameter("publish_rate_hz", 10.0);

        sub_speed_ = create_subscription<std_msgs::msg::Float64>(
            "/maneuver/target_speed", 10,
            [this](std_msgs::msg::Float64::ConstSharedPtr m)
            { target_speed_ = m->data; speed_t_ = now(); });

        sub_state_ = create_subscription<std_msgs::msg::String>(
            "/maneuver/state", 10,
            [this](std_msgs::msg::String::ConstSharedPtr m)
            { state_ = m->data; });

        // relative name: the launch remaps it per source, e.g. /feature/cruise/constraint
        pub_ = create_publisher<VelocityConstraint>("constraint", 10);
        // node clock, so under use_sim_time the rate follows the simulation
        timer_ = rclcpp::create_timer(this, get_clock(),
                                      rclcpp::Duration::from_seconds(1.0 / rate_hz), [this]
                                      { tick(); });
    }

private:
    void tick()
    {
        if (speed_t_.nanoseconds() == 0)
            return; // no target yet: publish nothing, the coordinator's staleness handling covers it

        VelocityConstraint c;
        c.header.stamp = now();
        c.source = source_;
        c.priority = 3; // restrictive speed constraint
        c.stop_required = false;
        c.stop_distance_m = 0.0;
        c.maximum_speed_mps = target_speed_;
        c.maximum_deceleration_mps2 = 0.0; // not used by the merge
        c.valid_until = speed_t_ + rclcpp::Duration::from_seconds(validity_window_);
        c.confidence = 1.0;
        c.reason = state_.empty() ? "target speed" : ("target speed, state=" + state_);
        pub_->publish(c);
    }

private:
    std::string source_;
    double validity_window_ = 1.0;
    double target_speed_ = 0.0;
    std::string state_;
    rclcpp::Time speed_t_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_speed_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
    rclcpp::Publisher<VelocityConstraint>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConstraintFromManeuver>());
    rclcpp::shutdown();
    return 0;
}