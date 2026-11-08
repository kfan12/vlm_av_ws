// trajectory_coordinator - merges every fresh VelocityConstraint into the
// planner's nominal speed profile and publishes the authorized path + profile
// the MPC tracks.
//
// Velocity only: the path passes through unchanged. Each constraint caps the
// whole profile at its maximum_speed_mps (pointwise minimum), so the result is
// never faster than the planner or any feature allows.
//
// Event-driven: one output per incoming profile. A timer re-publishing the last
// plan would make a dead local_planner look alive to the MPC and defeat its
// stale-path stop.
//
// Required topics: if one of them has no fresh constraint, the profile is held
// at 0 - a crashed feature must not silently fall back to the uncapped speed.

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <av_feature_msgs/msg/velocity_constraint.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using av_feature_msgs::msg::VelocityConstraint;

class TrajectoryCoordinator : public rclcpp::Node
{
public:
    TrajectoryCoordinator() : Node("trajectory_coordinator")
    {
        auto topics = declare_parameter("constraint_topics", std::vector<std::string>{});
        auto required = declare_parameter("required_constraint_topics", std::vector<std::string>{});
        hold_without_constraints_ = declare_parameter("hold_without_constraints", true);
        required_.insert(required.begin(), required.end());
        for (const auto &r : required_)
            if (std::find(topics.begin(), topics.end(), r) == topics.end())
                throw std::invalid_argument("required constraint topic not in constraint_topics: " + r);

        sub_path_ = create_subscription<nav_msgs::msg::Path>(
            "/plan/path_odom", 10, [this](nav_msgs::msg::Path::ConstSharedPtr m)
            { path_ = m; });
        sub_profile_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/plan/speed_profile", 10,
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m)
            { on_profile(m); });

        // the whole extension point: one subscription per configured topic
        for (const auto &topic : topics)
            subs_.push_back(create_subscription<VelocityConstraint>(
                topic, 10, [this, topic](VelocityConstraint::ConstSharedPtr m)
                { latest_[topic] = m; }));
        pub_path_ = create_publisher<nav_msgs::msg::Path>("/planning/authorized/path", 10);
        pub_profile_ = create_publisher<std_msgs::msg::Float32MultiArray>("/planning/authorized/speed_profile", 10);
        pub_binding_ = create_publisher<std_msgs::msg::String>("/planning/binding_constraint", 10);
    }

private:
    bool fresh(const VelocityConstraint::ConstSharedPtr &c) const
    {
        return c && now() <= rclcpp::Time(c->valid_until, RCL_ROS_TIME);
    }

    void on_profile(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &profile)
    {
        // local_planner publishes path then profile, equal length; anything
        // else is a torn pair and is dropped, the same check the MPC makes
        if (!path_ || path_->poses.size() != profile->data.size())
            return;
        std::vector<float> out = profile->data;
        std::string binding = "planner";
        float binding_cap = 1e9f;
        bool any_fresh = false;
        std::string missing;

        for (const auto &[topic, c] : latest_)
        {
            if (!fresh(c))
                continue;
            any_fresh = true;
            const float cap = static_cast<float>(c->maximum_speed_mps);
            for (auto &v : out)
                v = std::min(v, cap);
            if (cap < binding_cap)
            {
                binding_cap = cap;
                binding = c->source + ": " + c->reason;
            }
        }

        for (const auto &r : required_)
            if (!latest_.count(r) || !fresh(latest_.at(r)))
                missing += (missing.empty() ? "" : ", ") + r;

        if (!missing.empty() || (!any_fresh && hold_without_constraints_))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "holding: no fresh constraint on %s",
                                 missing.empty() ? "any topic" : missing.c_str());
            std::fill(out.begin(), out.end(), 0.0f);
            binding = "hold: " + (missing.empty() ? std::string("no constraints") : missing);
        }

        pub_path_->publish(*path_);
        std_msgs::msg::Float32MultiArray msg;
        msg.data = std::move(out);
        pub_profile_->publish(msg);
        std_msgs::msg::String b;
        b.data = binding;
        pub_binding_->publish(b);
    }

private:
    bool hold_without_constraints_ = true;
    std::set<std::string> required_;
    nav_msgs::msg::Path::ConstSharedPtr path_;
    std::map<std::string, VelocityConstraint::ConstSharedPtr> latest_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_profile_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_; // vector of subscriptions to VelocityConstraint topics
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_profile_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_binding_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryCoordinator>());
    rclcpp::shutdown();
    return 0;
}