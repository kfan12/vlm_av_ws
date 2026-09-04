// odom_compare - shift odometry sources that are in the wrong frame into the
// spawn-zeroed `odom` frame the rest of the stack uses, so RViz can overlay
// EKF / wheel / ground-truth odometry directly.
//
// Why this node exists: the Gazebo OdometryPublisher ground-truth plugin
// (/odom_truth) stamps its messages `frame_id: odom` but the pose it carries is
// the vehicle's ABSOLUTE world pose - it starts at the map spawn point, not at
// the origin. The other odometry sources (/odom wheel odometry, /odom_ekf)
// already use the REP-105 odom convention: origin = the vehicle pose at
// start-up, i.e. the spawn pose. So /odom_truth sits a constant offset (the
// spawn pose, ~3.5 m on urban_course) away from the rest, even at rest.
//
// By default this relays only the one topic that needs it:
//   /odom_truth --(world->odom, using map_origin_* = spawn)--> /odom_truth_spawn
// /odom_ekf and /odom go to RViz unchanged. Extra relays can be added via the
// `relays` param ("<in>:<out>:<world|odom>" entries).
//
// Visualization only. Nothing in the control path subscribes here - local_planner
// and mpc_tracker_v2 consume /odom_ekf directly and must keep doing so (ground
// truth is not available on a real vehicle).

#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
// q_out = Rz(theta) (x) q_in  (Hamilton product, planar yaw rotation about +z).
geometry_msgs::msg::Quaternion premul_rz(double theta,
                                         const geometry_msgs::msg::Quaternion &q)
{
    const double sz = std::sin(theta * 0.5), cz = std::cos(theta * 0.5);
    geometry_msgs::msg::Quaternion o;
    o.w = cz * q.w - sz * q.z;
    o.x = cz * q.x - sz * q.y;
    o.y = cz * q.y + sz * q.x;
    o.z = cz * q.z + sz * q.w;
    return o;
}

struct Relay
{
    std::string in_topic, out_topic;
    bool world_frame = false; // true: pose is world coords, apply world->odom
};
} // namespace

class OdomCompare : public rclcpp::Node
{
public:
    OdomCompare() : rclcpp::Node("odom_compare")
    {
        // Spawn pose = origin of the odom frame, same convention as MapModel and
        // the map_origin_* params fed to lane_node / local_planner.
        ox_ = declare_parameter("map_origin_x", 0.0);
        oy_ = declare_parameter("map_origin_y", 0.0);
        oyaw_ = declare_parameter("map_origin_yaw", 0.0);
        out_frame_ = declare_parameter("output_frame", std::string("odom"));
        out_child_ = declare_parameter("output_child_frame", std::string("base_link"));

        // "<in_topic>:<out_topic>:<world|odom>" per entry. Default: just the
        // ground-truth relay (the only source not already in the odom frame).
        const std::vector<std::string> spec_default = {
            "/odom_truth:/odom_truth_spawn:world",
        };
        const auto specs = declare_parameter("relays", spec_default);

        c_ = std::cos(-oyaw_);
        s_ = std::sin(-oyaw_);

        for (const auto &spec : specs)
        {
            const auto a = spec.find(':');
            const auto b = spec.rfind(':');
            if (a == std::string::npos || a == b)
            {
                RCLCPP_ERROR(get_logger(),
                             "ignoring relay spec '%s' (want in_topic:out_topic:frame)",
                             spec.c_str());
                continue;
            }
            Relay r;
            r.in_topic = spec.substr(0, a);
            r.out_topic = spec.substr(a + 1, b - a - 1);
            const std::string frame = spec.substr(b + 1);
            if (frame != "world" && frame != "odom")
            {
                RCLCPP_ERROR(get_logger(),
                             "ignoring relay spec '%s' (frame must be 'world' or 'odom')",
                             spec.c_str());
                continue;
            }
            r.world_frame = (frame == "world");

            auto pub = create_publisher<nav_msgs::msg::Odometry>(r.out_topic, 10);
            auto sub = create_subscription<nav_msgs::msg::Odometry>(
                r.in_topic, 10,
                [this, r, pub](nav_msgs::msg::Odometry::ConstSharedPtr msg)
                { pub->publish(convert(*msg, r.world_frame)); });
            pubs_.push_back(std::move(pub));
            subs_.push_back(std::move(sub));
            RCLCPP_INFO(get_logger(), "relay %s -> %s (%s)", r.in_topic.c_str(),
                        r.out_topic.c_str(),
                        r.world_frame ? "world->odom" : "pass-through");
        }
    }

private:
    nav_msgs::msg::Odometry convert(const nav_msgs::msg::Odometry &in, bool world_frame) const
    {
        nav_msgs::msg::Odometry out = in;
        out.header.frame_id = out_frame_;
        out.child_frame_id = out_child_;

        if (world_frame)
        {
            // pose_odom = T_world_odom^-1 * pose_world, with T_world_odom the
            // spawn pose. Rotation part is Rz(-oyaw).
            const double dx = in.pose.pose.position.x - ox_;
            const double dy = in.pose.pose.position.y - oy_;
            out.pose.pose.position.x = c_ * dx - s_ * dy;
            out.pose.pose.position.y = s_ * dx + c_ * dy;
            out.pose.pose.orientation = premul_rz(-oyaw_, in.pose.pose.orientation);
            // twist is expressed in child_frame_id (base_link), so the
            // world->odom shift does not touch it. Covariance is left as-is:
            // these topics feed RViz, not an estimator.
        }
        return out;
    }

    double ox_{}, oy_{}, oyaw_{}, c_{1.0}, s_{0.0};
    std::string out_frame_, out_child_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> pubs_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomCompare>());
    rclcpp::shutdown();
    return 0;
}
