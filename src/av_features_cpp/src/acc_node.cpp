// acc_node - adaptive cruise control from the front lidar.
//
// Each cloud: cluster it (the same av_common clustering AEB uses), keep the
// clusters inside the ego lane along the planned path, take the nearest one
// ahead as the lead, and track its gap with an alpha-beta filter, which also
// gives the closing speed. Then a constant-time-gap policy turns gap and lead
// speed into a speed cap, published as a VelocityConstraint.
//
// ACC only ever lowers speed (the coordinator takes the minimum) and never
// reacts to emergencies - that is aeb_node's job, on its own process and its
// own copy of the cloud. Without a lead it still publishes (a non-binding
// cap), so the coordinator can tell "no lead" from "ACC is dead".

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>

#include <av_feature_msgs/msg/velocity_constraint.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "av_common/geometry.hpp"
#include "av_common/point_cloud.hpp"
#include "av_common/projection.hpp"

using av_feature_msgs::msg::VelocityConstraint;

class AccNode : public rclcpp::Node
{
public:
    AccNode() : rclcpp::Node("acc_node")
    {
        // lidar extrinsic - mirrors sedan.urdf.xacro lidar_x / lidar_z
        mount_x_ = declare_parameter("lidar_x", 2.30);
        mount_z_ = declare_parameter("lidar_z", 0.60);
        filt_.min_range_m = declare_parameter("lidar_min_range_m", 0.3);
        filt_.max_range_m = declare_parameter("max_range_m", 45.0);
        filt_.max_lateral_m = declare_parameter("max_lateral_m", 6.0);
        filt_.min_height_m = declare_parameter("min_height_m", 0.3);
        filt_.max_height_m = declare_parameter("max_height_m", 3.0);
        voxel_m_ = declare_parameter("cluster_voxel_m", 0.5);
        min_points_ = declare_parameter("minimum_cluster_points", 3);

        lane_half_width_ = declare_parameter("lane_half_width_m", 1.3);
        bumper_x_ = declare_parameter("bumper_x_m", 2.25); // sedan.urdf.xacro ch_l / 2

        time_gap_ = declare_parameter("time_gap_sec", 1.8);
        standstill_gap_ = declare_parameter("standstill_gap_m", 5.0);
        min_gap_ = declare_parameter("min_gap_m", 3.0);
        gap_gain_ = declare_parameter("gap_gain_per_sec", 0.3);
        alpha_ = declare_parameter("tracker_alpha", 0.4);
        beta_ = declare_parameter("tracker_beta", 0.1);
        gate_ = declare_parameter("tracker_gate_m", 3.0);
        lost_after_ = declare_parameter("lost_after_sec", 0.5);
        validity_ = declare_parameter("constraint_validity_sec", 0.3);
        input_stale_ = declare_parameter("input_stale_sec", 0.5);

        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/front/points", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m)
            { on_cloud(*m); });
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom_ekf", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m)
            { odom_ = m; odom_t_ = now(); });
        sub_path_ = create_subscription<nav_msgs::msg::Path>(
            "/plan/path_odom", 10, [this](nav_msgs::msg::Path::ConstSharedPtr m)
            { path_ = m; path_t_ = now(); });

        pub_constraint_ = create_publisher<VelocityConstraint>("/feature/acc/constraint", 10);
        // [gap_m, lead_speed_mps, cap_mps, has_lead] - for recording and plots
        pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("/feature/acc/state", 10);
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/feature/acc/debug_markers", 10);
    }

private:
    struct Track
    {
        double gap = 0.0;  // bumper to the lead's nearest point, along the path [m]
        double rate = 0.0; // d(gap)/dt [m/s]
        rclcpp::Time t;
        Eigen::Vector2d p_odom{0, 0};
    };

    // the planned path in odom, extended straight past its end so a lead
    // beyond the perception window still projects onto "the lane ahead"
    av::geom::Polyline lane_path() const
    {
        av::geom::Polyline p;
        for (const auto &ps : path_->poses)
            p.push_back({ps.pose.position.x, ps.pose.position.y});

        if (p.size() >= 2)
        {
            Eigen::Vector2d d = (p.back() - p[p.size() - 2]).normalized();
            p.push_back(p.back() + d * (filt_.max_range_m + 10.0));
        }
        return p;
    }

    void on_cloud(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        const rclcpp::Time t = now();
        if (!odom_ || !path_ || (t - odom_t_).seconds() > input_stale_ || (t - path_t_).seconds() > input_stale_)
        {
            // publish nothing: the coordinator lists ACC as required and holds
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for fresh /odom_ekf and /plan/path_odom");
            return;
        }

        const auto &q = odom_->pose.pose.orientation;
        av::proj::Pose2d ego{odom_->pose.pose.position.x, odom_->pose.pose.position.y,
                             std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};

        const double v_ego = std::hypot(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y);

        const auto path = lane_path();
        if (path.size() < 2)
            return;

        const double s_ego = av::geom::project_point(path, {ego.x, ego.y}).s;

        // nearest in-lane cluster ahead, by arc length along the planned path
        std::optional<double> meas;
        Eigen::Vector2d lead_odom{0, 0};
        auto clusters = av::cloud::voxel_clusters(
            av::cloud::cloud_points_base(cloud, mount_x_, mount_z_, filt_), voxel_m_, min_points_);

        for (const auto &c : clusters)
        {
            Eigen::Vector2d p_odom = ego.odom_of({c.nearest_x, c.mean.y()});
            auto pr = av::geom::project_point(path, p_odom);
            if (std::abs(pr.lateral) > lane_half_width_ || pr.s <= s_ego)
                continue;
            double gap = pr.s - s_ego - bumper_x_;
            if (!meas || gap < *meas)
            {
                meas = gap;
                lead_odom = p_odom;
            }
        }

        update_track(meas, lead_odom, t, v_ego);
        publish(t, v_ego);
    }

    void update_track(const std::optional<double> &meas, const Eigen::Vector2d &p, const rclcpp::Time &t, double v_ego)
    {
        if (track_ && !meas && (t - track_->t).seconds() > lost_after_)
            track_.reset();
        if (!meas)
            return;
        if (track_)
        {
            const double dt = std::max(1e-3, (t - track_->t).seconds());
            const double pred = track_->gap + track_->rate * dt;
            const double r = *meas - pred;
            if (std::abs(r) <= gate_)
            {
                track_->gap = pred + alpha_ * r;
                track_->rate += beta_ * r / dt;
                track_->t = t;
                track_->p_odom = p;
                return;
            }
        }
        // new target: assume it is standing still until the filter learns
        // otherwise - the conservative guess for the cap
        track_ = Track{*meas, -v_ego, t, p};
    }

    void publish(const rclcpp::Time &t, double v_ego)
    {
        VelocityConstraint c;
        c.header.stamp = t;
        c.source = "acc";
        c.priority = 3;
        c.valid_until = t + rclcpp::Duration::from_seconds(validity_);
        c.confidence = 1.0;
        double lead_v = 0.0, cap = 1e3;
        if (track_)
        {
            lead_v = std::max(0.0, v_ego + track_->rate);
            const double desired_gap = standstill_gap_ + time_gap_ * v_ego;
            cap = track_->gap < min_gap_ ? 0.0
                                         : std::max(0.0, lead_v + gap_gain_ * (track_->gap - desired_gap));
            char why[96];
            std::snprintf(why, sizeof(why), "lead %.1f m at %.1f m/s, want %.1f m", track_->gap, lead_v, desired_gap);
            c.reason = why;
        }
        else
            c.reason = "no lead";
        c.maximum_speed_mps = cap;
        pub_constraint_->publish(c);

        std_msgs::msg::Float64MultiArray st;
        st.data = {track_ ? track_->gap : -1.0, lead_v, cap, track_ ? 1.0 : 0.0};
        pub_state_->publish(st);
        publish_markers(t, c.reason);
    }

    void publish_markers(const rclcpp::Time &t, const std::string &text)
    {
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = t;
        m.ns = "acc";
        m.lifetime = rclcpp::Duration::from_seconds(0.3);
        m.pose.orientation.w = 1.0;
        if (track_)
        {
            m.id = 0;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.pose.position.x = track_->p_odom.x();
            m.pose.position.y = track_->p_odom.y();
            m.pose.position.z = 0.75;
            m.scale.x = m.scale.y = 0.6;
            m.scale.z = 1.5;
            m.color.b = 1.0;
            m.color.g = 0.5;
            m.color.a = 0.7;
            arr.markers.push_back(m);
        }
        m.id = 1;
        m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        m.pose.position.x = odom_->pose.pose.position.x;
        m.pose.position.y = odom_->pose.pose.position.y;
        m.pose.position.z = 3.6; // above the arbiter's source label
        m.scale.z = 0.5;
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0;
        m.text = "ACC: " + text;
        arr.markers.push_back(m);
        pub_markers_->publish(arr);
    }

private:
    double mount_x_ = 2.30, mount_z_ = 0.60, voxel_m_ = 0.5;
    int min_points_ = 3;
    av::cloud::CloudFilter filt_;
    double lane_half_width_ = 1.3, bumper_x_ = 2.25;
    double time_gap_ = 1.8, standstill_gap_ = 5.0, min_gap_ = 3.0, gap_gain_ = 0.3;
    double alpha_ = 0.4, beta_ = 0.1, gate_ = 3.0, lost_after_ = 0.5, validity_ = 0.3, input_stale_ = 0.5;

    std::optional<Track> track_;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    nav_msgs::msg::Path::ConstSharedPtr path_;
    rclcpp::Time odom_t_{0, 0, RCL_ROS_TIME}, path_t_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Publisher<VelocityConstraint>::SharedPtr pub_constraint_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccNode>());
    rclcpp::shutdown();
    return 0;
}