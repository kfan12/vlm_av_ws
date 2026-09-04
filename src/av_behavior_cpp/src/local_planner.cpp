#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "av_common/debug_tap.hpp"
#include "av_common/geometry.hpp"
#include "av_common/json_io.hpp"
#include "av_common/map_model.hpp"
#include "av_common/projection.hpp"

using av::json;
using av::geom::Polyline;

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner() : Node("local_planner")
    {
        map_path_ = declare_parameter("map_path", std::string(""));
        spacing_ = declare_parameter("path_spacing", 0.5);
        smooth_window_ = declare_parameter("smooth_window", 9);
        splice_dist_ = declare_parameter("splice_dist", 25.0);
        bridge_hold_s_ = declare_parameter("bridge_hold_s", 15.0);
        lane_stale_s_ = declare_parameter("lane_stale_s", 1.5);
        spline_tail_extend_m_ = declare_parameter("spline_tail_extend_m", 45.0);
        crossfade_m_ = declare_parameter("path_crossfade_m", 4.0);
        // road-curvature assist (course-style maps, see build_course_lane)
        curve_splice_enable_ = declare_parameter("curve_splice_enable", false);
        curve_kappa_min_ = declare_parameter("curve_kappa_min", 0.02);
        curve_entry_lead_m_ = declare_parameter("curve_entry_lead_m", 2.0);
        curve_exit_extend_m_ = declare_parameter("curve_exit_extend_m", 8.0);
        curve_end_hysteresis_m_ = declare_parameter("curve_end_hysteresis_m", 10.0);
        curve_offcourse_max_m_ = declare_parameter("curve_offcourse_max_m", 6.0);
        a_lat_max_ = declare_parameter("a_lat_max", 2.0);
        a_comfort_ = declare_parameter("a_comfort", 3.0);
        a_accel_ = declare_parameter("a_accel", 2.0);
        v_cap_ = declare_parameter("v_cap", 8.0);

        double mo_x = declare_parameter("map_origin_x", 0.0);
        double mo_y = declare_parameter("map_origin_y", 0.0);
        double mo_yaw = declare_parameter("map_origin_yaw", 0.0);

        if (!map_path_.empty())
        {
            try
            {
                map_ = av::MapModel::load(map_path_, mo_x, mo_y, mo_yaw);
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(get_logger(), "map load failed: %s -- no slices", e.what());
            }
        }

        build_course_lane();

        sub_lane_ = create_subscription<nav_msgs::msg::Path>("/lane/path_odom", 10, [this](nav_msgs::msg::Path::ConstSharedPtr msg)
                                                             {
            lane_.clear();
            for (const auto &ps : msg->poses)
                lane_.emplace_back(ps.pose.position.x, ps.pose.position.y);
            lane_t_ = now().seconds(); });

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/odom_ekf", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
                                                                 {
            const auto &q  = msg->pose.pose.orientation;
            pose_ = av::proj::Pose2d{
                msg->pose.pose.position.x, msg->pose.pose.position.y,
                std::atan2(2 * (q.w * q.z + q.x * q.y), // quaternion -> yaw (Z component), no tf2 dependency
                           1 - 2 * (q.y * q.y + q.z * q.z))}; 
            ego_v_ = msg->twist.twist.linear.x; });

        pub_path_ = create_publisher<nav_msgs::msg::Path>("/plan/path_odom", 10); // the drivable plan (odom)
        pub_profile_ =
            create_publisher<std_msgs::msg::Float32MultiArray>("/plan/speed_profile", 10); // one speed per plan point
        tap_ = std::make_unique<av::DebugTap>(this);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] // 10 Hz control loop
                                   { tick(); });
    };

private:
    std::string map_path_;
    int smooth_window_ = 9;
    double spacing_, splice_dist_, bridge_hold_s_, lane_stale_s_,
        spline_tail_extend_m_, crossfade_m_, a_lat_max_, a_comfort_, a_accel_,
        v_cap_;
    bool curve_splice_enable_ = true;
    double curve_kappa_min_, curve_entry_lead_m_, curve_exit_extend_m_,
        curve_end_hysteresis_m_, curve_offcourse_max_m_;
    Polyline course_lane_;
    std::vector<double> course_s_, course_kappa_; // parallel arrays: arc length, |curvature| per course_lane_ sample

    std::optional<av::MapModel> map_;
    std::optional<av::proj::Pose2d> pose_;
    Polyline lane_;
    double lane_t_ = -1e9, ego_v_ = 0, last_spliced_t_ = -1e9;
    Polyline last_published_;

    void build_course_lane()
    {
        if (!map_ || map_->course_centerline.size() < 3)
            return;

        const auto &cl = map_->course_centerline;
        double half = map_->lane_width * 0.5;

        Polyline lane;
        for (size_t i = 0; i < cl.size(); ++i)
        {
            size_t j = std::min(i + 1, cl.size() - 1);
            size_t k = (j > 0 ? j - 1 : 0); // neighbours for a central-difference tangent (clamped at the ends)
            Eigen::Vector2d t = cl[j] - cl[k];
            double n = t.norm(); // local forward tangent of the road
            if (n < 1e-9)        // duplicate points
                continue;
            t /= n;                                      // unit tangent
            Eigen::Vector2d right_normal(t.y(), -t.x()); // unit tanget rotate -90 deg -> right direction vector
            lane.push_back(cl[i] + half * right_normal); // shift pt to right direction by half of lane width
        }

        course_lane_ = av::geom::resample_uniform(lane, spacing_);
        course_s_ = av::geom::arc_length(course_lane_);
        Polyline sm = av::geom::smooth_moving_avg(course_lane_, 9);

        course_kappa_.assign(course_lane_.size(), 0.0);
        for (size_t i = 1; i + 1 < sm.size(); ++i)
            course_kappa_[i] = std::abs(av::geom::curvature_menger(sm[i - 1], sm[i], sm[i + 1]));

        RCLCPP_INFO(get_logger(),
                    "course lane built: %zu pts, %.1f m (curve assist %s)", // one-shot sanity line: expect ~500 pts, ~244 m
                    course_lane_.size(), course_s_.empty() ? 0.0 : course_s_.back(),
                    curve_splice_enable_ ? "on" : "off");
    }

    std::optional<Polyline> curve_segment_ahead(std::string &id)
    {
        if (!curve_splice_enable_ || course_lane_.size() < 3)
            return std::nullopt;

        auto pr = av::geom::project_point(course_lane_, {pose_->x, pose_->y}); // foot point of ego on the course lane: .s, .seg, .dist
        if (pr.dist > curve_offcourse_max_m_)
            return std::nullopt;

        double s_ego = pr.s; // arc-length position along the course lane
        size_t n = course_lane_.size();
        size_t i_entry = n;

        // search ahead until splice_dist_
        for (size_t i = pr.seg; i < n && course_s_[i] <= s_ego + splice_dist_; ++i)
        {
            // find curvature > curve_kappa_min && arc length course_s_ > s_ego - 1.0
            if (course_s_[i] >= s_ego - 1.0 && course_kappa_[i] > curve_kappa_min_)
            {
                i_entry = i;
                break;
            }
        }

        if (i_entry == n) // nothing ahear
            return std::nullopt;

        // walk from the i_entry along the course, find i_exit when course_kappa_[i] is no more > curve_kappa_min
        size_t i_exit = i_entry;
        for (size_t i = i_entry; i < n; i++)
        {
            // if still curve, increas i_exit
            if (course_kappa_[i] > curve_kappa_min_)
                i_exit = i;
            else if (course_s_[i] - course_s_[i_exit] > curve_end_hysteresis_m_) // hysteresis
                break;
        }

        double s_start = std::max(course_s_[i_entry] - curve_entry_lead_m_, s_ego - 1.0); // start x m before the curve but not behind the ego
        double s_end = course_s_[i_exit] + curve_exit_extend_m_;

        Polyline seg;
        for (size_t i = 0; i < n; i++)
            if (course_s_[i] >= s_start && course_s_[i] <= s_end)
                seg.push_back(course_lane_[i]);
        if (seg.size() < 2)
            return std::nullopt;
        id = "curve@" + std::to_string(static_cast<int>(course_s_[i_entry]));
        return seg;
    }

    // ------------------------------------------------------------ splicing
    // Join the raw lane_ prefix (before the segment entry) to `spline`, then
    // extrapolate past the segment exit along its final heading.
    Polyline splice_and_extend(const Polyline &path, const Polyline &spline)
    {
        Polyline out;
        if (path.size() > 2)
        {
            double s_entry = av::geom::project_point(path, spline.front()).s; // where on the lane path the map segment begins
            auto s = av::geom::arc_length(path);

            for (size_t i = 0; i < path.size(); i++)
                if (s[i] < s_entry - 0.5)
                    out.push_back(path[i]); // 0.5 guard, prefix and segment don't overlap
        }

        for (const auto &p : spline)
            out.push_back(p);

        if (spline.size() >= 2)
        {
            Eigen::Vector2d dir = (spline.back() - spline[spline.size() - 2]).normalized(); // unit direction of the segment's last chord

            for (double s = 1.0; s <= spline_tail_extend_m_; s += 1.0)
                out.push_back(spline.back() + s * dir);
        }

        return out;
    }

    Polyline build_path(bool &spliced, std::string &spline_id)
    {
        spliced = false;
        Polyline path = lane_; // default output: raw perception, untouched
        if (!map_ || !pose_)
            return path;

        if (auto seg = curve_segment_ahead(spline_id)) // a curve is ahead in range
        {
            spliced = true;
            return splice_and_extend(path, *seg);
        }

        return path;
    }

    // ------------------------------------------------------- speed profile
    std::vector<float> build_profile(const Polyline &path, double v_zone)
    {
        size_t n = path.size();
        std::vector<float> v(n, static_cast<float>(std::min(v_cap_, v_zone)));
        if (n < 2)
            return v;

        double ds = spacing_;

        // 1. curvature cap
        for (size_t i = 1; i + 1 < n; ++i)
        {
            double k = std::abs(av::geom::curvature_menger(path[i - 1], path[i], path[i + 1])); // cuvature (1/radius)
            if (k > 1e-6)
                v[i] = std::min<float>(v[i], std::sqrt(a_lat_max_ / k)); // // a_lat = v^2*k <= a_lat_max  ->  v <= sqrt(a_lat_max/k). POINTWISE only.
        }

        // 2. terminal zero
        auto s = av::geom::arc_length(path);
        v[n - 1] = 0; // always ends stopped

        // 3. backward pass (brake)
        for (size_t i = n - 1; i > 0; --i)
        {
            v[i - 1] = std::min<float>(v[i - 1], std::sqrt(v[i] * v[i] + 2 * a_comfort_ * ds));
        }

        // 4. forward pass seeded with ego speed at the ego projection
        if (pose_)
        {
            size_t ego_i = 0;
            double ego_s = av::geom::project_point(path, {pose_->x, pose_->y}).s;
            for (size_t i = 0; i < n; ++i)
                if (s[i] <= ego_s)
                    ego_i = i;
            double run = std::max(ego_v_, 0.3);
            for (size_t i = ego_i; i < n; ++i)
            {
                run = std::min<double>(v[i], std::sqrt(run * run + 2 * a_accel_ * ds)); // ramp up
                v[i] = static_cast<float>(run);
            }
        }
        return v; // smooth, corner-aware, brakes early v profile
    }

    // Blend the first crossfade_m_ metres of a freshly-built path toward the
    // path published on the PREVIOUS tick, weight decaying from alpha_max at i=0
    // down to 0.
    Polyline crossfade(const Polyline &fresh)
    {
        if (last_published_.size() < 2 || fresh.size() < 2 || crossfade_m_ <= 0.0) // first tick, or blend disabled
        {
            last_published_ = fresh;
            return fresh;
        }

        const double alpha_max = 0.8;
        Polyline out = fresh;
        int n = std::min({static_cast<int>(out.size()),
                          static_cast<int>(last_published_.size()),
                          static_cast<int>(crossfade_m_ / spacing_) + 1});

        for (int i = 0; i < n; ++i)
        {
            double w = alpha_max * (1.0 - static_cast<double>(i) / n);
            out[i] = w * last_published_[i] + (1.0 - w) * fresh[i];
        }

        last_published_ = out;
        return out;
    }

    void tick()
    {
        double now_s = now().seconds();
        if (!pose_ || lane_.empty()) // no ego pose, or no lane yet -> can't plan
            return;

        bool bridging = (now_s - last_spliced_t_ < bridge_hold_s_);
        if (now_s - lane_t_ > lane_stale_s_ && !bridging)
            return;

        bool spliced = false;
        std::string spline_id;
        Polyline path = build_path(spliced, spline_id);
        if (path.size() < 2)
            return;
        if (spliced)
            last_spliced_t_ = now_s;

        path = av::geom::smooth_moving_avg(path, smooth_window_);
        path = av::geom::resample_uniform(path, spacing_);
        path = crossfade(path);

        double v_zone = map_ ? map_->speed_limit_at({pose_->x, pose_->y}) : v_cap_; // map speed zone cap (default = v_cap on the course)
        auto profile = build_profile(path, v_zone);

        nav_msgs::msg::Path pmsg;
        pmsg.header.stamp = now();
        pmsg.header.frame_id = "odom";
        for (size_t i = 0; i < path.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = pmsg.header;
            ps.pose.position.x = path[i].x();
            ps.pose.position.y = path[i].y();
            size_t j = std::min(i + 1, path.size() - 1);
            size_t k = (j > 0 ? j - 1 : 0);
            double yaw = std::atan2(path[j].y() - path[k].y(), path[j].x() - path[k].x());
            ps.pose.orientation.z = std::sin(yaw * 0.5);
            ps.pose.orientation.w = std::cos(yaw * 0.5);
            pmsg.poses.push_back(ps);
        }
        pub_path_->publish(pmsg);

        std_msgs::msg::Float32MultiArray array;
        array.data = profile;
        pub_profile_->publish(array);

        tap_->put("n_pts", static_cast<int>(path.size())); // watch this == profile length
        tap_->put("spliced", spliced);                     // true a few m before each curve, false a few m after
        tap_->put("spline", spline_id);                    // one stable id per curve
        tap_->put("v_zone", v_zone);
        tap_->flush();
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_lane_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_profile_;
    std::unique_ptr<av::DebugTap> tap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlanner>());
    rclcpp::shutdown();
    return 0;
}