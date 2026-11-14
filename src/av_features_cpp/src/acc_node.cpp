// acc_node - adaptive cruise control from the front lidar.
//
// Each cloud: group its points into objects (connected cells of the same
// voxel grid AEB clusters on), keep the objects with points inside the ego
// lane along the planned path, and track every one of them (associated by odom
// position). Each track filters its gap along the path (alpha-beta, which also
// gives the closing speed) and its odom position (which gives its own ground
// speed). The nearest accepted track is the lead, and a constant-time-gap
// policy turns its gap and speed into a speed cap, published as a
// VelocityConstraint.
//
// Accepted means the object has been inside the perceived path (the planner's
// path, about 12 m) or has been seen moving along the road. Past the perceived
// path, "the lane" is only the path's straight extension, which runs off the
// road at every bend; a stationary object there is far more likely roadside
// than a stopped car, so ACC leaves it alone until it moves or the perceived
// path reaches it. A stopped car far ahead is still AEB's, which brakes for it.
//
// Only a vehicle-sized object seen whole can count as moving: the part of a
// building inside the lane band, the lateral crop or the range limit slides
// along the facade as the car drives, which reads as ground motion.
//
// ACC only ever lowers speed (the coordinator takes the minimum) and never
// reacts to emergencies - that is aeb_node's job, on its own process and its
// own copy of the cloud. Without a lead it still publishes (a non-binding
// cap), so the coordinator can tell "no lead" from "ACC is dead".

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

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
        min_points_ = declare_parameter("minimum_cluster_points", 3); // per object
        max_vehicle_extent_ = declare_parameter("max_vehicle_extent_m", 7.5);

        lane_half_width_ = declare_parameter("lane_half_width_m", 1.3);
        bumper_x_ = declare_parameter("bumper_x_m", 2.25); // sedan.urdf.xacro ch_l / 2

        time_gap_ = declare_parameter("time_gap_sec", 1.8);
        standstill_gap_ = declare_parameter("standstill_gap_m", 5.0);
        min_gap_ = declare_parameter("min_gap_m", 3.0);
        gap_gain_ = declare_parameter("gap_gain_per_sec", 0.3);
        alpha_ = declare_parameter("tracker_alpha", 0.4);
        beta_ = declare_parameter("tracker_beta", 0.1);
        gate_ = declare_parameter("tracker_gate_m", 3.0);
        assoc_gate_ = declare_parameter("association_gate_m", 2.0);
        require_motion_ = declare_parameter("require_motion_beyond_path", true);
        moving_speed_ = declare_parameter("moving_speed_mps", 1.0);
        moving_confirm_ = declare_parameter("moving_confirm_sec", 1.0);
        max_tracks_ = declare_parameter("max_tracks", 32);
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
        // [gap_m, lead_speed_mps, cap_mps, has_lead,
        //  candidate_gap_m, candidate_speed_along_path_mps, candidate_flags, n_tracks,
        //  lead_flags, lead_speed_along_path_mps, lead_speed_across_path_mps,
        //  candidate_extent_m, lead_extent_m]
        // for recording and plots; candidate = nearest track, accepted or not;
        // speeds are ground speeds from the odom filter; flags: 1 seen moving,
        // 2 seen inside the perceived path, 4 now beyond it, 8 now cut by the
        // lateral crop or the range limit
        pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("/feature/acc/state", 10);
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/feature/acc/debug_markers", 10);
    }

private:
    struct Track
    {
        double gap = 0.0;  // bumper to the object's nearest point, along the path [m]
        double rate = 0.0; // d(gap)/dt [m/s]
        rclcpp::Time t;    // last measurement
        Eigen::Vector2d pos{0, 0}, vel{0, 0}; // odom position and ground velocity
        double moving_since = -1.0;           // [s] when its speed along the path went over moving_speed_mps
        bool moved = false;                   // sustained motion along the path seen (sticky)
        bool inside = false;                  // seen inside the perceived path (sticky)
        // latest measurement:
        bool beyond = false;       // past the perceived path
        bool truncated = false;    // cut by the lateral crop or the range limit
        double extent = 0.0;       // larger side of the object's bounding box [m]
        Eigen::Vector2d dir{1, 0}; // path direction at the object
    };

    struct Meas
    {
        double gap;        // bumper to the object's nearest in-lane point, along the path
        Eigen::Vector2d p; // mean of its in-lane points, odom
        bool beyond;
        bool truncated;
        double extent;
        Eigen::Vector2d dir;
    };

    static int flags(const Track &tr)
    {
        return (tr.moved ? 1 : 0) | (tr.inside ? 2 : 0) | (tr.beyond ? 4 : 0) | (tr.truncated ? 8 : 0);
    }

    static double along(const Track &tr) { return tr.vel.dot(tr.dir); }

    // connected objects: 8-neighbour components of the occupied voxel cells,
    // as lists of point indices
    std::vector<std::vector<size_t>> objects(const std::vector<Eigen::Vector3d> &pts) const
    {
        using Cell = std::pair<int, int>;
        std::map<Cell, std::vector<size_t>> cells;
        for (size_t i = 0; i < pts.size(); ++i)
            cells[{static_cast<int>(std::floor(pts[i].x() / voxel_m_)),
                   static_cast<int>(std::floor(pts[i].y() / voxel_m_))}]
                .push_back(i);

        std::vector<std::vector<size_t>> out;
        std::map<Cell, bool> done;
        for (const auto &[seed, _] : cells)
        {
            if (done[seed])
                continue;
            std::vector<size_t> obj;
            std::vector<Cell> stack{seed};
            done[seed] = true;
            while (!stack.empty())
            {
                const Cell c = stack.back();
                stack.pop_back();
                const auto &idx = cells.at(c);
                obj.insert(obj.end(), idx.begin(), idx.end());
                for (int dx = -1; dx <= 1; ++dx)
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        const Cell n{c.first + dx, c.second + dy};
                        if (cells.count(n) && !done[n])
                        {
                            done[n] = true;
                            stack.push_back(n);
                        }
                    }
            }
            out.push_back(std::move(obj));
        }
        return out;
    }

    // the planned path in odom, extended straight past its end so a lead
    // beyond the perception window still projects onto "the lane ahead";
    // s_end is the arc length where the perceived part ends
    av::geom::Polyline lane_path(double &s_end) const
    {
        av::geom::Polyline p;
        for (const auto &ps : path_->poses)
            p.push_back({ps.pose.position.x, ps.pose.position.y});

        s_end = 0.0;
        for (size_t i = 1; i < p.size(); ++i)
            s_end += (p[i] - p[i - 1]).norm();
        if (p.size() >= 2)
        {
            Eigen::Vector2d d = (p.back() - p[p.size() - 2]).normalized();
            p.push_back(p.back() + d * (filt_.max_range_m + 10.0));
        }
        return p;
    }

    bool accepted(const Track &tr) const { return !require_motion_ || tr.inside || tr.moved; }

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

        double s_end = 0.0;
        const auto path = lane_path(s_end);
        if (path.size() < 2)
            return;

        const double s_ego = av::geom::project_point(path, {ego.x, ego.y}).s;

        // every object with points in the lane ahead; gap by arc length along
        // the planned path to its nearest in-lane point
        std::vector<Meas> meas;
        const auto pts = av::cloud::cloud_points_base(cloud, mount_x_, mount_z_, filt_);
        const double edge = voxel_m_; // this close to a crop limit = cut by it

        for (const auto &obj : objects(pts))
        {
            if (static_cast<int>(obj.size()) < min_points_)
                continue;
            double x0 = std::numeric_limits<double>::max(), x1 = -x0, y0 = x0, y1 = -x0;
            bool truncated = false;
            double s_min = std::numeric_limits<double>::max();
            Eigen::Vector2d sum{0, 0};
            int n_in = 0;
            for (size_t i : obj)
            {
                const Eigen::Vector3d &p = pts[i];
                x0 = std::min(x0, p.x());
                x1 = std::max(x1, p.x());
                y0 = std::min(y0, p.y());
                y1 = std::max(y1, p.y());
                const double r = (p - Eigen::Vector3d(mount_x_, 0.0, mount_z_)).norm();
                truncated = truncated || r > filt_.max_range_m - edge || std::abs(p.y()) > filt_.max_lateral_m - edge;

                const Eigen::Vector2d q_odom = ego.odom_of({p.x(), p.y()});
                const auto pr = av::geom::project_point(path, q_odom);
                if (std::abs(pr.lateral) > lane_half_width_ || pr.s <= s_ego)
                    continue;
                s_min = std::min(s_min, pr.s);
                sum += q_odom;
                ++n_in;
            }
            if (n_in == 0)
                continue;
            const double h = av::geom::heading_at(path, s_min);
            meas.push_back({s_min - s_ego - bumper_x_, sum / n_in, s_min > s_end, truncated,
                            std::max(x1 - x0, y1 - y0), {std::cos(h), std::sin(h)}});
        }
        // when the track table is full, the nearest objects get the tracks
        std::sort(meas.begin(), meas.end(), [](const Meas &a, const Meas &b)
                  { return a.gap < b.gap; });

        update_tracks(meas, t, v_ego);
        publish(t, v_ego);
    }

    void update_tracks(const std::vector<Meas> &meas, const rclcpp::Time &t, double v_ego)
    {
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track &tr)
                                     { return (t - tr.t).seconds() > lost_after_; }),
                      tracks_.end());

        // greedy nearest-neighbour association in odom, against each track's
        // predicted position
        std::vector<std::tuple<double, size_t, size_t>> pairs;
        for (size_t i = 0; i < tracks_.size(); ++i)
        {
            const Eigen::Vector2d pred = predicted(tracks_[i], t);
            for (size_t j = 0; j < meas.size(); ++j)
            {
                const double d = (meas[j].p - pred).norm();
                if (d <= assoc_gate_)
                    pairs.emplace_back(d, i, j);
            }
        }
        std::sort(pairs.begin(), pairs.end());
        std::vector<bool> track_used(tracks_.size(), false), meas_used(meas.size(), false);
        for (const auto &[d, i, j] : pairs)
        {
            if (track_used[i] || meas_used[j])
                continue;
            track_used[i] = meas_used[j] = true;
            update(tracks_[i], meas[j], t, v_ego);
        }

        const size_t n_old = tracks_.size();
        for (size_t j = 0; j < meas.size(); ++j)
        {
            if (meas_used[j] || tracks_.size() >= static_cast<size_t>(std::max(1, max_tracks_)))
                continue;
            // new object: assume it is standing still until the filters learn
            // otherwise - the conservative guess for the cap
            Track tr;
            tr.gap = meas[j].gap;
            tr.rate = -v_ego;
            tr.t = t;
            tr.pos = meas[j].p;
            tr.beyond = meas[j].beyond;
            tr.truncated = meas[j].truncated;
            tr.extent = meas[j].extent;
            tr.dir = meas[j].dir;
            tr.inside = !meas[j].beyond;
            // an object that jumped out of an accepted track's gate (the lead
            // merged with or split from a neighbour, or a short reading)
            // inherits that track's standing, so a stopped lead past the
            // perceived path is not dropped
            for (size_t i = 0; i < n_old; ++i)
            {
                const Track &old = tracks_[i];
                if (!track_used[i] && (old.moved || old.inside) &&
                    (meas[j].p - predicted(old, t)).norm() <= 2.0 * assoc_gate_)
                {
                    tr.moved = tr.moved || old.moved;
                    tr.inside = tr.inside || old.inside;
                }
            }
            tracks_.push_back(tr);
        }
    }

    static Eigen::Vector2d predicted(const Track &tr, const rclcpp::Time &t)
    {
        return tr.pos + tr.vel * (t - tr.t).seconds();
    }

    void update(Track &tr, const Meas &m, const rclcpp::Time &t, double v_ego)
    {
        const double dt = std::max(1e-3, (t - tr.t).seconds());
        const double pred = tr.gap + tr.rate * dt;
        const double r = m.gap - pred;
        if (std::abs(r) <= gate_)
        {
            tr.gap = pred + alpha_ * r;
            tr.rate += beta_ * r / dt;
        }
        else
        {
            // same object, but its gap jumped: restart the gap estimate as
            // standing still, as for a new object
            tr.gap = m.gap;
            tr.rate = -v_ego;
        }

        const Eigen::Vector2d pp = tr.pos + tr.vel * dt;
        const Eigen::Vector2d rp = m.p - pp;
        tr.pos = pp + alpha_ * rp;
        tr.vel += beta_ * rp / dt;

        tr.beyond = m.beyond;
        tr.truncated = m.truncated;
        tr.extent = m.extent;
        tr.dir = m.dir;

        // moving along the road (either way), as a whole vehicle-sized object
        const double ts = t.seconds();
        if (std::abs(along(tr)) > moving_speed_ && !tr.truncated && tr.extent <= max_vehicle_extent_)
        {
            if (tr.moving_since < 0.0)
                tr.moving_since = ts;
            if (ts - tr.moving_since >= moving_confirm_)
                tr.moved = true;
        }
        else
            tr.moving_since = -1.0;

        tr.inside = tr.inside || !m.beyond;
        tr.t = t;
    }

    // nearest track by gap, optionally only among accepted ones
    const Track *nearest(bool only_accepted) const
    {
        const Track *best = nullptr;
        for (const auto &tr : tracks_)
            if ((!only_accepted || accepted(tr)) && (!best || tr.gap < best->gap))
                best = &tr;
        return best;
    }

    void publish(const rclcpp::Time &t, double v_ego)
    {
        VelocityConstraint c;
        c.header.stamp = t;
        c.source = "acc";
        c.priority = 3;
        c.valid_until = t + rclcpp::Duration::from_seconds(validity_);
        c.confidence = 1.0;
        const Track *lead = nearest(true);
        const Track *cand = nearest(false);
        double lead_v = 0.0, cap = 1e3;
        if (lead)
        {
            lead_v = std::max(0.0, v_ego + lead->rate);
            const double desired_gap = standstill_gap_ + time_gap_ * v_ego;
            cap = lead->gap < min_gap_ ? 0.0
                                       : std::max(0.0, lead_v + gap_gain_ * (lead->gap - desired_gap));
            char why[96];
            std::snprintf(why, sizeof(why), "lead %.1f m at %.1f m/s, want %.1f m", lead->gap, lead_v, desired_gap);
            c.reason = why;
        }
        else if (cand)
        {
            char why[96];
            std::snprintf(why, sizeof(why), "no lead (stationary object %.1f m ahead ignored)", cand->gap);
            c.reason = why;
        }
        else
            c.reason = "no lead";
        c.maximum_speed_mps = cap;
        pub_constraint_->publish(c);

        std_msgs::msg::Float64MultiArray st;
        st.data = {lead ? lead->gap : -1.0, lead_v, cap, lead ? 1.0 : 0.0,
                   cand ? cand->gap : -1.0, cand ? along(*cand) : 0.0,
                   cand ? double(flags(*cand)) : 0.0, double(tracks_.size()),
                   lead ? double(flags(*lead)) : 0.0,
                   lead ? along(*lead) : 0.0,
                   lead ? lead->dir.x() * lead->vel.y() - lead->dir.y() * lead->vel.x() : 0.0,
                   cand ? cand->extent : 0.0, lead ? lead->extent : 0.0};
        pub_state_->publish(st);
        publish_markers(t, c.reason, lead);
    }

    void publish_markers(const rclcpp::Time &t, const std::string &text, const Track *lead)
    {
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = t;
        m.ns = "acc";
        m.lifetime = rclcpp::Duration::from_seconds(0.3);
        m.pose.orientation.w = 1.0;
        // the lead in blue, every other in-lane track in grey
        int id = 2;
        for (const auto &tr : tracks_)
        {
            const bool is_lead = &tr == lead;
            m.id = is_lead ? 0 : id++;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.pose.position.x = tr.pos.x();
            m.pose.position.y = tr.pos.y();
            m.pose.position.z = 0.75;
            m.scale.x = m.scale.y = is_lead ? 0.6 : 0.4;
            m.scale.z = 1.5;
            m.color.r = is_lead ? 0.0 : 0.6;
            m.color.g = is_lead ? 0.5 : 0.6;
            m.color.b = is_lead ? 1.0 : 0.6;
            m.color.a = is_lead ? 0.7 : 0.4;
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
    double mount_x_ = 2.30, mount_z_ = 0.60, voxel_m_ = 0.5, max_vehicle_extent_ = 7.5;
    int min_points_ = 3;
    av::cloud::CloudFilter filt_;
    double lane_half_width_ = 1.3, bumper_x_ = 2.25;
    double time_gap_ = 1.8, standstill_gap_ = 5.0, min_gap_ = 3.0, gap_gain_ = 0.3;
    double alpha_ = 0.4, beta_ = 0.1, gate_ = 3.0, lost_after_ = 0.5, validity_ = 0.3, input_stale_ = 0.5;
    double assoc_gate_ = 2.0, moving_speed_ = 1.0, moving_confirm_ = 1.0;
    bool require_motion_ = true;
    int max_tracks_ = 32;

    std::vector<Track> tracks_;
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