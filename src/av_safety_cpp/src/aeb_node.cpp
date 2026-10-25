#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "av_common/projection.hpp"
#include "av_safety_cpp/braking_model.hpp"
#include "av_safety_cpp/collision_corridor.hpp"
#include "av_safety_cpp/depth_obstacle_extractor.hpp"
#include <av_feature_msgs/msg/aeb_status.hpp>

using av_feature_msgs::msg::AebStatus;

enum class AebState : uint8_t
{
    INACTIVE = AebStatus::INACTIVE,
    MONITORING = AebStatus::MONITORING,
    WARNING = AebStatus::WARNING,
    BRAKING = AebStatus::BRAKING,
    STOP_HOLD = AebStatus::STOP_HOLD,
    FAULT = AebStatus::FAULT,
};

class AebNode : public rclcpp::Node
{
public:
    AebNode() : rclcpp::Node("aeb_node")
    {
        // camera extrinsic/intrinsic - mirrors sedan.urdf.xacro, the same
        // duplication discipline every av_perception_cpp node follows
        double cam_x = declare_parameter("cam_x", 2.2);
        double cam_z = declare_parameter("cam_z", 1.4);
        double cam_pitch = declare_parameter("cam_pitch", 0.06);
        int img_w = declare_parameter("image_width", 424);
        int img_h = declare_parameter("image_height", 300);
        double hfov = declare_parameter("horizontal_fov", 1.9);

        T_base_cam_ = av::proj::make_T_base_cam(cam_x, 0.0, cam_z, cam_pitch);
        double fx = (img_w / 2.0) / std::tan(hfov / 2.0); // derived, never read from camera_info (ground rule)
        pin_ = {fx, fx, img_w / 2.0, img_h / 2.0};

        fp_.min_depth_m = declare_parameter("min_depth_m", 0.5);
        fp_.max_depth_m = declare_parameter("max_depth_m", 40.0);
        fp_.max_lateral_m = declare_parameter("max_lateral_m", 5.0);
        fp_.min_height_m = declare_parameter("min_obstacle_height_m", 0.15);
        fp_.max_height_m = declare_parameter("max_obstacle_height_m", 3.0);
        fp_.ground_removal_m = declare_parameter("ground_removal_threshold_m", 0.1);
        fp_.cluster_voxel_m = declare_parameter("cluster_voxel_m", 0.5);
        fp_.min_cluster_points = declare_parameter("minimum_cluster_points", 5);

        pred_t_ = declare_parameter("prediction_time_sec", 3.0);
        pred_dt_ = declare_parameter("prediction_dt_sec", 0.1);
        min_lookahead_ = declare_parameter("min_corridor_lookahead_m", 5.0);
        mpc_path_timeout_ = declare_parameter("mpc_path_timeout_sec", 0.3);

        foot_.vehicle_length_m = declare_parameter("vehicle_length_m", 4.5);
        foot_.vehicle_width_m = declare_parameter("vehicle_width_m", 1.8);
        foot_.lateral_margin_m = declare_parameter("lateral_margin_m", 0.3);
        foot_.longitudinal_margin_m = declare_parameter("longitudinal_margin_m", 0.5);

        t_response_ = declare_parameter("response_time_sec", 0.3);
        a_decel_ = declare_parameter("ego_emergency_decel_mps2", 6.0);
        d_offset_ = declare_parameter("distance_offset_m", 1.0);

        warning_ttc_ = declare_parameter("warning_ttc_sec", 2.5);
        brake_ttc_ = declare_parameter("brake_ttc_sec", 1.5);
        collision_hold_ = declare_parameter("collision_hold_sec", 1.0);
        clear_hysteresis_ = declare_parameter("clear_hysteresis_sec", 0.5);
        persistence_ticks_ = declare_parameter("persistence_ticks", 3);
        stop_hold_speed_ = declare_parameter("stop_hold_speed_mps", 0.1);

        rate_hz_ = declare_parameter("aeb_rate_hz", 20.0);
        odom_stale_ = declare_parameter("odom_stale_sec", 0.3);
        depth_stale_ = declare_parameter("depth_stale_sec", 0.5);

        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/front/depth/image_raw", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr m)
            { depth_ = m; depth_t_ = now(); });

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom_ekf", 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr m)
            { odom_ = m; odom_t_ = now(); });

        sub_mpc_path_ = create_subscription<nav_msgs::msg::Path>(
            "/mpc_predicted_path", 10,
            [this](nav_msgs::msg::Path::ConstSharedPtr m)
            { mpc_path_ = m; mpc_path_t_ = now(); });

        // Remap-friendly handle on the nominal controller's last command, so
        // BRAKING can preserve the current yaw rate instead of zeroing it
        // mid-curve. Today this is remapped to /cmd_vel
        // (MPC still publishes there directly); Later plan moves the remap target
        // to /control/mpc_cmd when the arbiter takes over /cmd_vel - this
        // node's code does not change between the two.
        sub_nominal_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
            "nominal_cmd", 10,
            [this](geometry_msgs::msg::Twist::ConstSharedPtr m)
            { last_nominal_angular_ = m->angular.z; });

        pub_status_ = create_publisher<AebStatus>("/safety/aeb_status", 10);
        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/control/aeb_cmd", 10);
        pub_obstacles_ = create_publisher<visualization_msgs::msg::MarkerArray>("/safety/aeb/debug_obstacles", 10);
        pub_corridor_ = create_publisher<visualization_msgs::msg::MarkerArray>("/safety/aeb/debug_corridor", 10);

        // /safety/aeb/debug_image: front RGB + swept corridor, clusters, d_safe
        // line and state banner. The RGB subscription exists only for this
        // view - no safety decision ever reads it.
        debug_image_ = declare_parameter("debug_image", true);
        if (debug_image_)
        {
            sub_rgb_ = create_subscription<sensor_msgs::msg::Image>(
                "/camera/front/image_raw", rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Image::ConstSharedPtr m)
                { rgb_ = m; });
            // BEST_EFFORT, depth 1, same reason as lane_node's /lane/debug_image:
            // a slow reliable RViz reader must never back up into tick()
            pub_debug_image_ = create_publisher<sensor_msgs::msg::Image>(
                "/safety/aeb/debug_image", rclcpp::QoS(1).best_effort());
        }

        auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]()
                                   { tick(); });
    }

private:
    double seconds_clear() const
    {
        return clear_count_ / std::max(1.0, rate_hz_);
    }

    std::string reason_string() const
    {
        switch (state_)
        {
        case AebState::INACTIVE:
            return "inactive";
        case AebState::MONITORING:
            return "monitoring, clear";
        case AebState::WARNING:
            return "warning: ttc or distance below soft threshold";
        case AebState::BRAKING:
            return "braking: ttc or distance below hard threshold";
        case AebState::STOP_HOLD:
            return "stopped, holding";
        case AebState::FAULT:
            return "fault: stale or missing input";
        }
        return "";
    }

    // Transition table. persist_count_ gates ENTRY into
    // WARNING/BRAKING (do not trigger from one noisy frame); clear_count_
    // gates EXIT back toward MONITORING (do not release the instant a cluster
    // flickers out for a frame - "use hold time and release hysteresis").
    //
    // STOP_HOLD releases on its own counter, corridor_clear_count_, which
    // counts ticks with NO obstacle in the corridor at all. At standstill TTC
    // is huge and d_safe is just distance_offset_m, so "no concern" is true
    // with an obstacle 1.5 m ahead - releasing on that would hand control back
    // to the nominal controller in front of the obstacle.
    void step_state(bool risk_now, bool warn_now, bool obstacle_in_corridor, double v)
    {
        bool any_concern = risk_now || warn_now;
        persist_count_ = any_concern ? persist_count_ + 1 : 0;
        clear_count_ = any_concern ? 0 : clear_count_ + 1;
        corridor_clear_count_ = obstacle_in_corridor ? 0 : corridor_clear_count_ + 1;

        switch (state_)
        {
        case AebState::INACTIVE:

        case AebState::FAULT:
            // A fault interrupts a latched intervention but never ends it:
            // resume BRAKING/STOP_HOLD (counters untouched) so a single late
            // depth frame cannot release a hold with the obstacle still there.
            state_ = state_before_fault_.value_or(AebState::MONITORING);
            state_before_fault_.reset();
            break;

        case AebState::MONITORING:
            if (risk_now && persist_count_ >= persistence_ticks_)
                state_ = AebState::BRAKING;
            else if (warn_now && persist_count_ >= persistence_ticks_)
                state_ = AebState::WARNING;
            break;

        case AebState::WARNING:
            if (risk_now && persist_count_ >= persistence_ticks_)
                state_ = AebState::BRAKING;
            else if (seconds_clear() >= clear_hysteresis_)
                state_ = AebState::MONITORING;
            break;

        case AebState::BRAKING:
            if (v < stop_hold_speed_)
                state_ = AebState::STOP_HOLD;
            // else: stays BRAKING. Release is only ever via STOP_HOLD's own
            // timer below - never directly back to MONITORING while still
            // moving ("do not automatically release AEB while the vehicle is
            // still moving quickly").
            break;

        case AebState::STOP_HOLD:
            if (corridor_clear_count_ / std::max(1.0, rate_hz_) > collision_hold_)
                state_ = AebState::MONITORING;
            break;
        }
    }

    void publish_fault(const std::string &reason)
    {
        if (state_ == AebState::BRAKING || state_ == AebState::STOP_HOLD)
            state_before_fault_ = state_;
        state_ = AebState::FAULT;
        AebStatus st;
        st.header.stamp = now();
        st.state = AebStatus::FAULT;
        st.emergency_active = false;
        st.obstacle_distance_m = -1.0; // no valid measurement, not "obstacle at 0 m"
        st.time_to_collision_sec = 1e9;
        st.reason = reason;
        pub_status_->publish(st);
        pub_cmd_->publish(geometry_msgs::msg::Twist{}); // zero cmd on fault

        DebugOverlay o;
        o.reason = reason;
        publish_debug_image(o);
    }

    void tick()
    {
        if (!odom_ || !depth_)
        {
            publish_fault("waiting for odom and depth");
            return;
        }
        if ((now() - odom_t_).seconds() > odom_stale_)
        {
            publish_fault("stale odom");
            return;
        }
        if ((now() - depth_t_).seconds() > depth_stale_)
        {
            publish_fault("stale depth");
            return;
        }

        double v = std::hypot(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y);
        double yaw_rate = odom_->twist.twist.angular.z;

        auto clusters = av::safety::extract_clusters(depth_, pin_, T_base_cam_, fp_);

        // Path A: MPC prediction, transformed odom -> base_link using THIS
        // tick's ego pose (good enough at AEB's few-metre, sub-second
        // horizon; the predicted path itself is only ~N*dt seconds old).

        av::geom::Polyline mpc_path_base;
        if (mpc_path_ && (now() - mpc_path_t_).seconds() < mpc_path_timeout_)
        {
            const auto &q = odom_->pose.pose.orientation;
            av::proj::Pose2d ego_pose{
                odom_->pose.pose.position.x,
                odom_->pose.pose.position.y,
                std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};

            for (const auto &pose_stamped : mpc_path_->poses)
            {
                const auto &p = pose_stamped.pose.position;
                Eigen::Vector2d p_base = ego_pose.base_of({p.x, p.y});
                mpc_path_base.push_back(p_base);
            }
        }

        // Path B: independent kinematic prediction - always available, never
        // depends on the MPC being healthy.
        // Never shorter than min_corridor_lookahead_m past the bumper +
        // longitudinal margin: the path starts at base_link (chassis center),
        // and check_corridor measures from half_l ahead of it.
        double half_l = foot_.vehicle_length_m / 2.0 + foot_.longitudinal_margin_m;
        auto kin_path = av::safety::kinematic_prediction(v, yaw_rate, pred_t_, pred_dt_,
                                                         half_l + min_lookahead_);

        av::safety::CorridorResult best;
        for (const auto *path : {&mpc_path_base, &kin_path})
        {
            if (path->size() < 2)
                continue;
            auto result = av::safety::check_corridor(*path, clusters, foot_);
            if (result.obstacle_in_corridor && result.nearest_distance_m < best.nearest_distance_m)
                best = result;
        }

        double ttc = best.obstacle_in_corridor
                         ? av::safety::time_to_collision(best.nearest_distance_m, v)
                         : 1e9;

        double d_safe = av::safety::safe_braking_distance(v, t_response_, a_decel_, d_offset_);

        bool risk_now = best.obstacle_in_corridor && ttc < brake_ttc_ && best.nearest_distance_m < d_safe;
        bool warn_now = best.obstacle_in_corridor && (ttc < warning_ttc_ || best.nearest_distance_m < d_safe * 1.5);

        step_state(risk_now, warn_now, best.obstacle_in_corridor, v);

        AebStatus st;
        st.header.stamp = now();
        st.state = static_cast<uint8_t>(state_);
        st.emergency_active = (state_ == AebState::BRAKING || state_ == AebState::STOP_HOLD);
        st.obstacle_distance_m = best.obstacle_in_corridor ? best.nearest_distance_m : -1.0;
        st.relative_speed_mps = v;
        st.time_to_collision_sec = std::min(ttc, 1e9);
        st.safe_braking_distance_m = d_safe;
        st.requested_deceleration_mps2 = (state_ == AebState::BRAKING || state_ == AebState::STOP_HOLD) ? a_decel_ : 0.0;
        // per-frame cluster index, not a tracked identity - it can change every tick
        st.obstacle_id = best.obstacle_in_corridor ? "cluster_" + std::to_string(best.nearest_obstacle_id) : "";
        st.reason = reason_string();
        pub_status_->publish(st);

        geometry_msgs::msg::Twist cmd{};
        // preserve yaw rate while braking, zero only longitudinal; once stopped
        // the Ackermann plugin cannot yaw, so a passed-through rate would only
        // swing the wheels to lock while the car sits still
        if (state_ == AebState::BRAKING)
        {
            cmd.angular.z = last_nominal_angular_;
        }
        pub_cmd_->publish(cmd);

        publish_debug_markers(clusters);

        DebugOverlay o;
        o.clusters = &clusters;
        o.mpc_path = &mpc_path_base;
        o.kin_path = &kin_path;
        o.best = best;
        o.ttc = ttc;
        o.d_safe = d_safe;
        o.v = v;
        o.reason = st.reason;
        publish_debug_image(o);
    }

    // Everything the debug image draws from one tick. Pointers are only valid
    // for the duration of publish_debug_image().
    struct DebugOverlay
    {
        const std::vector<av::safety::ObstacleCluster> *clusters = nullptr;
        const av::geom::Polyline *mpc_path = nullptr; // base_link, may be empty (stale MPC)
        const av::geom::Polyline *kin_path = nullptr; // base_link
        av::safety::CorridorResult best;
        double ttc = 1e9, d_safe = 0.0, v = 0.0;
        std::string reason;
    };

    static const char *state_name(AebState s)
    {
        switch (s)
        {
        case AebState::INACTIVE:
            return "INACTIVE";
        case AebState::MONITORING:
            return "MONITORING";
        case AebState::WARNING:
            return "WARNING";
        case AebState::BRAKING:
            return "BRAKING";
        case AebState::STOP_HOLD:
            return "STOP_HOLD";
        case AebState::FAULT:
            return "FAULT";
        }
        return "?";
    }

    static cv::Scalar state_color(AebState s) // rgb8
    {
        switch (s)
        {
        case AebState::MONITORING:
            return {40, 200, 80};
        case AebState::WARNING:
            return {255, 165, 0};
        case AebState::BRAKING:
            return {255, 40, 40};
        case AebState::STOP_HOLD:
            return {200, 40, 220};
        case AebState::FAULT:
            return {255, 230, 0};
        default:
            return {160, 160, 160};
        }
    }

    std::optional<cv::Point> to_pixel(double x, double y, double z) const
    {
        auto uv = av::proj::base_to_pixel(pin_, T_base_cam_, {x, y, z});
        if (!uv || std::abs(uv->x()) > 1e4 || std::abs(uv->y()) > 1e4)
            return std::nullopt;
        return cv::Point(static_cast<int>(std::lround(uv->x())), static_cast<int>(std::lround(uv->y())));
    }

    // Ground-plane outline of the swept corridor along `path`, from arc length
    // s0 (bumper + longitudinal margin, where check_corridor's distances start)
    // to the path end, half_w either side - the region check_corridor tests.
    std::vector<cv::Point> corridor_polygon(const av::geom::Polyline &path, double s0, double half_w) const
    {
        std::vector<cv::Point> left, right;
        double s = 0.0;
        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            Eigen::Vector2d d = path[i + 1] - path[i];
            double len = d.norm();
            if (len < 1e-9)
                continue;
            Eigen::Vector2d t = d / len, n(-t.y(), t.x());
            auto add = [&](const Eigen::Vector2d &c)
            {
                auto l = to_pixel(c.x() + n.x() * half_w, c.y() + n.y() * half_w, 0.0);
                auto r = to_pixel(c.x() - n.x() * half_w, c.y() - n.y() * half_w, 0.0);
                if (l && r)
                {
                    left.push_back(*l);
                    right.push_back(*r);
                }
            };
            if (s + len >= s0)
            {
                if (left.empty())
                    add(path[i] + t * std::max(0.0, s0 - s));
                add(path[i + 1]);
            }
            s += len;
        }
        if (left.size() < 2)
            return {};
        left.insert(left.end(), right.rbegin(), right.rend());
        return left;
    }

    // point + unit tangent at arc length s along path (nullopt past its end)
    static std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> point_at(
        const av::geom::Polyline &path, double s)
    {
        double acc = 0.0;
        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            Eigen::Vector2d d = path[i + 1] - path[i];
            double len = d.norm();
            if (len < 1e-9)
                continue;
            if (acc + len >= s)
                return std::make_pair(Eigen::Vector2d(path[i] + d * ((s - acc) / len)), Eigen::Vector2d(d / len));
            acc += len;
        }
        return std::nullopt;
    }

    // /safety/aeb/debug_image. Purely a view: at most once per new RGB frame,
    // and any failure here is logged and swallowed - drawing must never take
    // down the safety node.
    void publish_debug_image(const DebugOverlay &o)
    {
        if (!pub_debug_image_ || !rgb_ || rgb_ == last_debug_rgb_ || rgb_->encoding != "rgb8")
            return;
        last_debug_rgb_ = rgb_;
        try
        {
            cv::Mat img = cv::Mat(rgb_->height, rgb_->width, CV_8UC3,
                                  const_cast<uint8_t *>(rgb_->data.data()), rgb_->step)
                              .clone();
            const cv::Scalar col_state = state_color(state_);
            const cv::Scalar col_mpc(0, 220, 255), col_kin(255, 255, 255), col_dsafe(255, 230, 0),
                col_out(150, 220, 150), col_in(255, 140, 0), col_near(255, 30, 30);
            const double half_w = foot_.vehicle_width_m / 2.0 + foot_.lateral_margin_m;
            const double half_l = foot_.vehicle_length_m / 2.0 + foot_.longitudinal_margin_m;

            // swept corridors (translucent fill in the state colour, outline per source)
            std::vector<std::pair<std::vector<cv::Point>, cv::Scalar>> polys;
            if (o.kin_path)
                polys.push_back({corridor_polygon(*o.kin_path, half_l, half_w), col_kin});
            if (o.mpc_path)
                polys.push_back({corridor_polygon(*o.mpc_path, half_l, half_w), col_mpc});
            cv::Mat fill = img.clone();
            for (const auto &[poly, _] : polys)
                if (!poly.empty())
                    cv::fillPoly(fill, std::vector<std::vector<cv::Point>>{poly}, col_state, cv::LINE_AA);
            cv::addWeighted(fill, 0.28, img, 0.72, 0.0, img);
            for (const auto &[poly, col] : polys)
                if (!poly.empty())
                    cv::polylines(img, std::vector<std::vector<cv::Point>>{poly}, true, col, 1, cv::LINE_AA);

            // d_safe: where a cluster starts to count as a braking risk
            if (o.kin_path && o.d_safe > 0.0)
                if (auto pt = point_at(*o.kin_path, half_l + o.d_safe))
                {
                    const auto &[c, t] = *pt;
                    Eigen::Vector2d n(-t.y(), t.x());
                    auto a = to_pixel(c.x() + n.x() * half_w, c.y() + n.y() * half_w, 0.0);
                    auto b = to_pixel(c.x() - n.x() * half_w, c.y() - n.y() * half_w, 0.0);
                    if (a && b)
                    {
                        cv::line(img, *a, *b, col_dsafe, 2, cv::LINE_AA);
                        cv::putText(img, "d_safe", *b + cv::Point(4, 0), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                                    col_dsafe, 1, cv::LINE_AA);
                    }
                }

            // clusters: stalk from the ground to the cluster's mean height;
            // orange inside a corridor, red + distance for the nearest one
            // (drawn last, so the others never cover its label)
            int n_clusters = 0;
            if (o.clusters)
            {
                n_clusters = static_cast<int>(o.clusters->size());
                const int nearest_id = o.best.obstacle_in_corridor ? o.best.nearest_obstacle_id : -1;
                auto draw = [&](int i)
                {
                    const auto &c = (*o.clusters)[i];
                    bool nearest = i == nearest_id;
                    bool in_corridor = nearest;
                    for (const auto *p : {o.kin_path, o.mpc_path})
                        if (!in_corridor && p && p->size() >= 2 &&
                            av::safety::check_corridor(*p, {c}, foot_).obstacle_in_corridor)
                            in_corridor = true;
                    cv::Scalar col = nearest ? col_near : (in_corridor ? col_in : col_out);
                    auto g = to_pixel(c.centroid.x(), c.centroid.y(), 0.0);
                    auto top = to_pixel(c.centroid.x(), c.centroid.y(), std::max(0.3, c.centroid.z()));
                    if (!g || !top)
                        return;
                    cv::line(img, *g, *top, col, nearest ? 3 : 1, cv::LINE_AA);
                    cv::circle(img, *top, nearest ? 6 : 3, col, cv::FILLED, cv::LINE_AA);
                    if (nearest)
                    {
                        char lbl[32];
                        std::snprintf(lbl, sizeof(lbl), "%.1f m", o.best.nearest_distance_m);
                        cv::Point at = *top + cv::Point(-20, -12);
                        cv::putText(img, lbl, at, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
                        cv::putText(img, lbl, at, cv::FONT_HERSHEY_SIMPLEX, 0.55, col_near, 2, cv::LINE_AA);
                    }
                };
                for (int i = 0; i < n_clusters; ++i)
                    if (i != nearest_id)
                        draw(i);
                if (nearest_id >= 0 && nearest_id < n_clusters)
                    draw(nearest_id);
            }

            // banner: state + the numbers behind it
            cv::rectangle(img, {0, 0}, {img.cols, 42}, cv::Scalar(0, 0, 0), cv::FILLED);
            cv::putText(img, std::string("AEB ") + state_name(state_), {6, 17}, cv::FONT_HERSHEY_SIMPLEX,
                        0.55, col_state, 2, cv::LINE_AA);
            char buf[128], ttc_buf[16];
            std::snprintf(buf, sizeof(buf), "v %.1f m/s", o.v);
            cv::putText(img, buf, {img.cols - 90, 17}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            // which predicted paths fed the corridor this tick
            const char *paths = (o.mpc_path && o.mpc_path->size() >= 2) ? "mpc+kin" : "kin";
            if (o.ttc < 100.0)
                std::snprintf(ttc_buf, sizeof(ttc_buf), "%.2fs", o.ttc);
            else
                std::snprintf(ttc_buf, sizeof(ttc_buf), "inf");
            if (state_ == AebState::FAULT)
                std::snprintf(buf, sizeof(buf), "%s", o.reason.c_str());
            else if (o.best.obstacle_in_corridor)
                std::snprintf(buf, sizeof(buf), "dist %.2f  ttc %s  d_safe %.2f  [%s] %d cl",
                              o.best.nearest_distance_m, ttc_buf, o.d_safe, paths, n_clusters);
            else
                std::snprintf(buf, sizeof(buf), "corridor clear  d_safe %.2f  [%s] %d cl",
                              o.d_safe, paths, n_clusters);
            cv::putText(img, buf, {6, 35}, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(255, 255, 255), 1,
                        cv::LINE_AA);

            sensor_msgs::msg::Image out;
            out.header = rgb_->header;
            out.height = img.rows;
            out.width = img.cols;
            out.encoding = "rgb8";
            out.is_bigendian = 0;
            out.step = img.cols * 3;
            out.data.assign(img.data, img.data + out.step * img.rows);
            pub_debug_image_->publish(out);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "debug image skipped: %s", e.what());
        }
    }

    void publish_debug_markers(const std::vector<av::safety::ObstacleCluster> &clusters)
    {
        visualization_msgs::msg::MarkerArray obs_array;
        int id = 0;

        for (const auto &c : clusters)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "base_link";
            m.header.stamp = now();
            m.ns = "aeb_obstacles";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = c.centroid.x();
            m.pose.position.y = c.centroid.y();
            m.pose.position.z = c.centroid.z();
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.4;
            m.scale.y = 0.4;
            m.scale.z = 0.4;
            bool in_risk_state = (state_ == AebState::BRAKING || state_ == AebState::WARNING);
            m.color.r = in_risk_state ? 1.0 : 0.2;
            m.color.g = in_risk_state ? 0.2 : 0.8;
            m.color.b = 0.2;
            m.color.a = 0.8;
            m.lifetime = rclcpp::Duration::from_seconds(0.3);
            obs_array.markers.push_back(m);
        }
        pub_obstacles_->publish(obs_array);

        visualization_msgs::msg::MarkerArray corridor_array;
        visualization_msgs::msg::Marker box;
        box.header.frame_id = "base_link";
        box.header.stamp = now();
        box.ns = "aeb_corridor";
        box.id = 0;
        box.type = visualization_msgs::msg::Marker::CUBE;
        double len = foot_.vehicle_length_m + 2 * foot_.longitudinal_margin_m;
        box.pose.position.x = len / 2.0;
        box.pose.orientation.w = 1.0;
        box.scale.x = len;
        box.scale.y = foot_.vehicle_width_m + 2 * foot_.lateral_margin_m;
        box.scale.z = 0.05;
        box.color.r = (state_ == AebState::BRAKING) ? 1.0 : (state_ == AebState::WARNING ? 1.0 : 0.2);
        box.color.g = (state_ == AebState::BRAKING) ? 0.0 : (state_ == AebState::WARNING ? 0.6 : 0.8);
        box.color.b = 0.2;
        box.color.a = 0.25;
        box.lifetime = rclcpp::Duration::from_seconds(0.3);
        corridor_array.markers.push_back(box);
        pub_corridor_->publish(corridor_array);
    }

private:
    Eigen::Isometry3d T_base_cam_;
    av::proj::Pinhole pin_;
    av::safety::ObstacleFilterParams fp_;
    av::safety::FootprintParams foot_;
    double pred_t_ = 3.0, pred_dt_ = 0.1, min_lookahead_ = 5.0, mpc_path_timeout_ = 0.3;
    double t_response_ = 0.3, a_decel_ = 6.0, d_offset_ = 1.0;
    double warning_ttc_ = 2.5, brake_ttc_ = 1.5, collision_hold_ = 1.0,
           clear_hysteresis_ = 0.5, stop_hold_speed_ = 0.1;
    int persistence_ticks_ = 3;
    double rate_hz_ = 20.0, odom_stale_ = 0.3, depth_stale_ = 0.5;

    AebState state_ = AebState::INACTIVE;
    std::optional<AebState> state_before_fault_; // latched BRAKING/STOP_HOLD interrupted by a fault
    int persist_count_ = 0, clear_count_ = 0, corridor_clear_count_ = 0;
    double last_nominal_angular_ = 0.0;
    bool debug_image_ = true;

    sensor_msgs::msg::Image::ConstSharedPtr depth_;
    sensor_msgs::msg::Image::ConstSharedPtr rgb_, last_debug_rgb_; // debug image only
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    nav_msgs::msg::Path::ConstSharedPtr mpc_path_;
    rclcpp::Time depth_t_{0, 0, RCL_ROS_TIME}, odom_t_{0, 0, RCL_ROS_TIME}, mpc_path_t_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_, sub_rgb_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_mpc_path_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nominal_cmd_;

    rclcpp::Publisher<AebStatus>::SharedPtr pub_status_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_, pub_corridor_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AebNode>());
    rclcpp::shutdown();
    return 0;
}