#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "av_common/projection.hpp"
#include "av_common/debug_tap.hpp"
#include "av_common/geometry.hpp"
#include "av_perception_cpp/img_access.hpp"
#include "av_perception_cpp/ground_cal.hpp"

#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <fstream>

// ---------------------------------------------------------------- namespace helpers
namespace
{
    // Analytic expected ground distance for image row v, from the SAME
    // ground-plane model as ground_cal.hpp's estimate_pitch -- used here in
    // reverse, to decide sampling density for a row BEFORE any depth pixel in
    // that row has been read.
    // read documentation ground_plane_theory_and_dervations.md for derivation
    double expected_ground_distance(int v, const av::proj::Pinhole &pin, double cam_z, double pitch)
    {
        double s = std::sin(pitch) + ((v - pin.cy) / pin.fy) * std::cos(pitch); // same model as estimate_pitch
        if (s <= 1e-3)
            return 1e9;   // row points above the horizon -- treat as "far"
        return cam_z / s; // solved for distance instead of pitch
    }

    struct ChainPt
    {
        double x, y;
        bool yellow;
    }; // one ego-frame point, color-tagged
    using Chain = std::vector<ChainPt>; // one lane chain, in ego-frame coordinates

    Chain grid_dedup(const Chain &pts, double cell_m) // grid-based deduplication of points, keeping only one point per cell
    {
        struct cell
        {
            double sx = 0, sy = 0;
            int n = 0;
            bool yellow = false;
        };

        std::unordered_map<uint64_t, cell> grid; // sparse grid of cells, keyed by packed (ix,iy)
        auto key = [&](double x, double y) -> uint64_t
        {
            int32_t gx = static_cast<int32_t>(std::floor(x / cell_m));
            int32_t gy = static_cast<int32_t>(std::floor(y / cell_m));
            // pack both int32 cell indices into one 64-bit key. Cast through
            // uint32_t first: shifting a negative signed value is UB before
            // C++20, and this TU is built as C++17.
            return (static_cast<uint64_t>(static_cast<uint32_t>(gx)) << 32) |
                   static_cast<uint32_t>(gy);
        };

        for (const auto &pt : pts)
        {
            auto &c = grid[key(pt.x, pt.y)];
            c.sx += pt.x;
            c.sy += pt.y;
            c.n++;
            c.yellow = c.yellow || pt.yellow;
        }

        Chain out;
        for (const auto &[k, c] : grid)
        {
            out.push_back({c.sx / c.n, c.sy / c.n, c.yellow}); // average of all points in the cell
        }
        return out;
    }

    struct StopLineHit
    {
        bool found = false;
        double x = 0, y_center = 0; // where the bar sits, if any
    };

    StopLineHit detect_and_strip_stop_line(Chain &gridded, double x_max_m)
    {
        StopLineHit res;
        std::map<int, std::pair<double, double>> y_range; // 0.5 m x-bin -> (ymin, ymax
        std::map<int, int> counts;

        for (const auto &p : gridded)
        {
            if (p.yellow || p.x > x_max_m || p.x < 0.0)
                continue;
            int b = static_cast<int>(std::floor(p.x / 0.5)); // which 0.5m x-bin this point falls in
            auto it = y_range.find(b);

            if (it == y_range.end())     // first point in this bin
                y_range[b] = {p.y, p.y}; // min = max = p.y
            else
            {
                it->second.first = std::min(it->second.first, p.y);   // widen the bin's y-min
                it->second.second = std::max(it->second.second, p.y); // widen the bin's y-max
            }
            counts[b]++; // one more point in this bin
        }

        for (const auto &[b, yr] : y_range)
        {
            double span = yr.second - yr.first;
            if (span >= 2.0 && yr.first < 0.3 && yr.second > -0.3 && counts[b] >= 6)
            {
                res = {true, (b + 0.5) * 0.5, (yr.first + yr.second) / 2.0};
                break;
            }
        }

        if (res.found)
        {
            gridded.erase(std::remove_if(gridded.begin(), gridded.end(),
                                         [&](const ChainPt &p)
                                         { return std::abs(p.x - res.x) < 0.35; }),
                          gridded.end()); // strip the bar's points
        }

        return res;
    }
    std::vector<Chain> cluster_chains(Chain pts, double link_gap_m)
    {
        std::vector<Chain> chains;                 // compleleted Chains
        std::vector<bool> used(pts.size(), false); // which points have already been claimed.
        for (;;)
        {
            int seed = -1;
            double best_d2 = 1e18;

            for (size_t i = 0; i < pts.size(); i++)
            {
                if (used[i])
                    continue;
                double d2 = pts[i].x * pts[i].x + pts[i].y * pts[i].y;
                if (d2 < best_d2)
                {
                    best_d2 = d2;
                    seed = static_cast<int>(i);
                }
            }
            if (seed < 0)
                break;

            Chain chain{pts[seed]}; // start a new chain
            used[seed] = true;

            for (;;)
            {
                const auto &tail = chain.back();
                int best = -1;
                double best_d2b = link_gap_m * link_gap_m;

                for (size_t i = 0; i < pts.size(); i++)
                {
                    if (used[i])
                        continue;
                    double dx = pts[i].x - tail.x, dy = pts[i].y - tail.y;
                    double d2 = dx * dx + dy * dy;
                    if (d2 < best_d2b)
                    {
                        best_d2b = d2;
                        best = static_cast<int>(i);
                    }
                }

                if (best < 0)
                    break;
                chain.push_back(pts[best]);

                used[best] = true;
            }
            chains.push_back(std::move(chain));
        }

        return chains;
    }

    void merge_collinear(std::vector<Chain> &chains, double gap_white_m,
                         double gap_yellow_m, double angle_max_rad)
    {
        bool merged_any = true;

        while (merged_any)
        {
            merged_any = false;
            for (size_t i = 0; i < chains.size() && !merged_any; ++i)
            {
                for (size_t j = 0; j < chains.size() && !merged_any; ++j)
                {
                    if (i == j || chains[i].empty() || chains[j].empty())
                        continue;
                    const auto &a = chains[i];
                    const auto &b = chains[j];
                    if (a.size() < 2)
                        continue;
                    bool yellow = a.back().yellow || b.front().yellow;
                    double gap_max = yellow ? gap_yellow_m : gap_white_m;
                    double dx = b.front().x - a.back().x, dy = b.front().y - a.back().y;
                    double gap = std::hypot(dx, dy);
                    if (gap > gap_max || gap < 1e-6)
                        continue;
                    double h_chain = std::atan2(
                        a.back().y - a[a.size() - 2].y,
                        a.back().x - a[a.size() - 2].x);
                    double h_join = std::atan2(dy, dx);
                    if (std::abs(av::geom::wrap_angle(h_join - h_chain)) > angle_max_rad)
                        continue;

                    chains[i].insert(chains[i].end(), b.begin(), b.end()); // append b onto a
                    chains[j].clear();
                    merged_any = true;
                }
            }
        }

        chains.erase(std::remove_if(chains.begin(), chains.end(),
                                    [](const Chain &c)
                                    { return c.empty(); }),
                     chains.end());
    }

    void drop_stray_chains(std::vector<Chain> &chains, int min_pts, double min_extent_m)
    {
        auto extent = [](const Chain &c)
        { return std::hypot(c.back().x - c.front().x, c.back().y - c.front().y); };

        std::vector<Chain> kept; // survivors of size/extent filter
        for (Chain &c : chains)
            if (static_cast<int>(c.size()) >= min_pts && extent(c) >= min_extent_m)
                kept.push_back(std::move(c));
        // NOTE: the fallback below only runs when kept is empty, i.e. when the
        // loop above moved out of nothing -- so every chain here is still intact.
        if (kept.empty() && !chains.empty()) // everything failed, but there WAS evidence
        {
            auto longest = std::max_element(chains.begin(), chains.end(),
                                            [&](const Chain &a, const Chain &b)
                                            { return extent(a) < extent(b); }); // find the least-bad chain
            kept.push_back(std::move(*longest));                                // keep it anyway rather than publishing nothing
        }

        chains = std::move(kept);
    }

    Chain resample_chain_x(Chain c, double slice_m) // average per-pixel noise pts into per_slice estimate
    {
        if (c.size() < 2)
            return c;

        std::sort(c.begin(), c.end(), [](const ChainPt &a, const ChainPt &b)
                  { return a.x < b.x; }); // x-monotonic order

        Chain out;
        double sx = 0, sy = 0;
        double bin_start = c.front().x;
        int n = 0;
        bool yellow = false;

        auto flush = [&]()
        {
            if (n > 0)
            {
                out.push_back({sx / n, sy / n, yellow});
                sx = sy = 0;
                n = 0;
                yellow = false;
            }
        };

        for (const auto &p : c)
        {
            if (p.x - bin_start >= slice_m) // slice closed, flush the average value, reset the slice bin start
            {
                flush();
                bin_start = p.x;
            }
            sx += p.x; // accumulate during the slice
            sy += p.y;
            ++n;
            yellow = yellow || p.yellow;
        }

        flush(); // emit the final still open slice
        return out;
    }

    struct OffsetChain
    {
        Chain pts;
    };

    std::vector<OffsetChain> classify_and_offset(const std::vector<Chain> &chains,
                                                 double lane_half_m, double lane_mem_y, bool have_lane_mem)
    {
        const Chain *yellow = nullptr;
        for (const auto &c : chains)
            if (!c.empty() && c.front().yellow)
            {
                yellow = &c;
                break;
            }
        double yellow_near_y = yellow ? yellow->front().y : 0.0; // yellow's near-end y, the reference line

        std::vector<OffsetChain> out;
        for (const auto &c : chains)
        {
            if (c.empty())
                continue;

            Chain offset;
            offset.reserve(c.size());

            // Ego frame: +y is LEFT. Right-hand traffic (see
            // scripts/generate_course_world.py): the ego drives the right lane,
            // so the yellow centre line is to its LEFT (y > 0) and the ego
            // lane's own white edge is to its RIGHT (y < 0). "Drop the far side"
            // therefore means drop chains with the LARGER y, not the smaller.
            if (c.front().yellow) // yellow IS the ego lane's left boundary -- offset right (toward -y)
            {
                for (auto p : c)
                {
                    p.y -= lane_half_m;
                    offset.push_back(p);
                }
            }
            else if (yellow) // a yellow chain exists this frame
            {
                if (c.front().y > yellow_near_y) // left of yellow -- oncoming lane's edge, drop
                    continue;
                for (auto p : c) // this is the ego lane's right edge -- offset left (toward +y)
                {
                    p.y += lane_half_m;
                    offset.push_back(p);
                }
            }
            else // no yellow this frame -- fall back to lane_mem_y_
            {
                if (have_lane_mem && c.front().y >= lane_mem_y) // left of remembered centre -- drop
                    continue;
                for (auto p : c) // presumed ego lane right edge -- offset left (toward +y)
                {
                    p.y += lane_half_m;
                    offset.push_back(p);
                }
            }

            if (!offset.empty())
                out.push_back({std::move(offset)});
        }
        return out;
    }

    // x-bin merge: collapse to one centerline
    av::geom::Polyline xbin_merge(const std::vector<OffsetChain> &offsets, double bin_m)
    {
        std::map<int, std::pair<double, int>> bins; // x-bin -> (sum of y, counts)
        for (const auto &oc : offsets)
        {
            for (const auto &p : oc.pts)
            {
                auto &e = bins[static_cast<int>(std::floor(p.x / bin_m))]; // which bin this point votes to
                e.first += p.y;
                e.second += 1;
            }
        }
        av::geom::Polyline out;
        for (const auto &[b, e] : bins)
            out.push_back({(b + 0.5) * bin_m, e.first / e.second}); // bin center x, bin average y

        return out;
    }

    bool folds_back(const av::geom::Polyline &c)
    {
        if (c.size() < 3)
            return false;
        for (size_t i = 1; i + 1 < c.size(); ++i)
        {
            Eigen::Vector2d d1 = c[i] - c[i - 1], d2 = c[i + 1] - c[i];
            if (d1.norm() < 1e-6 || d2.norm() < 1e-6)
                continue;
            if (d1.normalized().dot(d2.normalized()) < 0.0) // negative dot product = turn > 90 deg
                return true;
        }

        return false;
    }
}

// ---------------------------------------------------------------- LaneNode
class LaneNode : public rclcpp::Node
{
public:
    LaneNode();

private:
    // ---------------------------------------------------------------- params
    struct Params
    {
        double cam_x, cam_z, cam_pitch, hfov;                       // camera x, z, pitch, horizontal FOV
        int img_w, img_h;                                           // image width, height
        int white_v_min, white_s_max;                               // HSV thresholds for white lane pixels
        int yellow_h_min, yellow_h_max, yellow_s_min, yellow_v_min; // HSV thresholds for yellow lane pixels
        int line_area_min_px;                                       // minimum area in pixels for a detected lane line
        double z_gate, x_min, x_max;                                // depth and lateral bounds for lane detection
        double far_stride_dist_m;
        int near_stride;
        std::string map_path;
        double map_origin_x, map_origin_y, map_origin_yaw;

        std::string dump_dir; // debug: image/cloud dump directory; empty = disabled
        int dump_every_n;     // dump 1 out of every N ticks (throttle -- images are heavy)

        double chain_grid_m;             // B.1 grid-dedup cell size
        double chain_stop_max_m;         // B.2 stop-line search range
        double chain_link_gap_m;         // B.3 clustering link gap
        double chain_merge_gap_white_m;  // B.4 white merge gap budget
        double chain_merge_gap_yellow_m; // B.4 yellow merge gap budget (must clear the dash cycle)
        double chain_merge_angle_rad;    // B.4 max join angle
        int chain_min_pts;               // B.5 minimum chain point count
        double chain_min_extent_m;       // B.5 minimum chain end-to-end length
        double chain_resample_m;         // B.6 x-slice width
        double lane_half_m;              // B.7 half lane width, the classification offset
        double chain_merge_bin_m;        // B.8 x-bin merge width
        int chain_min_center_pts;        // B.10 minimum published centerline points
        double path_reuse_s;             // B.11 how long to republish the last good path
    };

    struct ChainResult
    {                                  // return type from chain_centerline()
        av::geom::Polyline centerline; // ego frame -- tick() transforms to odom before publishing
        bool valid = false;            // false on fold-back or too few points
    };

    ChainResult chain_centerline();

    void on_rgb(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    av::proj::Pose2d pose_at(double stamp_sec) const; // nearst-timestamp pose from pose_hist_, or current_pose_ if none
    void ingest();
    void ingest_mask();
    void publish_path(const av::geom::Polyline &path); // odom-frame nav_msgs/Path publisher
    void tick();
    void dump_debug_frame(const cv::Mat &white_raw, const cv::Mat &yellow_raw,
                          const cv::Mat &white_filtered, const cv::Mat &yellow_filtered,
                          const cv::Mat &white_cloud_mask, const cv::Mat &yellow_cloud_mask);
    Params params_;
    av::proj::Pinhole pin_;
    Eigen::Isometry3d T_base_cam_;
    double pitch_ema_ = 0.0;
    bool have_pitch_ = false;
    bool info_checked_ = false;

    sensor_msgs::msg::Image::ConstSharedPtr rgb_msg_, depth_msg_;
    cv::Mat rgb_, depth_;
    std::vector<Eigen::Vector2d> white_cloud_, yellow_cloud_;

    struct PoseSample
    {
        double t;
        av::proj::Pose2d pose;
    };

    std::deque<PoseSample> pose_hist_; // last 60 seconds of pose samples
    av::proj::Pose2d frame_pose_;
    av::proj::Pose2d current_pose_;

    double lane_mem_y_ = 0.0; // chains: near-end lane-center lateral memory
    bool have_lane_mem_ = false;

    int ingest_tick_count_ = 0; // counts ingest_mask() calls, for the dump_every_n throttle
    int dump_seq_ = 0;          // sequential index of ACTUAL dumps written (no gaps, unlike the tick count)

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_, depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; // /lane/path_odom
    av::DebugTap debug_tap_;

    std::optional<Eigen::Vector2d> last_stop_line_ego_;
    rclcpp::Time last_path_time_;  // when last_path_ was published
    av::geom::Polyline last_path_; // last published odom-frame path, for the reuse window
};

LaneNode::LaneNode() : rclcpp::Node("lane_node"), debug_tap_(this) // constructor
{
    params_.cam_x = declare_parameter("cam_x", 1.9);          // URDF mount x (Day 2)
    params_.cam_z = declare_parameter("cam_z", 1.4);          // URDF mount z, also ground height post-fix
    params_.cam_pitch = declare_parameter("cam_pitch", 0.06); // URDF mount pitch, now the true value (Day 2 fix)
    params_.hfov = declare_parameter("camera_hfov", 1.6);     // must match the URDF's rgbd_camera hfov
    params_.img_w = declare_parameter("camera_width", 640);   // must match the URDF's image width
    params_.img_h = declare_parameter("camera_height", 360);  // must match the URDF's image height

    params_.white_s_max = declare_parameter("white_s_max", 60);           // white: S must stay below this
    params_.white_v_min = declare_parameter("white_v_min", 150);          // white: V must stay above this
    params_.yellow_h_min = declare_parameter("yellow_h_min", 20);         // yellow hue band, lower bound
    params_.yellow_h_max = declare_parameter("yellow_h_max", 38);         // yellow hue band, upper bound
    params_.yellow_s_min = declare_parameter("yellow_s_min", 80);         // yellow: S floor
    params_.yellow_v_min = declare_parameter("yellow_v_min", 120);        // yellow: V floor
    params_.line_area_min_px = declare_parameter("line_area_min_px", 50); // drop blobs smaller than this

    params_.z_gate = declare_parameter("z_gate", 0.15);                      // |z| tolerance around the ground plane
    params_.x_min = declare_parameter("depth_x_min", 0.5);                   // reject points closer than this (self-occlusion)
    params_.x_max = declare_parameter("depth_x_max", 33.0);                  // reject points farther than this
    params_.far_stride_dist_m = declare_parameter("far_stride_dist_m", 6.0); // stride switches at this ground distance
    params_.near_stride = declare_parameter("near_stride", 4);               // pixel stride used for near (dense-paint) rows

    params_.map_path = declare_parameter("map_path", std::string());   // MapModel::load() input
    params_.map_origin_x = declare_parameter("map_origin_x", 0.0);     // spawn x, zeroes map frame into odom
    params_.map_origin_y = declare_parameter("map_origin_y", 0.0);     // spawn y
    params_.map_origin_yaw = declare_parameter("map_origin_yaw", 0.0); // spawn yaw

    params_.chain_grid_m = declare_parameter("chain_grid_m", 0.15);                        // B.1
    params_.chain_stop_max_m = declare_parameter("chain_stop_max_m", 12.0);                // B.2
    params_.chain_link_gap_m = declare_parameter("chain_link_gap_m", 1.0);                 // B.3
    params_.chain_merge_gap_white_m = declare_parameter("chain_merge_gap_white_m", 2.5);   // B.4
    params_.chain_merge_gap_yellow_m = declare_parameter("chain_merge_gap_yellow_m", 6.0); // B.4
    params_.chain_merge_angle_rad = declare_parameter("chain_merge_angle_rad", 0.7);       // B.4
    params_.chain_min_pts = declare_parameter("chain_min_pts", 3);                         // B.5
    params_.chain_min_extent_m = declare_parameter("chain_min_extent_m", 1.0);             // B.5
    params_.chain_resample_m = declare_parameter("chain_resample_m", 0.75);                // B.6
    params_.lane_half_m = declare_parameter("lane_half_m", 1.75);                          // B.7
    params_.chain_merge_bin_m = declare_parameter("chain_merge_bin_m", 0.75);              // B.8
    params_.chain_min_center_pts = declare_parameter("chain_min_center_pts", 5);           // B.10
    params_.path_reuse_s = declare_parameter("path_reuse_s", 1.0);                         // B.11

    params_.dump_dir = declare_parameter("dump_dir", std::string()); // debug: set to enable ingest_mask() dumps
    params_.dump_every_n = declare_parameter("dump_every_n", 10);    // dump 1 out of every N ticks

    pin_ = av::percep::pinhole_from_config(params_.img_w, params_.img_h, params_.hfov);            // derived, not from camera_info
    T_base_cam_ = av::proj::make_T_base_cam(params_.cam_x, 0.0, params_.cam_z, params_.cam_pitch); // initial extrinsic

    auto qos = rclcpp::SensorDataQoS(); // best-effort, small queue: a dropped frame is harmless, a stall is not

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/front/image_raw", qos, std::bind(&LaneNode::on_rgb, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/front/depth/image_raw", qos, std::bind(&LaneNode::on_depth, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/front/camera_info", qos, std::bind(&LaneNode::on_camera_info, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom_ekf", 10, std::bind(&LaneNode::on_odom, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/lane/path_odom", 10);

    // Seed with the node clock so the reuse-window subtraction in tick() never
    // mixes clock sources. A default-constructed rclcpp::Time is RCL_SYSTEM_TIME
    // while this->now() is RCL_ROS_TIME, and subtracting the two throws.
    last_path_time_ = this->now();
}

void LaneNode::on_rgb(sensor_msgs::msg::Image::ConstSharedPtr msg) // callback for RGB image
{
    rgb_msg_ = msg;
}

void LaneNode::on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) // callback for depth image
{
    depth_msg_ = msg;
    if (rgb_msg_)
        tick();
}

void LaneNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) // callback for camera_info
{
    if (info_checked_)
        return; // only check once
    info_checked_ = true;
    av::proj::Pinhole from_info = av::proj::Pinhole::from_info(*msg);
    if (std::abs(from_info.fx - pin_.fx) > 1.0)
    {
        RCLCPP_WARN(get_logger(), "CameraInfo fx = %f disagrees with declared pinhole fx = %f", from_info.fx, pin_.fx);
    }
}

void LaneNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg) // callback for odometry
{
    double t = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    av::proj::Pose2d pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = tf2::getYaw(msg->pose.pose.orientation);
    pose_hist_.push_back({t, pose}); // append new sample to pose_hist_ as struct PoseSample
    while (pose_hist_.size() > 0 && t - pose_hist_.front().t > 60.0)
        pose_hist_.pop_front(); // keep last 60 seconds of samples
    current_pose_ = pose;       // newest sample is the current pose
}

av::proj::Pose2d LaneNode::pose_at(double stamp_sec) const // nearst-timestamp pose from pose_hist_, or current_pose_ if none
{
    if (pose_hist_.size() == 0)
        return current_pose_;
    auto best = std::min_element(pose_hist_.begin(), pose_hist_.end(),
                                 [stamp_sec](const PoseSample &a, const PoseSample &b)
                                 { return std::abs(a.t - stamp_sec) < std::abs(b.t - stamp_sec); });
    return best->pose;
}

void LaneNode::ingest()
{
    rgb_ = av::img::as_rgb(rgb_msg_);       // zero-copy view into rgb_msg_'s own buffer
    depth_ = av::img::as_depth(depth_msg_); // zero-copy view into

    double stamp = depth_msg_->header.stamp.sec + 1e-9 * depth_msg_->header.stamp.nanosec;
    frame_pose_ = pose_at(stamp); // pose AT capture time, not the latest pose

    auto p = av::percep::estimate_pitch(depth_, pin_, params_.cam_z); // one noisy per-frame pitch sample, or nullopt
    if (p)
    {
        pitch_ema_ = have_pitch_ ? 0.9 * pitch_ema_ + 0.1 * *p : *p; // EMA filter
        RCLCPP_INFO(get_logger(), "Estimated pitch: %.4f rad, EMA: %.4f rad", *p, pitch_ema_);
        have_pitch_ = true;

        // bypass estimation, use the known true pitch from URDF mount, for Day 5 accept check
        // pitch_ema_ = params_.cam_pitch;
        T_base_cam_ = av::proj::make_T_base_cam(params_.cam_x, 0.0, params_.cam_z, pitch_ema_);
    }

    debug_tap_.put("pitch_est", pitch_ema_); // exposed on /debug/lane_node for the Day 5 accept check
}

void LaneNode::ingest_mask()
{
    white_cloud_.clear(); // this frame's clouds only -- no cross-frame accumulation
    yellow_cloud_.clear();

    cv::Mat hsv;
    cv::cvtColor(rgb_, hsv, cv::COLOR_RGB2HSV); // one colorspace conversion, reused by both masks below

    cv::Mat white_mask, yellow_mask;
    cv::inRange(hsv, cv::Scalar(0, 0, params_.white_v_min),
                cv::Scalar(180, params_.white_s_max, 255), white_mask); // full hue range, gated on S/V only
    cv::inRange(hsv, cv::Scalar(params_.yellow_h_min, params_.yellow_s_min, params_.yellow_v_min),
                cv::Scalar(params_.yellow_h_max, 255, 255), yellow_mask); // narrow hue band + S/V floors

    // The per-pixel loop below indexes the masks with depth_ row/col ranges;
    // a size mismatch would be a silent out-of-bounds read. Skip the frame
    // instead -- the cleared clouds make chain_centerline() fall back to reuse.
    if (white_mask.size() != depth_.size())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "RGB mask %dx%d != depth %dx%d -- skipping frame",
                             white_mask.cols, white_mask.rows, depth_.cols, depth_.rows);
        return;
    }

    cv::Mat white_mask_raw = white_mask.clone(); // keep the PRE-blob-drop version for the debug dump
    cv::Mat yellow_mask_raw = yellow_mask.clone();

    auto drop_small_blobs = [&](cv::Mat &mask)
    {
        cv::Mat labels, stats, centroids;
        int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids); // label every blob in the mask
        for (int i = 1; i < n; ++i)                                               // label 0 is the background, skip it
            if (stats.at<int>(i, cv::CC_STAT_AREA) < params_.line_area_min_px)    // too small to be real paint
                mask.setTo(0, labels == i);                                       // erase the whole blob in one call
    };
    // Blank out any row that can NEVER be ground -- above the horizon --
    // using the SAME analytic model expected_ground_distance already relies
    // on. This is what actually removes the sky from the mask (drop_small_blobs
    // alone can't: the sky is one huge connected blob, nowhere near
    // line_area_min_px). Doing it here, before drop_small_blobs, means the
    // dumped "_filtered" mask visibly shows the sky gone, not just the
    // downstream point counters improving.
    for (int v = 0; v < white_mask.rows; ++v)
    {
        double expected_d = expected_ground_distance(v, pin_, params_.cam_z, pitch_ema_);
        if (expected_d >= 1e9) // sentinel from expected_ground_distance: this row points above the horizon
        {
            white_mask.row(v).setTo(0);
            yellow_mask.row(v).setTo(0);
        }
    }

    drop_small_blobs(white_mask);  // stray-speck suppression, white
    drop_small_blobs(yellow_mask); // stray-speck suppression, yellow

    // Image-space record of which pixels actually survived ALL the way
    // through the per-pixel loop below (depth-valid, on the ground plane,
    // in range) -- not just color+blob-drop, which is all white_mask/
    // yellow_mask show. Stays mostly zero at every pixel the stride skipped
    // entirely, so this will look sparse/dotted compared to the solid
    // color masks, not a bug -- it's marking EVALUATED-AND-ACCEPTED, not
    // "should be paint".
    cv::Mat white_mask_cloud = cv::Mat::zeros(white_mask.size(), CV_8UC1);
    cv::Mat yellow_mask_cloud = cv::Mat::zeros(yellow_mask.size(), CV_8UC1);

    int added = 0, rej_depth = 0, rej_zgate = 0; // counters surfaced on the debug tap
    for (int v = 0; v < depth_.rows; ++v)
    {
        double expected_d = expected_ground_distance(v, pin_, params_.cam_z, pitch_ema_); // analytic, no depth read yet
        if (expected_d >= 1e9)
            continue;                                                                  // above the horizon -- mask is already zeroed here, skip the row entirely (no per-pixel checks needed)
        int stride = expected_d > params_.far_stride_dist_m ? 1 : params_.near_stride; // far rows: dense; near rows: coarse
        for (int u = 0; u < depth_.cols; u += stride)
        {
            bool is_white = white_mask.at<uint8_t>(v, u) > 0;   // this pixel survived the white mask + blob filter
            bool is_yellow = yellow_mask.at<uint8_t>(v, u) > 0; // this pixel survived the yellow mask + blob filter
            if (!is_white && !is_yellow)
                continue; // neither color -- skip without touching depth_

            float d = depth_.at<float>(v, u); // only read depth for pixels that passed color
            if (!std::isfinite(d) || d < 0.1f)
            {
                ++rej_depth;
                continue;
            } // invalid or bogus-near depth

            Eigen::Vector3d p_cam = av::proj::pixel_depth_to_camera(pin_, u, v, d); // pixel+depth -> camera frame
            Eigen::Vector3d p_base = T_base_cam_ * p_cam;                           // camera frame -> ego/base_link frame
            if (std::abs(p_base.z()) > params_.z_gate)                              // reject points off the ground plane
            {
                ++rej_zgate;
                continue;
            } // off the ground plane -- reject
            if (p_base.x() < params_.x_min || p_base.x() > params_.x_max)
                continue; // out of trusted range -- reject

            (is_white ? white_cloud_ : yellow_cloud_).emplace_back(p_base.x(), p_base.y()); // keep the ego-frame x,y
            (is_white ? white_mask_cloud : yellow_mask_cloud).at<uint8_t>(v, u) = 255;      // mark it in image space too
            ++added;
        }
    }
    debug_tap_.put("added", added);         // Day 5 accept check: thousands on a straight
    debug_tap_.put("rej_depth", rej_depth); // should be bounded, not dominant
    debug_tap_.put("rej_zgate", rej_zgate); // should be bounded, not dominant

    ++ingest_tick_count_;
    if (!params_.dump_dir.empty() && ingest_tick_count_ % params_.dump_every_n == 0)
        dump_debug_frame(white_mask_raw, yellow_mask_raw, white_mask, yellow_mask,
                         white_mask_cloud, yellow_mask_cloud); // white_mask/yellow_mask are the FILTERED versions here
}

LaneNode::ChainResult LaneNode::chain_centerline()
{
    ChainResult res;

    Chain pts;
    for (const auto &p : white_cloud_)
        pts.push_back({p.x(), p.y(), false});
    for (const auto &p : yellow_cloud_)
        pts.push_back({p.x(), p.y(), true});

    auto gridded = grid_dedup(pts, params_.chain_grid_m);
    auto stop = detect_and_strip_stop_line(gridded, params_.chain_stop_max_m);
    if (stop.found)
        last_stop_line_ego_ = Eigen::Vector2d(stop.x, stop.y_center);

    auto chains = cluster_chains(gridded, params_.chain_link_gap_m);
    merge_collinear(chains, params_.chain_merge_gap_white_m,
                    params_.chain_merge_gap_yellow_m, params_.chain_merge_angle_rad);
    drop_stray_chains(chains, params_.chain_min_pts, params_.chain_min_extent_m);
    for (auto &c : chains)
        c = resample_chain_x(c, params_.chain_resample_m);

    auto offsets = classify_and_offset(chains, params_.lane_half_m, lane_mem_y_, have_lane_mem_);
    auto center = xbin_merge(offsets, params_.chain_merge_bin_m);

    if (folds_back(center) || static_cast<int>(center.size()) < params_.chain_min_center_pts)
    {
        res.valid = false; // bad frame -- caller falls back to path reuse
        return res;
    }

    lane_mem_y_ = have_lane_mem_ ? 0.5 * lane_mem_y_ + 0.5 * center.front().y() : center.front().y(); // update the fallback EMA
    have_lane_mem_ = true;                                                                            // EMA now has at least one sample
    res.centerline = std::move(center);                                                               // still in EGO frame -- tick() transforms to odom
    res.valid = true;

    return res;
}

// Writes one frame's raw RGB, both masks at every stage (raw color gate,
// after horizon-blank+blob-drop, and the final image-space cloud masks),
// plus the final white_cloud_/yellow_cloud_ JSON, into params_.dump_dir, for
// offline visualization with scripts/visualize_lane_dump.py. Gated by
// dump_dir being set AND the dump_every_n throttle, so it costs nothing
// when disabled.
void LaneNode::dump_debug_frame(const cv::Mat &white_raw, const cv::Mat &yellow_raw,
                                const cv::Mat &white_filtered, const cv::Mat &yellow_filtered,
                                const cv::Mat &white_cloud_mask, const cv::Mat &yellow_cloud_mask)
{
    char tag[16];
    std::snprintf(tag, sizeof(tag), "%05d", dump_seq_++); // sequential, independent of the (throttled) tick count
    const std::string base = params_.dump_dir + "/" + tag;

    cv::Mat bgr;
    cv::cvtColor(rgb_, bgr, cv::COLOR_RGB2BGR); // cv::imwrite expects BGR, rgb_ is RGB

    // cv::imwrite returns false (not an exception) on failure -- e.g. the
    // directory doesn't exist -- and ignoring that return value is exactly
    // what let a failed dump log as if it had succeeded.
    bool ok = true;
    ok &= cv::imwrite(base + "_rgb.png", bgr);
    ok &= cv::imwrite(base + "_white_raw.png", white_raw);
    ok &= cv::imwrite(base + "_yellow_raw.png", yellow_raw);
    ok &= cv::imwrite(base + "_white_filtered.png", white_filtered);
    ok &= cv::imwrite(base + "_yellow_filtered.png", yellow_filtered);
    ok &= cv::imwrite(base + "_white_cloud_mask.png", white_cloud_mask);   // image-space: pixels that made it into white_cloud_
    ok &= cv::imwrite(base + "_yellow_cloud_mask.png", yellow_cloud_mask); // image-space: pixels that made it into yellow_cloud_

    av::json j;
    for (const auto &p : white_cloud_)
        j["white"].push_back({p.x(), p.y()}); // ego-frame [x, y] pairs
    for (const auto &p : yellow_cloud_)
        j["yellow"].push_back({p.x(), p.y()});
    std::ofstream cloud_file(base + "_clouds.json");
    cloud_file << j.dump();
    ok &= cloud_file.good(); // same failure mode as imwrite: a bad stream just silently drops writes

    if (!ok)
    {
        RCLCPP_ERROR(get_logger(),
                     "dump_debug_frame: failed writing into '%s' -- does the directory exist and is it writable?",
                     params_.dump_dir.c_str());
        return; // do NOT log success below when the write actually failed
    }

    RCLCPP_INFO(get_logger(), "dumped debug frame %s (%zu white pts, %zu yellow pts) -> %s",
                tag, white_cloud_.size(), yellow_cloud_.size(), params_.dump_dir.c_str());
}

void LaneNode::publish_path(const av::geom::Polyline &path)
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = "odom"; // every downstream consumer expects odom frame
    msg.header.stamp = this->now();
    for (const auto &p : path)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg.header; // same stamp/frame on every pose in the path
        ps.pose.position.x = p.x();
        ps.pose.position.y = p.y();
        msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
}

void LaneNode::tick()
{
    ingest();      // wrap Mats, resolve frame_pose_, update pitch_ema_/T_base_cam_
    ingest_mask(); // build white_cloud_/yellow_cloud_ for THIS frame
    // Day 6 adds chain_centerline() + publish here.
    auto chain = chain_centerline();
    rclcpp::Time now = this->now();

    if (chain.valid)
    {
        av::geom::Polyline odom_path;
        for (const auto &p : chain.centerline)
            odom_path.push_back(frame_pose_.odom_of(p));        // ego -> odom
        odom_path = av::geom::resample_uniform(odom_path, 0.5); // even 0.5 m spacing for downstream consumers
        last_path_ = odom_path;
        last_path_time_ = now;
        publish_path(odom_path);
    }
    else if (!last_path_.empty() && (now - last_path_time_).seconds() < params_.path_reuse_s)
    {
        publish_path(last_path_); // still inside the reuse window -- republish the last good path
    }

    debug_tap_.flush();
}

// ---------------------------------------------------------------- main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneNode>());
    rclcpp::shutdown();
    return 0;
}
