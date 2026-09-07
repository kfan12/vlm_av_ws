#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "av_common/projection.hpp"
#include "av_common/debug_tap.hpp"
#include "av_common/geometry.hpp"
#include "av_common/json_io.hpp"
#include "av_common/map_model.hpp"
#include "av_perception_cpp/img_access.hpp"
#include "av_perception_cpp/ground_cal.hpp"

#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <chrono>
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
        std::map<int, std::vector<double>> ybins; // 0.5 m x-bin -> the white y values in it

        for (const auto &p : gridded)
        {
            if (p.yellow || p.x > x_max_m || p.x < 0.0)
                continue;
            ybins[static_cast<int>(std::floor(p.x / 0.5))].push_back(p.y);
        }

        for (auto &[b, ys] : ybins)
        {
            if (ys.size() < 6)
                continue;
            std::sort(ys.begin(), ys.end());
            double ymin = ys.front(), ymax = ys.back(), span = ymax - ymin;
            if (span < 2.0 || ymin >= 0.3 || ymax <= -0.3) // wide enough, straddling the ego line
                continue;
            // Reject two parallel edge lines masquerading as a bar: a real bar
            // FILLS the span, two thin lines leave the middle third empty.
            double lo = ymin + span / 3.0, hi = ymax - span / 3.0;
            size_t mid = std::count_if(ys.begin(), ys.end(),
                                       [&](double y)
                                       { return y > lo && y < hi; });
            if (mid < ys.size() / 5) // < 20% of points in the middle third -> not a bar
                continue;
            res = {true, (b + 0.5) * 0.5, (ymin + ymax) / 2.0};
            break;
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
                    bool yellow = a.back().yellow || b.front().yellow;
                    double gap_max = yellow ? gap_yellow_m : gap_white_m;
                    double dx = b.front().x - a.back().x, dy = b.front().y - a.back().y;
                    double gap = std::hypot(dx, dy);
                    if (gap > gap_max || gap < 1e-6)
                        continue;
                    // A chain with < 2 points has no heading to check - this is
                    // exactly the case for a thin near-field fragment (e.g. a
                    // single detected point in the sliver just past the hood,
                    // separated from the main dash chain by more than
                    // chain_link_gap_m). Skipping it outright here used to
                    // strand these fragments: they can never be `a` (no
                    // heading), and can never be picked up as `b` either since
                    // every other chain grows AWAY from the camera, not back
                    // toward it - so a real, unbroken dash line could get its
                    // nearest fragment permanently orphaned and then dropped by
                    // chain_min_pts, despite chain_merge_gap_yellow_m being
                    // generously larger than the dash period. Fall back to a
                    // gap-only test (no angle check - there's no direction to
                    // check against) instead of refusing the merge.
                    if (a.size() >= 2)
                    {
                        double h_chain = std::atan2(
                            a.back().y - a[a.size() - 2].y,
                            a.back().x - a[a.size() - 2].x);
                        double h_join = std::atan2(dy, dx);
                        if (std::abs(av::geom::wrap_angle(h_join - h_chain)) > angle_max_rad)
                            continue;
                    }

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

    // Forward-bin occupancy STRUCTURE of a lane boundary, for /lane/markings.
    // Counting occupied bins alone can't tell "solid, only 13 m detected" from
    // "dashed" -- both land near half. What separates them is the GAP pattern:
    // a solid line never leaves a multi-bin hole; the 3 m-paint / 4.5 m-gap
    // dash cycle always does.
    struct MarkingStats
    {
        int n_bins = 0;         // occupied bins
        double span_m = 0.0;    // first occupied bin to last
        double max_gap_m = 0.0; // longest run of empty bins strictly inside the span
    };

    MarkingStats marking_stats(const std::vector<Chain> &chains, double d_max_m, double bin_m)
    {
        const size_t nbins = static_cast<size_t>(d_max_m / bin_m) + 1;
        std::vector<char> hit(nbins, 0);
        for (const auto &c : chains)
            for (const auto &p : c)
            {
                if (p.x < 0.0)
                    continue;
                size_t b = static_cast<size_t>(p.x / bin_m);
                if (b < nbins)
                    hit[b] = 1;
            }
        MarkingStats s;
        int first = -1, last = -1;
        for (int i = 0; i < static_cast<int>(nbins); ++i)
            if (hit[i])
            {
                if (first < 0)
                    first = i;
                last = i;
                ++s.n_bins;
            }
        if (first < 0)
            return s;
        s.span_m = (last - first + 1) * bin_m;
        int run = 0;
        for (int i = first; i <= last; ++i)
        {
            if (hit[i])
                run = 0;
            else
                s.max_gap_m = std::max(s.max_gap_m, ++run * bin_m);
        }
        return s;
    }

    // solid: a long contiguous run, at most a detection-dropout hole. dashed:
    // a >=2 m empty run somewhere in the span (a solid line never has one; the
    // centre dash cycle always does). none: too little to tell -- a lone dash
    // or a stub reads "none" until the car moves and more comes into view.
    std::string marking_type(const MarkingStats &s)
    {
        if (s.n_bins < 2 || s.span_m < 1.5)
            return "none";
        if (s.max_gap_m >= 2.0)
            return "dashed";
        if (s.n_bins >= 6 && s.max_gap_m < 1.5)
            return "solid";
        return "none";
    }

    struct ClassifyResult
    {
        std::vector<OffsetChain> offsets; // centreline candidates, ready for xbin_merge
        MarkingStats right_edge;          // ego right-edge boundary structure
        MarkingStats center_line;         // yellow centre-line structure
    };

    ClassifyResult classify_and_offset(const std::vector<Chain> &chains,
                                       double lane_half_m, double lane_mem_y, bool have_lane_mem,
                                       double occ_d_max_m, double occ_bin_m)
    {
        const Chain *yellow = nullptr;
        for (const auto &c : chains)
            if (!c.empty() && c.front().yellow)
            {
                yellow = &c;
                break;
            }
        double yellow_near_y = yellow ? yellow->front().y : 0.0; // yellow's near-end y, the reference line

        std::vector<Chain> yellow_chains, right_chains; // for the occupancy counts below
        ClassifyResult res;
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
                yellow_chains.push_back(c);
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
                right_chains.push_back(c);
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
                right_chains.push_back(c);
                for (auto p : c) // presumed ego lane right edge -- offset left (toward +y)
                {
                    p.y += lane_half_m;
                    offset.push_back(p);
                }
            }

            if (!offset.empty())
                res.offsets.push_back({std::move(offset)});
        }

        res.center_line = marking_stats(yellow_chains, occ_d_max_m, occ_bin_m);
        res.right_edge = marking_stats(right_chains, occ_d_max_m, occ_bin_m);
        return res;
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
        double chain_d_max_m;            // trusted forward range for chain evidence + markings occupancy
        int smooth_window;               // centreline moving-average window, in points
        double depth_stale_warn_s;       // watchdog: WARN if no depth frame arrives for this long
        double rgb_depth_skew_max_s;     // skip a tick when |rgb stamp - depth stamp| exceeds this
    };

    struct ChainResult
    {                                                                        // return type from chain_centerline()
        av::geom::Polyline centerline;                                       // ego frame -- tick() transforms to odom before publishing
        bool valid = false;                                                  // false on fold-back or too few points
        MarkingStats right_edge;                                             // /lane/markings: ego right-edge boundary structure
        MarkingStats center_line;                                            // /lane/markings: yellow centre-line structure
        std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> stop_seg; // ego-frame stop-bar span, if detected
        std::optional<double> stop_x;                                        // ego-frame forward distance to the bar
    };

    ChainResult chain_centerline();

    void on_rgb(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    av::proj::Pose2d pose_at(double stamp_sec) const; // nearst-timestamp pose from pose_hist_, or current_pose_ if none
    void ingest();
    void ingest_mask();
    void publish_path(const av::geom::Polyline &path);                     // odom-frame nav_msgs/Path publisher
    void publish_markings(const ChainResult &chain);                       // /lane/markings JSON
    void publish_stop_line(const ChainResult &chain);                      // /lane/stop_line JSON (vision + optional map prior)
    double path_end_curvature(const av::geom::Polyline &ego_center) const; // wkappa tap/overlay field
    void publish_debug_markers(const ChainResult &chain, double wkappa);   // /lane/debug_markers
    void publish_debug_image(const ChainResult &chain, double wkappa);     // /lane/debug_image
    void dump_lane_cloud_jsonl(const ChainResult &chain, double wkappa);   // per-tick lane_cloud.jsonl
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
    // debug-only: latest steering command, purely for the /lane/debug_image
    // overlay so the commanded steer can be eyeballed against the detected
    // lane in the same frame. lane_node has no other reason to know about
    // mpc_tracker_v2 - this is a visualization convenience, not a control
    // dependency (nothing here feeds back into perception).
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_steer_sub_;
    double last_cmd_steer_ = 0.0;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;                         // /lane/path_odom
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr markings_pub_;                   // /lane/markings
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stop_line_pub_;                  // /lane/stop_line
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dbg_markers_pub_; // /lane/debug_markers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dbg_image_pub_;                // /lane/debug_image
    av::DebugTap debug_tap_;

    bool debug_markers_ = false, debug_image_ = false; // gate the two heavy debug surfaces
    std::optional<av::MapModel> map_;                  // loaded iff map_path is set; stop-line prior only
    std::ofstream lane_cloud_jsonl_;                   // per-tick cloud dump, opened lazily

    std::optional<Eigen::Vector2d> last_stop_line_ego_;
    rclcpp::Time last_path_time_;  // when last_path_ was published
    av::geom::Polyline last_path_; // last published odom-frame path, for the reuse window

    // Depth-staleness watchdog. tick() is driven ONLY by on_depth(), so a
    // stalled depth stream silently freezes /lane/path_odom (RGB-only consumers
    // keep working, which hides it). Wall-clock timestamps + a manual throttle
    // so the warning still fires when the sim clock itself is frozen.
    std::chrono::steady_clock::time_point last_depth_wall_{};
    std::chrono::steady_clock::time_point last_stall_warn_{};
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

LaneNode::LaneNode() : rclcpp::Node("lane_node"), debug_tap_(this) // constructor
{
    params_.cam_x = declare_parameter("cam_x", 2.2);          // URDF mount x (Day 2; 1.9->1.2->2.2 2026-09-05 to clear hood self-occlusion, must match sedan.urdf.xacro)
    params_.cam_z = declare_parameter("cam_z", 1.4);          // URDF mount z, also ground height post-fix
    params_.cam_pitch = declare_parameter("cam_pitch", 0.06); // URDF mount pitch, now the true value (Day 2 fix)
    params_.hfov = declare_parameter("camera_hfov", 1.9);     // must match the URDF's rgbd_camera hfov (1.6->1.9 2026-09-05: lowers fx=fy, widens vertical FOV, pulls the near-field ground limit forward - see sedan.urdf.xacro)
    params_.img_w = declare_parameter("camera_width", 424);   // must match the URDF's image width (424->500->400->424 2026-09-05: back to baseline, near coverage now comes from hfov instead of extra pixels)
    params_.img_h = declare_parameter("camera_height", 300);  // must match the URDF's image height (240->300->400->300 2026-09-05: back to baseline, ~1.0x the original 424x240 pixel-area baseline)

    params_.white_s_max = declare_parameter("white_s_max", 60);           // white: S must stay below this
    params_.white_v_min = declare_parameter("white_v_min", 150);          // white: V must stay above this
    params_.yellow_h_min = declare_parameter("yellow_h_min", 20);         // yellow hue band, lower bound
    params_.yellow_h_max = declare_parameter("yellow_h_max", 38);         // yellow hue band, upper bound
    params_.yellow_s_min = declare_parameter("yellow_s_min", 80);         // yellow: S floor
    params_.yellow_v_min = declare_parameter("yellow_v_min", 120);        // yellow: V floor
    params_.line_area_min_px = declare_parameter("line_area_min_px", 28); // drop blobs smaller than this (scaled from 50 @ 640x360; 22->28->33->43->35->28 2026-09-05 for 424x300, same area-scaling ratio: 50*(424*300)/(640*360))

    params_.z_gate = declare_parameter("z_gate", 0.15);                      // |z| tolerance around the ground plane
    params_.x_min = declare_parameter("depth_x_min", 0.5);                   // reject points closer than this (self-occlusion)
    params_.x_max = declare_parameter("depth_x_max", 33.0);                  // reject points farther than this
    params_.far_stride_dist_m = declare_parameter("far_stride_dist_m", 6.0); // stride switches at this ground distance
    params_.near_stride = declare_parameter("near_stride", 4);               // pixel stride used for near (dense-paint) rows

    params_.map_path = declare_parameter("map_path", std::string());   // MapModel::load() input
    // Origin of the odom frame in world coords = the map spawn pose. The launch
    // (av_stack / urban_world) reads it from maps/<world>.json and passes it in;
    // default 0.0 so a missing param is visibly wrong rather than silently
    // hardcoding one map's spawn (local_planner uses the same 0.0 default).
    params_.map_origin_x = declare_parameter("map_origin_x", 0.0);
    params_.map_origin_y = declare_parameter("map_origin_y", 0.0);
    params_.map_origin_yaw = declare_parameter("map_origin_yaw", 0.0);

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
    params_.chain_merge_bin_m = declare_parameter("chain_merge_bin_m", 1.5);               // B.8 -- 2x chain_resample_m so the merge genuinely averages, not re-bins the same grid
    params_.chain_min_center_pts = declare_parameter("chain_min_center_pts", 5);           // B.10
    params_.path_reuse_s = declare_parameter("path_reuse_s", 1.0);                         // B.11
    params_.chain_d_max_m = declare_parameter("chain_d_max_m", 20.0);                      // Day 6 pitfall: far depth noise
    params_.smooth_window = declare_parameter("smooth_window", 9);                         // ~6 m at 0.75 m spacing
    params_.depth_stale_warn_s = declare_parameter("depth_stale_warn_s", 1.5);             // watchdog: no-depth WARN threshold
    params_.rgb_depth_skew_max_s = declare_parameter("rgb_depth_skew_max_s", 0.20);        // skip tick past this rgb/depth stamp gap

    params_.dump_dir = declare_parameter("dump_dir", std::string()); // debug: set to enable ingest_mask() dumps
    params_.dump_every_n = declare_parameter("dump_every_n", 10);    // dump 1 out of every N ticks
    debug_markers_ = declare_parameter("debug_markers", true);       // /lane/debug_markers, publisher created below only if set
    debug_image_ = declare_parameter("debug_image", true);           // /lane/debug_image, same

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
    cmd_steer_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/mpc/cmd_steer", 10,
        [this](std_msgs::msg::Float64::ConstSharedPtr m)
        { last_cmd_steer_ = m->data; });
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/lane/path_odom", 10);
    markings_pub_ = create_publisher<std_msgs::msg::String>("/lane/markings", 10);
    stop_line_pub_ = create_publisher<std_msgs::msg::String>("/lane/stop_line", 10);
    if (debug_markers_)
        // best-effort for the same reason as debug_image below -- a slow RViz
        // must not back the publisher up into tick(). Markers carry a 0.5 s
        // lifetime so a dropped update self-clears.
        dbg_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/lane/debug_markers", rclcpp::QoS(1).best_effort());
    if (debug_image_)
        // BEST_EFFORT, depth 1: a full-frame image every tick on a RELIABLE
        // publisher will BLOCK tick() once a slow reliable subscriber (RViz
        // that can't keep up) stops acking and the history fills -- that was
        // the "lane_node stops publishing after a while" bug. A debug view may
        // drop frames; it must never stall the node. In RViz set the Image
        // display's Reliability Policy to "Best Effort" (or use rqt_image_view).
        dbg_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/lane/debug_image", rclcpp::QoS(1).best_effort());

    if (!params_.map_path.empty())
    {
        try
        {
            map_ = av::MapModel::load(params_.map_path, params_.map_origin_x,
                                      params_.map_origin_y, params_.map_origin_yaw);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "map load failed (%s) -- /lane/stop_line will be vision-only", e.what());
        }
    }

    // Seed with the node clock so the reuse-window subtraction in tick() never
    // mixes clock sources. A default-constructed rclcpp::Time is RCL_SYSTEM_TIME
    // while this->now() is RCL_ROS_TIME, and subtracting the two throws.
    last_path_time_ = this->now();

    watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]()
    {
        if (last_depth_wall_.time_since_epoch().count() == 0)
            return; // no depth received yet -- startup, not a stall
        const auto now_wall = std::chrono::steady_clock::now();
        const double gap = std::chrono::duration<double>(now_wall - last_depth_wall_).count();
        if (gap > params_.depth_stale_warn_s &&
            std::chrono::duration<double>(now_wall - last_stall_warn_).count() > 2.0)
        {
            last_stall_warn_ = now_wall;
            RCLCPP_WARN(get_logger(),
                        "no fresh depth for %.1f s -- tick() is depth-driven, so /lane/path_odom "
                        "is frozen (check the rgbd_camera sensor / ros_gz_bridge)", gap);
        }
    });
}

void LaneNode::on_rgb(sensor_msgs::msg::Image::ConstSharedPtr msg) // callback for RGB image
{
    rgb_msg_ = msg;
}

void LaneNode::on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) // callback for depth image
{
    depth_msg_ = msg;
    last_depth_wall_ = std::chrono::steady_clock::now(); // watchdog heartbeat
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
        // Throttled: this fires every frame otherwise, and a per-frame write to
        // a stalled terminal (Ctrl-S, or a slow console) blocks the whole spin
        // loop. The live value is on the debug tap as pitch_est.
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "estimated pitch %.4f rad (EMA %.4f)", *p, pitch_ema_);
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
    debug_tap_.put("added", added);                                         // Day 5 accept check: thousands on a straight
    debug_tap_.put("rej_depth", rej_depth);                                 // should be bounded, not dominant
    debug_tap_.put("rej_zgate", rej_zgate);                                 // should be bounded, not dominant
    debug_tap_.put("cloud_white", static_cast<int>(white_cloud_.size()));   // split so /lane/markings occ can be traced back
    debug_tap_.put("cloud_yellow", static_cast<int>(yellow_cloud_.size())); // to detection (this) vs classification (occ_right)

    ++ingest_tick_count_;
    if (!params_.dump_dir.empty() && ingest_tick_count_ % params_.dump_every_n == 0)
        dump_debug_frame(white_mask_raw, yellow_mask_raw, white_mask, yellow_mask,
                         white_mask_cloud, yellow_mask_cloud); // white_mask/yellow_mask are the FILTERED versions here
}

LaneNode::ChainResult LaneNode::chain_centerline()
{
    ChainResult res;

    // Cluster white and yellow SEPARATELY. On one mixed list, cluster_chains
    // (colour-blind nearest-neighbour) and merge_collinear (colour only sets
    // the gap budget) stitch the yellow centre dash onto the parallel white
    // right edge -- the merged chain then carries the yellow flag and is
    // offset the wrong way. Keeping the two colours apart until classification
    // is what the world's geometry requires.
    Chain wpts, ypts;
    for (const auto &p : white_cloud_)
        if (p.x() <= params_.chain_d_max_m) // trust bound for centreline geometry (Day 6 pitfall)
            wpts.push_back({p.x(), p.y(), false});
    for (const auto &p : yellow_cloud_)
        if (p.x() <= params_.chain_d_max_m)
            ypts.push_back({p.x(), p.y(), true});

    wpts = grid_dedup(wpts, params_.chain_grid_m);
    ypts = grid_dedup(ypts, params_.chain_grid_m);

    auto stop = detect_and_strip_stop_line(wpts, params_.chain_stop_max_m); // white points only
    if (stop.found)
    {
        last_stop_line_ego_ = Eigen::Vector2d(stop.x, stop.y_center);
        res.stop_x = stop.x;
        // A short lateral span centred on the bar, for the RViz / FPV overlay.
        res.stop_seg = std::make_pair(Eigen::Vector2d(stop.x, stop.y_center - params_.lane_half_m),
                                      Eigen::Vector2d(stop.x, stop.y_center + params_.lane_half_m));
    }

    auto wchains = cluster_chains(wpts, params_.chain_link_gap_m);
    merge_collinear(wchains, params_.chain_merge_gap_white_m,
                    params_.chain_merge_gap_yellow_m, params_.chain_merge_angle_rad);
    auto ychains = cluster_chains(ypts, params_.chain_link_gap_m);
    merge_collinear(ychains, params_.chain_merge_gap_white_m,
                    params_.chain_merge_gap_yellow_m, params_.chain_merge_angle_rad);

    std::vector<Chain> chains;
    chains.reserve(wchains.size() + ychains.size());
    for (auto &c : wchains)
        chains.push_back(std::move(c));
    for (auto &c : ychains)
        chains.push_back(std::move(c));

    drop_stray_chains(chains, params_.chain_min_pts, params_.chain_min_extent_m);
    for (auto &c : chains)
        c = resample_chain_x(c, params_.chain_resample_m);

    if (debug_tap_.enabled())
    {
        std::string s;
        for (const auto &c : chains)
        {
            if (c.empty())
                continue;
            char b[96];
            std::snprintf(b, sizeof(b), "[n=%zu %s f(%.1f,%.2f) b(%.1f,%.2f)] ",
                          c.size(), c.front().yellow ? "Y" : "W",
                          c.front().x, c.front().y, c.back().x, c.back().y);
            s += b;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "chains: %s", s.c_str());
    }

    auto cls = classify_and_offset(chains, params_.lane_half_m, lane_mem_y_, have_lane_mem_,
                                   params_.chain_d_max_m, params_.chain_resample_m);
    res.right_edge = cls.right_edge;   // reported even on an otherwise-invalid frame
    res.center_line = cls.center_line; // so /lane/markings keeps updating through dropouts

    auto center = xbin_merge(cls.offsets, params_.chain_merge_bin_m);

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

// /lane/markings: per-side boundary type from forward-bin occupancy. A solid
// line covers nearly every bin out to chain_d_max_m; the dashed centre line
// covers roughly a third (3 m paint in a 7.5 m cycle); no line, almost none.
void LaneNode::publish_markings(const ChainResult &chain)
{
    // quality: fraction of the observed span that is actually painted (~1.0
    // solid, ~0.4 dashed). type: from the gap STRUCTURE, not the raw ratio --
    // see marking_type().
    const double bin_m = params_.chain_resample_m;
    auto quality = [bin_m](const MarkingStats &s)
    {
        return s.span_m > 1e-6 ? s.n_bins / (s.span_m / bin_m) : 0.0;
    };
    av::json j = {
        {"stamp", this->now().seconds()},
        {"left", {{"type", marking_type(chain.center_line)}, {"quality", quality(chain.center_line)}}},
        {"right", {{"type", marking_type(chain.right_edge)}, {"quality", quality(chain.right_edge)}}},
        {"in_intersection", false}, // no intersections on the course world; kept for schema parity
    };
    markings_pub_->publish(av::to_msg(j));
}

// /lane/stop_line: vision distance from detect_and_strip_stop_line, optionally
// fused 0.7/0.3 with the map's course stop line once the two agree within 3 m.
void LaneNode::publish_stop_line(const ChainResult &chain)
{
    std::optional<double> vision_x = chain.stop_x; // ego-frame forward distance

    std::optional<double> map_x;
    std::string map_id;
    if (map_ && map_->course_stop_line)
    {
        Eigen::Vector2d sl = current_pose_.base_of(*map_->course_stop_line);
        if (sl.x() > 0.5 && sl.x() < 40.0 && std::abs(sl.y()) < 4.0)
        {
            map_x = sl.x();
            map_id = "course_stop";
        }
    }

    if (!vision_x && !map_x)
        return; // nothing to say -- stay silent until the bar is in view

    double dist;
    std::string source;
    if (vision_x && map_x && std::abs(*vision_x - *map_x) < 3.0)
    {
        dist = 0.7 * *vision_x + 0.3 * *map_x;
        source = "fused";
    }
    else if (vision_x)
    {
        dist = *vision_x;
        source = "vision";
    }
    else
    {
        dist = *map_x;
        source = "map";
    }
    av::json j = {{"stamp", this->now().seconds()}, {"distance", dist}, {"source", source}, {"map_id", map_id}};
    stop_line_pub_->publish(av::to_msg(j));
}

// Curvature of the last ~4 m of the ego centreline -- the honest chains-mode
// equivalent of the HTML plan's voxelwalk "wkappa": it says whether the frame
// captured a curve or only a straight stub.
double LaneNode::path_end_curvature(const av::geom::Polyline &c) const
{
    if (c.size() < 5)
        return 0.0;
    const size_t n = c.size();
    return av::geom::curvature_menger(c[n - 5], c[n - 3], c[n - 1]);
}

// RViz overlay (debug_markers param), everything in the odom frame,
// transformed with the CAPTURE-time pose so the markers line up with
// /lane/path_odom. No gate boxes -- that is a voxelwalk concept.
void LaneNode::publish_debug_markers(const ChainResult &chain, double wkappa)
{
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = this->now();

    auto base = [&](const char *ns, int id, int type)
    {
        Marker m;
        m.header.stamp = stamp;
        m.header.frame_id = "odom";
        m.ns = ns;
        m.id = id;
        m.type = type;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.lifetime = rclcpp::Duration::from_seconds(0.5); // self-clear if the node stalls
        return m;
    };
    auto pt = [](const Eigen::Vector2d &p, double z)
    {
        geometry_msgs::msg::Point q;
        q.x = p.x();
        q.y = p.y();
        q.z = z;
        return q;
    };
    auto rgba = [](float r, float g, float b, float a)
    {
        std_msgs::msg::ColorRGBA col;
        col.r = r;
        col.g = g;
        col.b = b;
        col.a = a;
        return col;
    };

    Marker mw = base("cloud_white", 0, Marker::POINTS);
    mw.scale.x = mw.scale.y = 0.12;
    mw.color = rgba(0.95f, 0.95f, 0.95f, 0.8f);
    for (const auto &p : white_cloud_)
        mw.points.push_back(pt(frame_pose_.odom_of(p), 0.02));
    if (mw.points.empty())
        mw.action = Marker::DELETE;
    arr.markers.push_back(mw);

    Marker my = base("cloud_yellow", 1, Marker::POINTS);
    my.scale.x = my.scale.y = 0.12;
    my.color = rgba(1.0f, 0.85f, 0.1f, 0.9f);
    for (const auto &p : yellow_cloud_)
        my.points.push_back(pt(frame_pose_.odom_of(p), 0.02));
    if (my.points.empty())
        my.action = Marker::DELETE;
    arr.markers.push_back(my);

    Marker mc = base("centerline", 2, Marker::LINE_STRIP);
    mc.scale.x = 0.08;
    mc.color = rgba(0.1f, 1.0f, 0.3f, 1.0f);
    // smoothed, like /lane/path_odom -- smoothing is rigid-frame-invariant so
    // smoothing the ego centreline then transforming == what tick() publishes.
    for (const auto &p : av::geom::smooth_moving_avg(chain.centerline, params_.smooth_window))
        mc.points.push_back(pt(frame_pose_.odom_of(p), 0.05));
    if (mc.points.size() < 2)
        mc.action = Marker::DELETE;
    arr.markers.push_back(mc);

    Marker ms = base("stop_line", 3, Marker::LINE_LIST);
    ms.scale.x = 0.15;
    ms.color = rgba(1.0f, 0.15f, 0.15f, 1.0f);
    if (chain.stop_seg)
    {
        ms.points.push_back(pt(frame_pose_.odom_of(chain.stop_seg->first), 0.05));
        ms.points.push_back(pt(frame_pose_.odom_of(chain.stop_seg->second), 0.05));
    }
    else
        ms.action = Marker::DELETE;
    arr.markers.push_back(ms);

    Marker mx = base("state", 4, Marker::TEXT_VIEW_FACING);
    mx.scale.z = 0.6;
    mx.color = chain.valid ? rgba(1.0f, 1.0f, 1.0f, 0.9f) : rgba(1.0f, 0.3f, 0.3f, 0.9f);
    char buf[96];
    std::snprintf(buf, sizeof(buf), "pts %zu  wk %+.3f  %s",
                  chain.centerline.size(), wkappa, chain.valid ? "valid" : "INVALID");
    mx.text = buf;
    mx.pose.position.x = current_pose_.x;
    mx.pose.position.y = current_pose_.y;
    mx.pose.position.z = 2.5;
    arr.markers.push_back(mx);

    dbg_markers_pub_->publish(arr);
}

// FPV overlay (debug_image param): the current camera frame with the ego cloud
// and centreline projected back through the pinhole. Hand-encoded rgb8 (no
// cv_bridge). Colour scalars are in RGB order because the output is rgb8.
void LaneNode::publish_debug_image(const ChainResult &chain, double wkappa)
{
    cv::Mat img = rgb_.clone(); // rgb8

    auto px = [&](const Eigen::Vector2d &p) -> std::optional<cv::Point>
    {
        auto uv = av::proj::base_to_pixel(pin_, T_base_cam_, {p.x(), p.y(), 0.0});
        if (!uv || uv->x() < -50 || uv->x() > img.cols + 50 || uv->y() < -50 || uv->y() > img.rows + 50)
            return std::nullopt;
        return cv::Point(static_cast<int>(uv->x()), static_cast<int>(uv->y()));
    };

    const cv::Scalar col_white(0, 255, 255), col_yellow(255, 200, 25),
        col_center(30, 255, 90), col_stop(255, 40, 40);

    for (const auto &p : white_cloud_)
        if (auto q = px(p))
            cv::circle(img, *q, 2, col_white, cv::FILLED);
    for (const auto &p : yellow_cloud_)
        if (auto q = px(p))
            cv::circle(img, *q, 2, col_yellow, cv::FILLED);

    std::optional<cv::Point> prev;
    for (const auto &p : av::geom::smooth_moving_avg(chain.centerline, params_.smooth_window)) // as published
    {
        auto q = px(p);
        if (q && prev)
            cv::line(img, *prev, *q, col_center, 3, cv::LINE_AA);
        if (q)
            cv::circle(img, *q, 4, col_center, cv::FILLED);
        prev = q;
    }

    if (chain.stop_seg)
    {
        auto a = px(chain.stop_seg->first), b = px(chain.stop_seg->second);
        if (a && b)
            cv::line(img, *a, *b, col_stop, 4, cv::LINE_AA);
    }

    char buf[96];
    std::snprintf(buf, sizeof(buf), "pts %zu  wk %+.3f  %s",
                  chain.centerline.size(), wkappa, chain.valid ? "valid" : "INVALID");
    cv::putText(img, buf, {8, 22}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                chain.valid ? cv::Scalar(255, 255, 255) : col_stop, 1, cv::LINE_AA);

    // latest /mpc/cmd_steer, purely for eyeballing the commanded steer
    // against the detected lane in the same frame (see cmd_steer_sub_).
    char steer_buf[48];
    std::snprintf(steer_buf, sizeof(steer_buf), "steer %+.3f rad", last_cmd_steer_);
    cv::putText(img, steer_buf, {8, 44}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(255, 255, 0), 1, cv::LINE_AA);

    sensor_msgs::msg::Image out;
    out.header.stamp = this->now();
    out.header.frame_id = rgb_msg_->header.frame_id;
    out.height = img.rows;
    out.width = img.cols;
    out.encoding = "rgb8";
    out.is_bigendian = 0;
    out.step = img.cols * 3;
    out.data.assign(img.data, img.data + out.step * img.rows);
    dbg_image_pub_->publish(out);
}

// Per-tick JSONL for offline replay: ego cloud snapshot + centreline + pose.
// Distinct from dump_debug_frame's per-N mask PNGs -- cheap enough every tick.
void LaneNode::dump_lane_cloud_jsonl(const ChainResult &chain, double wkappa)
{
    if (params_.dump_dir.empty())
        return;
    if (!lane_cloud_jsonl_.is_open())
        lane_cloud_jsonl_.open(params_.dump_dir + "/lane_cloud.jsonl", std::ios::app);
    if (!lane_cloud_jsonl_.is_open())
        return; // directory missing -- dump_debug_frame already logs that loudly

    auto r2 = [](double v)
    { return std::round(v * 100.0) / 100.0; };
    av::json pj = av::json::array();
    for (const auto &p : white_cloud_)
        pj.push_back({r2(p.x()), r2(p.y()), 0});
    for (const auto &p : yellow_cloud_)
        pj.push_back({r2(p.x()), r2(p.y()), 1});
    av::json wj = av::json::array();
    for (const auto &p : chain.centerline)
        wj.push_back({r2(p.x()), r2(p.y())});

    av::json line = {
        {"t", this->now().seconds()},
        {"pose", {frame_pose_.x, frame_pose_.y, frame_pose_.yaw}},
        {"wkappa", wkappa},
        {"valid", chain.valid},
        {"pts", pj},
        {"center", wj},
    };
    lane_cloud_jsonl_ << line.dump() << "\n";
}

void LaneNode::tick()
try
{
    const auto t_start = std::chrono::steady_clock::now(); // tick_ms on the tap: is the node or the camera the rate limit?

    // RGB/depth skew guard: ingest() resolves frame_pose_ from the DEPTH stamp
    // but ingest_mask() reads the latest RGB. If the two streams have drifted
    // apart (depth stalling under render load), a current-view detection would
    // be planted at a stale pose -- the "centreline stuck at the origin" bug.
    // Skip the frame; the reuse window covers a brief gap, a persistent one
    // SHOULD stop the path so the planner isn't fed garbage.
    {
        const double rgb_t = rgb_msg_->header.stamp.sec + 1e-9 * rgb_msg_->header.stamp.nanosec;
        const double dep_t = depth_msg_->header.stamp.sec + 1e-9 * depth_msg_->header.stamp.nanosec;
        if (std::abs(rgb_t - dep_t) > params_.rgb_depth_skew_max_s)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "RGB/depth stamp skew %.2f s > %.2f s -- skipping frame",
                                 std::abs(rgb_t - dep_t), params_.rgb_depth_skew_max_s);
            return;
        }
    }

    ingest();      // wrap Mats, resolve frame_pose_, update pitch_ema_/T_base_cam_
    ingest_mask(); // build white_cloud_/yellow_cloud_ for THIS frame

    ChainResult chain = chain_centerline();
    const double wkappa = path_end_curvature(chain.centerline);
    rclcpp::Time now = this->now();

    if (chain.valid)
    {
        av::geom::Polyline odom_path;
        for (const auto &p : chain.centerline)
            odom_path.push_back(frame_pose_.odom_of(p));                           // ego -> odom
        odom_path = av::geom::smooth_moving_avg(odom_path, params_.smooth_window); // kill x-bin jitter
        odom_path = av::geom::resample_uniform(odom_path, 0.5);                    // even 0.5 m spacing for downstream consumers
        last_path_ = odom_path;
        last_path_time_ = now;
        publish_path(odom_path);
    }
    else if (!last_path_.empty() && (now - last_path_time_).seconds() < params_.path_reuse_s)
    {
        publish_path(last_path_); // still inside the reuse window -- republish the last good path
    }

    publish_markings(chain);  // always -- occupancy is live even on an invalid frame
    publish_stop_line(chain); // silent unless a bar (or a map prior) is in view

    if (debug_markers_)
        publish_debug_markers(chain, wkappa);
    if (debug_image_)
        publish_debug_image(chain, wkappa);
    dump_lane_cloud_jsonl(chain, wkappa);

    debug_tap_.put("cloud", static_cast<int>(white_cloud_.size() + yellow_cloud_.size()));
    debug_tap_.put("occ_right", chain.right_edge.n_bins);
    debug_tap_.put("occ_center", chain.center_line.n_bins);
    debug_tap_.put("right_gap_m", chain.right_edge.max_gap_m);
    debug_tap_.put("center_gap_m", chain.center_line.max_gap_m);
    debug_tap_.put("wkappa", wkappa);
    debug_tap_.put("valid", chain.valid);
    debug_tap_.put("tick_ms", std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t_start)
                                  .count());
    debug_tap_.flush();
}
catch (const std::exception &e)
{
    // A single bad frame (OpenCV assertion, projection blow-up, ...) must not
    // std::terminate the whole node -- log it, drop the frame, keep spinning.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tick(): %s", e.what());
}

// ---------------------------------------------------------------- main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneNode>());
    rclcpp::shutdown();
    return 0;
}
