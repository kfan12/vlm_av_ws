#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <fstream>

#include "av_common/projection.hpp"
#include "av_common/debug_tap.hpp"
#include "av_common/geometry.hpp"
#include "av_perception_cpp/img_access.hpp"
#include "av_perception_cpp/ground_cal.hpp"

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
    };

    void on_rgb(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
    av::proj::Pose2d pose_at(double stamp_sec) const; // nearst-timestamp pose from pose_hist_, or current_pose_ if none
    void ingest();
    void ingest_mask();
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
    av::DebugTap debug_tap_;
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

void LaneNode::tick()
{
    ingest();      // wrap Mats, resolve frame_pose_, update pitch_ema_/T_base_cam_
    ingest_mask(); // build white_cloud_/yellow_cloud_ for THIS frame
    // Day 6 adds chain_centerline() + publish here.

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
