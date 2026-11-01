#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "av_common/projection.hpp"
#include "av_safety_cpp/depth_obstacle_extractor.hpp"
#include "av_safety_cpp/lidar_obstacle_extractor.hpp"

namespace
{
    struct Nearest
    {
        double x = -1.0; // -1 = nothing in the lane band
        double y = 0.0;
        int n_points = 0;
        int n_clusters = 0;
        double ms = 0.0; // extractor wall time
    };

    Nearest nearest_in_band(const std::vector<av::safety::ObstacleCluster> &cl, double half_band, double ms)
    {
        Nearest n;
        n.n_clusters = static_cast<int>(cl.size());
        n.ms = ms;
        for (const auto &c : cl)
            if (c.centroid.x() > 0.0 && std::abs(c.centroid.y()) < half_band && (n.x < 0 || c.centroid.x() < n.x))
            {
                n.x = c.centroid.x();
                n.y = c.centroid.y();
                n.n_points = c.n_points;
            }
        return n;
    }

    std::string json(const Nearest &n)
    {
        char b[160];
        std::snprintf(b, sizeof(b), "{\"x\": %.3f, \"y\": %.3f, \"n_points\": %d, \"n_clusters\": %d, \"ms\": %.2f}",
                      n.x, n.y, n.n_points, n.n_clusters, n.ms);
        return b;
    }
} // namespace

class ObstacleProbe : public rclcpp::Node
{
public:
    ObstacleProbe() : rclcpp::Node("obstacle_probe")
    {
        double cam_x = declare_parameter("cam_x", 2.2);
        double cam_z = declare_parameter("cam_z", 1.4);
        double cam_pitch = declare_parameter("cam_pitch", 0.06);
        int img_w = declare_parameter("image_width", 424);
        int img_h = declare_parameter("image_height", 300);
        double hfov = declare_parameter("horizontal_fov", 1.9);
        T_base_cam_ = av::proj::make_T_base_cam(cam_x, 0.0, cam_z, cam_pitch);
        double fx = (img_w / 2.0) / std::tan(hfov / 2.0);
        pin_ = {fx, fx, img_w / 2.0, img_h / 2.0};

        fp_.min_depth_m = declare_parameter("min_depth_m", 0.5);
        fp_.max_depth_m = declare_parameter("max_depth_m", 40.0);
        fp_.max_lateral_m = declare_parameter("max_lateral_m", 5.0);
        fp_.min_height_m = declare_parameter("min_obstacle_height_m", 0.15);
        fp_.max_height_m = declare_parameter("max_obstacle_height_m", 3.0);
        fp_.ground_removal_m = declare_parameter("ground_removal_threshold_m", 0.1);
        fp_.cluster_voxel_m = declare_parameter("cluster_voxel_m", 0.5);
        fp_.min_cluster_points = declare_parameter("minimum_cluster_points", 5);

        lp_.mount_x = declare_parameter("lidar_x", 2.30);
        lp_.mount_z = declare_parameter("lidar_z", 0.60);
        lp_.min_range_m = declare_parameter("lidar_min_range_m", 0.3);
        lp_.max_range_m = declare_parameter("lidar_max_range_m", 40.0);
        lp_.min_cluster_points = declare_parameter("lidar_minimum_cluster_points", 3);

        half_band_ = declare_parameter("lane_half_band_m", 1.2); // vehicle_width/2 + lateral_margin
        log_.open(declare_parameter("log_path", std::string("/tmp/obstacle_probe.jsonl")), std::ios::app);

        sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/front/depth/image_raw", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr m)
            { depth_ = m; });
        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/front/points", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m)
            { cloud_ = m; });
        sub_truth_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom_truth", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m)
            { truth_ = m; });
        pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/safety/probe/clusters", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]
                                   { sample(); });
    }

private:
    template <class F>
    static std::pair<std::vector<av::safety::ObstacleCluster>, double> timed(F f)
    {
        auto t0 = std::chrono::steady_clock::now();
        auto cl = f();
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        return {cl, ms};
    }

    void sample()
    {
        if (!truth_)
            return;
        Nearest d, l;
        std::vector<av::safety::ObstacleCluster> dcl, lcl;
        if (depth_)
        {
            auto [cl, ms] = timed([&]
                                  { return av::safety::extract_clusters(depth_, pin_, T_base_cam_, fp_); });
            dcl = cl;
            d = nearest_in_band(cl, half_band_, ms);
        }
        if (cloud_)
        {
            auto [cl, ms] = timed([&]
                                  { return av::safety::extract_lidar_clusters(*cloud_, lp_, fp_); });
            lcl = cl;
            l = nearest_in_band(cl, half_band_, ms);
        }
        double wall = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        const auto &p = truth_->pose.pose.position;
        char head[128];
        std::snprintf(head, sizeof(head), "{\"wall\": %.3f, \"car_x\": %.3f, \"car_y\": %.3f, ", wall, p.x, p.y);
        log_ << head << "\"depth\": " << json(d) << ", \"lidar\": " << json(l) << "}\n";
        log_.flush();
        publish_markers(dcl, lcl);
    }

    void publish_markers(const std::vector<av::safety::ObstacleCluster> &dcl,
                         const std::vector<av::safety::ObstacleCluster> &lcl)
    {
        visualization_msgs::msg::MarkerArray arr;
        int id = 0;
        auto add = [&](const std::vector<av::safety::ObstacleCluster> &cl, const char *ns, float r, float g, float b)
        {
            for (const auto &c : cl)
            {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = "base_link";
                m.header.stamp = now();
                m.ns = ns;
                m.id = id++;
                m.type = visualization_msgs::msg::Marker::CUBE;
                m.pose.position.x = c.centroid.x();
                m.pose.position.y = c.centroid.y();
                m.pose.position.z = c.centroid.z();
                m.pose.orientation.w = 1.0;
                m.scale.x = m.scale.y = m.scale.z = 0.25;
                m.color.r = r;
                m.color.g = g;
                m.color.b = b;
                m.color.a = 0.8;
                m.lifetime = rclcpp::Duration::from_seconds(0.4);
                arr.markers.push_back(m);
            }
        };
        add(dcl, "depth", 0.2f, 0.9f, 0.2f);
        add(lcl, "lidar", 0.2f, 0.5f, 1.0f);
        pub_markers_->publish(arr);
    }

    Eigen::Isometry3d T_base_cam_;
    av::proj::Pinhole pin_;
    av::safety::ObstacleFilterParams fp_;
    av::safety::LidarParams lp_;
    double half_band_ = 1.2;
    std::ofstream log_;

    sensor_msgs::msg::Image::ConstSharedPtr depth_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
    nav_msgs::msg::Odometry::ConstSharedPtr truth_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_truth_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleProbe>());
    rclcpp::shutdown();
    return 0;
}
