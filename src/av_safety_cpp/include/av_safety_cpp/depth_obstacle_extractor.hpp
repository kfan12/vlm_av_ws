#pragma once

// Depth image -> clustered obstacle points in base_link. Deliberately
// independent of av_perception_cpp (AEB must not share fate with the lane
// pipeline): this header duplicates the small zero-copy depth-access pattern
// instead of depending on av_perception_cpp/img_access.hpp.

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <map>

#include "av_common/projection.hpp"
#include "av_common/point_cloud.hpp"

namespace av::safety
{
    struct ObstacleFilterParams
    {
        double min_depth_m = 0.5;
        double max_depth_m = 40.0;
        double max_lateral_m = 5.0;
        double min_height_m = 0.15;
        double max_height_m = 3.0;
        double ground_removal_m = 0.1;
        double cluster_voxel_m = 0.5;
        int min_cluster_points = 5;
    };

    struct ObstacleCluster
    {
        Eigen::Vector3d centroid{0, 0, 0}; // base_link frame; x() holds the NEAREST point's depth, not the mean
        int n_points = 0;
    };

    // same zero-copy aliasing as av_perception_cpp/img_access.hpp::as_depth,
    // duplicated on purpose
    inline cv::Mat depth_as_mat(const sensor_msgs::msg::Image::ConstSharedPtr &m)
    {
        return cv::Mat(m->height, m->width, CV_32FC1,
                       const_cast<uint8_t *>(m->data.data()), m->step);
    }

    // Convert a vector of PointCluster to ObstacleCluster, keeping the nearest x
    inline std::vector<ObstacleCluster> to_obstacle_clusters(const std::vector<av::cloud::PointCluster> &in)
    {
        std::vector<ObstacleCluster> out;
        out.reserve(in.size());
        for (const auto &c : in)
        {
            ObstacleCluster o;
            o.centroid = c.mean;
            o.centroid.x() = c.nearest_x;
            o.n_points = c.n_points;
            out.push_back(o);
        }
        return out;
    }

    inline std::vector<ObstacleCluster> extract_clusters(
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_image,
        const av::proj::Pinhole &pin,
        const Eigen::Isometry3d &T_base_cam,
        const ObstacleFilterParams &fp,
        int stride = 2)
    {
        cv::Mat depth = depth_as_mat(depth_image);
        std::vector<Eigen::Vector3d> pts;
        const double min_h = std::max(fp.ground_removal_m, fp.min_height_m);
        for (int v = 0; v < depth.rows; v += stride)
        {
            for (int u = 0; u < depth.cols; u += stride)
            {
                float d = depth.at<float>(v, u);
                if (!std::isfinite(d) || d < fp.min_depth_m || d > fp.max_depth_m)
                    continue;
                Eigen::Vector3d p_base = T_base_cam * av::proj::pixel_depth_to_camera(pin, u, v, d);
                if (std::abs(p_base.y()) > fp.max_lateral_m)
                    continue;
                // base_link sits at ground level, so p_base.z() is height above ground
                if (p_base.z() < min_h || p_base.z() > fp.max_height_m)
                    continue;
                pts.push_back(p_base);
            }
        }
        return to_obstacle_clusters(av::cloud::voxel_clusters(pts, fp.cluster_voxel_m, fp.min_cluster_points));
    }

} // namespace av::safety