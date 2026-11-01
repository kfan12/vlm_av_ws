#pragma once
#include <algorithm>
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "av_common/point_cloud.hpp"
#include "av_safety_cpp/depth_obstacle_extractor.hpp"

namespace av::safety
{
    struct LidarParams
    {
        double mount_x = 2.30; // sedan.urdf.xacro lidar_x (base_link, unrotated mount)
        double mount_z = 0.60; // sedan.urdf.xacro lidar_z (height above the road)
        double min_range_m = 0.3;
        double max_range_m = 40.0;
        // far fewer points per obstacle than the depth image: a 0.6 m box at
        // 30 m is ~5 points, split across up to four 0.5 m cells
        int min_cluster_points = 3;
    };

    inline std::vector<ObstacleCluster> extract_lidar_clusters(const sensor_msgs::msg::PointCloud2 &cloud,
                                                               const LidarParams &lp,
                                                               const ObstacleFilterParams &fp)
    {
        av::cloud::CloudFilter f;
        f.min_range_m = lp.min_range_m;
        f.max_range_m = lp.max_range_m;
        f.max_lateral_m = fp.max_lateral_m;
        f.min_height_m = std::max(fp.ground_removal_m, fp.min_height_m);
        f.max_height_m = fp.max_height_m;
        auto pts = av::cloud::cloud_points_base(cloud, lp.mount_x, lp.mount_z, f);
        return to_obstacle_clusters(av::cloud::voxel_clusters(pts, fp.cluster_voxel_m, lp.min_cluster_points));
    }
} // namespace av::safety