#pragma once

// Point helpers shared by every node that turns range data into obstacles:
// PointCloud2 -> base_link points, and the 2D voxel clustering both the depth
// and the lidar obstacle extractors use, so a "cluster" means the same thing
// whichever sensor produced it.

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace av::cloud
{
    struct PointCluster
    {
        Eigen::Vector3d mean{0, 0, 0}; // base_link coordinates of the cluster's mean point
        double nearest_x = 0.0;        // nearest x coordinate of any point in the cluster
        int n_points = 0;              // number of points in the cluster
    };

    // Buckets points into an (x, y) grid of voxel_m cells and returns one
    // cluster per cell holding at least min_points. Height is ignored on
    // purpose: a pole and its base are one obstacle. Cells come out in
    // (ix, iy) order, so the result is deterministic.

    inline std::vector<PointCluster> voxel_clusters(const std::vector<Eigen::Vector3d> &pts,
                                                    double voxel_m, int min_points)
    {
        std::vector<PointCluster> out;

        std::map<std::pair<int, int>, std::vector<const Eigen::Vector3d *>> cells;

        for (const auto &p : pts)
        {
            int ix = static_cast<int>(std::floor(p.x() / voxel_m));
            int iy = static_cast<int>(std::floor(p.y() / voxel_m));
            cells[{ix, iy}].push_back(&p);
        }

        for (const auto &[_, cell] : cells)
        {
            if (static_cast<int>(cell.size()) < min_points)
                continue;

            PointCluster c;
            c.nearest_x = std::numeric_limits<double>::max();
            for (const auto *p : cell)
            {
                c.mean += *p;
                c.nearest_x = std::min(c.nearest_x, p->x());
            }
            c.mean /= static_cast<double>(cell.size());
            c.n_points = static_cast<int>(cell.size());
            out.push_back(c);
        }

        return out;
    }

    struct CloudFilter
    {
        double min_range_m = 0.0;   // minimum range to keep points
        double max_range_m = 40.0;  // maximum range to keep points
        double max_lateral_m = 5.0; // |y| in base_link coordinates to keep points
        double min_height_m = 0.15; // z in base_link coordinates to keep points = height above the ground
        double max_height_m = 3.0;
    };

    // Sensor-frame cloud -> base_link points that survive the filter. The
    // sensor is mounted UNROTATED at (mount_x, 0, mount_z) in base_link, so the
    // transform is a translation
    inline std::vector<Eigen::Vector3d> cloud_points_base(const sensor_msgs::msg::PointCloud2 &cloud,
                                                          double mount_x, double mount_z, const CloudFilter &f)
    {
        std::vector<Eigen::Vector3d> out;
        out.reserve(cloud.width * cloud.height / 4); // rough guess to avoid too many reallocs
        sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
        {
            const double x = *it_x, y = *it_y, z = *it_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            const double r = std::sqrt(x * x + y * y + z * z);
            if (r < f.min_range_m || r > f.max_range_m)
                continue;

            Eigen::Vector3d p_base(x + mount_x, y, z + mount_z);
            if (std::abs(p_base.y()) > f.max_lateral_m)
                continue;
            if (p_base.z() < f.min_height_m || p_base.z() > f.max_height_m)
                continue;

            out.push_back(p_base);
        }

        return out;
    }

} // namespace av::cloud