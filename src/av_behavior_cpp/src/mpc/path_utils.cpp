#include "mpc/path_utils.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <limits>

// convert nav_msgs::msg::Path to std::vector<PathPoint>
std::vector<PathPoint> path_msg_to_points(const nav_msgs::msg::Path &path)
{
    std::vector<PathPoint> points;
    double s = 0.0;

    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        const auto &pose = path.poses[i].pose;
        PathPoint p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.psi = tf2::getYaw(pose.orientation);
        if (i > 0)
        {
            double dx = p.x - points.back().x;
            double dy = p.y - points.back().y;
            s += std::hypot(dx, dy);
        }
        p.s = s;
        points.push_back(p);
    }
    return points;
}

// find the index of the closest point on the path to current vehicle state.
// Prefers a point within search_radius (keeps the search local/robust on
// self-intersecting paths), but if the vehicle has drifted farther than
// search_radius from EVERY point - large e_lat mid-turn, off-course - falls
// back to the true globally nearest point rather than silently returning
// index 0. Returning 0 here used to hand the solver a foot point unrelated
// to the vehicle's actual position, computing e_lat/e_head/kappa against the
// wrong part of the path and commanding a bogus correction: exactly the
// runaway divergence a large cross-track error should recover from, not
// trigger (2026-09-04 turn-settling diagnosis).
size_t find_closest_point(const std::vector<PathPoint> &path, const VehicleState &state, double search_radius)
{
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max(); // best within search_radius
    size_t best_idx = 0;
    double best_dist = std::numeric_limits<double>::max(); // best overall

    for (size_t i = 0; i < path.size(); ++i)
    {
        double dx = path[i].x - state.x;
        double dy = path[i].y - state.y;
        double dist = std::hypot(dx, dy);
        if (dist < best_dist)
        {
            best_dist = dist;
            best_idx = i;
        }
        if (dist < min_dist && dist <= search_radius)
        {
            min_dist = dist;
            closest_idx = i;
        }
    }

    return (min_dist <= search_radius) ? closest_idx : best_idx;
}

// resample a path uniformally along arc length, using linear interpolation, starting from start_idx
std::vector<PathPoint> resample_path(const std::vector<PathPoint> &path, size_t start_idx, int N, double ds)
{
    std::vector<PathPoint> resampled_path;
    if (path.empty() || start_idx >= path.size())
        return resampled_path;

    double s_start = path[start_idx].s;
    size_t idx = start_idx;

    for (int i = 0; i < N; ++i)
    {
        double s_target = s_start + i * ds;
        while (idx + 1 < path.size() && path[idx + 1].s < s_target)
            ++idx;
        if (idx + 1 >= path.size())
            resampled_path.push_back(path.back()); // last point if we run out of points
        else
        {
            double ratio = 1.0 * (s_target - path[idx].s) / (path[idx + 1].s - path[idx].s + 1e-9);
            PathPoint interpolated;
            interpolated.x = path[idx].x + ratio * (path[idx + 1].x - path[idx].x);
            interpolated.y = path[idx].y + ratio * (path[idx + 1].y - path[idx].y);
            interpolated.psi = path[idx].psi + ratio * (path[idx + 1].psi - path[idx].psi);
            interpolated.s = s_target;
            resampled_path.push_back(interpolated);
        }
    }

    return resampled_path;
}