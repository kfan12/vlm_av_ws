#pragma once
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include "mpc/vehicle_model.hpp"

struct PathPoint
{
    double x, y, psi, s; // position, heading, arc length along path
};

// convert nav_msgs::msg::Path to std::vector<PathPoint>
std::vector<PathPoint> path_msg_to_points(const nav_msgs::msg::Path &path);

// find the index of the closest point on the path to current vehicle state
size_t find_closest_point(
    const std::vector<PathPoint> &path,
    const VehicleState &state,
    double search_radius = 2.0);

// resample a path uniformally along arc length, using linear interpolation, starting from start_idx
std::vector<PathPoint> resample_path(
    const std::vector<PathPoint> &path,
    size_t start_idx,
    int N, // number of points to sample, horizon steps
    double ds);