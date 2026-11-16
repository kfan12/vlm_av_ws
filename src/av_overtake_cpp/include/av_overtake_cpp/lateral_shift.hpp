#pragma once

// Lateral offset of a lane path, as a function of a world-stable arc length.
//
// "World-stable" means the same piece of road keeps the same s from one lane
// path to the next: s = ego progress along the lane (odometry) + the point's
// arc length past the ego's projection. Indexing the offset from each path's
// first point instead would move the whole shift about 4 m forward with every
// camera frame, because lane_node's path always starts just ahead of the car.

#include <algorithm>
#include <cmath>

#include "av_common/geometry.hpp"

namespace av::overtake
{
    // 0->1 with zero slope and zero curvature at both ends(quintic), so the
    // shifted path has no curvature step where the shift starts or ends.
    inline double smoothstep5(double x)
    {
        x = std::clamp(x, 0.0, 1.0);
        return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
    }

    // d0 before s0, a quintic blend to d1 over `length`, d1 after.
    struct ShiftProfile
    {
        double d0 = 0.0, d1 = 0.0, s0 = 0.0, length = 1.0;

        double at(double s) const { return d0 + (d1 - d0) * smoothstep5((s - s0) / length); }
        bool done(double s) const { return s >= s0 + length; }
        bool zero() const { return d0 == 0.0 && d1 == 0.0; }
    };

    // A new shift toward d_target starting at s_now, from wherever `cur` is at
    // s_now, so the offset at the ego is continuous.
    inline ShiftProfile retarget(const ShiftProfile &cur, double s_now, double d_target, double length)
    {
        return {cur.at(s_now), d_target, s_now, std::max(length, 1e-3)};
    }

    // Arc length of q along path, negative for a point before the first point
    // (project_point clamps those to s = 0).
    inline double signed_s(const geom::Polyline &path, const Eigen::Vector2d &q)
    {
        const auto pr = geom::project_point(path, q);
        if (path.size() >= 2 && pr.seg == 0 && pr.s < 1e-6)
        {
            const Eigen::Vector2d d = (path[1] - path[0]).normalized();
            return (q - path[0]).dot(d); // negative if q is before the first point, positive if after
        }
        return pr.s;
    }

    // Every point moved to its left by prof.at(s_world), where
    // s_world = s_ego_world + (s_i - s_ego_on_path).
    inline geom::Polyline offset_path(const geom::Polyline &path, const ShiftProfile &prof,
                                      double s_ego_world, double s_ego_on_path)
    {
        geom::Polyline out;
        out.reserve(path.size());
        const auto s = geom::arc_length(path);
        for (size_t i = 0; i < path.size(); ++i)
        {
            const size_t j = std::min(i + 1, path.size() - 1);
            const size_t k = j > 0 ? j - 1 : 0;
            Eigen::Vector2d t = path[j] - path[k];
            const double n = t.norm();
            t = n < 1e-9 ? Eigen::Vector2d(1.0, 0.0) : Eigen::Vector2d(t / n);
            const Eigen::Vector2d left(-t.y(), t.x());
            out.push_back(path[i] + prof.at(s_ego_world + s[i] - s_ego_on_path) * left);
        }
        return out;
    }

} // namespace av::overtake