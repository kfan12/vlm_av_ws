#pragma once
// Polyline math shared by lane_node / local_planner / behavior /MPC -v2.
// All polylines are 2-D odom-frame point lists.

#include <cmath>
#include <cstddef>
#include <vector>
#include <Eigen/Dense>

namespace av::geom
{
    using Polyline = std::vector<Eigen::Vector2d>;

    inline std::vector<double> arc_length(const Polyline &p)
    {
        std::vector<double> s(p.size(), 0.0);
        for (size_t i = 1; i < p.size(); ++i)
        {
            // printf("arc_length %zu, at points %f, %f and %f, %f is %f\n", i, p[i - 1].x(), p[i - 1].y(), p[i].x(), p[i].y(), (p[i] - p[i - 1]).norm());
            s[i] = s[i - 1] + (p[i] - p[i - 1]).norm();
            // printf("arc_length s[%zu] = %f\n", i, s[i]);
        }
        return s;
    }

    inline Polyline resample_uniform(const Polyline &p, double spacing)
    {
        Polyline resampled;
        if (p.size() < 2 || spacing <= 0.0)
        {
            return p;
        }
        // calculate the arc length of the polyline
        std::vector<double> s = arc_length(p);
        double total_length = s.back();

        // first point is always included
        resampled.push_back(p.front());
        double target_length = spacing;
        size_t segment_index = 1;

        while (target_length < total_length)
        {
            while (segment_index < p.size() && s[segment_index] < target_length)
            {
                ++segment_index;
            }
            if (segment_index >= p.size())
            {
                break;
            }
            // interpolate between p[segment_index - 1] and p[segment_index]
            double segment_length = s[segment_index] - s[segment_index - 1];
            double t = (target_length - s[segment_index - 1]) / segment_length;
            auto new_point = (1 - t) * p[segment_index - 1] + t * p[segment_index];
            resampled.push_back(new_point);
            target_length += spacing;
        }
        // last point is always included
        resampled.push_back(p.back());
        return resampled;
    }

    // Menger curvature through three consecutive points (1/m).
    inline double curvature_menger(const Eigen::Vector2d &a,
                                   const Eigen::Vector2d &b,
                                   const Eigen::Vector2d &c)
    {
        double cross = (b.x() - a.x()) * (c.y() - a.y()) -
                       (b.y() - a.y()) * (c.x() - a.x());
        double d1 = (b - a).norm(), d2 = (c - b).norm(), d3 = (c - a).norm();
        double denom = d1 * d2 * d3;
        if (denom < 1e-9)
            return 0.0;
        return 2.0 * cross / denom; // signed; callers usually take |.|
    }

    struct PathProjection
    {
        double s = 0;       // arc length of the foot point
        double lateral = 0; // signed: + = left of path direction
        size_t seg = 0;     // segment index (foot between p[seg] and p[seg+1])
        double dist = 1e18; // |q - foot|
    };

    inline PathProjection project_point(const Polyline &path,
                                        const Eigen::Vector2d &q)
    {
        PathProjection best;
        if (path.size() < 2)
            return best;
        double s_acc = 0;
        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            Eigen::Vector2d a = path[i], d = path[i + 1] - path[i];
            double len = d.norm();
            if (len < 1e-9)
                continue;
            double t = std::clamp((q - a).dot(d) / (len * len), 0.0, 1.0);
            Eigen::Vector2d foot = a + t * d;
            double dist = (q - foot).norm();
            if (dist < best.dist)
            {
                double cross = d.x() * (q.y() - foot.y()) - d.y() * (q.x() - foot.x());
                best = {s_acc + t * len, (cross >= 0 ? dist : -dist), i, dist};
            }
            s_acc += len;
        }
        return best;
    }

    inline Eigen::Vector2d point_at(const Polyline &path, double s_query)
    {
        if (path.empty())
            return {0, 0};
        if (path.size() == 1 || s_query <= 0)
            return path.front();
        double s_acc = 0;
        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            double len = (path[i + 1] - path[i]).norm();
            if (s_acc + len >= s_query)
            {
                double t = (s_query - s_acc) / std::max(1e-9, len);
                return path[i] + t * (path[i + 1] - path[i]);
            }
            s_acc += len;
        }
        return path.back();
    }

    inline double heading_at(const Polyline &path, double s_query)
    {
        if (path.size() < 2)
            return 0.0;

        double s_acc = 0;

        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            double len = (path[i + 1] - path[i]).norm();
            if (s_acc + len >= s_query || i + 1 == path.size() - 1)
            {
                Eigen::Vector2d d = path[i + 1] - path[i];
                return std::atan2(d.y(), d.x());
            }
            s_acc += len;
        }
        return 0.0; // default heading if s_query is beyond the path
    }

    inline double wrap_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }
} // namespace av::geom
