#pragma once
// Read-only model of maps/<world>.json emitted by scripts/generate_urban_world.py.
// Schema: docs/v2_interfaces.md §Map. Map = prior knowledge only: gates ROIs,
// fills intersections with turn splines, grounds missions. Never replaces
// perception on the open road.
#include <fstream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "av_common/geometry.hpp"

namespace av
{

    class MapModel
    {
    public:
        struct Approach
        {
            double heading = 0;             // ego heading when using this approach
            std::vector<std::string> legal; // "straight" | "left" | "right"
            Eigen::Vector2d stop_line{0, 0};
            std::string light_id;                          // "" = unsignalized
            std::string sign;                              // "" | "stop" | "yield"
            std::map<std::string, geom::Polyline> splines; // turn -> polyline
        };
        struct Intersection
        {
            std::string id;
            Eigen::Vector2d center{0, 0};
            double half_size = 7.0; // intersection box half extent
            std::vector<Approach> approaches;
        };
        struct Light
        {
            std::string id;
            Eigen::Vector3d pos{0, 0, 0};
            double facing = 0; // heading the lamp FACES (toward traffic)
        };
        struct SpeedZone
        {
            double x0, y0, x1, y1, limit;
        };
        struct Crosswalk
        {
            std::string id;
            Eigen::Vector2d pos{0, 0};
            double heading = 0;
        };

        double lane_width = 3.5;
        double speed_limit_default = 8.0;
        int seed = 0;
        std::vector<Intersection> intersections;
        std::vector<Light> lights;
        std::vector<SpeedZone> speed_zones;
        std::vector<Crosswalk> crosswalks;
        // Course-style maps (generate_course_world.py): the two-way ROAD centerline,
        // sampled along the direction of travel. Empty for grid maps. Used by
        // local_planner's road-curvature assist (turns there are road geometry, not
        // intersections, so there are no turn splines to bridge them).
        geom::Polyline course_centerline;
        // Course-style maps also carry a single road stop line (the painted bar
        // near the course end). nullopt for grid maps. Transformed into the
        // odom frame like everything else; lane_node fuses it 0.7/0.3 with its
        // own vision detection on /lane/stop_line.
        std::optional<Eigen::Vector2d> course_stop_line;

        // Loads the map and transforms every coordinate from WORLD frame into the
        // ODOM frame, which is zeroed at the spawn pose (ox, oy, oyaw). The
        // generator emits the spawn in the map json; launch files pass it through
        // as map_origin_* params. Forgetting this made the stack believe it sat in
        // ix_0_0 at boot (2026-07-11 smoke test).
        static MapModel load(const std::string &path, double ox = 0.0,
                             double oy = 0.0, double oyaw = 0.0)
        {
            std::ifstream f(path);
            if (!f)
                throw std::runtime_error("MapModel: cannot open " + path);
            nlohmann::json j;
            f >> j;
            MapModel m;
            const double c = std::cos(-oyaw), s = std::sin(-oyaw);
            auto tf = [&](double x, double y) -> Eigen::Vector2d
            {
                double dx = x - ox, dy = y - oy;
                return {c * dx - s * dy, s * dx + c * dy};
            };
            auto tf_heading = [&](double h)
            {
                return std::atan2(std::sin(h - oyaw), std::cos(h - oyaw));
            };
            m.seed = j.value("seed", 0);
            m.lane_width = j.value("lane_width", 3.5);
            m.speed_limit_default = j.value("speed_limit_default", 8.0);
            for (const auto &ji : j.value("intersections", nlohmann::json::array()))
            {
                Intersection ix;
                ix.id = ji.value("id", "");
                ix.center = tf(ji.value("x", 0.0), ji.value("y", 0.0));
                ix.half_size = ji.value("half_size", 7.0);
                for (const auto &ja : ji.value("approaches", nlohmann::json::array()))
                {
                    Approach ap;
                    ap.heading = tf_heading(ja.value("heading", 0.0));
                    for (const auto &l : ja.value("legal", nlohmann::json::array()))
                        ap.legal.push_back(l.get<std::string>());
                    if (ja.contains("stop_line"))
                        ap.stop_line = tf(ja["stop_line"].value("x", 0.0),
                                          ja["stop_line"].value("y", 0.0));
                    ap.light_id = ja.value("light", "");
                    ap.sign = ja.value("sign", "");
                    if (ja.contains("splines"))
                        for (auto it = ja["splines"].begin(); it != ja["splines"].end(); ++it)
                        {
                            geom::Polyline pl;
                            for (const auto &pt : it.value())
                                pl.push_back(tf(pt[0].get<double>(), pt[1].get<double>()));
                            ap.splines[it.key()] = pl;
                        }
                    ix.approaches.push_back(std::move(ap));
                }
                m.intersections.push_back(std::move(ix));
            }
            for (const auto &jl : j.value("lights", nlohmann::json::array()))
            {
                Eigen::Vector2d p = tf(jl.value("x", 0.0), jl.value("y", 0.0));
                m.lights.push_back({jl.value("id", ""),
                                    {p.x(), p.y(), jl.value("z", 5.0)},
                                    tf_heading(jl.value("facing", 0.0))});
            }
            for (const auto &jz : j.value("speed_zones", nlohmann::json::array()))
            {
                // transform corners, keep the AABB (exact for oyaw = 0, which is what
                // the generator emits; approximate otherwise)
                Eigen::Vector2d a = tf(jz.value("x0", 0.0), jz.value("y0", 0.0));
                Eigen::Vector2d b = tf(jz.value("x1", 0.0), jz.value("y1", 0.0));
                m.speed_zones.push_back({std::min(a.x(), b.x()), std::min(a.y(), b.y()),
                                         std::max(a.x(), b.x()), std::max(a.y(), b.y()),
                                         jz.value("limit", 8.0)});
            }
            for (const auto &jc : j.value("crosswalks", nlohmann::json::array()))
                m.crosswalks.push_back({jc.value("id", ""),
                                        tf(jc.value("x", 0.0), jc.value("y", 0.0)),
                                        tf_heading(jc.value("heading", 0.0))});
            if (j.contains("course"))
                for (const auto &pt :
                     j["course"].value("centerline", nlohmann::json::array()))
                    m.course_centerline.push_back(
                        tf(pt[0].get<double>(), pt[1].get<double>()));
            if (j.contains("stop_line"))
                m.course_stop_line = tf(j["stop_line"].value("x", 0.0),
                                        j["stop_line"].value("y", 0.0));
            return m;
        }

        // Nearest intersection AHEAD of ego (within max_dist, roughly along heading).
        const Intersection *next_intersection(const Eigen::Vector2d &ego,
                                              double heading,
                                              double max_dist = 60.0) const
        {
            const Intersection *best = nullptr;
            double best_d = max_dist;
            Eigen::Vector2d dir(std::cos(heading), std::sin(heading));
            for (const auto &ix : intersections)
            {
                Eigen::Vector2d rel = ix.center - ego;
                double along = rel.dot(dir);
                if (along < -ix.half_size)
                    continue; // behind
                if (std::abs(rel.x() * dir.y() - rel.y() * dir.x()) > ix.half_size + 4.0)
                    continue; // laterally off-route
                if (along < best_d)
                {
                    best_d = along;
                    best = &ix;
                }
            }
            return best;
        }

        // The approach of `ix` whose stored heading matches ego heading (+-45 deg).
        const Approach *approach_for(const Intersection &ix, double ego_heading) const
        {
            const Approach *best = nullptr;
            double best_err = M_PI / 4;
            for (const auto &ap : ix.approaches)
            {
                double err = std::abs(geom::wrap_angle(ap.heading - ego_heading));
                if (err < best_err)
                {
                    best_err = err;
                    best = &ap;
                }
            }
            return best;
        }

        const Intersection *intersection_by_id(const std::string &id) const
        {
            for (const auto &ix : intersections)
                if (ix.id == id)
                    return &ix;
            return nullptr;
        }

        const Light *light_by_id(const std::string &id) const
        {
            for (const auto &l : lights)
                if (l.id == id)
                    return &l;
            return nullptr;
        }

        bool in_intersection(const Eigen::Vector2d &ego, double margin = 0.0) const
        {
            for (const auto &ix : intersections)
            {
                double h = ix.half_size + margin;
                if (std::abs(ego.x() - ix.center.x()) < h &&
                    std::abs(ego.y() - ix.center.y()) < h)
                    return true;
            }
            return false;
        }

        double speed_limit_at(const Eigen::Vector2d &ego) const
        {
            for (const auto &z : speed_zones)
                if (ego.x() >= z.x0 && ego.x() <= z.x1 && ego.y() >= z.y0 && ego.y() <= z.y1)
                    return z.limit;
            return speed_limit_default;
        }
    };

} // namespace av
