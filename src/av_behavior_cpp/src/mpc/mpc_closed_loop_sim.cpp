// mpc_closed_loop_sim - offline closed-loop check of MpcSolver against a plant
// with the MEASURED steering actuator lag (2026-10-18 open-loop step test in
// Gazebo: the AckermannSteering steering joint follows the command as a first
// order lag, tau ~= 1.0 s, steady-state ~98%, yaw rate exactly kinematic).
//
// Mirrors mpc_tracker_v2_node's control tick (closest point, resample at
// ds = v*dt, speed-scheduled lookahead, output steer slew) so a solver/weight
// change can be judged in seconds instead of a 5-minute Gazebo lap.
//
//   mpc_closed_loop_sim <lane.csv> [key=value ...]
//
// lane.csv: "x,y" per line, the reference lane (e.g. the map's right lane,
// written by tools/avkit/export_lane_csv.py). The controller sees a 0.5 m
// resampled window of it ahead of the car, refreshed at perception rate,
// like /plan/path_odom.
//
// Keys (defaults = config/mpc_params.yaml + measured plant):
//   v=3.0 tau_plant=1.0 tau_model=1.0 dead_plant=0.0 y0=0.3 noise=0.0
//   look_time=5.0 look_min=5.0 look_max=14.0 win_from=1.3 win_to=13.4
//   trace=<file.csv>
// tau_model=0 disables the solver's actuator model (the pre-2026-10-18 model).
// Prints e_lat RMS/max overall, on the first straight, and in the turns, and
// the late-vs-early RMS ratio on the first straight (> 1 = growing weave).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "mpc/mpc_solver.hpp"
#include "mpc/path_utils.hpp"
#include "mpc/vehicle_model.hpp"

namespace
{
    struct Lane
    {
        std::vector<double> x, y, s;
    };

    Lane load_lane(const char *path)
    {
        Lane L;
        std::ifstream f(path);
        std::string line;
        while (std::getline(f, line))
        {
            double x, y;
            if (std::sscanf(line.c_str(), "%lf,%lf", &x, &y) == 2)
            {
                if (!L.x.empty())
                    L.s.push_back(L.s.back() + std::hypot(x - L.x.back(), y - L.y.back()));
                else
                    L.s.push_back(0.0);
                L.x.push_back(x);
                L.y.push_back(y);
            }
        }
        return L;
    }

    // signed lateral error (+ = left of travel) and arc length of q on the lane
    void project(const Lane &L, double qx, double qy, double &s_out, double &lat_out)
    {
        double best = 1e18;
        for (size_t i = 0; i + 1 < L.x.size(); ++i)
        {
            double dx = L.x[i + 1] - L.x[i], dy = L.y[i + 1] - L.y[i];
            double l2 = dx * dx + dy * dy;
            if (l2 < 1e-12)
                continue;
            double t = std::clamp(((qx - L.x[i]) * dx + (qy - L.y[i]) * dy) / l2, 0.0, 1.0);
            double fx = L.x[i] + t * dx, fy = L.y[i] + t * dy;
            double d = std::hypot(qx - fx, qy - fy);
            if (d < best)
            {
                best = d;
                s_out = L.s[i] + t * std::sqrt(l2);
                double cross = dx * (qy - L.y[i]) - dy * (qx - L.x[i]);
                lat_out = cross >= 0 ? d : -d;
            }
        }
    }

    void at_s(const Lane &L, double s, double &x, double &y)
    {
        size_t i = 0;
        while (i + 2 < L.s.size() && L.s[i + 1] < s)
            ++i;
        double r = std::clamp((s - L.s[i]) / std::max(L.s[i + 1] - L.s[i], 1e-9), 0.0, 1.0);
        x = L.x[i] + r * (L.x[i + 1] - L.x[i]);
        y = L.y[i] + r * (L.y[i + 1] - L.y[i]);
    }

    double kappa_at(const Lane &L, double s)
    {
        double x0, y0, x1, y1, x2, y2;
        at_s(L, s - 1.0, x0, y0);
        at_s(L, s, x1, y1);
        at_s(L, s + 1.0, x2, y2);
        double a = std::atan2(y1 - y0, x1 - x0), b = std::atan2(y2 - y1, x2 - x1);
        double d = std::remainder(b - a, 2.0 * M_PI);
        return d / 1.0;
    }
} // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "usage: %s <lane.csv> [key=value ...]\n", argv[0]);
        return 2;
    }
    std::map<std::string, std::string> kv;
    for (int i = 2; i < argc; ++i)
    {
        std::string a = argv[i];
        auto eq = a.find('=');
        if (eq != std::string::npos)
            kv[a.substr(0, eq)] = a.substr(eq + 1);
    }
    auto num = [&](const char *k, double d)
    { return kv.count(k) ? std::atof(kv[k].c_str()) : d; };

    const Lane lane = load_lane(argv[1]);
    if (lane.x.size() < 3)
    {
        std::fprintf(stderr, "lane file has < 3 points\n");
        return 2;
    }

    // ---- controller config: mirrors config/mpc_params.yaml
    VehicleParams vp;
    vp.wheelbase = 2.7;
    vp.max_speed = 3.0;
    vp.max_accel = 1.2;
    vp.max_decel = -1.2;
    vp.max_steer = 0.6;
    MpcParams mp;
    mp.N = static_cast<int>(num("N", 50));
    mp.dt = 0.12;
    mp.w_lat = num("w_lat", 6.0);
    mp.w_head = num("w_head", 3.0);
    mp.w_speed = 1.0;
    mp.w_accel = 0.2;
    mp.w_steer = num("w_steer", 1.0);
    mp.w_accel_change = 0.4;
    mp.w_steer_change = num("w_steer_rate", 6.0);
    mp.w_accel_change_u0 = 0.0;
    mp.w_steer_change_u0 = num("w_steer_rate_u0", 16.0);
    mp.steer_rate_limit = num("steer_slew", 4.0);
    const double rate_hz = 30.0;
    mp.control_dt = 1.0 / rate_hz;
    mp.steer_curvature_margin_base = num("margin_base", 0.1);
    mp.steer_curvature_margin_slope = num("margin_slope", 2.0);
    mp.steer_curvature_margin_elat_slope = num("margin_elat_slope", 0.2);
    mp.steer_tau = num("tau_model", 1.0);
    mp.centered_heading_ref = num("centered", 0.0) > 0.5;
    const double look_time = num("look_time", 5.0), look_min = num("look_min", 5.0),
                 look_max = num("look_max", 14.0);
    const double steer_slew = mp.steer_rate_limit;
    MpcSolver solver(vp, mp);

    // ---- plant + scenario
    const double v_target = num("v", 3.0);
    const double tau_plant = num("tau_plant", 1.0);
    const double dead_plant = num("dead_plant", 0.0);
    const double noise = num("noise", 0.0);
    const double win_from = num("win_from", 1.3), win_to = num("win_to", 13.4);
    const double percep_hz = 8.5, sub_hz = 300.0;
    std::mt19937 rng(7);
    std::normal_distribution<double> gauss(0.0, 1.0);

    double x0, y0, x1, y1;
    at_s(lane, 0.0, x0, y0);
    at_s(lane, 1.0, x1, y1);
    const double psi0 = std::atan2(y1 - y0, x1 - x0);
    const double off = num("y0", 0.3); // initial lateral offset, + = left
    VehicleState st;
    st.x = x0 - std::sin(psi0) * off;
    st.y = y0 + std::cos(psi0) * off;
    st.psi = psi0;
    st.v = 0.0;
    double delta_act = 0.0;
    std::vector<double> dead_buf(static_cast<size_t>(std::max(1.0, dead_plant * sub_hz + 1)), 0.0);
    size_t dead_i = 0;

    double v_ref_cmd = 0.0, steer_cmd = 0.0, prev_accel = 0.0, prev_steer = 0.0, v_cmd = 0.0;
    double delta_est = 0.0; // node-side actuator estimate, driven by its own commands
    std::vector<PathPoint> seen;
    double next_percep = 0.0;
    FILE *trace = kv.count("trace") ? std::fopen(kv["trace"].c_str(), "w") : nullptr;
    if (trace)
        std::fprintf(trace, "t,s,lat,v,steer_cmd,delta_act,delta_est,kappa\n");

    const double lane_len = lane.s.back();
    double t = 0.0, s_ego = 0.0, lat = off;
    std::vector<double> lat_all, lat_straight_early, lat_straight_late, lat_turn;
    int fallbacks = 0;
    while (s_ego < lane_len - 25.0 && t < 600.0)
    {
        // perception: 0.5 m window [win_from, win_to] m from the car, like
        // /plan/path_odom (measured on urban_course: starts ~1.3 m ahead of
        // base_link, ends ~13.4 m ahead - the camera's lane-detection range)
        if (t >= next_percep)
        {
            next_percep += 1.0 / percep_hz;
            seen.clear();
            double acc = 0.0;
            for (double s = std::max(0.0, s_ego + win_from); s <= std::min(lane_len, s_ego + win_to); s += 0.5)
            {
                double px, py;
                at_s(lane, s, px, py);
                if (noise > 0.0)
                {
                    double ax, ay;
                    at_s(lane, s + 0.5, ax, ay);
                    double hd = std::atan2(ay - py, ax - px), n = noise * gauss(rng);
                    px += -std::sin(hd) * n;
                    py += std::cos(hd) * n;
                }
                if (!seen.empty())
                    acc += std::hypot(px - seen.back().x, py - seen.back().y);
                seen.push_back({px, py, 0.0, acc});
            }
            for (size_t i = 0; i + 1 < seen.size(); ++i)
                seen[i].psi = std::atan2(seen[i + 1].y - seen[i].y, seen[i + 1].x - seen[i].x);
            seen.back().psi = seen[seen.size() - 2].psi;
        }

        // ---- control tick (node parity)
        size_t closest = find_closest_point(seen, st, 3.0);
        double d = v_target - v_ref_cmd;
        double step = (d >= 0.0 ? 1.5 : 6.0) / rate_hz;
        v_ref_cmd += std::clamp(d, -step, step);
        solver.set_target_speed(v_ref_cmd);
        solver.set_lookahead(std::clamp(st.v * look_time, look_min, look_max));
        auto ref = resample_path(seen, closest, mp.N + 1, std::max(st.v * mp.dt, 0.3));
        st.delta = delta_est;
        auto r = solver.Solve(st, ref, prev_accel, prev_steer);
        if (r.used_fallback)
            ++fallbacks;
        prev_accel = r.accel;
        double max_d = steer_slew / rate_hz;
        steer_cmd += std::clamp(r.steer - steer_cmd, -max_d, max_d);
        prev_steer = steer_cmd;
        v_cmd = std::clamp(st.v + r.accel / rate_hz, 0.0, vp.max_speed);

        // ---- plant, integrated at sub_hz between ticks
        const int subs = static_cast<int>(sub_hz / rate_hz);
        for (int k = 0; k < subs; ++k)
        {
            const double h = 1.0 / sub_hz;
            dead_buf[dead_i] = steer_cmd;
            dead_i = (dead_i + 1) % dead_buf.size();
            const double u_delayed = dead_buf[dead_i];
            delta_act += (1.0 - std::exp(-h / tau_plant)) * (u_delayed - delta_act);
            st.x += st.v * std::cos(st.psi) * h;
            st.y += st.v * std::sin(st.psi) * h;
            st.psi += st.v * std::tan(delta_act) / vp.wheelbase * h;
            st.v += (v_cmd - st.v) * (1.0 - std::exp(-h / 0.3));
        }
        t += 1.0 / rate_hz;
        project(lane, st.x, st.y, s_ego, lat);

        if (st.v > 0.5)
        {
            lat_all.push_back(lat);
            const double k_here = std::max(std::abs(kappa_at(lane, s_ego)),
                                           std::abs(kappa_at(lane, s_ego + 4.0)));
            if (k_here > 0.02)
                lat_turn.push_back(lat);
            else if (s_ego > 10.0 && s_ego < 35.0)
                lat_straight_early.push_back(lat);
            else if (s_ego >= 35.0 && s_ego < 55.0)
                lat_straight_late.push_back(lat);
        }
        if (trace)
            std::fprintf(trace, "%.3f,%.2f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f\n", t, s_ego, lat, st.v,
                         steer_cmd, delta_act, delta_est, kappa_at(lane, s_ego));
        if (mp.steer_tau > 0.0)
            delta_est += (1.0 - std::exp(-(1.0 / rate_hz) / mp.steer_tau)) * (steer_cmd - delta_est);
        else
            delta_est = steer_cmd;
    }
    if (trace)
        std::fclose(trace);

    auto rms = [](const std::vector<double> &v)
    {
        double a = 0;
        for (double x : v)
            a += x * x;
        return v.empty() ? 0.0 : std::sqrt(a / v.size());
    };
    auto amax = [](const std::vector<double> &v)
    {
        double a = 0;
        for (double x : v)
            a = std::max(a, std::abs(x));
        return a;
    };
    const double early = rms(lat_straight_early), late = rms(lat_straight_late);
    std::printf("v=%.1f tau_plant=%.2f tau_model=%.2f dead=%.2f noise=%.2f | rms=%.3f max=%.3f | "
                "straight early=%.3f late=%.3f growth=%.2f | turn rms=%.3f max=%.3f | t=%.0fs done=%s fallbacks=%d\n",
                v_target, tau_plant, mp.steer_tau, dead_plant, noise, rms(lat_all), amax(lat_all),
                early, late, early > 1e-9 ? late / early : 0.0, rms(lat_turn), amax(lat_turn), t,
                s_ego >= lane_len - 25.0 ? "yes" : "NO", fallbacks);
    return 0;
}
