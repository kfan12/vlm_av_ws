#include <cmath>
#include <cstdio>
#include <vector>

#include "mpc/path_utils.hpp"
#include "mpc/vehicle_model.hpp"

static int fails = 0;
#define CHECK(cond, msg)                    \
    do                                      \
    {                                       \
        if (cond)                           \
            std::printf("PASS  %s\n", msg); \
        else                                \
        {                                   \
            std::printf("FAIL  %s\n", msg); \
            ++fails;                        \
        }                                   \
    } while (0)

int main()
{
    VehicleParams vp;
    vp.wheelbase = 2.7;
    vp.max_steer = 0.75;
    vp.max_speed = 8.0;
    vp.max_accel = 1.5;
    vp.max_decel = -6.0;

    // 1. straight rollout: zero steer goes straight, x integrates v*t
    VehicleState s;
    s.v = 2.0;
    for (int i = 0; i < 100; ++i) // integrate forward for 5 seconds, no control input, should move straight
        s = integrate_kinematic(s, {0.0, 0.0}, vp, 0.05);

    CHECK(std::abs(s.y) < 1e-9 && std::abs(s.psi) < 1e-9 &&
              std::abs(s.x - 10.0) < 1e-6,
          "straight rollout goes straight (x = v*t)");

    // 2. constant steer traces a circle of radius R = L / tan(delta):
    //    at heading pi/2 the position should be ~(R, R) for a left turn
    s = VehicleState{};
    s.v = 2.0;
    const double delta = 0.3;
    const double R = vp.wheelbase / std::tan(delta);

    while (s.psi < M_PI / 2)
        s = integrate_kinematic(s, {0.0, delta}, vp, 0.002);

    CHECK(std::abs(s.x - R) < 0.05 * R && std::abs(s.y - R) < 0.05 * R,
          "constant steer traces a circle of radius L/tan(delta)");

    // 3. clamping: control and speed limits hold
    VehicleControl control = clamp_control({99.0, 99.5}, vp);
    CHECK(control.delta == vp.max_steer && control.a == vp.max_accel,
          "control clamps to vehicle limits");

    s = VehicleState{};
    s.v = vp.max_speed;
    s = integrate_kinematic(s, {vp.max_accel, 0.0}, vp, 1.0);
    CHECK(s.v <= vp.max_speed + 1e-9, "speed clamps to vehicle limits");

    // 4. path utils: quarter circle, resampled uniformly
    nav_msgs::msg::Path path_msg;
    const double Rp = 15.0;
    const int n_samp = 79; // ~0.02 rad steps, last sample lands exactly on pi/2
    for (int k = 0; k <= n_samp; ++k)
    {
        double t = k * (M_PI / 2) / n_samp;
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = Rp * std::sin(t);
        ps.pose.position.y = Rp * (1.0 - std::cos(t));
        ps.pose.orientation.w = 1.0; // yaw unused by this check
        path_msg.poses.push_back(ps);
    }
    auto pts = path_msg_to_points(path_msg);
    CHECK(std::abs(pts.back().s - Rp * M_PI / 2) < 0.05,
          "arc-length of a quarter circle is R*pi/2");

    VehicleState ego;
    ego.x = Rp * std::sin(0.3);
    ego.y = Rp * (1.0 - std::cos(0.3)) + 0.4; // slightly off the path
    size_t closest_idx = find_closest_point(pts, ego, 2.0);
    CHECK(std::abs(pts[closest_idx].s - Rp * 0.3) < 0.5, "closest point lands at s=R*theta");

    auto ref = resample_path(pts, closest_idx, 25, 0.3);
    bool spacing_ok = ref.size() == 25;
    for (size_t i = 1; i + 1 < ref.size() && spacing_ok; ++i)
    {
        double d = std::hypot(ref[i].x - ref[i - 1].x, ref[i].y - ref[i - 1].y);
        spacing_ok = std::abs(d - 0.3) < 0.05;
    }
    CHECK(spacing_ok, "resampled path has uniform spacing");

    std::printf(fails ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASS\n", fails);

    return fails ? 1 : 0;
}