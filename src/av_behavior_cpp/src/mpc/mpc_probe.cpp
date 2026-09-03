// mpc_probe - Day 10 offline checks: the QP solves, uses OSQP (not the
// fallback), and its signs and feed-forward behave. Solve() expects
// reference_path[0] to BE the foot point (the node resamples from the
// closest point), so the probes construct references accordingly.
#include <cmath>
#include <cstdio>
#include <vector>

#include "mpc/mpc_solver.hpp"

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

static std::vector<PathPoint> straight(double x0, double len, double ds)
{
    std::vector<PathPoint> p;
    for (double s = 0.0; s <= len; s += ds)
        p.push_back({x0 + s, 0.0, 0.0, s});
    return p;
}

static std::vector<PathPoint> left_arc(double R, double len, double ds)
{
    std::vector<PathPoint> p; // starts at origin, heading +x, turning left
    for (double s = 0.0; s <= len; s += ds)
    {
        double th = s / R;
        p.push_back({R * std::sin(th), R * (1.0 - std::cos(th)), th, s});
    }
    return p;
}

int main()
{
    VehicleParams vp;
    vp.wheelbase = 2.7;
    vp.max_steer = 0.75;
    vp.max_speed = 8.0;
    vp.max_accel = 1.5;
    vp.max_decel = -6.0;
    MpcParams mp;
    mp.N = 25;
    mp.dt = 0.12;
    mp.target_speed = 2.5;
    mp.w_lat = 6.0;
    mp.w_head = 3.0;
    mp.w_speed = 1.0;
    mp.w_steer = 3.0;
    mp.w_steer_change = 6.0;
    mp.lookahead_m = 8.0;
    MpcSolver solver(vp, mp);

    // 1-3. offset left of a straight path -> steer right; mirrored -> mirrored
    VehicleState s;
    s.x = 5.0;
    s.y = 0.5; // half a meter LEFT of the path
    s.v = 2.0;
    auto ref = straight(5.0, 20.0, 0.5); // foot point at x=5
    auto r = solver.Solve(s, ref, 0.0, 0.0);
    CHECK(r.success, "QP solves on a straight");
    CHECK(!r.used_fallback, "OSQP path used (not the proportional fallback)");
    CHECK(r.steer < 0.0, "left offset steers right (negative)");
    s.y = -0.5;
    auto r2 = solver.Solve(s, ref, 0.0, 0.0);
    CHECK(r2.steer > 0.0 && std::abs(r2.steer + r.steer) < 1e-6,
          "mirrored offset gives mirrored steer");

    // 4. on-path on an r=15 left arc -> positive steer, near atan(L/R)
    s = VehicleState{};
    s.v = 2.0;
    auto arc = left_arc(15.0, 20.0, 0.5);
    auto ra = solver.Solve(s, arc, 0.0, 0.0);
    double ff = std::atan(vp.wheelbase / 15.0);
    CHECK(ra.success && !ra.used_fallback, "QP solves on an arc");
    CHECK(ra.steer > 0.3 * ff && ra.steer < 2.0 * ff,
          "curvature feed-forward steers left on a left arc");

    // 5. below target speed -> accelerate; above -> brake
    CHECK(ra.accel > 0.0, "below-target speed accelerates");
    s.v = 5.0;
    auto rb = solver.Solve(s, arc, 0.0, 0.0);
    CHECK(rb.accel < 0.0, "above-target speed brakes");

    // 6. Delta-u damping: a large previous steer pulls the command toward it
    s = VehicleState{};
    s.v = 2.0;
    auto rc0 = solver.Solve(s, straight(0.0, 20.0, 0.5), 0.0, 0.0);
    auto rc1 = solver.Solve(s, straight(0.0, 20.0, 0.5), 0.0, 0.4);
    CHECK(rc1.steer > rc0.steer, "previous-command term pulls the new command");

    // 7. degenerate path -> graceful failure, no crash
    auto rd = solver.Solve(s, {arc.front()}, 0.0, 0.0);
    CHECK(!rd.success, "1-point path returns failure");

    std::printf(fails ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASS\n", fails);
    return fails ? 1 : 0;
}
