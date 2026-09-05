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

    // 6b. w_steer_change_u0 damps ONLY the u_0-vs-previous-applied boundary,
    // independent of the within-horizon w_steer_change: a huge u0 weight
    // must pin delta_0 close to previous_steer, while the WITHIN-horizon
    // step delta_1-delta_0 (governed by the separate, much smaller
    // w_steer_change) is free to move more - proving the two are no longer
    // the same number doing double duty.
    {
        MpcParams mp_split = mp;
        mp_split.w_steer_change = 0.5;    // loose within-horizon smoothing
        mp_split.w_steer_change_u0 = 100.0; // very tight boundary-only weight
        MpcSolver split_solver(vp, mp_split);
        VehicleState offset;
        offset.x = 5.0;
        offset.y = 1.0; // wants real correction
        offset.v = 2.0;
        auto rsplit = split_solver.Solve(offset, ref, 0.0, 0.0); // previous_steer = 0.0
        CHECK(rsplit.success && !rsplit.used_fallback, "QP solves with split u0/within-horizon steer weights");
        // "close to previous_steer" judged relative to steers[1] (~-0.75,
        // right at max_steer) rather than an arbitrary absolute constant -
        // delta_0 landing at -0.057 while delta_1 swings to the hard limit
        // to compensate is exactly the pinning behavior this weight is for.
        CHECK(std::abs(rsplit.steer - 0.0) < 0.1,
              "huge w_steer_change_u0 pins delta_0 close to previous_steer");
        CHECK(rsplit.steers.size() >= 2 && std::abs(rsplit.steers[1] - rsplit.steers[0]) > std::abs(rsplit.steer),
              "the separate, loose within-horizon weight still lets delta_1 move more than the pinned delta_0 did");
    }

    // 7. degenerate path -> graceful failure, no crash
    auto rd = solver.Solve(s, {arc.front()}, 0.0, 0.0);
    CHECK(!rd.success, "1-point path returns failure");

    // 8. steer_rate_limit is a HARD bound on delta_0, not just a soft cost:
    // a big lateral offset that would otherwise want near-max_steer must be
    // clamped to what a tight rate limit allows over one control tick.
    {
        VehicleParams vp2 = vp;
        MpcParams mp_limited = mp;
        mp_limited.steer_rate_limit = 1.0; // rad/s, deliberately tight
        mp_limited.control_dt = 0.1;       // s
        MpcSolver limited_solver(vp2, mp_limited);
        VehicleState big_offset;
        big_offset.x = 5.0;
        big_offset.y = 2.0; // large left offset -> wants a big right correction
        big_offset.v = 2.0;
        auto rl = limited_solver.Solve(big_offset, ref, 0.0, 0.0); // previous_steer = 0.0
        const double allowed = mp_limited.steer_rate_limit * mp_limited.control_dt; // 0.1 rad
        CHECK(rl.success && !rl.used_fallback, "QP solves under a tight steer_rate_limit");
        CHECK(std::abs(rl.steer) <= allowed + 1e-3, // OSQP's convergence tolerance is ~1e-3/1e-4, not exact
              "steer_rate_limit hard-clamps delta_0 to the one-tick achievable swing");

        MpcParams mp_unlimited = mp;
        mp_unlimited.steer_rate_limit = 10.0; // effectively unconstrained default
        mp_unlimited.control_dt = 0.1;
        MpcSolver unlimited_solver(vp2, mp_unlimited);
        auto ru = unlimited_solver.Solve(big_offset, ref, 0.0, 0.0);
        CHECK(std::abs(ru.steer) > std::abs(rl.steer),
              "unconstrained solver commands more steer than the rate-limited one on the same offset");
    }

    // 9. steer_curvature_margin_base/_slope bound delta relative to the
    // path's OWN curvature feed-forward angle, not a flat +/-max_steer box,
    // and the margin GROWS with |kappa| (tightest on straights, loosest on
    // real curves): a large lateral offset on a STRAIGHT (kappa=0, margin =
    // base only) must stay capped near that tight base despite wanting a
    // much bigger correction, while the SAME base + a real slope on an
    // actual curve must still deliver close to that curve's own feed-forward
    // angle - the slope term is what lets a tight base not starve legitimate
    // curve-tracking.
    {
        VehicleParams vp3 = vp;
        MpcParams mp_margin = mp;
        mp_margin.steer_curvature_margin_base = 0.05; // rad, deliberately tight
        mp_margin.steer_curvature_margin_slope = 3.0; // same slope used on the arc case below
        MpcSolver margin_solver(vp3, mp_margin);

        VehicleState big_offset2;
        big_offset2.x = 5.0;
        big_offset2.y = 2.0; // large left offset on a STRAIGHT path -> delta_ff = 0, |kappa| = 0 -> margin = base
        big_offset2.v = 2.0;
        auto rm = margin_solver.Solve(big_offset2, ref, 0.0, 0.0);
        CHECK(rm.success && !rm.used_fallback, "QP solves under a tight steer_curvature_margin_base");
        // OSQP's default convergence tolerance is ~1e-3/1e-4, not exact -
        // 1e-3 here matches that instead of a numerically-unrealistic 1e-6.
        CHECK(std::abs(rm.steer) <= mp_margin.steer_curvature_margin_base + 1e-3,
              "tight base caps correction near zero on a straight (kappa=0), despite a large offset");

        MpcParams mp_margin_unbounded = mp;
        mp_margin_unbounded.steer_curvature_margin_base = 10.0; // effectively unconstrained
        MpcSolver unbounded_solver(vp3, mp_margin_unbounded);
        auto rmu = unbounded_solver.Solve(big_offset2, ref, 0.0, 0.0);
        CHECK(std::abs(rmu.steer) > std::abs(rm.steer),
              "unconstrained solver commands more correction than the curvature-margin-bounded one");

        // same tight base, but ON an r=15 arc (kappa~0.067): the slope term
        // should grow the margin enough to still reach close to the arc's
        // own feed-forward angle, not be crushed toward zero by the tight base.
        VehicleState on_arc{};
        on_arc.v = 2.0;
        auto rma = margin_solver.Solve(on_arc, arc, 0.0, 0.0);
        CHECK(rma.success && !rma.used_fallback, "QP solves on an arc under a tight base + real slope");
        CHECK(rma.steer > 0.5 * ff,
              "the slope term still allows near-full curvature feed-forward on a real curve despite a tight base");
    }

    // 10. steer_curvature_margin_elat_slope grows the margin with the
    // CURRENT measured e_lat0, uniformly across the horizon, so a car far
    // off track on a STRAIGHT (kappa=0, so the kappa-slope term contributes
    // nothing) still gets real correction authority instead of being stuck
    // at the tight base - "bring it back to track fast."
    {
        VehicleParams vp4 = vp;
        MpcParams mp_elat = mp;
        mp_elat.steer_curvature_margin_base = 0.05;  // tight
        mp_elat.steer_curvature_margin_slope = 0.0;  // isolate the e_lat term from the kappa term
        mp_elat.steer_curvature_margin_elat_slope = 0.2; // rad per metre of |e_lat0|
        MpcSolver elat_solver(vp4, mp_elat);

        VehicleState small_offset;
        small_offset.x = 5.0;
        small_offset.y = 0.05; // tiny offset -> elat contribution to margin is negligible
        small_offset.v = 2.0;
        auto rs = elat_solver.Solve(small_offset, ref, 0.0, 0.0);

        VehicleState large_offset;
        large_offset.x = 5.0;
        large_offset.y = 2.0; // large offset -> margin grows by 0.2*2.0 = 0.4 rad
        large_offset.v = 2.0;
        auto rl2 = elat_solver.Solve(large_offset, ref, 0.0, 0.0);

        CHECK(rs.success && rl2.success && !rs.used_fallback && !rl2.used_fallback,
              "QP solves under steer_curvature_margin_elat_slope");
        CHECK(std::abs(rl2.steer) > std::abs(rs.steer) + 0.1,
              "a large current lateral error widens the steer bound enough to command more correction "
              "than a small one, on the same (straight) path");
    }

    std::printf(fails ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASS\n", fails);
    return fails ? 1 : 0;
}
