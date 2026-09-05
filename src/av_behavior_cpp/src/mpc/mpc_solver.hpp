#pragma once
#include <Eigen/Core>
#include <vector>
#include "mpc/path_utils.hpp"
#include "mpc/vehicle_model.hpp"

struct MpcParams
{

    int N{15};
    double dt{0.1};
    double target_speed{0.6};

    double w_lat{8.0};          // weight for lateral error
    double w_head{3.0};         // weight for heading error
    double w_speed{1.0};        // weight for speed error
    double w_accel{0.2};        // weight for acceleration
    double w_steer{0.5};        // weight for steering
    double w_accel_change{0.4}; // weight for change in acceleration (within-horizon u_{i+1}-u_i)
    double w_steer_change{2.0}; // weight for change in steering (within-horizon u_{i+1}-u_i)

    // Boundary-only weights for (u_0 - previous APPLIED command)^2, separate
    // from w_accel_change/w_steer_change above (which, until now, doubled as
    // BOTH the u_0 boundary weight AND the within-horizon u_{i+1}-u_i
    // smoothness weight - one number for two different things). u_0 is what
    // actually reaches the plant next tick; u_1..u_{N-1} are just this
    // solve's internal plan, largely thrown away and replanned next tick.
    // Damping the REAL tick-to-tick output step more than the internal plan
    // shape targets actuator chatter without also flattening how much the
    // horizon is allowed to shape a correction internally. <= 0 falls back
    // to w_accel_change/w_steer_change (today's undifferentiated behavior),
    // so callers that don't set these (model_probe; mpc_probe except its own
    // dedicated check) are unaffected.
    double w_accel_change_u0{0.0};
    double w_steer_change_u0{0.0};

    // Cross-solve consistency: penalizes this horizon's controls for deviating
    // from what the PREVIOUS solve planned for the same future points (shifted
    // by one control period). Distinct from w_accel_change/w_steer_change,
    // which only anchor u_0 to the last APPLIED command and smooth WITHIN one
    // horizon - neither constrains steps 1..N-1 against the previous PLAN, which
    // is what lets consecutive solves disagree tick to tick even when u_0 is
    // damped (the receding-horizon "bang-bang" MPC oscillation). 0 = off
    // (default) - the previous horizon isn't tracked at all.
    double w_prev_track{0.0};


    // Heading-reference lookahead [m]. The heading reference is the bearing from
    // the foot point to a point this far ahead on the path (pure-pursuit style),
    // instead of the local foot-point tangent. A longer baseline rejects near-end
    // lateral jitter (which the heading gain otherwise saturates into a steering
    // limit cycle) and previews the upcoming path. <= 0 restores the local tangent.
    double lookahead_m{0.6};

    // Hard steering-RATE bound [rad/s], enforced as a linear inequality on
    // consecutive delta's - NOT the same thing as w_steer_change, which only
    // discourages large steer changes via a soft quadratic cost the optimizer
    // can still override when tracking error makes it worth it. Before this,
    // the model had NO notion of how fast the real actuator can move: delta_i
    // was a free variable at every horizon step, achievable instantly as far
    // as the QP knew, while the plant (sedan.urdf.xacro's steering joint +
    // the node's own output steer_slew_rps limiter) is rate-limited for real.
    // That let the QP plan corrections the plant would only partially deliver
    // (2026-09-04 turn-settling diagnosis - the magnitude-side counterpart of
    // this was the max_steer_rad-vs-steering_limit mismatch, fixed earlier
    // that session). Default is effectively unconstrained (10 rad/s, far above
    // any real steering actuator) so callers that don't set this (model_probe,
    // mpc_probe) keep their old behavior; the node wires this to the same
    // steer_slew_rps value that also rate-limits its output, so the QP and the
    // output stage agree on one number instead of the QP being unaware of it.
    double steer_rate_limit{10.0};

    // Real elapsed time [s] between control ticks, i.e. 1/control_rate_hz -
    // distinct from dt (the horizon's own, coarser discretization step) and
    // used ONLY for the delta_0-vs-previous_steer boundary rate bound, since
    // that transition happens over one real control tick, not one horizon
    // step. <= 0 falls back to dt (the old, less accurate behavior).
    double control_dt{0.0};

    // Hard steering bound RELATIVE TO PATH CURVATURE [rad], replacing the flat
    // +/-max_steer box: delta_i in [atan(L*kappa_i) - margin_i, atan(L*kappa_i)
    // + margin_i], clamped to +/-max_steer, where
    //   margin_i = steer_curvature_margin_base + steer_curvature_margin_slope * |kappa_i|
    // kappa_i comes from the reference path geometry (precomputed before the
    // QP is built), not a decision variable, so this stays a linear per-step
    // bound - a bound tied to the PREDICTED e_lat_i instead would multiply
    // two decision variables together (bilinear, non-convex, not solvable by
    // OSQP).
    // Effect: the QP can no longer command near-max_steer correction on a
    // gentle bend or straight just because tracking error is large - it's
    // capped to what the road geometry justifies plus this margin. The slope
    // term (2026-09-05) makes that cap TIGHTEST exactly on straights/gentle
    // bends (kappa~0, where a large correction is least justified - the
    // 2026-09-04 turn-settling failure mode: full-lock steer, 0.9 rad, on a
    // bend whose own curvature only called for ~0.18 rad) while loosening on
    // genuinely tight curves, which legitimately need more authority beyond
    // pure feed-forward. slope=0 recovers the old flat-margin behavior.
    // Large base default (10.0 rad) is effectively unconstrained, matching
    // steer_rate_limit's off-by-default convention, for callers that don't
    // set this (model_probe; mpc_probe except its own dedicated check).
    double steer_curvature_margin_base{10.0};
    double steer_curvature_margin_slope{0.0};

    // Extra margin per metre of CURRENT (measured, foot-point) lateral
    // error [rad/m]: margin_i = base + slope*|kappa_i| + elat_slope*|e_lat0|.
    // e_lat0 is a known scalar at solve time (like kappa_i) - the CURRENT
    // foot-point error, not the per-step PREDICTED e_lat_i (a decision
    // variable, which would make this bilinear/non-convex - see the note on
    // steer_curvature_margin_base above). Because e_lat0 doesn't vary with
    // horizon step i, this term shifts the WHOLE horizon's margin up
    // uniformly on a bad tick, so the QP isn't stuck at the tight
    // straight-road base while genuinely far off track - it can plan a
    // faster recovery, then the margin tightens back up again as e_lat0
    // shrinks on later ticks. 0 = off (the base/slope-on-kappa terms alone).
    double steer_curvature_margin_elat_slope{0.0};
};

struct MpcResult
{
    bool success{false};
    bool used_fallback{false};                  // true = proportional fallback was used, false = MPC solution was used
    double accel{0.0};                          // acceleration command [m/s^2]
    double steer{0.0};                          // steering command [rad]
    double e_lat{0.0};                          // foot-point lateral error at solve time [m], for diagnostics
    double e_head{0.0};                         // foot-point heading error at solve time [rad], for diagnostics
    std::vector<VehicleState> predicted_states; // for visualization/debugging
    std::vector<double> accels;                 // full control sequence [a_0..a_{N-1}]
    std::vector<double> steers;                 // full control sequence [delta_0..delta_{N-1}]
                                                 // (accels/steers: cross-solve tracking input for
                                                 // the NEXT call's w_prev_track term; empty when
                                                 // used_fallback is true - the fallback has no horizon)
};

class MpcSolver
{
public:
    explicit MpcSolver(const VehicleParams &vehicle_params, const MpcParams &mpc_params);

    MpcResult Solve(const VehicleState &current_state,
                    const std::vector<PathPoint> &reference_path,
                    double previous_accel,
                    double previous_steer);

    // Runtime override of the speed setpoint (from /maneuver/target_speed). The MPC
    // reads params only at startup, so the maneuver state machine's speed regime is
    // injected here each control tick when a fresh setpoint is available. Negative
    // values are clamped to 0 (never command reverse); the QP/stand-in still clamps
    // the resulting speed to [0, max_speed].
    void set_target_speed(double target_speed)
    {
        mp_.target_speed = std::max(0.0, target_speed);
    };

    // Runtime override of the heading-reference lookahead (maneuver-dependent: the
    // chord to the lookahead point cuts INSIDE curves by ~L^2/2R, so turns want a
    // short lookahead while straights want a long, noise-averaging one). Injected
    // per control tick from the planner's /maneuver/state, like the speed above.
    void set_lookahead(double lookahead_m)
    {
        mp_.lookahead_m = std::max(0.0, lookahead_m);
    };

private:
    // Single-step proportional law on [e_lat, e_head, v] — the pre-QP stand-in,
    // kept as the safety-net fallback whenever the OSQP solve fails to
    // initialize or converge (never leave the car without a command).
    MpcResult solve_proportional(
        const VehicleState &current_state,
        const std::vector<PathPoint> &reference_path,
        double previous_accel,
        double previous_steer);

    VehicleParams vp_;
    MpcParams mp_;

    // Previous solve's control sequence, for the w_prev_track cross-solve
    // consistency term. Invalidated (has_prev_traj_ = false) whenever a tick
    // falls back to solve_proportional, which has no horizon to hand back -
    // the term is skipped for the tick right after a fallback rather than
    // tracking a stale (or fabricated) trajectory.
    std::vector<double> prev_accels_, prev_steers_;
    bool has_prev_traj_{false};
};