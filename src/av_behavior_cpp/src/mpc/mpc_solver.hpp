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
    double w_accel_change{0.4}; // weight for change in acceleration
    double w_steer_change{2.0}; // weight for change in steering

    // Heading-reference lookahead [m]. The heading reference is the bearing from
    // the foot point to a point this far ahead on the path (pure-pursuit style),
    // instead of the local foot-point tangent. A longer baseline rejects near-end
    // lateral jitter (which the heading gain otherwise saturates into a steering
    // limit cycle) and previews the upcoming path. <= 0 restores the local tangent.
    double lookahead_m{0.6};
};

struct MpcResult
{
    bool success{false};
    bool used_fallback{false};                  // true = proportional fallback was used, false = MPC solution was used
    double accel{0.0};                          // acceleration command [m/s^2]
    double steer{0.0};                          // steering command [rad]
    std::vector<VehicleState> predicted_states; // for visualization/debugging
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
};