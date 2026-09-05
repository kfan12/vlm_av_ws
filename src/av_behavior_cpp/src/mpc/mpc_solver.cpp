#include "mpc/mpc_solver.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>
#include <Eigen/Sparse>
#include <algorithm>

namespace
{
    inline double wrap_pi(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }
} // namespace

MpcSolver::MpcSolver(const VehicleParams &vehicle_params, const MpcParams &mpc_params)
    : vp_(vehicle_params), mp_(mpc_params)
{
}

// Build and solve the linearized MPC as a QP problem.
// State error:[e_lat,e_head,e_speed]
// Control: [a, delta]
// total variables: (N+1)*3 + N*2

MpcResult MpcSolver::Solve(const VehicleState &current_state,
                           const std::vector<PathPoint> &reference_path,
                           double previous_accel,
                           double previous_steer)
{
    MpcResult result;
    if (reference_path.size() < 2)
    {
        result.success = false;
        return result;
    }

    const int n_state = 3;   // [e_lat, e_head, e_speed]
    const int n_control = 2; // [a, delta]
    const int N = mp_.N;     // Number of time steps
    const int n_vars = (N + 1) * n_state + N * n_control;

    const double L = vp_.wheelbase;
    const double dt = mp_.dt;

    const double v_ref = mp_.target_speed;                                          // Reference speed
    const double v_lin = std::clamp(current_state.v, vp_.min_speed, vp_.max_speed); // the operating point the lateral/heading dynamics are LINEARIZED around.

    // Reference tangent psi_ref from POINT geometry (not the poses' orientation,
    // which may be missing or wrong). Use a LOOKAHEAD bearing — the bearing from
    // point i to a point ~lookahead_m ahead on the path — instead of the raw local
    // segment tangent (ref[i]->ref[i+1], ~ds baseline). The local tangent turns a
    // couple cm of centerline noise into ~0.1-0.2 rad of reference-heading swing,
    // which the heading gain saturates into a steering limit cycle (the observed
    // straight-line oscillation). The long baseline averages that jitter out, and
    // curvature (kappa, a difference of tangents) inherits the de-noising. The node
    // schedules lookahead_m per maneuver (long on straights to kill noise, short in
    // turns to preserve curvature). lookahead_m <= 0 restores the local tangent.
    const int M = static_cast<int>(reference_path.size());
    const double look = mp_.lookahead_m;
    std::vector<double> psi_ref(M, 0.0);
    std::vector<double> kappa(M, 0.0);
    for (int i = 0; i < M; ++i)
    {
        int j = i + 1; // local tangent baseline#
        if (look > 0.0)
        {
            j = i;
            while (j < M - 1 && reference_path[j].s - reference_path[i].s < look)
                ++j;
        }

        if (j >= M)
            j = M - 1;
        if (j == i)
            psi_ref[i] = (i > 0) ? psi_ref[i - 1] : 0.0; // no point ahead, use previous tangent
        else
            psi_ref[i] = std::atan2(reference_path[j].y - reference_path[i].y,
                                    reference_path[j].x - reference_path[i].x);
    }

    // kappa = dpsi/ds = (psi[i+1]-psi[i])/(s[i+1]-s[i])
    for (int i = 0; i < M - 1; ++i)
    {
        double ds = reference_path[i + 1].s - reference_path[i].s;
        if (ds > 1e-6)
            kappa[i] = wrap_pi((psi_ref[i + 1] - psi_ref[i])) / ds;
    }

    /*
    z = [e_lat_0, e_head_0, e_speed_0,   ← state errors at step 0, e_speed is actual speed, not error
        e_lat_1, e_head_1, e_speed_1,   ← state errors at step 1
        ...
        e_lat_N, e_head_N, e_speed_N,   ← state errors at step N
        a_0,   δ_0,                     ← controls at step 0
        a_1,   δ_1,                     ← controls at step 1
        ...
        a_{N-1}, δ_{N-1}]               ← controls at step N-1

    */

    /*
    J = Σ_{i=0}^{N} [w_lat · e_lat(i)²  +  w_head · e_head(i)²  +  w_speed · e_speed(i)²]   // tracking error cost
            + Σ_{i=0}^{N-1} [w_accel · a(i)²  +  w_steer · δ(i)²]   // control effort cost
            + Σ_{i=0}^{N-2} [w_accel_change · Δa(i)²  +  w_steer_change · Δδ(i)²]  // control smoothness cost (Δa(i) = a(i+1)-a(i), Δδ(i) = δ(i+1)-δ(i))

    standard QP form: min 1/2 z' P z + q' z
    P = diag([w_lat, w_head, w_speed, ..., w_lat, w_head, w_speed, w_accel, w_steer, ..., w_accel, w_steer]) + Δu coupling
    q = [0, 0, -w_speed*v_ref, ..., 0, 0, -w_speed*v_ref, 0, 0, 0, ..., 0, 0, 0]   // linear term

    subject to linearized vehicle dynamics constraints: l  ≤  A · z  ≤  u
    where A is the linearized dynamics matrix, and l and u are the
    lower and upper bounds on the constraints.
    The linearized dynamics are derived from the vehicle model, and they
    relate the state at step i+1 to the state at step i and the control inputs at step i.
    The constraints ensure that the predicted states follow the vehicle dynamics given the control inputs.

    */

    // initial condition at the foot point
    const PathPoint &ref0 = reference_path[0];

    const double e_lat0 = -std::sin(ref0.psi) * (current_state.x - ref0.x) +
                          std::cos(ref0.psi) * (current_state.y - ref0.y);

    const double e_head0 = wrap_pi(current_state.psi - ref0.psi);
    const double v0 = current_state.v;

    // Cost matrix P and gradient q. Weights follow the (1/2)*w*(.)^2 convention
    // (OSQP applies the 1/2). P is built from triplets (setFromTriplets SUMS
    // duplicates) so the diagonal accumulates the state/effort weights plus the
    // control-rate (Δu) contributions. OsqpEigen takes the upper triangle, so
    // off-diagonal Δu coupling is emitted once as the upper (i<j) entry.
    const double w_da = mp_.w_accel_change; // Δaccel weight (within-horizon)
    const double w_ds = mp_.w_steer_change; // Δsteer weight (within-horizon)
    // boundary-only weights for (u_0 - previous applied)^2; <=0 falls back
    // to the within-horizon weight above (see w_accel_change_u0/
    // w_steer_change_u0 in mpc_solver.hpp for why these are separate).
    const double w_da0 = mp_.w_accel_change_u0 > 0.0 ? mp_.w_accel_change_u0 : w_da;
    const double w_ds0 = mp_.w_steer_change_u0 > 0.0 ? mp_.w_steer_change_u0 : w_ds;

    std::vector<Eigen::Triplet<double>> Ptr;
    Ptr.reserve(n_vars + 4 * N); // diagonal + Δu off-
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

    // state tracking consts (diagonal) + speed target (linear term in q)
    for (int i = 0; i < N + 1; ++i)
    {
        int idx_base = i * n_state;
        Ptr.emplace_back(idx_base + 0, idx_base + 0, mp_.w_lat);   // e_lat²
        Ptr.emplace_back(idx_base + 1, idx_base + 1, mp_.w_head);  // e_head²
        Ptr.emplace_back(idx_base + 2, idx_base + 2, mp_.w_speed); // e_speed²
        q[idx_base + 2] = -mp_.w_speed * v_ref;                    // linear term for speed target
    }

    // control effort (diagonal) + control-rate smoothness (Δu). The Δu penalty
    // includes the change from the PREVIOUS applied command (u_0 - prev)^2 —
    // this is what damps the tick-to-tick steering limit cycle, and is why the
    // solver takes previous_accel/previous_steer.

    auto ai = [&](int i)
    { return (N + 1) * n_state + i * n_control; }; // accel index
    auto si = [&](int i)
    { return (N + 1) * n_state + i * n_control + 1; }; // steer index

    for (int i = 0; i < N; ++i)
    {
        Ptr.emplace_back(ai(i), ai(i), mp_.w_accel); // a², diagonal
        Ptr.emplace_back(si(i), si(i), mp_.w_steer); // δ², diagonal
    }

    // boundary term (u_0 - prev)^2: diagonal += w, gradient += -w*prev.
    // Uses the u0-specific weight (falls back to the within-horizon weight
    // if unset) - this is the REAL tick-to-tick output step, distinct from
    // the within-horizon terms below.
    Ptr.emplace_back(ai(0), ai(0), w_da0); // a², diagonal
    Ptr.emplace_back(si(0), si(0), w_ds0); // δ², diagonal
    q[ai(0)] += -w_da0 * previous_accel;   // linear term
    q[si(0)] += -w_ds0 * previous_steer;   // linear term

    // within horizon (u_{i+1} - u{i})^2: diagonal += w on both, upper off-diag (i, i +1) -w;
    for (int i = 0; i < N - 1; ++i)
    {
        Ptr.emplace_back(ai(i), ai(i), w_da);         // a{i}
        Ptr.emplace_back(ai(i + 1), ai(i + 1), w_da); // a{i+1}
        Ptr.emplace_back(ai(i), ai(i + 1), -w_da);    // a{i}*a{i+1}
        Ptr.emplace_back(si(i), si(i), w_ds);         // s{i}
        Ptr.emplace_back(si(i + 1), si(i + 1), w_ds); // s{i+1}
        Ptr.emplace_back(si(i), si(i + 1), -w_ds);    // s{i}*s{i+1}
    }

    // cross-solve consistency (u_i - u_prev_shifted_i)^2: keeps this horizon
    // close to what the PREVIOUS solve planned for the same future points, one
    // control period on (index i+1 of the previous trajectory; the previous
    // trajectory's last point is held for i = N-1, past its own end). Separate
    // from the (u_0 - last applied command)^2 anchor above - that one tracks
    // what was actually DRIVEN, this one tracks what was previously PLANNED.
    // Off by default (w_prev_track = 0); skipped for one tick after a
    // proportional-fallback tick, when there is no previous horizon to track.
    if (has_prev_traj_ && mp_.w_prev_track > 0.0 &&
        static_cast<int>(prev_accels_.size()) == N &&
        static_cast<int>(prev_steers_.size()) == N)
    {
        const double w_pt = mp_.w_prev_track;
        for (int i = 0; i < N; ++i)
        {
            const int shifted = std::min(i + 1, N - 1);
            Ptr.emplace_back(ai(i), ai(i), w_pt);
            Ptr.emplace_back(si(i), si(i), w_pt);
            q[ai(i)] += -w_pt * prev_accels_[shifted];
            q[si(i)] += -w_pt * prev_steers_[shifted];
        }
    }

    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    P.setFromTriplets(Ptr.begin(), Ptr.end());
    P.makeCompressed(); // OSQP requires compressed sparse column format

    // constraints: l<= Az <= u
    // dynamics (n_state*N) + control bounds (n_control*N) + state_bounds
    // (n_state*(N+1)) + steering-rate bounds (N: 1 boundary row for
    // delta_0-vs-previous_steer, N-1 within-horizon delta_{i+1}-delta_i rows)
    const int n_constraints = n_state * N + n_vars + N;
    Eigen::SparseMatrix<double> A(n_constraints, n_vars); // n_constraints*n_vars
    Eigen::VectorXd l(n_constraints);
    Eigen::VectorXd u(n_constraints);

    std::vector<Eigen::Triplet<double>> Tr;
    Tr.reserve(n_state * N * 3 + n_vars + 2 * N); // rough estimate of nonzeros

    // system dynamics constraints: e_lat, e_head, v, 3 rows per step, N steps.
    // Coupling terms linearized around v_lin (current speed), NOT v_ref (target):
    // e_lat_{i+1} = e_lat_i + dt*v_lin*e_head_i
    // e_head_{i+1} = e_head_i + dt*(v_lin/L)*delta_i - dt*v_lin*kappa_i
    // v_{i+1} = v_i + dt*a_i   (speed dynamics unaffected)

    for (int i = 0; i < N; ++i)
    {
        const int row = i * n_state;                      // row for e_lat dynamics constraints in A
        const int xi = i * n_state;                       // e_lat_{i} position in z
        const int xi1 = (i + 1) * n_state;                // e_lat_{i + 1} position in z
        const int ui = (N + 1) * n_state + i * n_control; // a_{i} position in z
        const double k_i = kappa[std::min(i, M - 1)];     // kappa{i}, clamped by reference length

        // e_lat_{i+1} - e_lat_i - dt*v_lin*e_head_i = 0
        Tr.emplace_back(row + 0, xi1 + 0, 1.0);        // 1.0 -> e_lat_{i+1}
        Tr.emplace_back(row + 0, xi + 0, -1.0);        // -1.0 -> e_lat_{i}
        Tr.emplace_back(row + 0, xi + 1, -dt * v_lin); // -dt * v_lin->e_head_ { i }
        l(row + 0) = u(row + 0) = 0.0;                 // right side of equation

        // e_head_{i+1} - e_head_i - dt*(v_lin/L)*delta_i = -dt*v_lin*k_i
        Tr.emplace_back(row + 1, xi1 + 1, 1.0);            // 1.0 -> e_head_{i+1}
        Tr.emplace_back(row + 1, xi + 1, -1.0);            // -1.0 -> e_head_{i}
        Tr.emplace_back(row + 1, ui + 1, -dt * v_lin / L); // -dt * v_lin/L->delta { i }
        l(row + 1) = u(row + 1) = -dt * v_lin * k_i;       // right side of equation

        // v_{i+1} - v_i - dt*a_i = 0
        Tr.emplace_back(row + 2, xi1 + 2, 1.0); // 1.0 -> v_{i+1}
        Tr.emplace_back(row + 2, xi + 2, -1.0); // -1.0 -> v_{i}
        Tr.emplace_back(row + 2, ui + 0, -dt);  // -dt ->a { i }
        l(row + 2) = u(row + 2) = 0.0;          // right side of equation
    }

    // boundary conditions
    // e_lat_0, e_head_0,  v_0  started at row = N*n_state
    const int base = N * n_state; // implemented dynamic constraints
    for (int i = 0; i < n_vars; ++i)
    {
        Tr.emplace_back(base + i, i, 1.0);
        l(base + i) = -OsqpEigen::INFTY;
        u(base + i) = OsqpEigen::INFTY; // initialize
    }

    // initial condition at the foot point
    l(base + 0) = u(base + 0) = e_lat0;
    l(base + 1) = u(base + 1) = e_head0;
    l(base + 2) = u(base + 2) = v0;

    // speed bounds: 0 <= v <= max_speed, for all steps after the pinned step 0
    for (int i = 1; i <= N; ++i)
    {
        l(base + i * n_state + 2) = 0.0;
        u(base + i * n_state + 2) = vp_.max_speed;
    }

    // control bounds: max_decel <= a <= max_accel, for all steps.
    // delta is bounded relative to path curvature, not a flat +/-max_steer
    // box - see steer_curvature_margin_base/_slope/_elat_slope in
    // mpc_solver.hpp. The margin grows with |kappa_i| (tightest on straights,
    // loosest on genuinely tight curves) AND with the CURRENT measured
    // |e_lat0| (uniformly across the whole horizon), so a car that's
    // genuinely far off track gets more correction authority to recover
    // quickly, while a car that's on track stays capped tight even through a
    // straight/gentle stretch.
    const double elat_margin = mp_.steer_curvature_margin_elat_slope * std::abs(e_lat0);
    for (int i = 0; i < N; ++i)
    {
        l(base + (N + 1) * n_state + i * n_control + 0) = vp_.max_decel;
        u(base + (N + 1) * n_state + i * n_control + 0) = vp_.max_accel;

        const double k_i = kappa[std::min(i, M - 1)];
        const double delta_ff = std::atan(L * k_i);
        const double margin_i = mp_.steer_curvature_margin_base +
                                mp_.steer_curvature_margin_slope * std::abs(k_i) +
                                elat_margin;
        l(base + (N + 1) * n_state + i * n_control + 1) =
            std::clamp(delta_ff - margin_i, -vp_.max_steer, vp_.max_steer);
        u(base + (N + 1) * n_state + i * n_control + 1) =
            std::clamp(delta_ff + margin_i, -vp_.max_steer, vp_.max_steer);
    }

    // steering-rate bounds: |delta change| <= steer_rate_limit * (elapsed time).
    // Hard linear constraint, not a soft cost - the QP's own plan now can't
    // ask for a swing faster than the plant (joint velocity + the node's
    // output steer_slew_rps) can actually deliver, closing the plan-vs-plant
    // rate gap the model previously had no notion of.
    const int rate_base = base + n_vars; // after dynamics + all bound rows
    const double dt0 = mp_.control_dt > 0.0 ? mp_.control_dt : dt; // real tick interval for u_0's transition
    const double rate_bound0 = mp_.steer_rate_limit * dt0;
    const double rate_bound = mp_.steer_rate_limit * dt; // within-horizon steps, spaced by dt

    // delta_0 - previous_steer, bounded by the real one-tick achievable swing
    Tr.emplace_back(rate_base, si(0), 1.0);
    l(rate_base) = previous_steer - rate_bound0;
    u(rate_base) = previous_steer + rate_bound0;

    // delta_{i+1} - delta_i for i = 0..N-2, bounded by the horizon-step swing
    for (int i = 0; i < N - 1; ++i)
    {
        const int row = rate_base + 1 + i;
        Tr.emplace_back(row, si(i + 1), 1.0);
        Tr.emplace_back(row, si(i), -1.0);
        l(row) = -rate_bound;
        u(row) = rate_bound;
    }

    A.setFromTriplets(Tr.begin(), Tr.end());
    A.makeCompressed();

    // OSQP solver setup
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);
    if (!solver.data()->setHessianMatrix(P) ||           // HessianMatrix (cost matrix)
        !solver.data()->setGradient(q) ||                // Gradient (linear cost vector)
        !solver.data()->setLinearConstraintsMatrix(A) || // Constraints Matrix
        !solver.data()->setLowerBound(l) ||              // lower bounds
        !solver.data()->setUpperBound(u) ||              // higher bounds
        !solver.initSolver())
    { // fall back to proportional control - no horizon to hand back
        has_prev_traj_ = false;
        return solve_proportional(current_state, reference_path, previous_accel, previous_steer);
    }

    // Run the solve. solveProblem() returning NoError only means "no runtime
    // error": a primal-infeasible or max-iteration exit still returns NoError
    // with a garbage solution vector, so also require the status to be Solved
    // before trusting z. Either failure routes to the proportional safety net so
    // the car is never left without a command.
    const OsqpEigen::ErrorExitFlag exit_flag = solver.solveProblem();
    const OsqpEigen::Status status = solver.getStatus();
    if (exit_flag != OsqpEigen::ErrorExitFlag::NoError ||
        (status != OsqpEigen::Status::Solved &&
         status != OsqpEigen::Status::SolvedInaccurate))
    {
        has_prev_traj_ = false; // no horizon to hand back
        return solve_proportional(current_state, reference_path, previous_accel, previous_steer);
    }

    Eigen::VectorXd z = solver.getSolution();

    // Extract first control input (a_0, delta_0) and predicted states
    result.accel = std::clamp(z[(N + 1) * n_state + 0], vp_.max_decel, vp_.max_accel);
    result.steer = std::clamp(z[(N + 1) * n_state + 1], -vp_.max_steer, vp_.max_steer);
    result.e_lat = e_lat0;
    result.e_head = e_head0;
    result.success = true;

    // Predicted preview: roll the NONLINEAR model out under the QP's control
    // sequence (the QP states are linearized errors, not poses).
    VehicleState s = current_state;
    for (int i = 0; i < N; ++i)
    {
        const int ui = (N + 1) * n_state + i * n_control;
        VehicleControl uc{std::clamp(z[ui + 0], vp_.max_decel, vp_.max_accel),
                          std::clamp(z[ui + 1], -vp_.max_steer, vp_.max_steer)};
        s = integrate_kinematic(s, uc, vp_, dt);
        result.predicted_states.push_back(s);
        result.accels.push_back(uc.a);
        result.steers.push_back(uc.delta);
    }

    // hand this horizon to the NEXT call's w_prev_track term
    prev_accels_ = result.accels;
    prev_steers_ = result.steers;
    has_prev_traj_ = true;

    return result;
}

MpcResult MpcSolver::solve_proportional(const VehicleState &current_state,
                                        const std::vector<PathPoint> &reference_path,
                                        double previous_accel,
                                        double previous_steer)
{
    MpcResult result;
    result.used_fallback = true;

    VehicleState s = current_state;
    double best_steer = 0.0;
    double best_accel = 0.0;

    // Reference foot point = closest path point (ref[0], where resampling began).
    // Take the path tangent from the geometry (ref[0] -> lookahead point), not from
    // the poses' orientation, so this works even if the drawn path has no headings.
    //
    // Heading reference = bearing from ref0 to a point ~mp_.lookahead_m ahead on the
    // path (pure-pursuit style). Using the LOCAL tangent (ref0 -> ref1, ~0.1 m
    // baseline) turns a couple cm of near-end lateral noise into ~0.1-0.2 rad of
    // reference-heading swing, which the heading gain then saturates into a steering
    // limit cycle. The longer lookahead baseline averages that jitter out and previews
    // the path. mp_.lookahead_m <= 0 restores the old local tangent for A/B testing.
    const auto &ref0 = reference_path.front();
    size_t look_idx;

    if (mp_.lookahead_m > 0.0)
    {
        look_idx = reference_path.size() - 1; // fall back to the farthest point
        for (size_t i = 1; i < reference_path.size(); ++i)
        {
            double d = std::hypot(reference_path[i].x - ref0.x,
                                  reference_path[i].y - ref0.y);
            if (d >= mp_.lookahead_m)
            {
                look_idx = i;
                break;
            }
        }
    }
    else
    {
        look_idx = std::min<size_t>(1, reference_path.size() - 1);
    }

    const auto &ref_look = reference_path[look_idx];
    double psi_ref = std::atan2(ref_look.y - ref0.y, ref_look.x - ref0.x);

    // Cross-track error: signed lateral offset in the path frame (+ = left of path).
    double dx = s.x - ref0.x;
    double dy = s.y - ref0.y;
    double e_lat = -std::sin(psi_ref) * dx + std::cos(psi_ref) * dy;

    // Heading error, normalized to [-pi, pi].
    double e_head = s.psi - psi_ref;
    while (e_head > M_PI)
        e_head -= 2 * M_PI;
    while (e_head < -M_PI)
        e_head += 2 * M_PI;

    // Gains from the cost-weight ratios (state weight / control weight). e_lat and
    // e_head together act as PD on cross-track error (e_head ~ d(e_lat)/dt), so the
    // law is well-damped without a separate derivative term. Steering positive = left,
    // so a robot left of / pointing left of the path steers right (negative).
    double k_lat = mp_.w_lat / std::max(mp_.w_steer, 1e-6);
    double k_head = mp_.w_head / std::max(mp_.w_steer, 1e-6);
    double k_speed = mp_.w_speed / std::max(mp_.w_accel, 1e-6);

    best_steer = std::clamp(-(k_lat * e_lat + k_head * e_head),
                            -vp_.max_steer, vp_.max_steer);

    // Speed: the third state slot is ACTUAL speed (not an error) — same as the QP,
    // where the target is applied through the gradient q, not the state. The
    // proportional accel drives actual speed toward target_speed.
    best_accel = std::clamp(k_speed * (mp_.target_speed - s.v),
                            vp_.max_decel, vp_.max_accel);

    // Blend with the previous command — stand-in for the QP's control-rate
    // (smoothness) weights. A higher rate-weight => stronger blend toward the
    // previous command => smoother / less oscillation, which is exactly the knob
    // the Day 15 tuning rules reach for (raise weight_steer_rate to damp wobble).
    // Normalized to [0,1) so it can't run away; kRateRef sets where the default
    // weights sit (w_steer_change=2 -> ~0.33, matching the old fixed 0.3).
    const double kRateRef = 4.0;
    double smooth_steer = mp_.w_steer_change / (mp_.w_steer_change + kRateRef);
    double smooth_accel = mp_.w_accel_change / (mp_.w_accel_change + kRateRef);
    best_accel = (1.0 - smooth_accel) * best_accel + smooth_accel * previous_accel;
    best_steer = (1.0 - smooth_steer) * best_steer + smooth_steer * previous_steer;

    result.accel = best_accel;
    result.steer = best_steer;
    result.e_lat = e_lat;
    result.e_head = e_head;
    result.success = true;

    // Constant-control rollout for the /mpc_predicted_path preview.
    VehicleControl u{best_accel, best_steer};
    for (int i = 0; i < mp_.N; ++i)
    {
        s = integrate_kinematic(s, u, vp_, mp_.dt);
        result.predicted_states.push_back(s);
    }

    return result;
}
