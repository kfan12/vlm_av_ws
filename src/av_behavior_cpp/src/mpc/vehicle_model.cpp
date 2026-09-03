#include "mpc/vehicle_model.hpp"
#include <algorithm>
#include <cmath>

VehicleState integrate_kinematic(
    const VehicleState &state,
    const VehicleControl &control,
    const VehicleParams &params,
    double dt)
{
    VehicleControl us = clamp_control(control, params);
    VehicleState next;
    next.x = state.x + state.v * std::cos(state.psi) * dt;
    next.y = state.y + state.v * std::sin(state.psi) * dt;
    next.psi = state.psi + state.v * std::tan(us.delta) / params.wheelbase * dt;
    next.v = std::clamp(state.v + us.a * dt, params.min_speed, params.max_speed);
    return next;
}

VehicleControl clamp_control(const VehicleControl &control, const VehicleParams &params)
{
    VehicleControl clamped;
    clamped.a = std::clamp(control.a, params.max_decel, params.max_accel);
    clamped.delta = std::clamp(control.delta, -params.max_steer, params.max_steer);
    return clamped;
}