#pragma once
#include <Eigen/Core>

struct VehicleState
{
    double x{0.0}, y{0.0}, psi{0.0}, v{0.0};
};

struct VehicleControl
{
    double a{0.0}, delta{0.0};
};

struct VehicleParams
{
    double wheelbase{2.7};  // m
    double max_steer{0.5};  // rad
    double max_speed{5.0};  // m/s
    double min_speed{0.0};  // m/s
    double max_accel{1.2};  // m/s^2
    double max_decel{-1.8}; // m/s^2
};

// advanced state by dt using euler integration, return new state
VehicleState integrate_kinematic(
    const VehicleState &state,
    const VehicleControl &control,
    const VehicleParams &params,
    double dt);

// clamp control inputs to vehicle limits
VehicleControl clamp_control(
    const VehicleControl &control,
    const VehicleParams &params);
