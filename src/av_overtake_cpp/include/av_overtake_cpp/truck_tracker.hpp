#pragma once

// One vehicle in the right lane, tracked along the lane in world-stable arc
// length (see lateral_shift.hpp): its rear position, its speed and its length.
//
// The front lidar sees the vehicle's rear face while the ego is behind it, and
// its left side (front corner included) once the ego is in the left lane. Beside
// it, the vehicle leaves the 120 deg field of view entirely, so the track
// carries on by prediction; front_bound() adds the distance the vehicle could
// have gained since it was last seen by accelerating at accel_bound up to
// speed_gain_bound above its last known speed, which is what the merge
// decision uses. The speed gain must be bounded: with acceleration alone the
// margin grows with the square of the blind time and outruns any speed
// advantage, so the ego would never merge back.

#include <algorithm>
#include <cmath>
#include <optional>

namespace av::overtake
{
    class TruckTracker
    {
    public:
        struct Params
        {
            double alpha = 0.4;            // position gain
            double beta = 0.1;             // speed gain
            double gate_m = 2.5;           // a measurement further from the prediction is another object
            double length_prior_m = 6.0;   // assumed length until the whole side has been seen
            double accel_bound = 0.3;      // [m/s^2] the most the vehicle is assumed to accelerate unseen ...
            double speed_gain_bound = 0.5; // [m/s] ... and the most it is assumed to speed up in total
        };

        // explicitly construct with parameters
        explicit TruckTracker(const Params &p) : p_(p) {}
        bool tracked() const { return tracked_; }
        void reset() { tracked_ = false; }

        void start(double t, double s_rear, double v_init)
        {
            tracked_ = true;
            s_rear_ = s_rear;
            v_ = v_init;
            t_ = t;
            length_ = p_.length_prior_m;
            length_seen_ = false;
        }
        double rear_at(double t) const { return s_rear_ + v_ * (t - t_); }
        double front_at(double t) const { return rear_at(t) + length_; }
        double front_bound(double t) const
        {
            const double dt = std::max(0.0, t - t_);
            const double a = std::max(1e-6, p_.accel_bound);
            const double t_ramp = p_.speed_gain_bound / a; // time to gain the whole speed bound
            const double extra = dt <= t_ramp
                                     ? 0.5 * a * dt * dt
                                     : 0.5 * a * t_ramp * t_ramp + p_.speed_gain_bound * (dt - t_ramp);
            return front_at(t) + extra;
        }

    private:
        Params p_;
        bool tracked_ = false, length_seen_ = false;
        double s_rear_ = 0.0, v_ = 0.0, length_ = 6.0, t_ = 0.0;
    };
} // namespace av::overtake