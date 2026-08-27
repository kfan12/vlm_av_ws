#pragma once
// Hysteresis latch — port of the v1 sign_latch.py discipline:
//  * k consecutive identical observations to switch the latched value
//  * "no observation" (nullopt) NEVER clears a latched value inside lock_dist
//  * level-triggered: the latched value holds until actively replaced
#include <optional>

namespace av
{

    template <typename T>
    class HysteresisLatch
    {
    public:
        HysteresisLatch(int k_switch, double lock_dist, T initial)
            : k_(k_switch), lock_dist_(lock_dist), value_(initial) {}

        // observation: nullopt = nothing seen this frame. distance: current range to
        // the observed object (drives the lock rule). Returns the latched value.
        T update(const std::optional<T> &observation, double distance)
        {
            if (!observation.has_value())
            {
                // Inside lock distance an absent observation cannot erode the latch.
                if (distance > lock_dist_)
                {
                    candidate_.reset();
                    count_ = 0;
                }
                return value_;
            }
            if (*observation == value_)
            {
                candidate_.reset();
                count_ = 0;
                latched_ = true;
                return value_;
            }
            if (candidate_.has_value() && *candidate_ == *observation)
            {
                if (++count_ >= k_)
                {
                    value_ = *observation;
                    latched_ = true;
                    candidate_.reset();
                    count_ = 0;
                }
            }
            else
            {
                candidate_ = observation;
                count_ = 1;
            }
            return value_;
        }

        const T &value() const { return value_; }
        bool latched() const { return latched_; }
        void reset(T v)
        {
            value_ = v;
            latched_ = false;
            candidate_.reset();
            count_ = 0;
        }

    private:
        int k_;
        double lock_dist_;
        T value_;
        bool latched_ = false;
        std::optional<T> candidate_;
        int count_ = 0;
    };

} // namespace av