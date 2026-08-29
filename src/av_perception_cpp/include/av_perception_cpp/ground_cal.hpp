#pragma once
// Ground-plane pitch auto-calibration.
//  * 2026-07-11 finding (v1 hit the same, KB "published cx/fx ratio" note):
//    the bridged camera_info K did not match the configured sensor ->
//    intrinsics derived from the configured width/height/hfov instead of
//    read from camera_info. Retested 2026-08-28 against the live bridge at
//    640x360 and 848x480 (hfov=1.6): published K matched this formula
//    exactly in both cases, no mismatch reproduced on the current
//    Gazebo/ros_gz_bridge versions. Kept as the source of truth regardless,
//    since it is harmless and removes camera_info as a dependency;
//  * historical: the effective camera pitch measured from ground depth rows
//    (~0.03 rad) used to differ from the URDF mount value (0.06). Root-caused
//    2026-08-28: front wheels mounted via steering_link with an extra
//    -ch_h/2 z offset that rear wheels never got, forcing base_link off
//    level by ~-0.028 rad at rest (confirmed via odom_truth) - not
//    suspension (this model has none), a wheel-mount geometry bug. Fixed the
//    same day in sedan.urdf.xacro (chassis_joint = wheel_radius,
//    steering_link z_off = 0, imu/camera reparented under chassis with
//    heights expressed relative to it) so base_link now sits exactly at
//    ground level (odom_truth position.z = 0, pitch ~ 0) and cam_z is
//    simultaneously correct as both the base_link-relative and
//    ground-relative camera height. estimate_pitch is kept regardless of
//    the fix, since online correction is cheap insurance against whatever
//    the next geometry or physics-engine quirk turns out to be.
// Model per ground pixel: cam_z / depth = sin(p) + ((v - cy)/fy) cos(p).
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

#include "av_common/projection.hpp"

namespace av::percep
{

    inline std::optional<double> estimate_pitch(const cv::Mat &depth32f,
                                                const av::proj::Pinhole &pin,
                                                double cam_z)
    {
        std::vector<double> est;
        const double fracs_u[] = {0.35, 0.5, 0.65};
        const double fracs_v[] = {0.92, 0.80, 0.68};
        for (double fu : fracs_u)
            for (double fv : fracs_v)
            {
                int u = static_cast<int>(depth32f.cols * fu);
                int v = static_cast<int>(depth32f.rows * fv);
                float d = depth32f.at<float>(v, u);
                if (!std::isfinite(d) || d < 0.5f || d > 30.f)
                    continue;
                // cos(p) ~= 1 for p < 0.1 rad (error < 0.5 %)
                double s = cam_z / d - (v - pin.cy) / pin.fy;
                if (s < -0.05 || s > 0.25)
                    continue; // not ground (obstacle/curb/hole)
                est.push_back(std::asin(std::clamp(s, -1.0, 1.0)));
            }
        if (est.size() < 3)
            return std::nullopt;
        std::nth_element(est.begin(), est.begin() + est.size() / 2, est.end());
        return est[est.size() / 2];
    }

    inline av::proj::Pinhole pinhole_from_config(int w, int h, double hfov)
    {
        double fx = (w / 2.0) / std::tan(hfov / 2.0);
        return {fx, fx, w / 2.0, h / 2.0};
    }

} // namespace av::percep