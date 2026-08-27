#pragma once

// camera projection chain - c++ port of vlm_planner_py/projection.py math
// Frames: optical (z fwd, x right, y down) -> camera (x fwd, y left, z up), pitch down -> base_link-> odom
// the extrinsic is built from fixed URDF mount params (no TF)

#include <optional>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/camera_info.hpp>

namespace av::proj
{

    struct Pinhole
    {
        double fx = 0, fy = 0, cx = 0, cy = 0;
        static Pinhole from_info(const sensor_msgs::msg::CameraInfo &info)
        {
            return {info.k[0], info.k[4], info.k[2], info.k[5]};
        }

        bool valid() const
        {
            return fx > 1.0 && fy > 1.0;
        }
    };

    inline Eigen::Vector3d pixel_depth_to_camera(const Pinhole &p, double u, double v, double depth)
    {
        return {(u - p.cx) * depth / p.fx,
                (v - p.cy) * depth / p.fy,
                depth};
    }

    // T_base_cam maps Optical frame to base_link frame, given a pitch-down camera mount angle (radians)
    inline Eigen::Isometry3d make_T_base_cam(double cam_x, double cam_y, double cam_z, double mount_pitch)
    {
        // optical -> camera link: opt z -> cam x, opt x -> cam -y, opt y -> cam -z
        Eigen::Matrix3d R_opt_cam;
        R_opt_cam << 0, 0, 1,
            -1, 0, 0,
            0, -1, 0;

        Eigen::Isometry3d T_base_cam = Eigen::Isometry3d::Identity();
        // rotation block: pitch-down mount rotation (about base_link's Y axis)
        // composed with the fixed optical->camera-link axis remap above.
        T_base_cam.linear() = Eigen::AngleAxisd(mount_pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() * R_opt_cam;
        // translation block: where the camera origin sits in base_link, from URDF mount xyz.
        T_base_cam.translation() = Eigen::Vector3d(cam_x, cam_y, cam_z);
        return T_base_cam;
    }
    // base_link -> pixel (for map-gated ROIs). nullopt when behind the camera.
    inline std::optional<Eigen::Vector2d> base_to_pixel(
        const Pinhole &p, const Eigen::Isometry3d &T_base_cam,
        const Eigen::Vector3d &p_base)
    {
        Eigen::Vector3d pc = T_base_cam.inverse() * p_base;
        if (pc.z() < 0.3)
            return std::nullopt;
        return Eigen::Vector2d(p.fx * pc.x() / pc.z() + p.cx,
                               p.fy * pc.y() / pc.z() + p.cy);
    }

    struct Pose2d
    {
        double x = 0, y = 0, yaw = 0;
        Eigen::Vector2d odom_of(const Eigen::Vector2d &p_base) const
        {
            double c = std::cos(yaw), s = std::sin(yaw);
            return {x + c * p_base.x() - s * p_base.y(),
                    y + s * p_base.x() + c * p_base.y()};
        }
        Eigen::Vector2d base_of(const Eigen::Vector2d &p_odom) const
        {
            double c = std::cos(yaw), s = std::sin(yaw);
            double dx = p_odom.x() - x, dy = p_odom.y() - y;
            return {c * dx + s * dy, -s * dx + c * dy};
        }
    };

} // namespace av::proj