#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "mpc/mpc_solver.hpp"
#include "mpc/path_utils.hpp"
#include "mpc/vehicle_model.hpp"

class MpcTrackerV2 : public rclcpp::Node
{
public:
    MpcTrackerV2() : Node("mpc_tracker_v2")
    {
        rate_hz_ = declare_parameter("control_rate_hz", 30.0);      // control loop frequency [Hz]
        N_ = declare_parameter("horizon_steps", 25);                // MPC prediction horizon length [steps]
        double dt = declare_parameter("dt", 0.12);                  // MPC step time [s]
        double wb = declare_parameter("wheelbase_m", 2.7);          // front-to-rear axle distance [m]
        double max_spd = declare_parameter("max_speed_mps", 3.0);   // speed cap fed to the solver [m/s]
        double max_acc = declare_parameter("max_accel_mps2", 1.2);  // max forward acceleration [m/s^2]
        double max_dec = declare_parameter("max_decel_mps2", -1.2); // max braking, negative [m/s^2]
        double max_str = declare_parameter("max_steer_rad", 0.75);  // steering angle limit [rad]

        MpcParams mp{};
        mp.N = N_;
        mp.dt = dt;
        dt_ = dt; // kept for reference-path resampling: horizon node spacing = v * dt_
        mp.target_speed = 0.0;

        mp.w_lat = declare_parameter("weight_lateral_error", 6.0);       // cost weight on cross-track error e_lat
        mp.w_head = declare_parameter("weight_heading_error", 3.0);      // cost weight on heading error e_head
        mp.w_speed = declare_parameter("weight_speed_error", 1.0);       // cost weight on speed error e_speed
        mp.w_accel = declare_parameter("weight_accel", 0.2);             // cost weight on acceleration effort a
        mp.w_steer = declare_parameter("weight_steer", 3.0);             // cost weight on steering effort delta
        mp.w_accel_change = declare_parameter("weight_accel_rate", 0.4); // cost weight on accel change, smoothness
        mp.w_steer_change = declare_parameter("weight_steer_rate", 6.0); // cost weight on steer change, damps wobble
        mp.w_prev_track = declare_parameter("weight_prev_track", 0.0);  // cross-solve consistency vs the previous horizon; 0 = off (see mpc_solver.hpp)
        mp.lookahead_m = declare_parameter("heading_lookahead_m", 5.0);  // heading-reference lookahead on straights [m]
        look_turn_ = declare_parameter("heading_lookahead_turn_m", 5.0); // heading-reference lookahead in turns [m]
        look_time_ = declare_parameter("heading_lookahead_time_s", 3.2); // lookahead as time headway, dist = v * this [s]
        look_min_ = declare_parameter("heading_lookahead_min_m", 5.0);   // lower clamp on the computed lookahead [m]
        look_max_ = declare_parameter("heading_lookahead_max_m", 14.0);  // upper clamp on the computed lookahead [m]
        steer_slew_ = declare_parameter("steer_slew_rps", 2.0);          // max steering-command slew rate [rad/s]

        VehicleParams vp{};
        vp.wheelbase = wb;
        vp.max_speed = max_spd;
        vp.max_accel = max_acc;
        vp.max_decel = max_dec;
        vp.max_steer = max_str;
        vp_ = vp;
        solver_ = std::make_unique<MpcSolver>(vp, mp);

        path_timeout_ = declare_parameter("path_timeout_sec", 2.0);
        hold_without_maneuver_ = declare_parameter("hold_without_maneuver", true);
        search_radius_ = declare_parameter("closest_point_search_radius_m", 3.0);
        goal_tol_ = declare_parameter("goal_tolerance_m", 0.5);
        accel_rate_ = declare_parameter("target_speed_accel_rate_mps2", 1.5);
        decel_rate_ = declare_parameter("target_speed_decel_rate_mps2", 6.0);

        sub_path_ = create_subscription<nav_msgs::msg::Path>(
            "/plan/path_odom", 10,
            [this](nav_msgs::msg::Path::SharedPtr m)
            {
                path_ = m;
                path_time_ = now();
            });

        sub_profile_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/plan/speed_profile", 10,
            [this](std_msgs::msg::Float32MultiArray::SharedPtr m)
            {
                profile_ = m->data;
            });

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom_ekf", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr m)
            { odom_ = m; });

        sub_maneuver_ = create_subscription<std_msgs::msg::Float64>(
            "/maneuver/target_speed", 10,
            [this](std_msgs::msg::Float64::SharedPtr m)
            {
                maneuver_speed_ = m->data;
                maneuver_time_ = now();
            });

        sub_state_ = create_subscription<std_msgs::msg::String>(
            "/maneuver/state", 10, [this](std_msgs::msg::String::SharedPtr m)
            {
                turning_ = m->data == "left" || m->data == "right";
                state_time_ = now();
            });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pred_pub_ = create_publisher<nav_msgs::msg::Path>("/mpc_predicted_path", 10);
        ref_pub_ = create_publisher<nav_msgs::msg::Path>("/mpc_reference_path", 10);
        dbg_vref_pub_ = create_publisher<std_msgs::msg::Float64>("/mpc/v_ref_ramped", 10);
        dbg_accel_pub_ = create_publisher<std_msgs::msg::Float64>("/mpc/cmd_accel", 10);
        dbg_steer_pub_ = create_publisher<std_msgs::msg::Float64>("/mpc/cmd_steer", 10);
        // turn-exit settling diagnostics (see mpc/README.md discussion): foot-point
        // error at solve time, plus the lookahead actually used this tick, so a
        // slow post-turn settle can be told apart from an oscillating one and
        // correlated against the lookahead value in effect.
        dbg_elat_pub_ = create_publisher<std_msgs::msg::Float64>("/mpc/e_lat", 10);
        dbg_ehead_pub_ = create_publisher<std_msgs::msg::Float64>("/mpc/e_head", 10);
        dbg_lookahead_pub_ = create_publisher<std_msgs::msg::Float64>("/mpc/lookahead_used", 10);

        auto period = std::chrono::duration<double>(1.0 / rate_hz_);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]()
            { control_tick(); });
    }

private:
    // public vref and control command for debug
    void publish_debug(double vref, double accel, double steer)
    {
        std_msgs::msg::Float64 m;
        m.data = vref;
        dbg_vref_pub_->publish(m);
        m.data = accel;
        dbg_accel_pub_->publish(m);
        m.data = steer;
        dbg_steer_pub_->publish(m);
    }

    // publish stop cmd
    void stop_cmd()
    {
        geometry_msgs::msg::Twist cmd{};
        cmd_pub_->publish(cmd);
        publish_debug(v_ref_cmd_, 0.0, 0.0);
    }

    void control_tick()
    {
        // quite is odom is not available
        if (!odom_)
            return;
        double v_now = std::hypot(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y);
        // stop cmd if path is not right
        if (!path_ || path_->poses.empty())
        {
            stop_cmd();
            return;
        }
        // stop cmd if path is stale
        if ((now() - path_time_).seconds() > path_timeout_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "path stale, stopping");
            stop_cmd();
            return;
        }

        // update vehicle state
        VehicleState state;
        state.x = odom_->pose.pose.position.x;
        state.y = odom_->pose.pose.position.y;
        const auto &q = odom_->pose.pose.orientation;
        state.psi = std::atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z));
        state.v = v_now;

        // update path points
        auto pts = path_msg_to_points(*path_);
        if (pts.empty())
        {
            stop_cmd();
            return;
        }

        size_t closest = find_closest_point(pts, state, search_radius_);

        // ---- reference speed: profile at foot point, min maneuver setpoint
        bool profile_ok = profile_.size() == pts.size();
        double v_profile = profile_ok ? profile_[closest] : 0.0;
        bool profile_end_zero = profile_ok && profile_.back() <= 0.01;
        double desired = profile_ok ? v_profile : 2.0; // conservative w/o profile
        bool maneuver_fresh = maneuver_time_.nanoseconds() > 0 &&
                              (now() - maneuver_time_).seconds() < 1.0;

        if (maneuver_fresh)
            desired = std::min(desired, maneuver_speed_);
        else if (hold_without_maneuver_)
        {
            // no live sign chain: hold (slew ramps to 0). Fail-safe replacement for
            // the removed stale-safety estop; also the hold-at-spawn gate.
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "no fresh /maneuver/target_speed - holding");
            desired = 0.0;
        }
        desired = std::clamp(desired, 0.0, vp_.max_speed);

        // ---- stop-point arrival predicate (v1 deadlock fix)
        const auto &goal = path_->poses.back().pose.position;
        double dist_to_goal = std::hypot(goal.x - state.x, goal.y - state.y);
        bool stop_commanded_here =
            profile_ok && profile_[closest] <= 0.05 && profile_end_zero;
        if (stop_commanded_here && dist_to_goal < goal_tol_ + 2.0 && v_now < 0.1)
        {
            // arrived at a commanded stop: hold brake, stay in control, no latch -
            // a new plan with a receding stop point resumes automatically
            stop_cmd();
            return;
        }

        // slew v_ref (v1 KB §2.20 pattern, asymmetric)
        const double dt_tick = 1.0 / rate_hz_;
        double d = desired - v_ref_cmd_;
        double step = (d >= 0.0 ? accel_rate_ : decel_rate_) * dt_tick;
        v_ref_cmd_ += std::clamp(d, -step, step);

        // set solver speed
        solver_->set_target_speed(v_ref_cmd_);
        // heading lookahead: while turning use the fixed turn lookahead, else a
        // speed-based lookahead clamped to [min, max]. A stale /maneuver/state
        // (topic gone silent) must not latch us in turn mode, so the turn flag
        // only counts when it is fresh.
        bool turning_fresh = state_time_.nanoseconds() > 0 &&
                             (now() - state_time_).seconds() < 1.0;
        double lookahead_used =
            (turning_ && turning_fresh)
                ? look_turn_
                : std::clamp(state.v * look_time_, look_min_, look_max_);
        solver_->set_lookahead(lookahead_used);
        {
            std_msgs::msg::Float64 m;
            m.data = lookahead_used;
            dbg_lookahead_pub_->publish(m);
        }

        double ds = std::max(state.v * dt_, 0.3);
        auto ref = resample_path(pts, closest, N_ + 1, ds);

        nav_msgs::msg::Path ref_msg;
        ref_msg.header.stamp = now();
        ref_msg.header.frame_id = "odom";
        for (const auto &p : ref)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = ref_msg.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ref_msg.poses.push_back(ps);
        }
        ref_pub_->publish(ref_msg);

        auto result = solver_->Solve(state, ref, prev_accel_, prev_steer_);
        if (!result.success)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "MPC solve failed");
            stop_cmd();
            return;
        }
        if (result.used_fallback != last_fallback_)
        {
            RCLCPP_INFO(get_logger(), "control source: %s",
                        result.used_fallback ? "PROPORTIONAL fallback" : "OSQP QP");
            last_fallback_ = result.used_fallback;
        }

        prev_accel_ = result.accel;

        // actuator-aware steering slew: never command a steer step the joint
        // cannot follow (model-plant mismatch = the limit-cycle driver)
        double max_dsteer = steer_slew_ * dt_tick;
        steer_cmd_ += std::clamp(result.steer - steer_cmd_, -max_dsteer, max_dsteer);
        prev_steer_ = steer_cmd_;

        double angular_z = state.v / vp_.wheelbase * std::tan(steer_cmd_);
        double new_v =
            std::clamp(state.v + result.accel / rate_hz_, 0.0, vp_.max_speed);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = new_v;
        cmd.angular.z = angular_z;
        cmd_pub_->publish(cmd);
        publish_debug(v_ref_cmd_, result.accel, steer_cmd_);
        {
            std_msgs::msg::Float64 m;
            m.data = result.e_lat;
            dbg_elat_pub_->publish(m);
            m.data = result.e_head;
            dbg_ehead_pub_->publish(m);
        }

        nav_msgs::msg::Path pred;
        pred.header.stamp = now();
        pred.header.frame_id = "odom";
        for (const auto &s : result.predicted_states)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = pred.header;
            ps.pose.position.x = s.x;
            ps.pose.position.y = s.y;
            pred.poses.push_back(ps);
        }
        pred_pub_->publish(pred);
    }

    double rate_hz_, path_timeout_, search_radius_, goal_tol_,
        accel_rate_, decel_rate_, look_turn_,
        steer_slew_ = 0.7, look_time_ = 1.8, look_min_ = 4.0, look_max_ = 14.0;
    int N_;
    double dt_ = 0.1; // MPC step time [s], mirror of mp.dt for path resampling
    bool hold_without_maneuver_ = true, turning_ = false, last_fallback_ = false;
    double prev_accel_ = 0, prev_steer_ = 0, v_ref_cmd_ = 0, steer_cmd_ = 0;
    double maneuver_speed_ = 0.0;

    VehicleParams vp_;
    std::unique_ptr<MpcSolver> solver_;

    nav_msgs::msg::Path::SharedPtr path_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

    std::vector<float> profile_;
    rclcpp::Time path_time_{0, 0, RCL_ROS_TIME};

    nav_msgs::msg::Odometry::SharedPtr odom_;

    rclcpp::Time maneuver_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time state_time_{0, 0, RCL_ROS_TIME}; // last /maneuver/state stamp

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_profile_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_maneuver_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pred_pub_, ref_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dbg_vref_pub_, dbg_accel_pub_, dbg_steer_pub_,
        dbg_elat_pub_, dbg_ehead_pub_, dbg_lookahead_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MpcTrackerV2>());
    rclcpp::shutdown();
    return 0;
}