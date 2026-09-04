// odom_noise - overlay Gaussian measurement noise on wheel odometry's twist
// (vx, vy, wz), independent of the slip1/slip2 mechanism in
// sedan_description/urdf/sedan.urdf.xacro.
//
// The two are different EKF-vs-wheel-odom failure modes, not substitutes:
// slip1/slip2 breaks the AckermannSteering plugin's no-slip KINEMATIC
// ASSUMPTION itself under load, producing a systematic (non-zero-mean) yaw
// error that only shows up in real turns. This node instead adds honest
// zero-mean sensor noise on top of an otherwise-correct no-slip model - the
// kind of thing a Kalman filter is specifically good at rejecting, so an
// EKF-beats-wheel-odom demo built on noise alone is a milder, less
// mechanistically interesting test than slip (see the 2026-09-04
// turn-settling diagnosis notes in mpc_solver/mpc_tracker_v2_node). Useful on
// its own, or layered with slip once that is re-enabled.
//
// Subscribes to the pristine wheel odometry the Gazebo bridge publishes it as
// (default /odom_wheel_raw) and republishes a noisy copy on /odom, the topic
// every existing consumer (the EKF's odom0, RViz, odom_compare) already
// expects - a pure drop-in relay, no other config changes needed beyond the
// bridge's ros_topic_name rename in bridge_sedan.yaml.
//
// Pose (position/orientation) is left untouched: the EKF's odom0_config fuses
// only twist vx/vy (see ekf_sedan.yaml), so that's the only channel this
// experiment needs noisy, and a physically honest noisy position would need
// to integrate the twist noise into a random walk, not just jitter each
// message independently.

#include <random>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class OdomNoise : public rclcpp::Node
{
public:
    OdomNoise() : rclcpp::Node("odom_noise"), gen_(std::random_device{}())
    {
        in_topic_ = declare_parameter("in_topic", std::string("/odom_wheel_raw"));
        out_topic_ = declare_parameter("out_topic", std::string("/odom"));
        // defaults: a modest fraction of the sedan's cruise speed / yaw rate,
        // not derived - tune empirically the same way slip1/slip2 was.
        stddev_vx_ = declare_parameter("stddev_linear_x", 0.03);  // m/s
        stddev_vy_ = declare_parameter("stddev_linear_y", 0.03);  // m/s
        stddev_wz_ = declare_parameter("stddev_angular_z", 0.03); // rad/s

        dist_vx_ = std::normal_distribution<double>(0.0, stddev_vx_);
        dist_vy_ = std::normal_distribution<double>(0.0, stddev_vy_);
        dist_wz_ = std::normal_distribution<double>(0.0, stddev_wz_);

        pub_ = create_publisher<nav_msgs::msg::Odometry>(out_topic_, 10);
        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            in_topic_, 10,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            { pub_->publish(add_noise(*msg)); });

        RCLCPP_INFO(get_logger(),
                    "odom_noise: %s -> %s, stddev [vx=%.3f vy=%.3f wz=%.3f]",
                    in_topic_.c_str(), out_topic_.c_str(), stddev_vx_, stddev_vy_, stddev_wz_);
    }

private:
    nav_msgs::msg::Odometry add_noise(nav_msgs::msg::Odometry msg)
    {
        msg.twist.twist.linear.x += dist_vx_(gen_);
        msg.twist.twist.linear.y += dist_vy_(gen_);
        msg.twist.twist.angular.z += dist_wz_(gen_);

        // report the injected variance so a covariance-aware consumer (the
        // EKF) actually weighs this source down, instead of fusing noisy
        // data still tagged with Gazebo's near-zero original covariance.
        // twist.covariance is row-major [vx,vy,vz,wx,wy,wz]x6; diagonal i*6+i.
        msg.twist.covariance[0 * 6 + 0] = stddev_vx_ * stddev_vx_;
        msg.twist.covariance[1 * 6 + 1] = stddev_vy_ * stddev_vy_;
        msg.twist.covariance[5 * 6 + 5] = stddev_wz_ * stddev_wz_;

        return msg;
    }

    std::string in_topic_, out_topic_;
    double stddev_vx_, stddev_vy_, stddev_wz_;
    std::mt19937 gen_;
    std::normal_distribution<double> dist_vx_, dist_vy_, dist_wz_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomNoise>());
    rclcpp::shutdown();
    return 0;
}
