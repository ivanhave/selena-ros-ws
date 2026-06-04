#pragma once

#include <atomic>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "robot_navigation/navigation_provider.hpp"

namespace robot_navigation
{

// ── OdometryNavigator ─────────────────────────────────────────────────────────
// NavigationProvider implementation that uses wheel-encoder odometry.
// Subscribes to /diff_drive_controller/odom for pose and velocity.
//
// move_to(target):
//   1. If target is behind the robot (angle_diff > π/2): drive backward, no rotation.
//      This allows linear exit from a strip without rotating (mandatory safety constraint).
//   2. Otherwise: rotate_to(direction_to_target), then drive forward.
//   3. Final rotate_to(target.yaw) to align heading at destination.
//
// rotate_to(yaw): rotates in place at ROTATE_SPEED_RADPS, settles when error < ANGLE_TOL_RAD.
//
// To switch to Nav2: replace with Nav2Navigator — zero changes elsewhere.

class OdometryNavigator : public NavigationProvider
{
public:
    explicit OdometryNavigator(rclcpp::Node::SharedPtr node);

    Pose2D get_pose() override;
    bool   move_to(Pose2D target) override;
    bool   align_to_yaw(float target_yaw) override;
    bool   rotate_to(float target_yaw, float angle_tol = ANGLE_TOL_RAD);
    void   abort() override;
    void   reset_abort() override;

private:
    static constexpr float SPEED_MPS          = 0.1f;
    static constexpr float ARRIVAL_TOL_M      = 0.02f;
    static constexpr float STOPPED_VEL_MPS    = 0.01f;
    static constexpr float ROTATE_SPEED_RADPS = 0.3f;
    static constexpr float ROTATE_KP          = 2.0f;   // P gain: full speed at 0.15 rad error
    static constexpr float ANGLE_TOL_RAD      = 0.05f;  // general rotation tolerance (~2.9°)
    static constexpr float PLANT_ALIGN_TOL    = 0.02f;  // pre-plant alignment tolerance (~1.1°)
    static constexpr float DECEL_DIST_M       = 0.30f;  // start linear decel this far from target
    static constexpr float MIN_SPEED_FRACTION = 0.25f;  // minimum drive speed (fraction of SPEED_MPS)
    static constexpr float TRACK_KP           = 2.0f;   // P gain for heading correction during drive
    static constexpr float MAX_TRACK_WZ       = 0.30f;  // max angular correction while moving (rad/s)
    static constexpr int   MOVE_TIMEOUT_MS    = 120000;
    static constexpr int   STOP_TIMEOUT_MS    = 3000;
    static constexpr int   ROTATE_TIMEOUT_MS  = 10000;

    void publish_velocity(float vx, float wz = 0.0f);

    rclcpp::Node::SharedPtr node_;
    mutable std::mutex mutex_;
    std::atomic<bool> abort_flag_{false};

    float odom_x_   = 0.0f;
    float odom_y_   = 0.0f;
    float odom_yaw_ = 0.0f;
    float odom_vx_  = 0.0f;
    float odom_vz_  = 0.0f;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
};

} // namespace robot_navigation
